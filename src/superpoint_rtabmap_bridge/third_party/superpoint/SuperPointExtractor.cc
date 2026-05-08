#include "SuperPointExtractor.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace ORB_SLAM3
{

SuperPointExtractor::SuperPointExtractor(
  const std::string & model_path, int nfeatures, float scale_factor, int nlevels,
  float confidence_threshold)
: confidence_threshold_(confidence_threshold),
  features_(nfeatures),
  scale_factor_(scale_factor),
  levels_(nlevels)
{
  try {
    model_ = torch::jit::load(model_path);
    model_.eval();
  } catch (const c10::Error & e) {
    std::cerr << "Error loading SuperPoint model: " << e.what() << std::endl;
    throw;
  }

  scale_factors_.resize(levels_);
  level_sigma2_.resize(levels_);
  scale_factors_[0] = 1.0f;
  level_sigma2_[0] = 1.0f;

  for (int i = 1; i < levels_; ++i) {
    scale_factors_[i] = scale_factors_[i - 1] * scale_factor_;
    level_sigma2_[i] = scale_factors_[i] * scale_factors_[i];
  }

  inv_scale_factors_.resize(levels_);
  inv_level_sigma2_.resize(levels_);
  for (int i = 0; i < levels_; ++i) {
    inv_scale_factors_[i] = 1.0f / scale_factors_[i];
    inv_level_sigma2_[i] = 1.0f / level_sigma2_[i];
  }

  image_pyramid_.resize(levels_);
}

int SuperPointExtractor::operator()(
  cv::InputArray _image, cv::InputArray, std::vector<cv::KeyPoint> & keypoints,
  cv::OutputArray _descriptors, std::vector<int> &)
{
  torch::InferenceMode guard;

  cv::Mat image = _image.getMat();
  if (image.empty()) {
    return -1;
  }

  cv::Mat gray_image;
  if (image.channels() > 1) {
    cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);
  } else {
    gray_image = image.clone();
  }

  ComputePyramid(gray_image);

  std::vector<cv::KeyPoint> all_keypoints;
  cv::Mat all_descriptors;

  for (int level = 0; level < levels_; ++level) {
    const cv::Mat & img_level = image_pyramid_[level];
    torch::Tensor input_tensor = PreprocessImage(img_level);

    std::vector<torch::jit::IValue> inputs;
    inputs.push_back(input_tensor);
    auto output = model_.forward(inputs).toTuple();

    torch::Tensor semi = output->elements()[0].toTensor().detach().to(torch::kCPU).contiguous();
    torch::Tensor dense_desc =
      output->elements()[1].toTensor().detach().to(torch::kCPU).contiguous();

    torch::Tensor heatmap = SemiToHeatmap(semi);
    std::vector<cv::KeyPoint> level_keypoints = ExtractKeypoints(heatmap, confidence_threshold_);
    if (level_keypoints.empty()) {
      continue;
    }

    cv::Mat level_desc =
      ExtractDescriptors(dense_desc, level_keypoints, img_level.rows, img_level.cols);

    const float scale = scale_factors_[level];
    for (auto & keypoint : level_keypoints) {
      keypoint.pt.x *= scale;
      keypoint.pt.y *= scale;
      keypoint.octave = level;
      keypoint.size = 8.0f * scale;
      all_keypoints.push_back(keypoint);
    }

    if (all_descriptors.empty()) {
      all_descriptors = level_desc.clone();
    } else {
      cv::vconcat(all_descriptors, level_desc, all_descriptors);
    }
  }

  if (all_keypoints.empty()) {
    _descriptors.release();
    return 0;
  }

  if (all_descriptors.rows != static_cast<int>(all_keypoints.size())) {
    std::cerr << "Descriptor/Keypoint size mismatch." << std::endl;
    _descriptors.release();
    return 0;
  }

  const std::vector<int> keep_idx = ApplyANMS(all_keypoints, features_);
  keypoints.clear();
  cv::Mat final_desc(static_cast<int>(keep_idx.size()), all_descriptors.cols, all_descriptors.type());
  for (size_t i = 0; i < keep_idx.size(); ++i) {
    const int idx = keep_idx[i];
    keypoints.push_back(all_keypoints[idx]);
    all_descriptors.row(idx).copyTo(final_desc.row(static_cast<int>(i)));
  }

  final_desc.copyTo(_descriptors);
  return static_cast<int>(keypoints.size());
}

torch::Tensor SuperPointExtractor::PreprocessImage(const cv::Mat & image)
{
  cv::Mat float_img;
  image.convertTo(float_img, CV_32F, 1.0 / 255.0);

  auto tensor_img = torch::from_blob(
    float_img.data, {1, 1, float_img.rows, float_img.cols},
    torch::TensorOptions().dtype(torch::kFloat32));

  return tensor_img.clone();
}

torch::Tensor SuperPointExtractor::SemiToHeatmap(const torch::Tensor & semi)
{
  torch::Tensor x = semi.detach().to(torch::kCPU).contiguous();

  if (x.dim() == 4 && x.size(1) == 1) {
    return x.squeeze(0).squeeze(0).contiguous();
  }

  if (x.dim() == 4 && x.size(1) == 65) {
    auto prob = torch::softmax(x, 1);
    prob = prob.slice(1, 0, 64);

    const int hc = static_cast<int>(prob.size(2));
    const int wc = static_cast<int>(prob.size(3));

    prob = prob.permute({0, 2, 3, 1}).contiguous();
    prob = prob.view({1, hc, wc, 8, 8});
    prob = prob.permute({0, 1, 3, 2, 4}).contiguous();
    prob = prob.view({1, hc * 8, wc * 8});

    return prob.squeeze(0).contiguous();
  }

  if (x.dim() == 2) {
    return x.contiguous();
  }

  if (x.dim() == 3) {
    return x.squeeze(0).contiguous();
  }

  throw std::runtime_error("SuperPoint semi tensor shape not supported.");
}

std::vector<cv::KeyPoint> SuperPointExtractor::ExtractKeypoints(
  const torch::Tensor & heatmap, float threshold)
{
  torch::Tensor scores = heatmap.detach().to(torch::kCPU).contiguous();
  if (scores.dim() != 2) {
    throw std::runtime_error("Heatmap must be 2D.");
  }

  const int height = static_cast<int>(scores.size(0));
  const int width = static_cast<int>(scores.size(1));
  auto scores_acc = scores.accessor<float, 2>();

  std::vector<cv::KeyPoint> raw_keypoints;
  raw_keypoints.reserve(4096);

  const int border = 2;
  for (int h = border; h < height - border; ++h) {
    for (int w = border; w < width - border; ++w) {
      const float score = scores_acc[h][w];
      if (score > threshold) {
        raw_keypoints.emplace_back(
          cv::Point2f(static_cast<float>(w), static_cast<float>(h)), 8.0f, -1.0f, score, 0, -1);
      }
    }
  }

  std::sort(
    raw_keypoints.begin(), raw_keypoints.end(),
    [](const cv::KeyPoint & a, const cv::KeyPoint & b) {return a.response > b.response;});

  const int nms_dist = 2;
  std::vector<unsigned char> occupied(height * width, 0);
  std::vector<cv::KeyPoint> filtered;
  filtered.reserve(raw_keypoints.size());

  for (const auto & keypoint : raw_keypoints) {
    const int x = static_cast<int>(std::round(keypoint.pt.x));
    const int y = static_cast<int>(std::round(keypoint.pt.y));
    if (occupied[y * width + x]) {
      continue;
    }

    filtered.push_back(keypoint);
    const int x0 = std::max(0, x - nms_dist);
    const int x1 = std::min(width - 1, x + nms_dist);
    const int y0 = std::max(0, y - nms_dist);
    const int y1 = std::min(height - 1, y + nms_dist);
    for (int yy = y0; yy <= y1; ++yy) {
      for (int xx = x0; xx <= x1; ++xx) {
        occupied[yy * width + xx] = 1;
      }
    }
  }

  return filtered;
}

cv::Mat SuperPointExtractor::ExtractDescriptors(
  const torch::Tensor & descriptors, const std::vector<cv::KeyPoint> & keypoints, int image_h,
  int image_w)
{
  if (keypoints.empty()) {
    return cv::Mat();
  }

  torch::Tensor desc = descriptors.detach().to(torch::kCPU).contiguous();
  if (desc.dim() != 4) {
    throw std::runtime_error("Descriptor tensor must be 4D.");
  }

  if (desc.size(0) != 1) {
    throw std::runtime_error("Only batch size 1 is supported.");
  }

  desc = desc.squeeze(0);  // [C,Hc,Wc]
  const int channels = static_cast<int>(desc.size(0));
  const int hc = static_cast<int>(desc.size(1));
  const int wc = static_cast<int>(desc.size(2));
  const float scale_x = static_cast<float>(wc) / static_cast<float>(image_w);
  const float scale_y = static_cast<float>(hc) / static_cast<float>(image_h);

  cv::Mat output(static_cast<int>(keypoints.size()), channels, CV_32F);
  auto desc_acc = desc.accessor<float, 3>();

  for (size_t i = 0; i < keypoints.size(); ++i) {
    const float xf = std::clamp(keypoints[i].pt.x * scale_x, 0.0f, static_cast<float>(wc - 1));
    const float yf = std::clamp(keypoints[i].pt.y * scale_y, 0.0f, static_cast<float>(hc - 1));
    const int x = static_cast<int>(std::round(xf));
    const int y = static_cast<int>(std::round(yf));

    float norm = 0.0f;
    for (int c = 0; c < channels; ++c) {
      const float value = desc_acc[c][y][x];
      output.at<float>(static_cast<int>(i), c) = value;
      norm += value * value;
    }

    norm = std::sqrt(std::max(norm, 1e-12f));
    for (int c = 0; c < channels; ++c) {
      output.at<float>(static_cast<int>(i), c) /= norm;
    }
  }

  return output;
}

std::vector<int> SuperPointExtractor::ApplyANMS(
  const std::vector<cv::KeyPoint> & keypoints, int num_features)
{
  if (static_cast<int>(keypoints.size()) <= num_features) {
    std::vector<int> keep(keypoints.size());
    std::iota(keep.begin(), keep.end(), 0);
    return keep;
  }

  std::vector<float> radius(keypoints.size(), std::numeric_limits<float>::max());
  for (size_t i = 0; i < keypoints.size(); ++i) {
    for (size_t j = 0; j < i; ++j) {
      if (keypoints[j].response <= keypoints[i].response) {
        continue;
      }
      const float dx = keypoints[i].pt.x - keypoints[j].pt.x;
      const float dy = keypoints[i].pt.y - keypoints[j].pt.y;
      const float dist2 = dx * dx + dy * dy;
      if (dist2 < radius[i]) {
        radius[i] = dist2;
      }
    }
  }

  std::vector<int> order(keypoints.size());
  std::iota(order.begin(), order.end(), 0);
  std::sort(
    order.begin(), order.end(),
    [&](int a, int b) {return radius[a] > radius[b];});
  order.resize(num_features);
  return order;
}

void SuperPointExtractor::ComputePyramid(cv::Mat image)
{
  image_pyramid_[0] = image.clone();
  for (int level = 1; level < levels_; ++level) {
    const float scale = inv_scale_factors_[level];
    const cv::Size sz(
      static_cast<int>(std::round(image.cols * scale)),
      static_cast<int>(std::round(image.rows * scale)));
    cv::resize(image, image_pyramid_[level], sz, 0, 0, cv::INTER_LINEAR);
  }
}

}  // namespace ORB_SLAM3
