#include "SuperPointExtractor.h"
#include <iostream>
#include <algorithm>
#include <numeric>
#include <limits>
#include <cmath>
#include <stdexcept>

namespace ORB_SLAM3 {

SuperPointExtractor::SuperPointExtractor(const std::string& model_path,
                                         int nfeatures,
                                         float scaleFactor,
                                         int nlevels,
                                         float confidence_threshold)
    : mConfidenceThreshold(confidence_threshold),
      mnFeatures(nfeatures),
      mfScaleFactor(scaleFactor),
      mnLevels(nlevels)
{
    try {
        mModel = torch::jit::load(model_path);
        mModel.eval();
    }
    catch (const c10::Error& e) {
        std::cerr << "Error loading SuperPoint model: " << e.what() << std::endl;
        throw;
    }

    mvScaleFactors.resize(mnLevels);
    mvLevelSigma2.resize(mnLevels);
    mvScaleFactors[0] = 1.0f;
    mvLevelSigma2[0] = 1.0f;

    for(int i = 1; i < mnLevels; i++)
    {
        mvScaleFactors[i] = mvScaleFactors[i - 1] * mfScaleFactor;
        mvLevelSigma2[i] = mvScaleFactors[i] * mvScaleFactors[i];
    }

    mvInvScaleFactors.resize(mnLevels);
    mvInvLevelSigma2.resize(mnLevels);
    for(int i = 0; i < mnLevels; i++)
    {
        mvInvScaleFactors[i] = 1.0f / mvScaleFactors[i];
        mvInvLevelSigma2[i] = 1.0f / mvLevelSigma2[i];
    }

    mvImagePyramid.resize(mnLevels);
}

int SuperPointExtractor::operator()(cv::InputArray _image, cv::InputArray _mask,
                                    std::vector<cv::KeyPoint>& keypoints,
                                    cv::OutputArray _descriptors,
                                    std::vector<int>& vLappingArea)
{
    torch::InferenceMode guard;

    cv::Mat image = _image.getMat();
    if(image.empty())
        return -1;

    cv::Mat grayImage;
    if(image.channels() > 1)
        cv::cvtColor(image, grayImage, cv::COLOR_BGR2GRAY);
    else
        grayImage = image.clone();

    ComputePyramid(grayImage);

    std::vector<cv::KeyPoint> allKeypoints;
    cv::Mat allDescriptors;

    for(int level = 0; level < mnLevels; ++level)
    {
        const cv::Mat& imgLevel = mvImagePyramid[level];

        torch::Tensor inputTensor = PreprocessImage(imgLevel);

        std::vector<torch::jit::IValue> inputs;
        inputs.push_back(inputTensor);

        auto output = mModel.forward(inputs).toTuple();

        torch::Tensor semi = output->elements()[0].toTensor().detach().to(torch::kCPU).contiguous();
        torch::Tensor dense_desc = output->elements()[1].toTensor().detach().to(torch::kCPU).contiguous();

        torch::Tensor heatmap = SemiToHeatmap(semi);

        std::vector<cv::KeyPoint> levelKps = ExtractKeypoints(heatmap, mConfidenceThreshold);

        if(levelKps.empty())
            continue;

        cv::Mat levelDesc = ExtractDescriptors(dense_desc, levelKps, imgLevel.rows, imgLevel.cols);

        float scale = mvScaleFactors[level];

        for(size_t i = 0; i < levelKps.size(); ++i)
        {
            levelKps[i].pt.x *= scale;
            levelKps[i].pt.y *= scale;
            levelKps[i].octave = level;
            levelKps[i].size = 8.0f * scale;
            allKeypoints.push_back(levelKps[i]);
        }

        if(allDescriptors.empty())
            allDescriptors = levelDesc.clone();
        else
            cv::vconcat(allDescriptors, levelDesc, allDescriptors);
    }

    if(allKeypoints.empty())
    {
        _descriptors.release();
        return 0;
    }

    if(allDescriptors.rows != (int)allKeypoints.size())
    {
        std::cerr << "Descriptor/Keypoint size mismatch." << std::endl;
        _descriptors.release();
        return 0;
    }

    std::vector<int> keepIdx = ApplyANMS(allKeypoints, mnFeatures);

    keypoints.clear();
    cv::Mat finalDesc((int)keepIdx.size(), allDescriptors.cols, allDescriptors.type());

    for(size_t i = 0; i < keepIdx.size(); ++i)
    {
        int idx = keepIdx[i];
        keypoints.push_back(allKeypoints[idx]);
        allDescriptors.row(idx).copyTo(finalDesc.row((int)i));
    }

    finalDesc.copyTo(_descriptors);
    return static_cast<int>(keypoints.size());
}

torch::Tensor SuperPointExtractor::PreprocessImage(const cv::Mat& image)
{
    cv::Mat float_img;
    image.convertTo(float_img, CV_32F, 1.0 / 255.0);

    auto tensor_img = torch::from_blob(
        float_img.data,
        {1, 1, float_img.rows, float_img.cols},
        torch::TensorOptions().dtype(torch::kFloat32)
    );

    // 关键：必须 clone，否则 float_img 释放后 tensor 悬空
    return tensor_img.clone();
}

torch::Tensor SuperPointExtractor::SemiToHeatmap(const torch::Tensor& semi)
{
    torch::Tensor x = semi.detach().to(torch::kCPU).contiguous();

    // 情况1：已经是 [1,1,H,W]
    if(x.dim() == 4 && x.size(1) == 1)
    {
        return x.squeeze(0).squeeze(0).contiguous(); // [H,W]
    }

    // 情况2：标准 SuperPoint semi 输出 [1,65,Hc,Wc]
    if(x.dim() == 4 && x.size(1) == 65)
    {
        auto prob = torch::softmax(x, 1);          // [1,65,Hc,Wc]
        prob = prob.slice(1, 0, 64);               // 去掉 dustbin -> [1,64,Hc,Wc]

        const int Hc = static_cast<int>(prob.size(2));
        const int Wc = static_cast<int>(prob.size(3));

        prob = prob.permute({0, 2, 3, 1}).contiguous();     // [1,Hc,Wc,64]
        prob = prob.view({1, Hc, Wc, 8, 8});                // [1,Hc,Wc,8,8]
        prob = prob.permute({0, 1, 3, 2, 4}).contiguous();  // [1,Hc,8,Wc,8]
        prob = prob.view({1, Hc * 8, Wc * 8});              // [1,H,W]

        return prob.squeeze(0).contiguous();                // [H,W]
    }

    // 情况3：已经是 [H,W]
    if(x.dim() == 2)
        return x.contiguous();

    // 情况4：可能是 [1,H,W]
    if(x.dim() == 3)
        return x.squeeze(0).contiguous();

    throw std::runtime_error("SuperPoint semi tensor shape not supported.");
}

std::vector<cv::KeyPoint> SuperPointExtractor::ExtractKeypoints(const torch::Tensor& heatmap, float threshold)
{
    torch::Tensor scores = heatmap.detach().to(torch::kCPU).contiguous();

    if(scores.dim() != 2)
        throw std::runtime_error("Heatmap must be 2D.");

    const int height = static_cast<int>(scores.size(0));
    const int width  = static_cast<int>(scores.size(1));

    auto scores_acc = scores.accessor<float,2>();

    std::vector<cv::KeyPoint> raw_keypoints;
    raw_keypoints.reserve(4096);

    const int border = 2;
    for(int h = border; h < height - border; ++h)
    {
        for(int w = border; w < width - border; ++w)
        {
            float score = scores_acc[h][w];
            if(score > threshold)
            {
                raw_keypoints.emplace_back(
                    cv::Point2f(static_cast<float>(w), static_cast<float>(h)),
                    8.0f, -1.0f, score, 0, -1
                );
            }
        }
    }

    // 按响应值排序
    std::sort(raw_keypoints.begin(), raw_keypoints.end(),
              [](const cv::KeyPoint& a, const cv::KeyPoint& b)
              {
                  return a.response > b.response;
              });

    // 简单 NMS，避免点过密
    const int nms_dist = 2;
    std::vector<unsigned char> occupied(height * width, 0);

    std::vector<cv::KeyPoint> keypoints;
    keypoints.reserve(raw_keypoints.size());

    for(const auto& kp : raw_keypoints)
    {
        int x = static_cast<int>(std::round(kp.pt.x));
        int y = static_cast<int>(std::round(kp.pt.y));

        if(occupied[y * width + x])
            continue;

        keypoints.push_back(kp);

        int x0 = std::max(0, x - nms_dist);
        int x1 = std::min(width - 1, x + nms_dist);
        int y0 = std::max(0, y - nms_dist);
        int y1 = std::min(height - 1, y + nms_dist);

        for(int yy = y0; yy <= y1; ++yy)
            for(int xx = x0; xx <= x1; ++xx)
                occupied[yy * width + xx] = 1;
    }

    return keypoints;
}

cv::Mat SuperPointExtractor::ExtractDescriptors(const torch::Tensor& descriptors,
                                                const std::vector<cv::KeyPoint>& keypoints,
                                                int image_h, int image_w)
{
    if(keypoints.empty())
        return cv::Mat();

    torch::Tensor desc = descriptors.detach().to(torch::kCPU).contiguous();

    // 统一成 [1,C,Hd,Wd]
    if(desc.dim() == 3)
        desc = desc.unsqueeze(0);

    if(desc.dim() != 4)
        throw std::runtime_error("Descriptor tensor must be 4D [1,C,H,W].");

    const int desc_dim = static_cast<int>(desc.size(1));
    const int Hd = static_cast<int>(desc.size(2));
    const int Wd = static_cast<int>(desc.size(3));

    auto desc_acc = desc.accessor<float,4>();

    cv::Mat out_desc(static_cast<int>(keypoints.size()), desc_dim, CV_32F);

    for(size_t i = 0; i < keypoints.size(); ++i)
    {
        const cv::KeyPoint& kp = keypoints[i];

        float x_desc = 0.0f;
        float y_desc = 0.0f;

        if(image_w > 1) x_desc = kp.pt.x * static_cast<float>(Wd - 1) / static_cast<float>(image_w - 1);
        if(image_h > 1) y_desc = kp.pt.y * static_cast<float>(Hd - 1) / static_cast<float>(image_h - 1);

        int x0 = std::max(0, std::min(Wd - 1, static_cast<int>(std::floor(x_desc))));
        int y0 = std::max(0, std::min(Hd - 1, static_cast<int>(std::floor(y_desc))));
        int x1 = std::min(Wd - 1, x0 + 1);
        int y1 = std::min(Hd - 1, y0 + 1);

        float dx = x_desc - static_cast<float>(x0);
        float dy = y_desc - static_cast<float>(y0);

        float* row = out_desc.ptr<float>(static_cast<int>(i));
        float norm2 = 0.0f;

        for(int d = 0; d < desc_dim; ++d)
        {
            float v00 = desc_acc[0][d][y0][x0];
            float v01 = desc_acc[0][d][y0][x1];
            float v10 = desc_acc[0][d][y1][x0];
            float v11 = desc_acc[0][d][y1][x1];

            float v0 = v00 * (1.0f - dx) + v01 * dx;
            float v1 = v10 * (1.0f - dx) + v11 * dx;
            float v  = v0 * (1.0f - dy) + v1 * dy;

            row[d] = v;
            norm2 += v * v;
        }

        float norm = std::sqrt(norm2) + 1e-8f;
        for(int d = 0; d < desc_dim; ++d)
            row[d] /= norm;
    }

    return out_desc;
}

std::vector<int> SuperPointExtractor::ApplyANMS(const std::vector<cv::KeyPoint>& keypoints, const int num_features)
{
    if(keypoints.size() <= static_cast<size_t>(num_features))
    {
        std::vector<int> keep(keypoints.size());
        for(size_t i = 0; i < keypoints.size(); ++i) keep[i] = (int)i;
        return keep;
    }

    std::vector<int> indices((int)keypoints.size());
    for(int i = 0; i < (int)keypoints.size(); ++i) indices[i] = i;

    std::sort(indices.begin(), indices.end(),
              [&keypoints](int i1, int i2) {
                  return keypoints[i1].response > keypoints[i2].response;
              });

    indices.resize(num_features);
    return indices;
}

void SuperPointExtractor::ComputePyramid(cv::Mat image)
{
    for (int level = 0; level < mnLevels; ++level)
    {
        float scale = mvInvScaleFactors[level];
        cv::Size sz(cvRound((float)image.cols * scale), cvRound((float)image.rows * scale));
        cv::resize(image, mvImagePyramid[level], sz, 0, 0, cv::INTER_LINEAR);
    }
}

} // namespace ORB_SLAM3