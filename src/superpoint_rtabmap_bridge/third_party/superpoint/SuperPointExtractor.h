#ifndef SUPERPOINT_RTABMAP_BRIDGE_SUPERPOINTEXTRACTOR_H_
#define SUPERPOINT_RTABMAP_BRIDGE_SUPERPOINTEXTRACTOR_H_

#include "FeatureExtractor.h"

#include <opencv2/opencv.hpp>
#include <torch/script.h>

#include <string>
#include <vector>

namespace ORB_SLAM3
{

class SuperPointExtractor : public FeatureExtractor
{
public:
  SuperPointExtractor(
    const std::string & model_path, int nfeatures, float scale_factor, int nlevels,
    float confidence_threshold = 0.001f);

  ~SuperPointExtractor() override = default;

  int operator()(
    cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint> & keypoints,
    cv::OutputArray descriptors, std::vector<int> & lapping_area) override;

  std::vector<float> GetScaleFactors() override {return scale_factors_;}
  std::vector<float> GetInverseScaleFactors() override {return inv_scale_factors_;}
  std::vector<float> GetScaleSigmaSquares() override {return level_sigma2_;}
  std::vector<float> GetInverseScaleSigmaSquares() override {return inv_level_sigma2_;}

  int GetLevels() override {return levels_;}
  float GetScaleFactor() override {return scale_factor_;}
  const std::vector<cv::Mat> & GetImagePyramid() override {return image_pyramid_;}

protected:
  void ComputePyramid(cv::Mat image) override;

private:
  torch::Tensor PreprocessImage(const cv::Mat & image);
  torch::Tensor SemiToHeatmap(const torch::Tensor & semi);
  std::vector<cv::KeyPoint> ExtractKeypoints(const torch::Tensor & heatmap, float threshold);
  cv::Mat ExtractDescriptors(
    const torch::Tensor & descriptors, const std::vector<cv::KeyPoint> & keypoints, int image_h,
    int image_w);
  std::vector<int> ApplyANMS(const std::vector<cv::KeyPoint> & keypoints, int num_features);

  torch::jit::Module model_;
  float confidence_threshold_;
  int features_;
  float scale_factor_;
  int levels_;

  std::vector<float> scale_factors_;
  std::vector<float> inv_scale_factors_;
  std::vector<float> level_sigma2_;
  std::vector<float> inv_level_sigma2_;
  std::vector<cv::Mat> image_pyramid_;
};

}  // namespace ORB_SLAM3

#endif  // SUPERPOINT_RTABMAP_BRIDGE_SUPERPOINTEXTRACTOR_H_
