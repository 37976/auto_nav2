#ifndef SUPERPOINT_RTABMAP_BRIDGE_FEATUREEXTRACTOR_H_
#define SUPERPOINT_RTABMAP_BRIDGE_FEATUREEXTRACTOR_H_

#include <opencv2/opencv.hpp>

#include <vector>

namespace ORB_SLAM3
{

class FeatureExtractor
{
public:
  virtual ~FeatureExtractor() = default;

  virtual int operator()(
    cv::InputArray image, cv::InputArray mask, std::vector<cv::KeyPoint> & keypoints,
    cv::OutputArray descriptors, std::vector<int> & lapping_area) = 0;

  virtual std::vector<float> GetScaleFactors() = 0;
  virtual std::vector<float> GetInverseScaleFactors() = 0;
  virtual std::vector<float> GetScaleSigmaSquares() = 0;
  virtual std::vector<float> GetInverseScaleSigmaSquares() = 0;

  virtual int GetLevels() = 0;
  virtual float GetScaleFactor() = 0;
  virtual const std::vector<cv::Mat> & GetImagePyramid() = 0;

protected:
  virtual void ComputePyramid(cv::Mat image) = 0;
};

}  // namespace ORB_SLAM3

#endif  // SUPERPOINT_RTABMAP_BRIDGE_FEATUREEXTRACTOR_H_
