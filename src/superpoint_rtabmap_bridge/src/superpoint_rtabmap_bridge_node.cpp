#include "SuperPointExtractor.h"

#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <rclcpp/rclcpp.hpp>
#include <rtabmap/core/Compression.h>
#include <rtabmap_conversions/MsgConversion.h>
#include <rtabmap_msgs/msg/point3f.hpp>
#include <rtabmap_msgs/msg/rgbd_image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace
{

using RGBDPolicy = message_filters::sync_policies::ApproximateTime<
  sensor_msgs::msg::Image,
  sensor_msgs::msg::Image>;

bool readDepthMeters(
  const sensor_msgs::msg::Image & depth_msg, int u, int v, double depth_scale, float depth_min,
  float depth_max, float & depth_m)
{
  if (u < 0 || v < 0 || u >= static_cast<int>(depth_msg.width) || v >= static_cast<int>(depth_msg.height)) {
    return false;
  }

  if (depth_msg.encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
    const auto * raw = reinterpret_cast<const uint16_t *>(depth_msg.data.data() + v * depth_msg.step);
    const uint16_t value = raw[u];
    if (value == 0) {
      return false;
    }
    depth_m = static_cast<float>(value * depth_scale);
  } else if (depth_msg.encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    const auto * raw = reinterpret_cast<const float *>(depth_msg.data.data() + v * depth_msg.step);
    depth_m = raw[u];
    if (!std::isfinite(depth_m) || depth_m <= 0.0f) {
      return false;
    }
  } else {
    return false;
  }

  if (!std::isfinite(depth_m) || depth_m < depth_min || depth_m > depth_max) {
    return false;
  }
  return true;
}

}  // namespace

class SuperPointRtabmapBridgeNode : public rclcpp::Node
{
public:
  SuperPointRtabmapBridgeNode()
  : Node("superpoint_rtabmap_bridge_node")
  {
    declare_parameter<std::string>("rgb_topic", "/camera/camera/color/image_raw");
    declare_parameter<std::string>("depth_topic", "/camera/camera/aligned_depth_to_color/image_raw");
    declare_parameter<std::string>("camera_info_topic", "/camera/camera/color/camera_info");
    declare_parameter<std::string>("output_rgbd_topic", "/superpoint/rgbd_image");
    declare_parameter<std::string>(
      "superpoint_model_file",
      "/home/xu/project/sp_orb_slam_localization/vendor/superpoint_orb_slam3/weights/superpoint.ts");
    declare_parameter<int>("max_features", 400);
    declare_parameter<int>("pyramid_levels", 1);
    declare_parameter<double>("scale_factor", 1.2);
    declare_parameter<double>("confidence_threshold", 0.001);
    declare_parameter<double>("depth_scale", 0.001);
    declare_parameter<double>("depth_min_m", 0.15);
    declare_parameter<double>("depth_max_m", 5.0);
    declare_parameter<int>("sync_queue_size", 10);
    declare_parameter<bool>("force_gray", true);
    declare_parameter<double>("output_rate_hz", 1.0);

    const auto rgb_topic = get_parameter("rgb_topic").as_string();
    const auto depth_topic = get_parameter("depth_topic").as_string();
    const auto camera_info_topic = get_parameter("camera_info_topic").as_string();
    const auto output_rgbd_topic = get_parameter("output_rgbd_topic").as_string();
    const auto superpoint_model_file = get_parameter("superpoint_model_file").as_string();
    const auto max_features = get_parameter("max_features").as_int();
    const auto pyramid_levels = get_parameter("pyramid_levels").as_int();
    const auto scale_factor = static_cast<float>(get_parameter("scale_factor").as_double());
    const auto confidence_threshold =
      static_cast<float>(get_parameter("confidence_threshold").as_double());
    depth_scale_ = get_parameter("depth_scale").as_double();
    depth_min_m_ = static_cast<float>(get_parameter("depth_min_m").as_double());
    depth_max_m_ = static_cast<float>(get_parameter("depth_max_m").as_double());
    force_gray_ = get_parameter("force_gray").as_bool();
    const auto sync_queue_size = get_parameter("sync_queue_size").as_int();
    output_period_sec_ = 1.0 / std::max(0.01, get_parameter("output_rate_hz").as_double());

    extractor_ = std::make_unique<ORB_SLAM3::SuperPointExtractor>(
      superpoint_model_file, max_features, scale_factor, pyramid_levels, confidence_threshold);

    pub_ = create_publisher<rtabmap_msgs::msg::RGBDImage>(output_rgbd_topic, rclcpp::SensorDataQoS());

    rgb_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, rgb_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
    depth_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(
      this, depth_topic, rclcpp::SensorDataQoS().get_rmw_qos_profile());
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      camera_info_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&SuperPointRtabmapBridgeNode::cameraInfoCallback, this, std::placeholders::_1));

    sync_ = std::make_shared<message_filters::Synchronizer<RGBDPolicy>>(
      RGBDPolicy(sync_queue_size), *rgb_sub_, *depth_sub_);
    sync_->registerCallback(
      std::bind(&SuperPointRtabmapBridgeNode::rgbdCallback, this, std::placeholders::_1,
      std::placeholders::_2));

    RCLCPP_INFO(get_logger(), "SuperPoint model: %s", superpoint_model_file.c_str());
    RCLCPP_INFO(get_logger(), "RGB topic: %s", rgb_topic.c_str());
    RCLCPP_INFO(get_logger(), "Depth topic: %s", depth_topic.c_str());
    RCLCPP_INFO(get_logger(), "Camera info topic: %s", camera_info_topic.c_str());
    RCLCPP_INFO(get_logger(), "Output RTAB-Map RGBDImage topic: %s", output_rgbd_topic.c_str());
    RCLCPP_INFO(get_logger(), "Bridge publish rate limited to %.3f Hz", 1.0 / output_period_sec_);
  }

private:
  void rgbdCallback(
    const sensor_msgs::msg::Image::ConstSharedPtr & rgb_msg,
    const sensor_msgs::msg::Image::ConstSharedPtr & depth_msg)
  {
    auto camera_info_msg = last_camera_info_;
    if (!camera_info_msg) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Waiting for camera_info before publishing SuperPoint RGBDImage.");
      return;
    }

    const double current_stamp =
      static_cast<double>(rgb_msg->header.stamp.sec) +
      static_cast<double>(rgb_msg->header.stamp.nanosec) * 1e-9;
    if (last_publish_stamp_sec_ && (current_stamp - *last_publish_stamp_sec_) < output_period_sec_) {
      return;
    }

    cv_bridge::CvImageConstPtr rgb_bridge;
    try {
      rgb_bridge = cv_bridge::toCvShare(rgb_msg, rgb_msg->encoding);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "RGB cv_bridge conversion failed: %s", e.what());
      return;
    }

    cv::Mat image = rgb_bridge->image;
    if (image.empty()) {
      return;
    }

    cv::Mat extractor_input = image;
    if (force_gray_) {
      if (image.channels() == 3) {
        cv::cvtColor(image, extractor_input, cv::COLOR_BGR2GRAY);
      } else if (image.channels() == 4) {
        cv::cvtColor(image, extractor_input, cv::COLOR_BGRA2GRAY);
      } else {
        extractor_input = image.clone();
      }
    }

    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    std::vector<int> lapping_area;
    const int feature_count = (*extractor_)(extractor_input, cv::noArray(), keypoints, descriptors, lapping_area);
    if (feature_count <= 0 || keypoints.empty() || descriptors.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "SuperPoint produced no valid features for current frame.");
      return;
    }

    const float fx = static_cast<float>(camera_info_msg->k[0]);
    const float fy = static_cast<float>(camera_info_msg->k[4]);
    const float cx = static_cast<float>(camera_info_msg->k[2]);
    const float cy = static_cast<float>(camera_info_msg->k[5]);

    std::vector<cv::KeyPoint> filtered_keypoints;
    std::vector<rtabmap_msgs::msg::Point3f> filtered_points;
    cv::Mat filtered_descriptors;

    filtered_keypoints.reserve(keypoints.size());
    filtered_points.reserve(keypoints.size());

    for (int i = 0; i < descriptors.rows; ++i) {
      const cv::KeyPoint & keypoint = keypoints[static_cast<size_t>(i)];
      const int u = static_cast<int>(std::round(keypoint.pt.x));
      const int v = static_cast<int>(std::round(keypoint.pt.y));

      float depth_m = 0.0f;
      if (!readDepthMeters(*depth_msg, u, v, depth_scale_, depth_min_m_, depth_max_m_, depth_m)) {
        continue;
      }

      rtabmap_msgs::msg::Point3f point;
      point.x = (static_cast<float>(u) - cx) * depth_m / fx;
      point.y = (static_cast<float>(v) - cy) * depth_m / fy;
      point.z = depth_m;

      filtered_keypoints.push_back(keypoint);
      filtered_points.push_back(point);

      if (filtered_descriptors.empty()) {
        filtered_descriptors = descriptors.row(i).clone();
      } else {
        filtered_descriptors.push_back(descriptors.row(i));
      }
    }

    if (filtered_keypoints.empty() || filtered_descriptors.empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "SuperPoint found features, but none had valid aligned depth.");
      return;
    }

    rtabmap_msgs::msg::RGBDImage msg;
    msg.header = rgb_msg->header;
    msg.rgb_camera_info = *camera_info_msg;
    msg.depth_camera_info = *camera_info_msg;
    msg.rgb = *rgb_msg;
    msg.depth = *depth_msg;
    rtabmap_conversions::keypointsToROS(filtered_keypoints, msg.key_points);
    msg.points = filtered_points;
    msg.descriptors = rtabmap::compressData(filtered_descriptors);

    pub_->publish(std::move(msg));
    last_publish_stamp_sec_ = current_stamp;
    ++published_frames_;
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Published SuperPoint RGBDImage frame #%zu with %zu valid features.",
      published_frames_, filtered_keypoints.size());
  }

  void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    last_camera_info_ = msg;
  }

  double depth_scale_{0.001};
  float depth_min_m_{0.15f};
  float depth_max_m_{5.0f};
  bool force_gray_{true};
  double output_period_sec_{1.0};
  std::size_t published_frames_{0};
  std::optional<double> last_publish_stamp_sec_;

  std::unique_ptr<ORB_SLAM3::SuperPointExtractor> extractor_;
  rclcpp::Publisher<rtabmap_msgs::msg::RGBDImage>::SharedPtr pub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;

  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub_;
  std::shared_ptr<message_filters::Synchronizer<RGBDPolicy>> sync_;
  sensor_msgs::msg::CameraInfo::ConstSharedPtr last_camera_info_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SuperPointRtabmapBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
