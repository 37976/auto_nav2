#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sophus/se3.hpp>

#include "System.h"

#include "cv_bridge/cv_bridge.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace
{

const char * trackingStateToString(const int state)
{
  switch (state) {
    case ORB_SLAM3::Tracking::SYSTEM_NOT_READY:
      return "SYSTEM_NOT_READY";
    case ORB_SLAM3::Tracking::NO_IMAGES_YET:
      return "NO_IMAGES_YET";
    case ORB_SLAM3::Tracking::NOT_INITIALIZED:
      return "NOT_INITIALIZED";
    case ORB_SLAM3::Tracking::OK:
      return "OK";
    case ORB_SLAM3::Tracking::RECENTLY_LOST:
      return "RECENTLY_LOST";
    case ORB_SLAM3::Tracking::LOST:
      return "LOST";
    case ORB_SLAM3::Tracking::OK_KLT:
      return "OK_KLT";
    default:
      return "UNKNOWN";
  }
}

}  // namespace

class SuperPointLocalizationNode : public rclcpp::Node
{
public:
  SuperPointLocalizationNode()
  : Node("superpoint_localization_node")
  {
    const auto package_share_dir =
      ament_index_cpp::get_package_share_directory("sp_orb_slam_localization");
    const auto default_vocabulary_path = package_share_dir + "/vendor/Vocabulary/ORBvoc.txt";
    const auto default_model_path = package_share_dir + "/vendor/weights/superpoint.ts";

    declare_parameter<std::string>("image_topic", "/camera/image_raw");
    declare_parameter<std::string>("depth_topic", "/camera/aligned_depth_to_color/image_raw");
    declare_parameter<std::string>("sensor_mode", "rgbd");
    declare_parameter<std::string>("vocabulary_file", default_vocabulary_path);
    declare_parameter<std::string>("settings_file", "");
    declare_parameter<std::string>("superpoint_model_file", default_model_path);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("child_frame", "camera_link");
    declare_parameter<bool>("publish_tf", false);
    declare_parameter<bool>("force_gray", true);
    declare_parameter<double>("max_depth_age_sec", 1.0);
    declare_parameter<bool>("localization_only", true);

    image_topic_ = get_parameter("image_topic").as_string();
    depth_topic_ = get_parameter("depth_topic").as_string();
    sensor_mode_ = get_parameter("sensor_mode").as_string();
    vocabulary_file_ = get_parameter("vocabulary_file").as_string();
    settings_file_ = get_parameter("settings_file").as_string();
    superpoint_model_file_ = get_parameter("superpoint_model_file").as_string();
    map_frame_ = get_parameter("map_frame").as_string();
    child_frame_ = get_parameter("child_frame").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    force_gray_ = get_parameter("force_gray").as_bool();
    max_depth_age_sec_ = get_parameter("max_depth_age_sec").as_double();
    localization_only_ = get_parameter("localization_only").as_bool();

    if (sensor_mode_ == "rgbd" || sensor_mode_ == "RGBD") {
      use_rgbd_ = true;
    } else if (sensor_mode_ == "monocular" || sensor_mode_ == "mono") {
      use_rgbd_ = false;
    } else {
      throw std::runtime_error("Parameter 'sensor_mode' must be 'rgbd' or 'monocular'.");
    }

    if (vocabulary_file_.empty()) {
      vocabulary_file_ = default_vocabulary_path;
    }

    if (superpoint_model_file_.empty()) {
      superpoint_model_file_ = default_model_path;
    }

    if (settings_file_.empty()) {
      throw std::runtime_error("Parameter 'settings_file' must point to your camera yaml.");
    }

    if (!superpoint_model_file_.empty()) {
      setenv("ORB_SLAM3_SP_MODEL_PATH", superpoint_model_file_.c_str(), 1);
    }

    const auto sensor = use_rgbd_ ? ORB_SLAM3::System::RGBD : ORB_SLAM3::System::MONOCULAR;
    slam_ = std::make_unique<ORB_SLAM3::System>(
      vocabulary_file_, settings_file_, sensor, false);

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("sp_orb_slam/pose", 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("sp_orb_slam/odom", 10);

    if (publish_tf_) {
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

    image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&SuperPointLocalizationNode::imageCallback, this, std::placeholders::_1));

    if (use_rgbd_) {
      depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_,
        rclcpp::SensorDataQoS(),
        std::bind(&SuperPointLocalizationNode::depthCallback, this, std::placeholders::_1));
    }

    RCLCPP_INFO(get_logger(), "Sensor mode: %s", use_rgbd_ ? "RGB-D" : "Monocular");
    RCLCPP_INFO(get_logger(), "Subscribed image topic: %s", image_topic_.c_str());
    if (use_rgbd_) {
      RCLCPP_INFO(get_logger(), "Subscribed depth topic: %s", depth_topic_.c_str());
    }
    RCLCPP_INFO(get_logger(), "Vocabulary file: %s", vocabulary_file_.c_str());
    RCLCPP_INFO(get_logger(), "Settings file: %s", settings_file_.c_str());
    RCLCPP_INFO(get_logger(), "SuperPoint model: %s", superpoint_model_file_.c_str());
    RCLCPP_INFO(get_logger(), "Localization only: %s", localization_only_ ? "true" : "false");
  }

  ~SuperPointLocalizationNode() override
  {
    if (slam_) {
      slam_->Shutdown();
    }
  }

private:
  void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_depth_msg_ = msg;
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    ++received_frames_;

    cv_bridge::CvImageConstPtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge conversion failed: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image;
    if (image.empty()) {
      return;
    }

    if (force_gray_ && image.channels() == 3) {
      cv::cvtColor(image, image, cv::COLOR_BGR2GRAY);
    } else if (force_gray_ && image.channels() == 4) {
      cv::cvtColor(image, image, cv::COLOR_BGRA2GRAY);
    }

    const double stamp = rclcpp::Time(msg->header.stamp).seconds();
    Sophus::SE3f Tcw;
    if (use_rgbd_) {
      sensor_msgs::msg::Image::SharedPtr depth_msg;
      {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        depth_msg = latest_depth_msg_;
      }

      if (!depth_msg) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Frames=%zu published=%zu waiting for aligned depth frames on %s.",
          received_frames_, published_frames_, depth_topic_.c_str());
        return;
      }

      const double depth_stamp = rclcpp::Time(depth_msg->header.stamp).seconds();
      const double depth_age = std::abs(stamp - depth_stamp);
      if (depth_age > max_depth_age_sec_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Frames=%zu published=%zu depth is too old (%.3fs > %.3fs).",
          received_frames_, published_frames_, depth_age, max_depth_age_sec_);
        return;
      }

      cv_bridge::CvImageConstPtr depth_ptr;
      try {
        depth_ptr = cv_bridge::toCvShare(depth_msg, depth_msg->encoding);
      } catch (const cv_bridge::Exception & e) {
        RCLCPP_ERROR(get_logger(), "depth cv_bridge conversion failed: %s", e.what());
        return;
      }

      cv::Mat depth = depth_ptr->image;
      if (depth.empty()) {
        return;
      }

      if (depth.size() != image.size()) {
        cv::resize(depth, depth, image.size(), 0.0, 0.0, cv::INTER_NEAREST);
      }

      Tcw = slam_->TrackRGBD(image, depth, stamp);
    } else {
      Tcw = slam_->TrackMonocular(image, stamp);
    }
    const int tracking_state = slam_->GetTrackingState();

    const bool has_valid_tracking =
      tracking_state == ORB_SLAM3::Tracking::OK ||
      tracking_state == ORB_SLAM3::Tracking::RECENTLY_LOST ||
      tracking_state == ORB_SLAM3::Tracking::OK_KLT;

    if (!has_valid_tracking) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Frames=%zu published=%zu tracking=%s(%d) still initializing, pose is not published yet.",
        received_frames_, published_frames_, trackingStateToString(tracking_state), tracking_state);
      return;
    }

    const Eigen::Matrix4f pose_matrix = Tcw.matrix();
    if (!pose_matrix.allFinite()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Frames=%zu published=%zu tracking=%s(%d) pose invalid, still waiting for stable localization.",
        received_frames_, published_frames_, trackingStateToString(tracking_state), tracking_state);
      return;
    }

    if (localization_only_ && !localization_mode_activated_) {
      slam_->ActivateLocalizationMode();
      localization_mode_activated_ = true;
      RCLCPP_INFO(get_logger(), "Activated localization-only mode after first valid pose.");
    }

    const Sophus::SE3f Twc = Tcw.inverse();
    const Eigen::Vector3f t = Twc.translation();
    const Eigen::Quaternionf q(Twc.rotationMatrix());

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    pose_msg.header.frame_id = map_frame_;
    pose_msg.pose.position.x = t.x();
    pose_msg.pose.position.y = t.y();
    pose_msg.pose.position.z = t.z();
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();
    pose_pub_->publish(pose_msg);
    ++published_frames_;

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = child_frame_;
    odom_msg.pose.pose = pose_msg.pose;
    odom_pub_->publish(odom_msg);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Frames=%zu published=%zu tracking=%s(%d) pose=(%.3f, %.3f, %.3f)",
      received_frames_, published_frames_, trackingStateToString(tracking_state), tracking_state,
      t.x(), t.y(), t.z());

    if (publish_tf_ && tf_broadcaster_) {
      geometry_msgs::msg::TransformStamped tf_msg;
      tf_msg.header = pose_msg.header;
      tf_msg.child_frame_id = child_frame_;
      tf_msg.transform.translation.x = t.x();
      tf_msg.transform.translation.y = t.y();
      tf_msg.transform.translation.z = t.z();
      tf_msg.transform.rotation = pose_msg.pose.orientation;
      tf_broadcaster_->sendTransform(tf_msg);
    }
  }

  std::string image_topic_;
  std::string depth_topic_;
  std::string sensor_mode_;
  std::string vocabulary_file_;
  std::string settings_file_;
  std::string superpoint_model_file_;
  std::string map_frame_;
  std::string child_frame_;
  bool publish_tf_{false};
  bool force_gray_{true};
  bool use_rgbd_{true};
  bool localization_only_{true};
  bool localization_mode_activated_{false};
  double max_depth_age_sec_{1.0};
  size_t received_frames_{0};
  size_t published_frames_{0};

  std::mutex depth_mutex_;
  sensor_msgs::msg::Image::SharedPtr latest_depth_msg_;
  std::unique_ptr<ORB_SLAM3::System> slam_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<SuperPointLocalizationNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "superpoint_localization_node failed: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
