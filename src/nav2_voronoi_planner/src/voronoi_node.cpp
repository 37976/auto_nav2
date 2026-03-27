// Voronoi 规划节点的 ROS2 调度层：
// 负责订阅输入、触发重规划，以及发布路径和骨架结果。
#include "nav2_voronoi_planner/voronoi_node.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_voronoi_planner/voronoi_path_utils.hpp"

namespace nav2_voronoi_planner
{

VoronoiNode::VoronoiNode()
: Node("voronoi")
{
  robot_radius_ = this->declare_parameter<double>("robot_radius", 0.20);
  occ_threshold_ = this->declare_parameter<int>("occ_threshold", 50);
  unknown_is_obstacle_ = this->declare_parameter<bool>("unknown_is_obstacle", true);
  publish_debug_path2_ = this->declare_parameter<bool>("publish_debug_path2", true);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.2);
  plan_period_ms_ = this->declare_parameter<double>("plan_period_ms", 500.0);
  replan_min_move_ = this->declare_parameter<double>("replan_min_move", 0.15);

  planner_ = std::make_unique<VoronoiGridPlanner>(VoronoiGridPlanner::Config{
      robot_radius_,
      occ_threshold_,
      unknown_is_obstacle_});

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  skeleton_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/voronoi_skeleton", 1);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
  path2_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path2", 10);

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/combined_grid", rclcpp::SensorDataQoS(),
    std::bind(&VoronoiNode::mapCallback, this, std::placeholders::_1));
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
    "/goal_pose", 10,
    std::bind(&VoronoiNode::goalCallback, this, std::placeholders::_1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 50,
    std::bind(&VoronoiNode::odomCallback, this, std::placeholders::_1));

  plan_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(static_cast<int>(plan_period_ms_)),
    std::bind(&VoronoiNode::planTimerCallback, this));

  RCLCPP_INFO(this->get_logger(), "VoronoiNode started.");
  RCLCPP_INFO(this->get_logger(), "Subscribed: /combined_grid /goal_pose /odom");
  RCLCPP_INFO(this->get_logger(), "Publishing: /path /path2 /voronoi_skeleton");
  RCLCPP_INFO(this->get_logger(), "Plan period: %.1f ms", plan_period_ms_);
}

void VoronoiNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  map_ = msg;
  has_map_ = true;
  map_dirty_ = true;

  if (has_goal_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Received combined_grid: %u x %u, resolution=%.3f",
      map_->info.width, map_->info.height, map_->info.resolution);
    need_replan_ = true;
  }
}

void VoronoiNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  odom_ = msg;
  has_odom_ = true;
}

void VoronoiNode::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);

  last_goal_ = *goal_msg;
  if (last_goal_.header.frame_id.empty() && has_map_) {
    last_goal_.header.frame_id = map_->header.frame_id;
  }

  goal_reached_ = false;
  has_goal_ = true;
  goal_dirty_ = true;
  need_replan_ = true;
}

void VoronoiNode::tryPlanWithSnapshot(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & map_local,
  const nav_msgs::msg::Odometry::SharedPtr & odom_local,
  const geometry_msgs::msg::PoseStamped & goal_local)
{
  if (!map_local || !odom_local) {
    return;
  }

  const double dx = odom_local->pose.pose.position.x - goal_local.pose.position.x;
  const double dy = odom_local->pose.pose.position.y - goal_local.pose.position.y;
  const double distance = std::hypot(dx, dy);

  if (distance <= goal_tolerance_) {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      goal_reached_ = true;
      has_goal_ = false;
      goal_dirty_ = false;
      need_replan_ = false;
      map_dirty_ = false;
    }

    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = map_local->header.frame_id;
    empty_path.header.stamp = this->now();
    path_pub_->publish(empty_path);
    path2_pub_->publish(empty_path);

    publishStopCmd();
    RCLCPP_INFO(this->get_logger(), "Goal reached. Stop replanning.");
    return;
  }

  geometry_msgs::msg::PoseStamped start;
  start.header.frame_id = map_local->header.frame_id;
  start.header.stamp = this->now();
  start.pose = odom_local->pose.pose;

  geometry_msgs::msg::PoseStamped goal = goal_local;
  if (goal.header.frame_id.empty()) {
    goal.header.frame_id = map_local->header.frame_id;
  }

  nav_msgs::msg::Path plan;
  nav_msgs::msg::OccupancyGrid skeleton;
  if (!planner_->makePlanFromMap(
      *map_local, start, goal, plan, &skeleton, this->get_logger()))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Voronoi replanning failed.");
    return;
  }

  skeleton.header.stamp = this->now();
  skeleton_pub_->publish(skeleton);
  path_pub_->publish(plan);

  if (publish_debug_path2_) {
    nav_msgs::msg::Path smooth = smoothPathBSpline(plan, static_cast<int>(plan.poses.size()), 3);
    nav_msgs::msg::Path path2 = downsamplePath(smooth, 2);
    path2_pub_->publish(path2);
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    goal_dirty_ = false;
    need_replan_ = false;
    map_dirty_ = false;
    last_plan_x_ = odom_local->pose.pose.position.x;
    last_plan_y_ = odom_local->pose.pose.position.y;
    has_last_plan_pose_ = true;
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "Published Voronoi path, size = %zu", plan.poses.size());
}

void VoronoiNode::planTimerCallback()
{
  nav_msgs::msg::OccupancyGrid::SharedPtr map_local;
  nav_msgs::msg::Odometry::SharedPtr odom_local;
  geometry_msgs::msg::PoseStamped goal_local;

  bool goal_reached_local = false;
  bool has_map_local = false;
  bool has_odom_local = false;
  bool has_goal_local = false;
  bool goal_dirty_local = false;
  bool need_replan_local = false;
  bool map_dirty_local = false;
  bool has_last_plan_pose_local = false;
  double last_plan_x_local = 0.0;
  double last_plan_y_local = 0.0;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    goal_reached_local = goal_reached_;
    has_map_local = has_map_;
    has_odom_local = has_odom_;
    has_goal_local = has_goal_;
    goal_dirty_local = goal_dirty_;
    need_replan_local = need_replan_;
    map_dirty_local = map_dirty_;
    has_last_plan_pose_local = has_last_plan_pose_;
    last_plan_x_local = last_plan_x_;
    last_plan_y_local = last_plan_y_;

    if (has_map_) {
      map_local = map_;
    }
    if (has_odom_) {
      odom_local = odom_;
    }
    if (has_goal_) {
      goal_local = last_goal_;
    }
  }

  if (goal_reached_local) {
    return;
  }
  if (!has_map_local) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "No /combined_grid received yet.");
    return;
  }
  if (!has_odom_local) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "No /odom received yet.");
    return;
  }
  if (!has_goal_local) {
    return;
  }
  if (!need_replan_local && !map_dirty_local) {
    return;
  }

  if (has_last_plan_pose_local && !map_dirty_local && !goal_dirty_local) {
    const double dx = odom_local->pose.pose.position.x - last_plan_x_local;
    const double dy = odom_local->pose.pose.position.y - last_plan_y_local;
    const double moved = std::hypot(dx, dy);
    if (moved < replan_min_move_) {
      return;
    }
  }

  tryPlanWithSnapshot(map_local, odom_local, goal_local);
}

void VoronoiNode::publishStopCmd()
{
  geometry_msgs::msg::Twist stop_cmd;
  stop_cmd.linear.x = 0.0;
  stop_cmd.linear.y = 0.0;
  stop_cmd.linear.z = 0.0;
  stop_cmd.angular.x = 0.0;
  stop_cmd.angular.y = 0.0;
  stop_cmd.angular.z = 0.0;

  for (int i = 0; i < 5; ++i) {
    cmd_vel_pub_->publish(stop_cmd);
  }
}

}  // namespace nav2_voronoi_planner
