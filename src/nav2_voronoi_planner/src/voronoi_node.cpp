// Voronoi 规划节点的 ROS2 调度层：
// 负责订阅输入、触发重规划，以及发布路径和骨架结果。
#include "nav2_voronoi_planner/voronoi_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_voronoi_planner/util.hpp"
#include "nav2_voronoi_planner/voronoi_path_utils.hpp"

namespace nav2_voronoi_planner
{

VoronoiNode::VoronoiNode()
: Node("voronoi")
{
  robot_radius_ = this->declare_parameter<double>("robot_radius", 0.14);
  clearance_margin_ = this->declare_parameter<double>("clearance_margin", 0.03);
  occ_threshold_ = this->declare_parameter<int>("occ_threshold", 50);
  unknown_is_obstacle_ = this->declare_parameter<bool>("unknown_is_obstacle", true);
  publish_debug_path2_ = this->declare_parameter<bool>("publish_debug_path2", true);
  goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.2);
  plan_period_ms_ = this->declare_parameter<double>("plan_period_ms", 500.0);
  replan_min_move_ = this->declare_parameter<double>("replan_min_move", 0.15);
  trunk_safety_penalty_scale_ = this->declare_parameter<double>(
    "trunk_safety_penalty_scale", 0.06);
  connector_candidate_count_ = this->declare_parameter<int>("connector_candidate_count", 0);
  path_smoothing_control_step_ = this->declare_parameter<int>("path_smoothing_control_step", 2);
  stable_map_replan_period_ms_ = this->declare_parameter<double>(
    "stable_map_replan_period_ms", 3000.0);
  map_significant_change_cells_ = this->declare_parameter<int>(
    "map_significant_change_cells", 50);
  path_obstacle_check_distance_m_ = this->declare_parameter<double>(
    "path_obstacle_check_distance_m", 2.0);
  path_switch_min_improvement_m_ = this->declare_parameter<double>(
    "path_switch_min_improvement_m", 0.5);

  planner_ = std::make_unique<VoronoiGridPlanner>(VoronoiGridPlanner::Config{
      robot_radius_,
      clearance_margin_,
      occ_threshold_,
      unknown_is_obstacle_,
      trunk_safety_penalty_scale_,
      connector_candidate_count_});

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
  RCLCPP_INFO(
    this->get_logger(),
    "Clearance rule: robot_radius=%.2f m, extra_margin=%.2f m, connector_candidates=%d (0=all)",
    robot_radius_, clearance_margin_, connector_candidate_count_);
  RCLCPP_INFO(
    this->get_logger(),
    "Map replan gate: stable_period=%.0f ms, significant_change_cells=%d, "
    "path_check=%.2f m, switch_improvement=%.2f m",
    stable_map_replan_period_ms_, map_significant_change_cells_,
    path_obstacle_check_distance_m_, path_switch_min_improvement_m_);
  RCLCPP_INFO(this->get_logger(), "Subscribed: /combined_grid /goal_pose /odom");
  RCLCPP_INFO(this->get_logger(), "Publishing: /path /path2 /voronoi_skeleton");
  RCLCPP_INFO(this->get_logger(), "Plan period: %.1f ms", plan_period_ms_);
}

void VoronoiNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  bool request_replan = false;
  bool significant_change = false;
  bool path_blocked = false;
  bool stop_for_path_block = false;
  int changed_cells = 0;
  int blocked_path_index = -1;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (has_goal_) {
      significant_change = !has_map_ || isSignificantMapChange(*map_, *msg, &changed_cells);
      path_blocked =
        has_odom_ && has_published_plan_ &&
        isPathBlockedByMap(
          last_published_plan_, *odom_, *msg, path_obstacle_check_distance_m_,
          &blocked_path_index);

      const auto now = this->now();
      const bool slow_replan_due =
        !has_last_map_replan_request_ ||
        ((now - last_map_replan_request_time_).seconds() * 1000.0 >=
        stable_map_replan_period_ms_);

      request_replan = goal_dirty_ || path_blocked || significant_change || slow_replan_due;
      if (request_replan) {
        last_map_replan_request_time_ = now;
        has_last_map_replan_request_ = true;
      }
      stop_for_path_block = path_blocked;
    }

    map_ = msg;
    has_map_ = true;

    if (request_replan) {
      map_dirty_ = true;
      need_replan_ = true;
    }
  }

  if (stop_for_path_block) {
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = msg->header.frame_id.empty() ? "map" : msg->header.frame_id;
    empty_path.header.stamp = this->now();
    path_pub_->publish(empty_path);
    path2_pub_->publish(empty_path);
    publishStopCmd();
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Current path blocked near path index %d. Stop and request immediate replan.",
      blocked_path_index);
  }

  if (request_replan) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Map update scheduled replan: %u x %u, resolution=%.3f, changed_cells=%d, "
      "significant=%s, path_blocked=%s",
      msg->info.width, msg->info.height, msg->info.resolution,
      changed_cells, significant_change ? "true" : "false",
      path_blocked ? "true" : "false");
  }
}

int VoronoiNode::classifyMapCell(int8_t value) const
{
  if (value < 0) {
    return unknown_is_obstacle_ ? 1 : -1;
  }
  return (value >= occ_threshold_) ? 1 : 0;
}

bool VoronoiNode::isSignificantMapChange(
  const nav_msgs::msg::OccupancyGrid & previous,
  const nav_msgs::msg::OccupancyGrid & current,
  int * changed_cells) const
{
  if (changed_cells) {
    *changed_cells = 0;
  }

  if (
    previous.info.width != current.info.width ||
    previous.info.height != current.info.height ||
    previous.info.resolution != current.info.resolution ||
    previous.info.origin.position.x != current.info.origin.position.x ||
    previous.info.origin.position.y != current.info.origin.position.y ||
    previous.data.size() != current.data.size())
  {
    if (changed_cells) {
      *changed_cells = static_cast<int>(current.data.size());
    }
    return true;
  }

  int changes = 0;
  for (size_t i = 0; i < current.data.size(); ++i) {
    if (classifyMapCell(previous.data[i]) != classifyMapCell(current.data[i])) {
      ++changes;
      if (changes >= map_significant_change_cells_) {
        if (changed_cells) {
          *changed_cells = changes;
        }
        return true;
      }
    }
  }

  if (changed_cells) {
    *changed_cells = changes;
  }
  return false;
}

bool VoronoiNode::isPathBlockedByMap(
  const nav_msgs::msg::Path & path,
  const nav_msgs::msg::Odometry & odom,
  const nav_msgs::msg::OccupancyGrid & map,
  double check_distance_m,
  int * blocked_path_index) const
{
  if (blocked_path_index) {
    *blocked_path_index = -1;
  }
  if (path.poses.empty() || map.info.resolution <= 0.0 || map.data.empty()) {
    return false;
  }

  const double robot_x = odom.pose.pose.position.x;
  const double robot_y = odom.pose.pose.position.y;
  size_t closest_index = 0;
  double best_dist_sq = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & position = path.poses[i].pose.position;
    const double dx = position.x - robot_x;
    const double dy = position.y - robot_y;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      closest_index = i;
    }
  }

  const int width = static_cast<int>(map.info.width);
  const int height = static_cast<int>(map.info.height);
  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  const double max_check_distance = std::max(0.0, check_distance_m);

  double traversed_distance = 0.0;
  for (size_t i = closest_index; i < path.poses.size(); ++i) {
    if (i > closest_index) {
      const auto & prev = path.poses[i - 1].pose.position;
      const auto & cur = path.poses[i].pose.position;
      traversed_distance += std::hypot(cur.x - prev.x, cur.y - prev.y);
      if (traversed_distance > max_check_distance) {
        break;
      }
    }

    const auto & position = path.poses[i].pose.position;
    const int x = ContXY2Disc(position.x - origin_x, resolution);
    const int y = ContXY2Disc(position.y - origin_y, resolution);
    if (x < 0 || x >= width || y < 0 || y >= height) {
      if (blocked_path_index) {
        *blocked_path_index = static_cast<int>(i);
      }
      return true;
    }

    const int index = y * width + x;
    if (classifyMapCell(map.data[index]) == 1) {
      if (blocked_path_index) {
        *blocked_path_index = static_cast<int>(i);
      }
      return true;
    }
  }

  return false;
}

double VoronoiNode::pathLengthFromClosestPose(
  const nav_msgs::msg::Path & path,
  const geometry_msgs::msg::Pose & pose) const
{
  if (path.poses.size() < 2) {
    return 0.0;
  }

  size_t closest_index = 0;
  double best_dist_sq = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < path.poses.size(); ++i) {
    const auto & position = path.poses[i].pose.position;
    const double dx = position.x - pose.position.x;
    const double dy = position.y - pose.position.y;
    const double dist_sq = dx * dx + dy * dy;
    if (dist_sq < best_dist_sq) {
      best_dist_sq = dist_sq;
      closest_index = i;
    }
  }

  double length = std::sqrt(best_dist_sq);
  for (size_t i = closest_index + 1; i < path.poses.size(); ++i) {
    const auto & prev = path.poses[i - 1].pose.position;
    const auto & cur = path.poses[i].pose.position;
    length += std::hypot(cur.x - prev.x, cur.y - prev.y);
  }

  return length;
}

double VoronoiNode::pathLength(const nav_msgs::msg::Path & path) const
{
  if (path.poses.size() < 2) {
    return 0.0;
  }

  double length = 0.0;
  for (size_t i = 1; i < path.poses.size(); ++i) {
    const auto & prev = path.poses[i - 1].pose.position;
    const auto & cur = path.poses[i].pose.position;
    length += std::hypot(cur.x - prev.x, cur.y - prev.y);
  }

  return length;
}

void VoronoiNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  bool goal_reached_now = false;
  std::string frame_id;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    odom_ = msg;
    has_odom_ = true;

    if (has_goal_) {
      const double dx = msg->pose.pose.position.x - last_goal_.pose.position.x;
      const double dy = msg->pose.pose.position.y - last_goal_.pose.position.y;
      const double distance = std::hypot(dx, dy);

      if (distance <= goal_tolerance_) {
        goal_reached_ = true;
        has_goal_ = false;
        goal_dirty_ = false;
        need_replan_ = false;
        map_dirty_ = false;
        has_published_plan_ = false;
        goal_reached_now = true;
        frame_id = has_map_ ? map_->header.frame_id : last_goal_.header.frame_id;
      }
    }
  }

  if (goal_reached_now) {
    nav_msgs::msg::Path empty_path;
    empty_path.header.frame_id = frame_id;
    empty_path.header.stamp = this->now();
    path_pub_->publish(empty_path);
    path2_pub_->publish(empty_path);

    publishStopCmd();
    RCLCPP_INFO(this->get_logger(), "Goal reached from odom. Stop replanning.");
  }
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
      has_published_plan_ = false;
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

  nav_msgs::msg::Path previous_plan;
  bool has_previous_plan = false;
  bool goal_dirty_snapshot = false;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    previous_plan = last_published_plan_;
    has_previous_plan = has_published_plan_;
    goal_dirty_snapshot = goal_dirty_;
  }

  const nav_msgs::msg::Path smoothing_control_path =
    downsamplePath(plan, std::max(1, path_smoothing_control_step_));
  nav_msgs::msg::Path published_plan = smoothPathBSpline(
    smoothing_control_path, static_cast<int>(plan.poses.size()), 3);

  bool previous_plan_blocked = false;
  if (has_previous_plan && !previous_plan.poses.empty()) {
    previous_plan_blocked = isPathBlockedByMap(
      previous_plan, *odom_local, *map_local, path_obstacle_check_distance_m_,
      nullptr);
  }

  const double new_path_length = pathLength(published_plan);
  const double previous_remaining_length =
    has_previous_plan ?
    pathLengthFromClosestPose(previous_plan, odom_local->pose.pose) :
    std::numeric_limits<double>::infinity();

  if (
    has_previous_plan && !goal_dirty_snapshot && !previous_plan_blocked &&
    new_path_length + path_switch_min_improvement_m_ >= previous_remaining_length)
  {
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
      "Keep current path to avoid oscillation: current_remaining=%.2f m, "
      "new=%.2f m, switch_threshold=%.2f m",
      previous_remaining_length, new_path_length, path_switch_min_improvement_m_);
    return;
  }

  skeleton.header.stamp = this->now();
  skeleton_pub_->publish(skeleton);
  path_pub_->publish(published_plan);

  if (publish_debug_path2_) {
    nav_msgs::msg::Path path2 = downsamplePath(published_plan, 2);
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
    last_published_plan_ = published_plan;
    has_published_plan_ = true;
  }

  RCLCPP_INFO_THROTTLE(
    this->get_logger(), *this->get_clock(), 2000,
    "Published Voronoi path, raw size = %zu, smooth controls = %zu, smooth size = %zu",
    plan.poses.size(), smoothing_control_path.poses.size(), published_plan.poses.size());
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

  const double goal_dx = odom_local->pose.pose.position.x - goal_local.pose.position.x;
  const double goal_dy = odom_local->pose.pose.position.y - goal_local.pose.position.y;
  const double goal_distance = std::hypot(goal_dx, goal_dy);
  if (goal_distance <= goal_tolerance_) {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      goal_reached_ = true;
      has_goal_ = false;
      goal_dirty_ = false;
      need_replan_ = false;
      map_dirty_ = false;
      has_published_plan_ = false;
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
