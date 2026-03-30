// ROS2 Voronoi 规划节点的声明，以及节点运行时状态的定义。
#pragma once

#include <memory>
#include <mutex>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

#include "nav2_voronoi_planner/voronoi_grid_planner.hpp"

namespace nav2_voronoi_planner
{

class VoronoiNode : public rclcpp::Node
{
public:
  VoronoiNode();

private:
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg);
  void tryPlanWithSnapshot(
    const nav_msgs::msg::OccupancyGrid::SharedPtr & map_local,
    const nav_msgs::msg::Odometry::SharedPtr & odom_local,
    const geometry_msgs::msg::PoseStamped & goal_local);
  void planTimerCallback();
  void publishStopCmd();

  std::mutex data_mutex_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  geometry_msgs::msg::PoseStamped last_goal_;

  bool has_map_ {false};
  bool has_odom_ {false};
  bool has_goal_ {false};
  bool goal_dirty_ {false};
  bool goal_reached_ {false};

  double robot_radius_ {0.20};
  int occ_threshold_ {50};
  bool unknown_is_obstacle_ {true};
  bool publish_debug_path2_ {true};
  double goal_tolerance_ {0.2};
  double trunk_safety_penalty_scale_ {0.06};

  rclcpp::TimerBase::SharedPtr plan_timer_;

  bool need_replan_ {false};
  bool map_dirty_ {false};
  double plan_period_ms_ {500.0};
  double replan_min_move_ {0.15};
  double last_plan_x_ {0.0};
  double last_plan_y_ {0.0};
  bool has_last_plan_pose_ {false};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path2_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr skeleton_pub_;

  std::unique_ptr<VoronoiGridPlanner> planner_;
};

}  // namespace nav2_voronoi_planner
