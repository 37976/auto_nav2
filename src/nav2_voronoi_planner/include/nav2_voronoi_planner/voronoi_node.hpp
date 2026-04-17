// ROS2 Voronoi 规划节点的声明，以及节点运行时状态的定义。
#pragma once

#include <memory>
#include <mutex>

#include "geometry_msgs/msg/pose_stamped.hpp"
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
  int classifyMapCell(int8_t value) const;
  bool isSignificantMapChange(
    const nav_msgs::msg::OccupancyGrid & previous,
    const nav_msgs::msg::OccupancyGrid & current,
    int * changed_cells) const;
  bool isPathBlockedByMap(
    const nav_msgs::msg::Path & path,
    const nav_msgs::msg::Odometry & odom,
    const nav_msgs::msg::OccupancyGrid & map,
    double check_distance_m,
    int * blocked_path_index) const;
  double pathLengthFromClosestPose(
    const nav_msgs::msg::Path & path,
    const geometry_msgs::msg::Pose & pose) const;
  double pathLength(const nav_msgs::msg::Path & path) const;

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
  double clearance_margin_ {0.01};
  int occ_threshold_ {50};
  bool unknown_is_obstacle_ {true};
  bool publish_debug_path2_ {true};
  double goal_tolerance_ {0.2};
  double trunk_safety_penalty_scale_ {0.06};
  int connector_candidate_count_ {0};
  bool enable_local_map_cropping_ {true};
  double local_crop_min_padding_m_ {2.0};
  double local_crop_detour_ratio_ {0.5};
  double local_crop_max_padding_m_ {8.0};
  double local_crop_expansion_factor_ {1.8};
  int local_crop_max_expansions_ {2};
  bool enable_local_map_downsampling_ {false};
  int local_map_downsample_factor_ {2};
  int path_smoothing_control_step_ {2};
  double stable_map_replan_period_ms_ {3000.0};
  int map_significant_change_cells_ {50};
  double path_obstacle_check_distance_m_ {2.0};
  double path_switch_min_improvement_m_ {0.5};

  rclcpp::TimerBase::SharedPtr plan_timer_;

  bool need_replan_ {false};
  bool map_dirty_ {false};
  double plan_period_ms_ {500.0};
  double replan_min_move_ {0.15};
  double last_plan_x_ {0.0};
  double last_plan_y_ {0.0};
  bool has_last_plan_pose_ {false};
  rclcpp::Time last_map_replan_request_time_ {0, 0, RCL_ROS_TIME};
  bool has_last_map_replan_request_ {false};
  nav_msgs::msg::Path last_published_plan_;
  bool has_published_plan_ {false};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path2_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr skeleton_pub_;

  std::unique_ptr<VoronoiGridPlanner> planner_;
};

}  // namespace nav2_voronoi_planner
