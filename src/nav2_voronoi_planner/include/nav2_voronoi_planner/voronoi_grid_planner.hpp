// Voronoi 骨架构建与栅格路径规划的对外接口声明。
#pragma once

#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/logger.hpp"

#include "nav2_voronoi_planner/voronoi_path_utils.hpp"
#include "nav2_voronoi_planner/voronoi_types.hpp"

namespace nav2_voronoi_planner
{

class VoronoiGridPlanner
{
public:
  struct Config
  {
    double robot_radius {0.20};
    int occ_threshold {50};
    bool unknown_is_obstacle {true};
    double trunk_safety_penalty_scale {0.06};
  };

  explicit VoronoiGridPlanner(Config config);

  bool makePlanFromMap(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & plan,
    nav_msgs::msg::OccupancyGrid * skeleton,
    const rclcpp::Logger & logger) const;

private:
  using ParentMap = std::unordered_map<int, int>;

  int toIndex(int x, int y, int w) const;
  GridPoint fromIndex(int idx, int w) const;
  bool isObstacle(int8_t v) const;
  bool isFreeCell(int x, int y, const nav_msgs::msg::OccupancyGrid & grid) const;
  bool canTraverseBetweenCells(
    int x0, int y0, int x1, int y1,
    const nav_msgs::msg::OccupancyGrid & grid) const;
  bool lineOfSightFree(
    int x0, int y0, int x1, int y1,
    const nav_msgs::msg::OccupancyGrid & grid) const;
  GridPath reconstructGridPath(
    const ParentMap & parent,
    int start_idx,
    int goal_idx,
    int w) const;
  bool findNearestReachableVoronoiPoint(
    const GridPoint & start,
    const std::vector<std::vector<VoronoiData>> & gvd_map,
    const nav_msgs::msg::OccupancyGrid & grid,
    GridPoint & voronoi_pt,
    GridPath & connector_path) const;
  bool searchVoronoiOnly(
    const GridPoint & start_v,
    const GridPoint & goal_v,
    const std::vector<std::vector<VoronoiData>> & gvd_map,
    const nav_msgs::msg::OccupancyGrid & grid,
    GridPath & voronoi_path) const;
  void appendPathNoDuplicate(GridPath & dst, const GridPath & src) const;
  std::vector<std::vector<VoronoiData>> buildVoronoiDiagramFromOccupancyGrid(
    const nav_msgs::msg::OccupancyGrid & grid,
    const rclcpp::Logger & logger) const;
  void populateVoronoiSkeleton(
    const std::vector<std::vector<VoronoiData>> & gvd_map,
    const nav_msgs::msg::OccupancyGrid & src_grid,
    nav_msgs::msg::OccupancyGrid & skeleton) const;
  void getStartAndEndConfigurations(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double resolution,
    double origin_x,
    double origin_y,
    int * start_x,
    int * start_y,
    int * end_x,
    int * end_y) const;

  static bool isInside(int x, int y, int w, int h);

  Config config_;
};

}  // namespace nav2_voronoi_planner
