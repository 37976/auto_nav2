// Voronoi 骨架构建与栅格路径规划的对外接口声明。
#pragma once

#include <cstddef>
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
    double clearance_margin {0.03};
    int occ_threshold {50};
    bool unknown_is_obstacle {true};
    double trunk_safety_penalty_scale {0.06};
    int connector_candidate_count {0};
    bool enable_local_map_cropping {true};
    double local_crop_min_padding_m {2.0};
    double local_crop_detour_ratio {0.5};
    double local_crop_max_padding_m {8.0};
    double local_crop_expansion_factor {1.8};
    int local_crop_max_expansions {2};
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

  struct VoronoiConnectorCandidate
  {
    GridPoint point;
    GridPath connector_path;
    double connector_cost {0.0};
  };

  struct CropBounds
  {
    int min_x {0};
    int max_x {-1};
    int min_y {0};
    int max_y {-1};
  };

  int toIndex(int x, int y, int w) const;
  GridPoint fromIndex(int idx, int w) const;
  bool isObstacle(int8_t v) const;
  bool isFreeCell(int x, int y, const nav_msgs::msg::OccupancyGrid & grid) const;
  bool findNearestFreeCell(
    int input_x,
    int input_y,
    const nav_msgs::msg::OccupancyGrid & grid,
    int max_radius_cells,
    GridPoint & free_cell) const;
  bool isSafeCell(
    int x, int y,
    const nav_msgs::msg::OccupancyGrid & grid,
    const std::vector<std::vector<VoronoiData>> * gvd_map,
    double min_clearance) const;
  bool canTraverseBetweenCells(
    int x0, int y0, int x1, int y1,
    const nav_msgs::msg::OccupancyGrid & grid,
    const std::vector<std::vector<VoronoiData>> * gvd_map,
    double min_clearance) const;
  bool lineOfSightFree(
    int x0, int y0, int x1, int y1,
    const nav_msgs::msg::OccupancyGrid & grid,
    const std::vector<std::vector<VoronoiData>> * gvd_map,
    double min_clearance) const;
  GridPath makeLineGridPath(int x0, int y0, int x1, int y1) const;
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
  std::vector<VoronoiConnectorCandidate> findReachableVoronoiCandidates(
    const GridPoint & start,
    const std::vector<std::vector<VoronoiData>> & gvd_map,
    const nav_msgs::msg::OccupancyGrid & grid,
    size_t max_candidates) const;
  bool searchVoronoiOnly(
    const GridPoint & start_v,
    const GridPoint & goal_v,
    const std::vector<std::vector<VoronoiData>> & gvd_map,
    const nav_msgs::msg::OccupancyGrid & grid,
    GridPath & voronoi_path) const;
  bool searchBestVoronoiRoute(
    const std::vector<VoronoiConnectorCandidate> & start_candidates,
    const std::vector<VoronoiConnectorCandidate> & goal_candidates,
    const std::vector<std::vector<VoronoiData>> & gvd_map,
    const nav_msgs::msg::OccupancyGrid & grid,
    GridPath & start_connector,
    GridPath & trunk_path,
    GridPath & goal_connector,
    double & total_route_length_m) const;
  bool makePlanOnGrid(
    const nav_msgs::msg::OccupancyGrid & map,
    const GridPoint & start_grid,
    const GridPoint & goal_grid,
    nav_msgs::msg::Path & plan,
    nav_msgs::msg::OccupancyGrid * skeleton,
    const rclcpp::Logger & logger) const;
  void appendPathNoDuplicate(GridPath & dst, const GridPath & src) const;
  std::vector<std::vector<VoronoiData>> buildVoronoiDiagramFromOccupancyGrid(
    const nav_msgs::msg::OccupancyGrid & grid,
    const rclcpp::Logger & logger) const;
  void populateVoronoiSkeleton(
    const std::vector<std::vector<VoronoiData>> & gvd_map,
    const nav_msgs::msg::OccupancyGrid & src_grid,
    nav_msgs::msg::OccupancyGrid & skeleton) const;
  CropBounds computeCropBounds(
    const nav_msgs::msg::OccupancyGrid & grid,
    const GridPoint & start_grid,
    const GridPoint & goal_grid,
    double padding_m) const;
  bool cropBoundsCoverWholeMap(
    const CropBounds & bounds,
    const nav_msgs::msg::OccupancyGrid & grid) const;
  nav_msgs::msg::OccupancyGrid extractSubGrid(
    const nav_msgs::msg::OccupancyGrid & grid,
    const CropBounds & bounds) const;
  void populateEmbeddedSkeleton(
    const nav_msgs::msg::OccupancyGrid & local_skeleton,
    const nav_msgs::msg::OccupancyGrid & full_grid,
    const CropBounds & bounds,
    nav_msgs::msg::OccupancyGrid & skeleton) const;
  double computeCropPaddingMeters(
    const GridPoint & start_grid,
    const GridPoint & goal_grid,
    double resolution,
    int expansion_step) const;
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
