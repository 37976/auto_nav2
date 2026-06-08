// 核心栅格规划实现：
// 负责构建 Voronoi 骨架、搜索连接段和主干段，并组装完整路径。
#include "nav2_voronoi_planner/voronoi_grid_planner.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "nav2_voronoi_planner/util.hpp"

namespace nav2_voronoi_planner
{
namespace
{

using QueueIndex = std::pair<double, int>;

bool isVoronoiSeedObstacle(int8_t value)
{
  return value >= 100;
}

}  // namespace

VoronoiGridPlanner::VoronoiGridPlanner(Config config)
: config_(config)
{
}

int VoronoiGridPlanner::toIndex(int x, int y, int w) const
{
  return y * w + x;
}

GridPoint VoronoiGridPlanner::fromIndex(int idx, int w) const
{
  GridPoint point;
  point.x = idx % w;
  point.y = idx / w;
  return point;
}

bool VoronoiGridPlanner::isObstacle(int8_t v) const
{
  if (v < 0) {
    return config_.unknown_is_obstacle;
  }
  return static_cast<int>(v) >= config_.occ_threshold;
}

bool VoronoiGridPlanner::isInside(int x, int y, int w, int h)
{
  return x >= 0 && y >= 0 && x < w && y < h;
}

bool VoronoiGridPlanner::isFreeCell(
  int x, int y,
  const nav_msgs::msg::OccupancyGrid & grid) const
{
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);

  if (!isInside(x, y, w, h)) {
    return false;
  }

  return !isObstacle(grid.data[x + y * w]);
}

bool VoronoiGridPlanner::findNearestFreeCell(
  int input_x,
  int input_y,
  const nav_msgs::msg::OccupancyGrid & grid,
  int max_radius_cells,
  GridPoint & free_cell) const
{
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);

  if (!isInside(input_x, input_y, w, h)) {
    return false;
  }
  if (isFreeCell(input_x, input_y, grid)) {
    free_cell = GridPoint{input_x, input_y};
    return true;
  }

  int best_x = input_x;
  int best_y = input_y;
  int best_dist_sq = std::numeric_limits<int>::max();
  const int radius = std::max(0, max_radius_cells);

  for (int dy = -radius; dy <= radius; ++dy) {
    for (int dx = -radius; dx <= radius; ++dx) {
      const int x = input_x + dx;
      const int y = input_y + dy;
      const int dist_sq = dx * dx + dy * dy;
      if (dist_sq > radius * radius || dist_sq >= best_dist_sq) {
        continue;
      }
      if (!isFreeCell(x, y, grid)) {
        continue;
      }

      best_x = x;
      best_y = y;
      best_dist_sq = dist_sq;
    }
  }

  if (best_dist_sq == std::numeric_limits<int>::max()) {
    return false;
  }

  free_cell = GridPoint{best_x, best_y};
  return true;
}

bool VoronoiGridPlanner::isSafeCell(
  int x, int y,
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<std::vector<VoronoiData>> * gvd_map,
  double min_clearance) const
{
  if (!isFreeCell(x, y, grid)) {
    return false;
  }

  if (gvd_map == nullptr) {
    return true;
  }

  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  if (!isInside(x, y, w, h)) {
    return false;
  }

  return (*gvd_map)[x][y].dist >= min_clearance;
}

bool VoronoiGridPlanner::canTraverseBetweenCells(
  int x0, int y0, int x1, int y1,
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<std::vector<VoronoiData>> * gvd_map,
  double min_clearance) const
{
  if (!isSafeCell(x1, y1, grid, gvd_map, min_clearance)) {
    return false;
  }

  const int dx = x1 - x0;
  const int dy = y1 - y0;
  if (std::abs(dx) != 1 || std::abs(dy) != 1) {
    return true;
  }

  return isSafeCell(x0 + dx, y0, grid, gvd_map, min_clearance) &&
    isSafeCell(x0, y0 + dy, grid, gvd_map, min_clearance);
}

bool VoronoiGridPlanner::lineOfSightFree(
  int x0, int y0, int x1, int y1,
  const nav_msgs::msg::OccupancyGrid & grid,
  const std::vector<std::vector<VoronoiData>> * gvd_map,
  double min_clearance) const
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int x = x0;
  int y = y0;

  while (true) {
    if (!isSafeCell(x, y, grid, gvd_map, min_clearance)) {
      return false;
    }

    if (x == x1 && y == y1) {
      break;
    }

    const int prev_x = x;
    const int prev_y = y;
    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }

    if (!canTraverseBetweenCells(prev_x, prev_y, x, y, grid, gvd_map, min_clearance)) {
      return false;
    }
  }

  return true;
}

GridPath VoronoiGridPlanner::makeLineGridPath(int x0, int y0, int x1, int y1) const
{
  GridPath path;
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int x = x0;
  int y = y0;

  while (true) {
    path.push_back(GridPoint{x, y});

    if (x == x1 && y == y1) {
      break;
    }

    const int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x += sx;
    }
    if (e2 < dx) {
      err += dx;
      y += sy;
    }
  }

  return path;
}

GridPath VoronoiGridPlanner::reconstructGridPath(
  const ParentMap & parent,
  int start_idx,
  int goal_idx,
  int w) const
{
  GridPath path;
  int current = goal_idx;
  path.push_back(fromIndex(current, w));

  while (current != start_idx) {
    const auto it = parent.find(current);
    if (it == parent.end()) {
      return {};
    }
    current = it->second;
    path.push_back(fromIndex(current, w));
  }

  std::reverse(path.begin(), path.end());
  return path;
}

bool VoronoiGridPlanner::findNearestReachableVoronoiPoint(
  const GridPoint & start,
  const std::vector<std::vector<VoronoiData>> & gvd_map,
  const nav_msgs::msg::OccupancyGrid & grid,
  GridPoint & voronoi_pt,
  GridPath & connector_path) const
{
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  const double min_clearance = std::max(
    config_.robot_radius + config_.clearance_margin,
    grid.info.resolution * 1.5);

  if (!isInside(start.x, start.y, w, h) || !isFreeCell(start.x, start.y, grid)) {
    return false;
  }

  std::priority_queue<QueueIndex, std::vector<QueueIndex>, std::greater<QueueIndex>> open;
  std::vector<double> g_score(w * h, std::numeric_limits<double>::infinity());
  ParentMap parent;

  const int start_idx = toIndex(start.x, start.y, w);
  g_score[start_idx] = 0.0;
  open.push({0.0, start_idx});

  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty()) {
    const auto [cur_cost, cur_idx] = open.top();
    open.pop();

    if (cur_cost > g_score[cur_idx]) {
      continue;
    }

    const GridPoint cur = fromIndex(cur_idx, w);

    if (gvd_map[cur.x][cur.y].is_voronoi) {
      voronoi_pt = cur;
      connector_path = reconstructGridPath(parent, start_idx, cur_idx, w);
      return !connector_path.empty();
    }

    for (int k = 0; k < 8; ++k) {
      const int nx = cur.x + dx[k];
      const int ny = cur.y + dy[k];

      if (!isInside(nx, ny, w, h)) {
        continue;
      }
      if (!canTraverseBetweenCells(cur.x, cur.y, nx, ny, grid, &gvd_map, min_clearance)) {
        continue;
      }

      const double step = (k < 4) ? 1.0 : std::sqrt(2.0);
      const int nidx = toIndex(nx, ny, w);
      const double next_cost = cur_cost + step;

      if (next_cost < g_score[nidx]) {
        g_score[nidx] = next_cost;
        parent[nidx] = cur_idx;
        open.push({next_cost, nidx});
      }
    }
  }

  return false;
}

std::vector<VoronoiGridPlanner::VoronoiConnectorCandidate>
VoronoiGridPlanner::findReachableVoronoiCandidates(
  const GridPoint & start,
  const std::vector<std::vector<VoronoiData>> & gvd_map,
  const nav_msgs::msg::OccupancyGrid & grid,
  size_t max_candidates) const
{
  std::vector<VoronoiConnectorCandidate> candidates;
  const bool collect_all_candidates = (max_candidates == 0);

  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  const double min_clearance = std::max(
    config_.robot_radius + config_.clearance_margin,
    grid.info.resolution * 1.5);

  if (!isInside(start.x, start.y, w, h) || !isFreeCell(start.x, start.y, grid)) {
    return candidates;
  }

  std::priority_queue<QueueIndex, std::vector<QueueIndex>, std::greater<QueueIndex>> open;
  std::vector<double> g_score(w * h, std::numeric_limits<double>::infinity());
  ParentMap parent;

  const int start_idx = toIndex(start.x, start.y, w);
  g_score[start_idx] = 0.0;
  open.push({0.0, start_idx});

  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty() && (collect_all_candidates || candidates.size() < max_candidates)) {
    const auto [cur_cost, cur_idx] = open.top();
    open.pop();

    if (cur_cost > g_score[cur_idx]) {
      continue;
    }

    const GridPoint cur = fromIndex(cur_idx, w);

    if (gvd_map[cur.x][cur.y].is_voronoi) {
      candidates.push_back(
        VoronoiConnectorCandidate{
          cur,
          reconstructGridPath(parent, start_idx, cur_idx, w),
          cur_cost});
      continue;
    }

    for (int k = 0; k < 8; ++k) {
      const int nx = cur.x + dx[k];
      const int ny = cur.y + dy[k];

      if (!isInside(nx, ny, w, h)) {
        continue;
      }
      if (!canTraverseBetweenCells(cur.x, cur.y, nx, ny, grid, &gvd_map, min_clearance)) {
        continue;
      }

      const double step = (k < 4) ? 1.0 : std::sqrt(2.0);
      const int nidx = toIndex(nx, ny, w);
      const double next_cost = cur_cost + step;

      if (next_cost < g_score[nidx]) {
        g_score[nidx] = next_cost;
        parent[nidx] = cur_idx;
        open.push({next_cost, nidx});
      }
    }
  }

  return candidates;
}

bool VoronoiGridPlanner::searchVoronoiOnly(
  const GridPoint & start_v,
  const GridPoint & goal_v,
  const std::vector<std::vector<VoronoiData>> & gvd_map,
  const nav_msgs::msg::OccupancyGrid & grid,
  GridPath & voronoi_path) const
{
  if (gvd_map.empty()) {
    return false;
  }

  const int w = static_cast<int>(gvd_map.size());
  const int h = static_cast<int>(gvd_map[0].size());
  const double min_clearance = std::max(
    config_.robot_radius + config_.clearance_margin,
    grid.info.resolution * 1.5);

  if (!isInside(start_v.x, start_v.y, w, h) || !isInside(goal_v.x, goal_v.y, w, h)) {
    return false;
  }
  if (!gvd_map[start_v.x][start_v.y].is_voronoi || !gvd_map[goal_v.x][goal_v.y].is_voronoi) {
    return false;
  }

  std::priority_queue<QueueIndex, std::vector<QueueIndex>, std::greater<QueueIndex>> open;
  std::vector<double> g_score(w * h, std::numeric_limits<double>::infinity());
  ParentMap parent;

  auto heuristic = [&](int x, int y) {
      return std::hypot(static_cast<double>(x - goal_v.x), static_cast<double>(y - goal_v.y));
    };

  const int start_idx = toIndex(start_v.x, start_v.y, w);
  const int goal_idx = toIndex(goal_v.x, goal_v.y, w);

  g_score[start_idx] = 0.0;
  open.push({heuristic(start_v.x, start_v.y), start_idx});

  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  while (!open.empty()) {
    const auto [f_score, cur_idx] = open.top();
    (void)f_score;
    open.pop();

    if (cur_idx == goal_idx) {
      voronoi_path = reconstructGridPath(parent, start_idx, goal_idx, w);
      return !voronoi_path.empty();
    }

    const GridPoint cur = fromIndex(cur_idx, w);

    for (int k = 0; k < 8; ++k) {
      const int nx = cur.x + dx[k];
      const int ny = cur.y + dy[k];

      if (!isInside(nx, ny, w, h)) {
        continue;
      }
      if (!gvd_map[nx][ny].is_voronoi) {
        continue;
      }
      if (!canTraverseBetweenCells(cur.x, cur.y, nx, ny, grid, &gvd_map, min_clearance)) {
        continue;
      }

      const double move_cost = (k < 4) ? 1.0 : std::sqrt(2.0);
      const double clearance = gvd_map[nx][ny].dist;
      const double safety_penalty =
        (clearance > 1e-6) ? (config_.trunk_safety_penalty_scale / clearance) : 1000.0;
      const double tentative_g = g_score[cur_idx] + move_cost + safety_penalty;
      const int nidx = toIndex(nx, ny, w);

      if (tentative_g < g_score[nidx]) {
        g_score[nidx] = tentative_g;
        parent[nidx] = cur_idx;
        open.push({tentative_g + heuristic(nx, ny), nidx});
      }
    }
  }

  return false;
}

bool VoronoiGridPlanner::searchBestVoronoiRoute(
  const std::vector<VoronoiConnectorCandidate> & start_candidates,
  const std::vector<VoronoiConnectorCandidate> & goal_candidates,
  const std::vector<std::vector<VoronoiData>> & gvd_map,
  const nav_msgs::msg::OccupancyGrid & grid,
  GridPath & start_connector,
  GridPath & trunk_path,
  GridPath & goal_connector,
  double & total_route_length_m) const
{
  total_route_length_m = std::numeric_limits<double>::infinity();

  if (gvd_map.empty() || start_candidates.empty() || goal_candidates.empty()) {
    return false;
  }

  const int w = static_cast<int>(gvd_map.size());
  const int h = static_cast<int>(gvd_map[0].size());
  const double min_clearance = std::max(
    config_.robot_radius + config_.clearance_margin,
    grid.info.resolution * 1.5);

  std::priority_queue<QueueIndex, std::vector<QueueIndex>, std::greater<QueueIndex>> open;
  std::vector<double> g_score(w * h, std::numeric_limits<double>::infinity());
  std::vector<int> parent(w * h, -1);
  std::unordered_map<int, size_t> source_lookup;
  std::unordered_map<int, size_t> goal_lookup;

  for (size_t i = 0; i < goal_candidates.size(); ++i) {
    const auto & candidate = goal_candidates[i];
    if (!isInside(candidate.point.x, candidate.point.y, w, h)) {
      continue;
    }
    goal_lookup[toIndex(candidate.point.x, candidate.point.y, w)] = i;
  }

  for (size_t i = 0; i < start_candidates.size(); ++i) {
    const auto & candidate = start_candidates[i];
    if (!isInside(candidate.point.x, candidate.point.y, w, h)) {
      continue;
    }

    const int idx = toIndex(candidate.point.x, candidate.point.y, w);
    if (candidate.connector_cost < g_score[idx]) {
      g_score[idx] = candidate.connector_cost;
      parent[idx] = idx;
      source_lookup[idx] = i;
      open.push({candidate.connector_cost, idx});
    }
  }

  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  double best_total_cost = std::numeric_limits<double>::infinity();
  int best_goal_idx = -1;
  size_t best_goal_candidate = 0;

  while (!open.empty()) {
    const auto [cur_cost, cur_idx] = open.top();
    open.pop();

    if (cur_cost > g_score[cur_idx]) {
      continue;
    }
    if (cur_cost >= best_total_cost) {
      break;
    }

    const auto goal_it = goal_lookup.find(cur_idx);
    if (goal_it != goal_lookup.end()) {
      const double total_cost = cur_cost + goal_candidates[goal_it->second].connector_cost;
      if (total_cost < best_total_cost) {
        best_total_cost = total_cost;
        best_goal_idx = cur_idx;
        best_goal_candidate = goal_it->second;
      }
    }

    const GridPoint cur = fromIndex(cur_idx, w);
    for (int k = 0; k < 8; ++k) {
      const int nx = cur.x + dx[k];
      const int ny = cur.y + dy[k];

      if (!isInside(nx, ny, w, h)) {
        continue;
      }
      if (!gvd_map[nx][ny].is_voronoi) {
        continue;
      }
      if (!canTraverseBetweenCells(cur.x, cur.y, nx, ny, grid, &gvd_map, min_clearance)) {
        continue;
      }

      const double move_cost = (k < 4) ? 1.0 : std::sqrt(2.0);
      const double tentative_g = g_score[cur_idx] + move_cost;
      const int nidx = toIndex(nx, ny, w);

      if (tentative_g < g_score[nidx]) {
        g_score[nidx] = tentative_g;
        parent[nidx] = cur_idx;
        open.push({tentative_g, nidx});
      }
    }
  }

  if (best_goal_idx < 0) {
    return false;
  }

  GridPath best_trunk_path;
  int current = best_goal_idx;
  while (true) {
    best_trunk_path.push_back(fromIndex(current, w));

    const int next = parent[current];
    if (next < 0) {
      return false;
    }
    if (next == current) {
      break;
    }
    current = next;
  }
  std::reverse(best_trunk_path.begin(), best_trunk_path.end());

  const int source_idx = toIndex(best_trunk_path.front().x, best_trunk_path.front().y, w);
  const auto source_it = source_lookup.find(source_idx);
  if (source_it == source_lookup.end()) {
    return false;
  }

  start_connector = start_candidates[source_it->second].connector_path;
  trunk_path = best_trunk_path;
  goal_connector = goal_candidates[best_goal_candidate].connector_path;
  std::reverse(goal_connector.begin(), goal_connector.end());
  total_route_length_m = best_total_cost * grid.info.resolution;

  return true;
}

bool VoronoiGridPlanner::makePlanOnGrid(
  const nav_msgs::msg::OccupancyGrid & map,
  const GridPoint & start_grid,
  const GridPoint & goal_grid,
  nav_msgs::msg::Path & plan,
  nav_msgs::msg::OccupancyGrid * skeleton,
  const rclcpp::Logger & logger) const
{
  plan.header.frame_id = map.header.frame_id;
  plan.header.stamp = map.header.stamp;
  plan.poses.clear();

  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  const int size_x = static_cast<int>(map.info.width);
  const int size_y = static_cast<int>(map.info.height);

  if (!isInside(start_grid.x, start_grid.y, size_x, size_y)) {
    RCLCPP_DEBUG(logger, "Start out of map: (%d, %d)", start_grid.x, start_grid.y);
    return false;
  }
  if (!isInside(goal_grid.x, goal_grid.y, size_x, size_y)) {
    RCLCPP_DEBUG(logger, "Goal out of map: (%d, %d)", goal_grid.x, goal_grid.y);
    return false;
  }
  if (!isFreeCell(start_grid.x, start_grid.y, map)) {
    RCLCPP_DEBUG(
      logger,
      "Start is not free in planning grid: (%d, %d).",
      start_grid.x, start_grid.y);
    return false;
  }
  if (!isFreeCell(goal_grid.x, goal_grid.y, map)) {
    RCLCPP_DEBUG(
      logger,
      "Goal is not free in planning grid: (%d, %d).",
      goal_grid.x, goal_grid.y);
    return false;
  }

  const auto gvd_map = buildVoronoiDiagramFromOccupancyGrid(map, logger);
  if (gvd_map.empty()) {
    RCLCPP_DEBUG(logger, "Failed to build Voronoi diagram from planning grid.");
    return false;
  }

  if (skeleton != nullptr) {
    populateVoronoiSkeleton(gvd_map, map, *skeleton);
  }

  GridPath start_connector;
  GridPath goal_connector;
  GridPath trunk_path;

  const size_t connector_candidate_count =
    (config_.connector_candidate_count <= 0) ?
    0 : static_cast<size_t>(config_.connector_candidate_count);
  const auto start_candidates = findReachableVoronoiCandidates(
    start_grid, gvd_map, map, connector_candidate_count);
  const auto goal_candidates = findReachableVoronoiCandidates(
    goal_grid, gvd_map, map, connector_candidate_count);

  if (start_candidates.empty()) {
    RCLCPP_DEBUG(logger, "Cannot connect start to Voronoi skeleton.");
    return false;
  }
  if (goal_candidates.empty()) {
    RCLCPP_DEBUG(logger, "Cannot connect goal to Voronoi skeleton.");
    return false;
  }

  double total_route_length_m = 0.0;
  if (!searchBestVoronoiRoute(
      start_candidates, goal_candidates, gvd_map, map,
      start_connector, trunk_path, goal_connector, total_route_length_m))
  {
    RCLCPP_DEBUG(
      logger,
      "Cannot find globally best Voronoi route from %zu start candidates to %zu goal candidates.",
      start_candidates.size(), goal_candidates.size());
    return false;
  }

  GridPath full_path;
  appendPathNoDuplicate(full_path, start_connector);
  appendPathNoDuplicate(full_path, trunk_path);
  appendPathNoDuplicate(full_path, goal_connector);

  if (full_path.empty()) {
    RCLCPP_DEBUG(logger, "Merged final path is empty.");
    return false;
  }

  PopulateGridPath(full_path, plan.header, resolution, origin_x, origin_y, plan);
  if (!plan.poses.empty()) {
    const double goal_world_x = DiscXY2Cont(goal_grid.x, resolution) + origin_x;
    const double goal_world_y = DiscXY2Cont(goal_grid.y, resolution) + origin_y;
    plan.poses.back().pose.position.x = goal_world_x;
    plan.poses.back().pose.position.y = goal_world_y;
  }

  RCLCPP_DEBUG(
    logger,
    "Voronoi candidate plan success: map=%d x %d, start_candidates=%zu, goal_candidates=%zu, "
    "start_connector=%zu, trunk=%zu, goal_connector=%zu, total=%zu, route_length=%.2f m",
    size_x, size_y, start_candidates.size(), goal_candidates.size(),
    start_connector.size(), trunk_path.size(), goal_connector.size(), full_path.size(),
    total_route_length_m);

  return true;
}

void VoronoiGridPlanner::appendPathNoDuplicate(GridPath & dst, const GridPath & src) const
{
  if (src.empty()) {
    return;
  }

  if (dst.empty()) {
    dst = src;
    return;
  }

  size_t start_i = 0;
  if (dst.back() == src.front()) {
    start_i = 1;
  }

  for (size_t i = start_i; i < src.size(); ++i) {
    dst.push_back(src[i]);
  }
}

VoronoiGridPlanner::CropBounds VoronoiGridPlanner::computeCropBounds(
  const nav_msgs::msg::OccupancyGrid & grid,
  const GridPoint & start_grid,
  const GridPoint & goal_grid,
  double padding_m) const
{
  CropBounds bounds;

  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  const double resolution = grid.info.resolution;
  const int padding_cells = (resolution > 0.0) ?
    std::max(1, static_cast<int>(std::ceil(std::max(0.0, padding_m) / resolution))) :
    1;

  bounds.min_x = std::max(0, std::min(start_grid.x, goal_grid.x) - padding_cells);
  bounds.max_x = std::min(w - 1, std::max(start_grid.x, goal_grid.x) + padding_cells);
  bounds.min_y = std::max(0, std::min(start_grid.y, goal_grid.y) - padding_cells);
  bounds.max_y = std::min(h - 1, std::max(start_grid.y, goal_grid.y) + padding_cells);

  // Align crop windows to the same global coarse-grid phase so optional
  // downsampling does not make narrow passages flicker as the window shifts.
  const int downsample_factor = std::max(1, config_.local_map_downsample_factor);
  if (config_.enable_local_map_downsampling && downsample_factor > 1) {
    bounds.min_x = std::max(0, (bounds.min_x / downsample_factor) * downsample_factor);
    bounds.min_y = std::max(0, (bounds.min_y / downsample_factor) * downsample_factor);

    bounds.max_x = std::min(
      w - 1,
      (((bounds.max_x + 1) + downsample_factor - 1) / downsample_factor) * downsample_factor - 1);
    bounds.max_y = std::min(
      h - 1,
      (((bounds.max_y + 1) + downsample_factor - 1) / downsample_factor) * downsample_factor - 1);
  }

  return bounds;
}

bool VoronoiGridPlanner::cropBoundsCoverWholeMap(
  const CropBounds & bounds,
  const nav_msgs::msg::OccupancyGrid & grid) const
{
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);

  return bounds.min_x <= 0 && bounds.min_y <= 0 &&
         bounds.max_x >= (w - 1) && bounds.max_y >= (h - 1);
}

nav_msgs::msg::OccupancyGrid VoronoiGridPlanner::extractSubGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  const CropBounds & bounds) const
{
  nav_msgs::msg::OccupancyGrid sub_grid;
  sub_grid.header = grid.header;
  sub_grid.info = grid.info;

  const int full_w = static_cast<int>(grid.info.width);
  const int sub_w = bounds.max_x - bounds.min_x + 1;
  const int sub_h = bounds.max_y - bounds.min_y + 1;

  sub_grid.info.width = static_cast<uint32_t>(sub_w);
  sub_grid.info.height = static_cast<uint32_t>(sub_h);
  sub_grid.info.origin.position.x =
    grid.info.origin.position.x + bounds.min_x * grid.info.resolution;
  sub_grid.info.origin.position.y =
    grid.info.origin.position.y + bounds.min_y * grid.info.resolution;
  sub_grid.data.assign(sub_w * sub_h, -1);

  for (int y = 0; y < sub_h; ++y) {
    for (int x = 0; x < sub_w; ++x) {
      const int src_x = bounds.min_x + x;
      const int src_y = bounds.min_y + y;
      sub_grid.data[x + y * sub_w] = grid.data[src_x + src_y * full_w];
    }
  }

  return sub_grid;
}

nav_msgs::msg::OccupancyGrid VoronoiGridPlanner::downsampleGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  int factor) const
{
  if (factor <= 1) {
    return grid;
  }

  nav_msgs::msg::OccupancyGrid downsampled_grid;
  downsampled_grid.header = grid.header;
  downsampled_grid.info = grid.info;

  const int src_w = static_cast<int>(grid.info.width);
  const int src_h = static_cast<int>(grid.info.height);
  const int dst_w = std::max(1, (src_w + factor - 1) / factor);
  const int dst_h = std::max(1, (src_h + factor - 1) / factor);

  downsampled_grid.info.width = static_cast<uint32_t>(dst_w);
  downsampled_grid.info.height = static_cast<uint32_t>(dst_h);
  downsampled_grid.info.resolution = grid.info.resolution * factor;
  downsampled_grid.data.assign(dst_w * dst_h, -1);

  for (int y = 0; y < dst_h; ++y) {
    for (int x = 0; x < dst_w; ++x) {
      bool has_obstacle = false;
      bool has_unknown = false;
      int8_t max_free_value = 0;
      bool has_free_value = false;

      const int src_x0 = x * factor;
      const int src_y0 = y * factor;
      const int src_x1 = std::min(src_w, src_x0 + factor);
      const int src_y1 = std::min(src_h, src_y0 + factor);

      for (int sy = src_y0; sy < src_y1; ++sy) {
        for (int sx = src_x0; sx < src_x1; ++sx) {
          const int8_t value = grid.data[sx + sy * src_w];
          if (isObstacle(value)) {
            has_obstacle = true;
            break;
          }
          if (value < 0) {
            has_unknown = true;
            continue;
          }

          if (!has_free_value || value > max_free_value) {
            max_free_value = value;
            has_free_value = true;
          }
        }
        if (has_obstacle) {
          break;
        }
      }

      const int dst_idx = x + y * dst_w;
      if (has_obstacle) {
        downsampled_grid.data[dst_idx] = 100;
      } else if (has_unknown) {
        downsampled_grid.data[dst_idx] = -1;
      } else if (has_free_value) {
        downsampled_grid.data[dst_idx] = max_free_value;
      } else {
        downsampled_grid.data[dst_idx] = 0;
      }
    }
  }

  return downsampled_grid;
}

GridPoint VoronoiGridPlanner::downsampleGridPoint(
  const GridPoint & point,
  int factor,
  const nav_msgs::msg::OccupancyGrid & downsampled_grid) const
{
  GridPoint downsampled_point;

  const int w = static_cast<int>(downsampled_grid.info.width);
  const int h = static_cast<int>(downsampled_grid.info.height);
  const int safe_factor = std::max(1, factor);

  downsampled_point.x = std::min(w - 1, std::max(0, point.x / safe_factor));
  downsampled_point.y = std::min(h - 1, std::max(0, point.y / safe_factor));

  return downsampled_point;
}

void VoronoiGridPlanner::populateEmbeddedSkeleton(
  const nav_msgs::msg::OccupancyGrid & local_skeleton,
  const nav_msgs::msg::OccupancyGrid & full_grid,
  nav_msgs::msg::OccupancyGrid & skeleton) const
{
  skeleton.header = full_grid.header;
  skeleton.info = full_grid.info;

  const int full_w = static_cast<int>(full_grid.info.width);
  const int full_h = static_cast<int>(full_grid.info.height);
  skeleton.data.assign(full_w * full_h, -1);

  for (int y = 0; y < full_h; ++y) {
    for (int x = 0; x < full_w; ++x) {
      const int idx = x + y * full_w;
      if (isObstacle(full_grid.data[idx])) {
        skeleton.data[idx] = 100;
      }
    }
  }

  const double full_resolution = full_grid.info.resolution;
  const double local_resolution = local_skeleton.info.resolution;
  const int scale = (full_resolution > 0.0) ?
    std::max(1, static_cast<int>(std::llround(local_resolution / full_resolution))) :
    1;
  const int offset_x = (full_resolution > 0.0) ?
    static_cast<int>(std::llround(
    (local_skeleton.info.origin.position.x - full_grid.info.origin.position.x) / full_resolution)) :
    0;
  const int offset_y = (full_resolution > 0.0) ?
    static_cast<int>(std::llround(
    (local_skeleton.info.origin.position.y - full_grid.info.origin.position.y) / full_resolution)) :
    0;

  const int local_w = static_cast<int>(local_skeleton.info.width);
  const int local_h = static_cast<int>(local_skeleton.info.height);
  for (int y = 0; y < local_h; ++y) {
    for (int x = 0; x < local_w; ++x) {
      const int local_idx = x + y * local_w;
      if (local_skeleton.data[local_idx] != 0) {
        continue;
      }

      const int full_x0 = offset_x + x * scale;
      const int full_y0 = offset_y + y * scale;
      const int full_x1 = std::min(full_w, full_x0 + scale);
      const int full_y1 = std::min(full_h, full_y0 + scale);

      for (int fy = std::max(0, full_y0); fy < full_y1; ++fy) {
        for (int fx = std::max(0, full_x0); fx < full_x1; ++fx) {
          skeleton.data[fx + fy * full_w] = 0;
        }
      }
    }
  }
}

double VoronoiGridPlanner::computeCropPaddingMeters(
  const GridPoint & start_grid,
  const GridPoint & goal_grid,
  double resolution,
  int expansion_step) const
{
  const double safety_padding = std::max(
    config_.robot_radius + config_.clearance_margin,
    resolution * 2.0);
  const double start_goal_distance = std::hypot(
    static_cast<double>(goal_grid.x - start_grid.x),
    static_cast<double>(goal_grid.y - start_grid.y)) * resolution;
  const double detour_ratio = std::max(0.0, config_.local_crop_detour_ratio);
  const double expansion_factor = std::max(1.0, config_.local_crop_expansion_factor);
  const double detour_max_padding = std::max(0.0, config_.local_crop_max_padding_m);

  double detour_padding = std::max(
    0.0,
    std::max(config_.local_crop_min_padding_m, start_goal_distance * detour_ratio));

  if (expansion_step > 0) {
    detour_padding *= std::pow(expansion_factor, expansion_step);
  }
  if (detour_max_padding > 0.0) {
    detour_padding = std::min(detour_padding, detour_max_padding);
  }

  return safety_padding + detour_padding;
}

std::vector<std::vector<VoronoiData>> VoronoiGridPlanner::buildVoronoiDiagramFromOccupancyGrid(
  const nav_msgs::msg::OccupancyGrid & grid,
  const rclcpp::Logger & logger) const
{
  const int w = static_cast<int>(grid.info.width);
  const int h = static_cast<int>(grid.info.height);
  const double resolution = grid.info.resolution;

  std::vector<std::vector<VoronoiData>> gvd_map;
  if (w <= 0 || h <= 0) {
    return gvd_map;
  }

  gvd_map.resize(w, std::vector<VoronoiData>(h));

  struct SeedInfo
  {
    int ox;
    int oy;
  };

  struct QueueNode
  {
    double dist;
    int x;
    int y;
    int seed_x;
    int seed_y;

    bool operator>(const QueueNode & other) const
    {
      return dist > other.dist;
    }
  };

  const double inf = std::numeric_limits<double>::infinity();
  std::vector<std::vector<double>> dist_map(w, std::vector<double>(h, inf));
  std::vector<std::vector<SeedInfo>> seed_map(w, std::vector<SeedInfo>(h, {-1, -1}));
  std::priority_queue<QueueNode, std::vector<QueueNode>, std::greater<QueueNode>> open;

  const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      const int idx = x + y * w;
      if (isVoronoiSeedObstacle(grid.data[idx])) {
        dist_map[x][y] = 0.0;
        seed_map[x][y] = {x, y};
        open.push({0.0, x, y, x, y});
      }
    }
  }

  if (open.empty()) {
    RCLCPP_DEBUG(logger, "No hard obstacle seed found in /combined_grid.");
    return {};
  }

  while (!open.empty()) {
    const QueueNode cur = open.top();
    open.pop();

    if (cur.dist > dist_map[cur.x][cur.y]) {
      continue;
    }

    for (int k = 0; k < 8; ++k) {
      const int nx = cur.x + dx[k];
      const int ny = cur.y + dy[k];

      if (!isInside(nx, ny, w, h)) {
        continue;
      }

      const double nd = std::hypot(
        static_cast<double>(nx - cur.seed_x),
        static_cast<double>(ny - cur.seed_y));

      if (nd < dist_map[nx][ny]) {
        dist_map[nx][ny] = nd;
        seed_map[nx][ny] = {cur.seed_x, cur.seed_y};
        open.push({nd, nx, ny, cur.seed_x, cur.seed_y});
      }
    }
  }

  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      gvd_map[x][y].dist = dist_map[x][y] * resolution;
      gvd_map[x][y].is_voronoi = false;
    }
  }

  const double min_clearance = std::max(
    config_.robot_radius + config_.clearance_margin,
    resolution * 1.5);
  std::vector<std::vector<uint8_t>> candidate(w, std::vector<uint8_t>(h, 0));

  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      const int idx = x + y * w;
      if (isObstacle(grid.data[idx]) || gvd_map[x][y].dist < min_clearance) {
        continue;
      }

      const auto center_seed = seed_map[x][y];
      if (center_seed.ox < 0 || center_seed.oy < 0) {
        continue;
      }

      int different_seed_neighbors = 0;
      int valid_neighbors = 0;

      for (int k = 0; k < 8; ++k) {
        const int nx = x + dx[k];
        const int ny = y + dy[k];

        if (!isInside(nx, ny, w, h)) {
          continue;
        }

        const int nidx = nx + ny * w;
        if (isObstacle(grid.data[nidx]) || gvd_map[nx][ny].dist < min_clearance) {
          continue;
        }

        const auto neigh_seed = seed_map[nx][ny];
        if (neigh_seed.ox < 0 || neigh_seed.oy < 0) {
          continue;
        }

        ++valid_neighbors;
        if (neigh_seed.ox != center_seed.ox || neigh_seed.oy != center_seed.oy) {
          ++different_seed_neighbors;
        }
      }

      if (valid_neighbors >= 2 && different_seed_neighbors >= 2) {
        candidate[x][y] = 1;
      }
    }
  }

  auto countCandidateNeighbors =
    [&](int x, int y, const std::vector<std::vector<uint8_t>> & img) {
      int count = 0;
      for (int k = 0; k < 8; ++k) {
        const int nx = x + dx[k];
        const int ny = y + dy[k];
        if (isInside(nx, ny, w, h) && img[nx][ny]) {
          ++count;
        }
      }
      return count;
    };

  for (int iter = 0; iter < 6; ++iter) {
    std::vector<std::pair<int, int>> to_remove;

    for (int x = 0; x < w; ++x) {
      for (int y = 0; y < h; ++y) {
        if (!candidate[x][y]) {
          continue;
        }

        const int degree = countCandidateNeighbors(x, y, candidate);
        if (degree == 0) {
          to_remove.push_back({x, y});
        } else if (degree == 1 && gvd_map[x][y].dist < (min_clearance + 2.0 * resolution)) {
          to_remove.push_back({x, y});
        }
      }
    }

    if (to_remove.empty()) {
      break;
    }

    for (const auto & point : to_remove) {
      candidate[point.first][point.second] = 0;
    }
  }


  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      gvd_map[x][y].is_voronoi = (candidate[x][y] != 0);
    }
  }

  return gvd_map;
}

void VoronoiGridPlanner::populateVoronoiSkeleton(
  const std::vector<std::vector<VoronoiData>> & gvd_map,
  const nav_msgs::msg::OccupancyGrid & src_grid,
  nav_msgs::msg::OccupancyGrid & skeleton) const
{
  skeleton.header.frame_id = src_grid.header.frame_id;
  skeleton.header.stamp = src_grid.header.stamp;
  skeleton.info = src_grid.info;

  const int w = static_cast<int>(src_grid.info.width);
  const int h = static_cast<int>(src_grid.info.height);
  skeleton.data.assign(w * h, -1);

  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      const int idx = x + y * w;
      if (isObstacle(src_grid.data[idx])) {
        skeleton.data[idx] = 100;
      } else if (gvd_map[x][y].is_voronoi) {
        skeleton.data[idx] = 0;
      }
    }
  }
}

visualization_msgs::msg::Marker VoronoiGridPlanner::extractSkeletonMarker(
  const nav_msgs::msg::OccupancyGrid & skeleton) const
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = skeleton.header.frame_id;
  marker.header.stamp = skeleton.header.stamp;
  marker.ns = "voronoi_skeleton";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.02;
  marker.color.r = 0.0;
  marker.color.g = 1.0;
  marker.color.b = 0.0;
  marker.color.a = 1.0;

  const int w = static_cast<int>(skeleton.info.width);
  const int h = static_cast<int>(skeleton.info.height);
  const double resolution = skeleton.info.resolution;
  const double origin_x = skeleton.info.origin.position.x;
  const double origin_y = skeleton.info.origin.position.y;

  auto toWorld = [&](int x, int y) -> geometry_msgs::msg::Point {
    geometry_msgs::msg::Point p;
    p.x = origin_x + (static_cast<double>(x) + 0.5) * resolution;
    p.y = origin_y + (static_cast<double>(y) + 0.5) * resolution;
    p.z = 0.0;
    return p;
  };

  auto isSkeleton = [&](int x, int y) -> bool {
    return skeleton.data[x + y * w] == 0;
  };

  for (int x = 0; x < w; ++x) {
    for (int y = 0; y < h; ++y) {
      if (!isSkeleton(x, y)) {
        continue;
      }

      const geometry_msgs::msg::Point from = toWorld(x, y);

      // 右邻居
      if (x + 1 < w && isSkeleton(x + 1, y)) {
        marker.points.push_back(from);
        marker.points.push_back(toWorld(x + 1, y));
      }

      // 下邻居
      if (y + 1 < h && isSkeleton(x, y + 1)) {
        marker.points.push_back(from);
        marker.points.push_back(toWorld(x, y + 1));
      }
    }
  }

  return marker;
}

void VoronoiGridPlanner::getStartAndEndConfigurations(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  double resolution,
  double origin_x,
  double origin_y,
  int * start_x,
  int * start_y,
  int * end_x,
  int * end_y) const
{
  *start_x = ContXY2Disc(start.pose.position.x - origin_x, resolution);
  *start_y = ContXY2Disc(start.pose.position.y - origin_y, resolution);
  *end_x = ContXY2Disc(goal.pose.position.x - origin_x, resolution);
  *end_y = ContXY2Disc(goal.pose.position.y - origin_y, resolution);
}

bool VoronoiGridPlanner::makePlanFromMap(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & plan,
  nav_msgs::msg::OccupancyGrid * skeleton,
  const rclcpp::Logger & logger) const
{
  plan.header.frame_id = map.header.frame_id;
  plan.header.stamp = map.header.stamp;
  plan.poses.clear();

  const double resolution = map.info.resolution;
  const double origin_x = map.info.origin.position.x;
  const double origin_y = map.info.origin.position.y;
  const int size_x = static_cast<int>(map.info.width);
  const int size_y = static_cast<int>(map.info.height);

  int start_x = 0;
  int start_y = 0;
  int goal_x = 0;
  int goal_y = 0;
  getStartAndEndConfigurations(
    start, goal, resolution, origin_x, origin_y,
    &start_x, &start_y, &goal_x, &goal_y);

  if (!isInside(start_x, start_y, size_x, size_y)) {
    RCLCPP_DEBUG(logger, "Start out of map: (%d, %d)", start_x, start_y);
    return false;
  }
  if (!isInside(goal_x, goal_y, size_x, size_y)) {
    RCLCPP_DEBUG(logger, "Goal out of map: (%d, %d)", goal_x, goal_y);
    return false;
  }
  if (!isFreeCell(start_x, start_y, map)) {
    GridPoint adjusted_start;
    const int search_radius = std::max(1, ContXY2Disc(config_.robot_radius * 2.0, resolution));
    if (!findNearestFreeCell(start_x, start_y, map, search_radius, adjusted_start)) {
      RCLCPP_DEBUG(
        logger,
        "Start is occupied or unknown, and no nearby free cell was found: (%d, %d).",
        start_x, start_y);
      return false;
    }

    RCLCPP_DEBUG(
      logger,
      "Start is occupied or unknown; use nearest free cell (%d, %d) instead of (%d, %d).",
      adjusted_start.x, adjusted_start.y, start_x, start_y);
    start_x = adjusted_start.x;
    start_y = adjusted_start.y;
  }
  if (!isFreeCell(goal_x, goal_y, map)) {
    GridPoint adjusted_goal;
    const int search_radius = std::max(1, ContXY2Disc(config_.robot_radius * 3.0, resolution));
    if (!findNearestFreeCell(goal_x, goal_y, map, search_radius, adjusted_goal)) {
      RCLCPP_DEBUG(
        logger,
        "Goal is occupied or unknown, and no nearby free cell was found: (%d, %d).",
        goal_x, goal_y);
      return false;
    }

    RCLCPP_DEBUG(
      logger,
      "Goal is occupied or unknown; use nearest free cell (%d, %d) instead of (%d, %d).",
      adjusted_goal.x, adjusted_goal.y, goal_x, goal_y);
    goal_x = adjusted_goal.x;
    goal_y = adjusted_goal.y;
  }

  const GridPoint start_grid{start_x, start_y};
  const GridPoint goal_grid{goal_x, goal_y};
  auto finalizeGoalPose = [&](nav_msgs::msg::Path & candidate_plan) {
      if (!candidate_plan.poses.empty()) {
        candidate_plan.poses.back() = goal;
      }
    };

  if (!config_.enable_local_map_cropping) {
    const bool success = makePlanOnGrid(map, start_grid, goal_grid, plan, skeleton, logger);
    if (success) {
      finalizeGoalPose(plan);
    }
    return success;
  }

  CropBounds previous_bounds;
  bool has_previous_bounds = false;
  const int local_attempts = std::max(0, config_.local_crop_max_expansions) + 1;

  for (int attempt = 0; attempt < local_attempts; ++attempt) {
    const double padding_m = computeCropPaddingMeters(start_grid, goal_grid, resolution, attempt);
    const CropBounds bounds = computeCropBounds(map, start_grid, goal_grid, padding_m);

    if (cropBoundsCoverWholeMap(bounds, map)) {
      RCLCPP_DEBUG(
        logger,
        "Voronoi crop attempt %d reached full map coverage (padding=%.2f m); switch to full map.",
        attempt + 1, padding_m);
      break;
    }

    if (
      has_previous_bounds &&
      bounds.min_x == previous_bounds.min_x &&
      bounds.max_x == previous_bounds.max_x &&
      bounds.min_y == previous_bounds.min_y &&
      bounds.max_y == previous_bounds.max_y)
    {
      continue;
    }
    previous_bounds = bounds;
    has_previous_bounds = true;

    const int crop_w = bounds.max_x - bounds.min_x + 1;
    const int crop_h = bounds.max_y - bounds.min_y + 1;
    RCLCPP_DEBUG(
      logger,
      "Voronoi crop attempt %d/%d: padding=%.2f m, window=[%d:%d, %d:%d], size=%d x %d",
      attempt + 1, local_attempts, padding_m,
      bounds.min_x, bounds.max_x, bounds.min_y, bounds.max_y, crop_w, crop_h);

    const nav_msgs::msg::OccupancyGrid local_map = extractSubGrid(map, bounds);
    const GridPoint local_start{start_grid.x - bounds.min_x, start_grid.y - bounds.min_y};
    const GridPoint local_goal{goal_grid.x - bounds.min_x, goal_grid.y - bounds.min_y};

    const int downsample_factor = std::max(1, config_.local_map_downsample_factor);
    const bool try_downsample =
      config_.enable_local_map_downsampling &&
      downsample_factor > 1 &&
      crop_w >= downsample_factor &&
      crop_h >= downsample_factor;

    if (try_downsample) {
      const nav_msgs::msg::OccupancyGrid downsampled_local_map =
        downsampleGrid(local_map, downsample_factor);
      const GridPoint downsampled_start =
        downsampleGridPoint(local_start, downsample_factor, downsampled_local_map);
      const GridPoint downsampled_goal =
        downsampleGridPoint(local_goal, downsample_factor, downsampled_local_map);

      nav_msgs::msg::OccupancyGrid downsampled_local_skeleton;
      nav_msgs::msg::OccupancyGrid * downsampled_local_skeleton_ptr =
        (skeleton != nullptr) ? &downsampled_local_skeleton : nullptr;
      if (makePlanOnGrid(
          downsampled_local_map, downsampled_start, downsampled_goal,
          plan, downsampled_local_skeleton_ptr, logger))
      {
        finalizeGoalPose(plan);
        if (skeleton != nullptr) {
          populateEmbeddedSkeleton(downsampled_local_skeleton, map, *skeleton);
        }

        RCLCPP_DEBUG(
          logger,
          "Voronoi downsampled local crop success on attempt %d/%d with %d x %d window, factor=%d.",
          attempt + 1, local_attempts, crop_w, crop_h, downsample_factor);
        return true;
      }

      RCLCPP_DEBUG(
        logger,
        "Voronoi downsampled local crop failed on attempt %d/%d; retry same window at original resolution.",
        attempt + 1, local_attempts);
    }

    nav_msgs::msg::OccupancyGrid local_skeleton;
    nav_msgs::msg::OccupancyGrid * local_skeleton_ptr = (skeleton != nullptr) ? &local_skeleton : nullptr;
    if (makePlanOnGrid(local_map, local_start, local_goal, plan, local_skeleton_ptr, logger)) {
      finalizeGoalPose(plan);
      if (skeleton != nullptr) {
        populateEmbeddedSkeleton(local_skeleton, map, *skeleton);
      }

      RCLCPP_DEBUG(
        logger,
        "Voronoi local crop success on attempt %d/%d with %d x %d window.",
        attempt + 1, local_attempts, crop_w, crop_h);
      return true;
    }
  }

  RCLCPP_DEBUG(
    logger,
    "Voronoi local crop planning failed after %d attempt(s); fallback to full map %d x %d.",
    local_attempts, size_x, size_y);
  const bool success = makePlanOnGrid(map, start_grid, goal_grid, plan, skeleton, logger);
  if (success) {
    finalizeGoalPose(plan);
  }
  return success;
}

}  // namespace nav2_voronoi_planner
