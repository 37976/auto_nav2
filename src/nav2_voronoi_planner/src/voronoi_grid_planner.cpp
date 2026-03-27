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

bool VoronoiGridPlanner::canTraverseBetweenCells(
  int x0, int y0, int x1, int y1,
  const nav_msgs::msg::OccupancyGrid & grid) const
{
  if (!isFreeCell(x1, y1, grid)) {
    return false;
  }

  const int dx = x1 - x0;
  const int dy = y1 - y0;
  if (std::abs(dx) != 1 || std::abs(dy) != 1) {
    return true;
  }

  return isFreeCell(x0 + dx, y0, grid) && isFreeCell(x0, y0 + dy, grid);
}

bool VoronoiGridPlanner::lineOfSightFree(
  int x0, int y0, int x1, int y1,
  const nav_msgs::msg::OccupancyGrid & grid) const
{
  int dx = std::abs(x1 - x0);
  int dy = std::abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  int x = x0;
  int y = y0;

  while (true) {
    if (!isFreeCell(x, y, grid)) {
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

    if (!canTraverseBetweenCells(prev_x, prev_y, x, y, grid)) {
      return false;
    }
  }

  return true;
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
      if (!canTraverseBetweenCells(cur.x, cur.y, nx, ny, grid)) {
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
      if (!canTraverseBetweenCells(cur.x, cur.y, nx, ny, grid)) {
        continue;
      }

      const double move_cost = (k < 4) ? 1.0 : std::sqrt(2.0);
      const double clearance = gvd_map[nx][ny].dist;
      const double safety_penalty = (clearance > 1e-6) ? (0.15 / clearance) : 1000.0;
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
      if (isObstacle(grid.data[idx])) {
        dist_map[x][y] = 0.0;
        seed_map[x][y] = {x, y};
        open.push({0.0, x, y, x, y});
      }
    }
  }

  if (open.empty()) {
    RCLCPP_WARN(logger, "No obstacle cell found in /combined_grid.");
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

  const double min_clearance = std::max(config_.robot_radius, resolution * 1.5);
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
    RCLCPP_WARN(logger, "Start out of map: (%d, %d)", start_x, start_y);
    return false;
  }
  if (!isInside(goal_x, goal_y, size_x, size_y)) {
    RCLCPP_WARN(logger, "Goal out of map: (%d, %d)", goal_x, goal_y);
    return false;
  }
  if (!isFreeCell(start_x, start_y, map)) {
    RCLCPP_WARN(logger, "Start is occupied or unknown.");
    return false;
  }
  if (!isFreeCell(goal_x, goal_y, map)) {
    RCLCPP_WARN(logger, "Goal is occupied or unknown.");
    return false;
  }

  const auto gvd_map = buildVoronoiDiagramFromOccupancyGrid(map, logger);
  if (gvd_map.empty()) {
    RCLCPP_WARN(logger, "Failed to build Voronoi diagram from /combined_grid.");
    return false;
  }

  if (skeleton != nullptr) {
    populateVoronoiSkeleton(gvd_map, map, *skeleton);
  }

  const GridPoint start_grid{start_x, start_y};
  const GridPoint goal_grid{goal_x, goal_y};

  GridPoint start_voronoi;
  GridPoint goal_voronoi;
  GridPath start_connector;
  GridPath goal_connector;
  GridPath trunk_path;

  if (!findNearestReachableVoronoiPoint(
      start_grid, gvd_map, map, start_voronoi, start_connector))
  {
    RCLCPP_WARN(logger, "Cannot connect start to Voronoi skeleton.");
    return false;
  }

  if (!findNearestReachableVoronoiPoint(
      goal_grid, gvd_map, map, goal_voronoi, goal_connector))
  {
    RCLCPP_WARN(logger, "Cannot connect goal to Voronoi skeleton.");
    return false;
  }

  const double start_goal_dist = std::hypot(
    static_cast<double>(goal_x - start_x),
    static_cast<double>(goal_y - start_y));

  if (start_goal_dist < 6.0 && lineOfSightFree(start_x, start_y, goal_x, goal_y, map)) {
    PopulateGridPath({start_grid, goal_grid}, plan.header, resolution, origin_x, origin_y, plan);
    if (!plan.poses.empty()) {
      plan.poses.back() = goal;
    }
    return !plan.poses.empty();
  }

  if (!searchVoronoiOnly(start_voronoi, goal_voronoi, gvd_map, map, trunk_path)) {
    RCLCPP_WARN(logger, "Cannot find Voronoi trunk path from start skeleton to goal skeleton.");
    return false;
  }

  std::reverse(goal_connector.begin(), goal_connector.end());

  GridPath full_path;
  appendPathNoDuplicate(full_path, start_connector);
  appendPathNoDuplicate(full_path, trunk_path);
  appendPathNoDuplicate(full_path, goal_connector);

  if (full_path.empty()) {
    RCLCPP_WARN(logger, "Merged final path is empty.");
    return false;
  }

  PopulateGridPath(full_path, plan.header, resolution, origin_x, origin_y, plan);
  if (!plan.poses.empty()) {
    plan.poses.back() = goal;
  }

  RCLCPP_INFO(
    logger,
    "Voronoi 3-stage plan success: start_connector=%zu, trunk=%zu, goal_connector=%zu, total=%zu",
    start_connector.size(), trunk_path.size(), goal_connector.size(), full_path.size());

  return true;
}

}  // namespace nav2_voronoi_planner
