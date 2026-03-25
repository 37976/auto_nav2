#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "nav2_voronoi_planner/voronoi.hpp"
#include "nav2_voronoi_planner/util.hpp"

namespace nav2_voronoi_planner
{

class VoronoiNode : public rclcpp::Node
{
public:
  VoronoiNode()
  : Node("voronoi")
  {
    // 参数
    robot_radius_ = this->declare_parameter<double>("robot_radius", 0.20);
    occ_threshold_ = this->declare_parameter<int>("occ_threshold", 50);
    unknown_is_obstacle_ = this->declare_parameter<bool>("unknown_is_obstacle", true);
    publish_debug_path2_ = this->declare_parameter<bool>("publish_debug_path2", true);
    goal_tolerance_ = this->declare_parameter<double>("goal_tolerance", 0.2);
    plan_period_ms_ = this->declare_parameter<double>("plan_period_ms", 500.0);
    replan_min_move_ = this->declare_parameter<double>("replan_min_move", 0.15);

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    skeleton_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/voronoi_skeleton", 1);

    // 订阅 / 发布，尽量对齐你现有 astar 节点接口
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/combined_grid", rclcpp::SensorDataQoS(),
      std::bind(&VoronoiNode::mapCallback, this, std::placeholders::_1));

    goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      std::bind(&VoronoiNode::goalCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 50,
      std::bind(&VoronoiNode::odomCallback, this, std::placeholders::_1));

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10);
    path2_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path2", 10);

    voronoi_planner_ = std::make_unique<Voronoi>();

    plan_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(plan_period_ms_)),
      std::bind(&VoronoiNode::planTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "VoronoiNode started.");
    RCLCPP_INFO(this->get_logger(), "Subscribed: /combined_grid /goal_pose /odom");
    RCLCPP_INFO(this->get_logger(), "Publishing: /path /path2 /voronoi_skeleton");
    RCLCPP_INFO(this->get_logger(), "Plan period: %.1f ms", plan_period_ms_);
  }

private:
  struct ObstacleSeed
  {
    int ox;
    int oy;
  };

  struct GridPoint
  {
    int x;
    int y;

    bool operator==(const GridPoint & other) const
    {
      return x == other.x && y == other.y;
    }
  };

  struct GridPointHash
  {
    std::size_t operator()(const GridPoint & p) const
    {
      return std::hash<int>()(p.x * 73856093 ^ p.y * 19349663);
    }
  };

  using GridPath = std::vector<GridPoint>;

  int toIndex(int x, int y, int w) const
  {
    return y * w + x;
  }

  GridPoint fromIndex(int idx, int w) const
  {
    GridPoint p;
    p.x = idx % w;
    p.y = idx / w;
    return p;
  }

  bool isFreeCell(int x, int y, const nav_msgs::msg::OccupancyGrid & grid) const
  {
    const int w = static_cast<int>(grid.info.width);
    const int h = static_cast<int>(grid.info.height);

    if (!isInside(x, y, w, h)) {
      return false;
    }

    const int8_t v = grid.data[x + y * w];
    return !isObstacle(v);
  }

  bool lineOfSightFree(int x0, int y0, int x1, int y1, const nav_msgs::msg::OccupancyGrid & grid) const
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

      int e2 = 2 * err;
      if (e2 > -dy) {
        err -= dy;
        x += sx;
      }
      if (e2 < dx) {
        err += dx;
        y += sy;
      }
    }

    return true;
  }

  GridPath reconstructGridPath(
    const std::unordered_map<int, int> & parent,
    int start_idx,
    int goal_idx,
    int w) const
  {
    GridPath path;
    int cur = goal_idx;
    path.push_back(fromIndex(cur, w));

    while (cur != start_idx) {
      auto it = parent.find(cur);
      if (it == parent.end()) {
        return {};
      }
      cur = it->second;
      path.push_back(fromIndex(cur, w));
    }

    std::reverse(path.begin(), path.end());
    return path;
  }

  bool findNearestReachableVoronoiPoint(
  const GridPoint & start,
  const std::vector<std::vector<VoronoiData>> & gvd_map,
  const nav_msgs::msg::OccupancyGrid & grid,
  GridPoint & voronoi_pt,
  GridPath & connector_path)
  {
    const int w = static_cast<int>(grid.info.width);
    const int h = static_cast<int>(grid.info.height);

    if (!isInside(start.x, start.y, w, h) || !isFreeCell(start.x, start.y, grid)) {
      return false;
    }

    using QNode = std::pair<double, int>;  // cost, idx
    std::priority_queue<QNode, std::vector<QNode>, std::greater<QNode>> open;

    std::vector<double> g_score(w * h, std::numeric_limits<double>::infinity());
    std::unordered_map<int, int> parent;

    const int start_idx = toIndex(start.x, start.y, w);
    g_score[start_idx] = 0.0;
    open.push({0.0, start_idx});

    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!open.empty()) {
      auto [cur_cost, cur_idx] = open.top();
      open.pop();

      if (cur_cost > g_score[cur_idx]) {
        continue;
      }

      GridPoint cur = fromIndex(cur_idx, w);

      // 起点自己如果就在骨架上，也允许直接返回
      if (gvd_map[cur.x][cur.y].is_voronoi) {
        voronoi_pt = cur;
        connector_path = reconstructGridPath(parent, start_idx, cur_idx, w);
        return !connector_path.empty();
      }

      for (int k = 0; k < 8; ++k) {
        int nx = cur.x + dx[k];
        int ny = cur.y + dy[k];

        if (!isInside(nx, ny, w, h)) {
          continue;
        }
        if (!isFreeCell(nx, ny, grid)) {
          continue;
        }

        double step = (k < 4) ? 1.0 : std::sqrt(2.0);
        int nidx = toIndex(nx, ny, w);
        double ng = cur_cost + step;

        if (ng < g_score[nidx]) {
          g_score[nidx] = ng;
          parent[nidx] = cur_idx;
          open.push({ng, nidx});
        }
      }
    }

    return false;
  }

  bool searchVoronoiOnly(
  const GridPoint & start_v,
  const GridPoint & goal_v,
  const std::vector<std::vector<VoronoiData>> & gvd_map,
  GridPath & voronoi_path)
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

    using QNode = std::pair<double, int>;  // f, idx
    std::priority_queue<QNode, std::vector<QNode>, std::greater<QNode>> open;

    std::vector<double> g_score(w * h, std::numeric_limits<double>::infinity());
    std::unordered_map<int, int> parent;

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
      auto [f, cur_idx] = open.top();
      open.pop();

      if (cur_idx == goal_idx) {
        voronoi_path = reconstructGridPath(parent, start_idx, goal_idx, w);
        return !voronoi_path.empty();
      }

      GridPoint cur = fromIndex(cur_idx, w);

      for (int k = 0; k < 8; ++k) {
        int nx = cur.x + dx[k];
        int ny = cur.y + dy[k];

        if (!isInside(nx, ny, w, h)) {
          continue;
        }

        // 只允许骨架点
        if (!gvd_map[nx][ny].is_voronoi) {
          continue;
        }

        double move_cost = (k < 4) ? 1.0 : std::sqrt(2.0);

        // 可选：鼓励远离障碍，dist 越大越安全，代价越小
        double clearance = gvd_map[nx][ny].dist;
        double safety_penalty = 0.0;
        if (clearance > 1e-6) {
          safety_penalty = 0.15 / clearance;
        } else {
          safety_penalty = 1000.0;
        }

        double tentative_g = g_score[cur_idx] + move_cost + safety_penalty;
        int nidx = toIndex(nx, ny, w);

        if (tentative_g < g_score[nidx]) {
          g_score[nidx] = tentative_g;
          parent[nidx] = cur_idx;
          double f_score = tentative_g + heuristic(nx, ny);
          open.push({f_score, nidx});
        }
      }
    }

    return false;
  }

  void appendPathNoDuplicate(GridPath & dst, const GridPath & src)
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

  void PopulateGridPath(
  const GridPath & searched_result,
  const std_msgs::msg::Header & header,
  double resolution,
  double origin_x,
  double origin_y,
  nav_msgs::msg::Path & plan)
  {
    plan.poses.clear();
    plan.header = header;

    if (searched_result.empty()) {
      return;
    }

    for (size_t i = 0; i < searched_result.size(); ++i) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = header;
      pose_stamped.pose.position.x = DiscXY2Cont(searched_result[i].x, resolution) + origin_x;
      pose_stamped.pose.position.y = DiscXY2Cont(searched_result[i].y, resolution) + origin_y;
      pose_stamped.pose.position.z = 0.0;

      double yaw = 0.0;
      if (i + 1 < searched_result.size()) {
        double dx = static_cast<double>(searched_result[i + 1].x - searched_result[i].x);
        double dy = static_cast<double>(searched_result[i + 1].y - searched_result[i].y);
        yaw = std::atan2(dy, dx);
      } else if (i > 0) {
        double dx = static_cast<double>(searched_result[i].x - searched_result[i - 1].x);
        double dy = static_cast<double>(searched_result[i].y - searched_result[i - 1].y);
        yaw = std::atan2(dy, dx);
      }

      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, yaw);
      pose_stamped.pose.orientation = tf2::toMsg(quaternion);

      plan.poses.push_back(pose_stamped);
    }
  }



  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    map_ = msg;
    has_map_ = true;
    map_dirty_ = true;

    if (!planner_inited_) {
      voronoi_planner_->Init(
        static_cast<int>(map_->info.width),
        static_cast<int>(map_->info.height),
        robot_radius_);
      planner_inited_ = true;
    }


    if (has_goal_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "Received combined_grid: %u x %u, resolution=%.3f",
        map_->info.width, map_->info.height, map_->info.resolution);
    }


    if (has_goal_) {
      //tryPlan();
      need_replan_ = true;
    }
  }

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    odom_ = msg;
    has_odom_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    last_goal_ = *goal_msg;
    if (last_goal_.header.frame_id.empty() && has_map_) {
      last_goal_.header.frame_id = map_->header.frame_id;
    }
    goal_reached_ = false;
    has_goal_ = true;
    need_replan_ = true;
    //tryPlan();
  }

  void tryPlanWithSnapshot(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & map_local,
  const nav_msgs::msg::Odometry::SharedPtr & odom_local,
  const geometry_msgs::msg::PoseStamped & goal_local)
  {
    if (!map_local || !odom_local) {
      return;
    }

    double dx = odom_local->pose.pose.position.x - goal_local.pose.position.x;
    double dy = odom_local->pose.pose.position.y - goal_local.pose.position.y;
    double distance = std::hypot(dx, dy);

    if (distance <= goal_tolerance_) {
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        goal_reached_ = true;
        has_goal_ = false;
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
    if (!makePlanFromMap(*map_local, start, goal, plan)) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Voronoi replanning failed.");
      return;
    }

    path_pub_->publish(plan);

    if (publish_debug_path2_) {
      nav_msgs::msg::Path smooth = smoothPathBSpline(plan, static_cast<int>(plan.poses.size()), 3);
      nav_msgs::msg::Path path2 = downsamplePath(smooth, 2);
      path2_pub_->publish(path2);
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
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

  void planTimerCallback()
  {
    nav_msgs::msg::OccupancyGrid::SharedPtr map_local;
    nav_msgs::msg::Odometry::SharedPtr odom_local;
    geometry_msgs::msg::PoseStamped goal_local;

    bool goal_reached_local = false;
    bool has_map_local = false;
    bool has_odom_local = false;
    bool has_goal_local = false;
    bool need_replan_local = false;
    bool map_dirty_local = false;

    {
      std::lock_guard<std::mutex> lock(data_mutex_);

      goal_reached_local = goal_reached_;
      has_map_local = has_map_;
      has_odom_local = has_odom_;
      has_goal_local = has_goal_;
      need_replan_local = need_replan_;
      map_dirty_local = map_dirty_;

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
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "No /combined_grid received yet.");
      return;
    }

    if (!has_odom_local) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 3000, "No /odom received yet.");
      return;
    }

    if (!has_goal_local) {
      return;
    }

    // 没有请求重规划，也没有地图变化，则不算
    if (!need_replan_local && !map_dirty_local) {
      return;
    }

    // 可选：如果机器人位移很小，而且不是地图变化，可跳过本轮
    if (has_last_plan_pose_ && !map_dirty_local) {
      double dx = odom_local->pose.pose.position.x - last_plan_x_;
      double dy = odom_local->pose.pose.position.y - last_plan_y_;
      double moved = std::hypot(dx, dy);
      if (moved < replan_min_move_) {
        return;
      }
    }

    tryPlanWithSnapshot(map_local, odom_local, goal_local);
  }



    // 三次B样条基函数（均匀 clamped knot 情况下，使用 de Boor 更通用）
  static double deBoorCox(int i, int k, double t, const std::vector<double>& knots)
  {
    if (k == 0) {
      if ((knots[i] <= t && t < knots[i + 1]) ||
          (t == knots.back() && knots[i] <= t && t <= knots[i + 1])) {
        return 1.0;
      }
      return 0.0;
    }

    double denom1 = knots[i + k] - knots[i];
    double denom2 = knots[i + k + 1] - knots[i + 1];

    double term1 = 0.0;
    double term2 = 0.0;

    if (denom1 > 1e-9) {
      term1 = (t - knots[i]) / denom1 * deBoorCox(i, k - 1, t, knots);
    }
    if (denom2 > 1e-9) {
      term2 = (knots[i + k + 1] - t) / denom2 * deBoorCox(i + 1, k - 1, t, knots);
    }

    return term1 + term2;
  }

  // 构造clamped均匀节点向量
  static std::vector<double> makeClampedUniformKnots(int n_ctrl, int degree)
  {
    // 控制点数量 n_ctrl，阶数 degree
    // 节点数 = n_ctrl + degree + 1
    int m = n_ctrl + degree + 1;
    std::vector<double> knots(m, 0.0);

    int interior = m - 2 * (degree + 1);
    for (int i = 0; i <= degree; ++i) {
      knots[i] = 0.0;
      knots[m - 1 - i] = 1.0;
    }

    for (int j = 0; j < interior; ++j) {
      knots[degree + 1 + j] = static_cast<double>(j + 1) / (interior + 1);
    }

    return knots;
  }

  // 弦长参数化，得到[0,1]上的参数
  static std::vector<double> chordLengthParameterize(
    const std::vector<geometry_msgs::msg::PoseStamped>& poses)
  {
    std::vector<double> params;
    if (poses.empty()) {
      return params;
    }

    params.resize(poses.size(), 0.0);
    double total = 0.0;

    for (size_t i = 1; i < poses.size(); ++i) {
      double dx = poses[i].pose.position.x - poses[i - 1].pose.position.x;
      double dy = poses[i].pose.position.y - poses[i - 1].pose.position.y;
      total += std::hypot(dx, dy);
      params[i] = total;
    }

    if (total < 1e-9) {
      for (size_t i = 0; i < poses.size(); ++i) {
        params[i] = (poses.size() == 1) ? 0.0 : static_cast<double>(i) / (poses.size() - 1);
      }
      return params;
    }

    for (double& v : params) {
      v /= total;
    }

    return params;
  }

  // 用B样条对路径做平滑重采样
  static nav_msgs::msg::Path smoothPathBSpline(
    const nav_msgs::msg::Path& input_path,
    int output_points = -1,
    int degree = 3)
  {
    nav_msgs::msg::Path output_path;
    output_path.header = input_path.header;

    const auto& poses = input_path.poses;
    const int n = static_cast<int>(poses.size());

    if (n < 3) {
      output_path.poses = poses;
      return output_path;
    }

    // 阶数不能超过控制点数-1
    degree = std::min(degree, n - 1);
    if (degree < 1) {
      output_path.poses = poses;
      return output_path;
    }

    // 默认输出点数：和输入一致；你也可以给更大的数让曲线更密
    if (output_points <= 0) {
      output_points = n;
    }
    output_points = std::max(output_points, 2);

    // 控制点直接取原路径点
    std::vector<double> ctrl_x(n), ctrl_y(n);
    for (int i = 0; i < n; ++i) {
      ctrl_x[i] = poses[i].pose.position.x;
      ctrl_y[i] = poses[i].pose.position.y;
    }

    // 用clamped均匀节点向量
    std::vector<double> knots = makeClampedUniformKnots(n, degree);

    // 仅用于确定采样分布时参考原路径弦长
    std::vector<double> chord_t = chordLengthParameterize(poses);

    output_path.poses.clear();
    output_path.poses.reserve(output_points);

    for (int s = 0; s < output_points; ++s) {
      // 采样参数：均匀采样
      double t = (output_points == 1) ? 0.0 : static_cast<double>(s) / (output_points - 1);

      double x = 0.0;
      double y = 0.0;

      for (int i = 0; i < n; ++i) {
        double b = deBoorCox(i, degree, t, knots);
        x += b * ctrl_x[i];
        y += b * ctrl_y[i];
      }

      geometry_msgs::msg::PoseStamped pose;
      pose.header = input_path.header;
      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = 0.0;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, 0.0);
      pose.pose.orientation = tf2::toMsg(q);

      output_path.poses.push_back(pose);
    }

    // 强制首尾点与原路径一致，避免终点漂移
    if (!output_path.poses.empty()) {
      output_path.poses.front() = poses.front();
      output_path.poses.back() = poses.back();
    }

    // 为每个点估计朝向
    for (size_t i = 0; i + 1 < output_path.poses.size(); ++i) {
      double dx = output_path.poses[i + 1].pose.position.x - output_path.poses[i].pose.position.x;
      double dy = output_path.poses[i + 1].pose.position.y - output_path.poses[i].pose.position.y;
      double yaw = std::atan2(dy, dx);

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, yaw);
      output_path.poses[i].pose.orientation = tf2::toMsg(q);
    }

    if (output_path.poses.size() >= 2) {
      output_path.poses.back().pose.orientation =
        output_path.poses[output_path.poses.size() - 2].pose.orientation;
    }

    return output_path;
  }

  void publishVoronoiSkeleton(
  const std::vector<std::vector<VoronoiData>> & gvd_map,
  const nav_msgs::msg::OccupancyGrid & src_grid)
  {
    nav_msgs::msg::OccupancyGrid skeleton;
    skeleton.header.frame_id = src_grid.header.frame_id;
    skeleton.header.stamp = this->now();
    skeleton.info = src_grid.info;

    const int w = static_cast<int>(src_grid.info.width);
    const int h = static_cast<int>(src_grid.info.height);

    skeleton.data.assign(w * h, -1);

    for (int x = 0; x < w; ++x) {
      for (int y = 0; y < h; ++y) {
        int idx = x + y * w;

        // 障碍物显示为 100
        if (isObstacle(src_grid.data[idx])) {
          skeleton.data[idx] = 100;
        }
        // 骨架显示为 0 或 50 都行，这里设成 0 方便和障碍区分
        else if (gvd_map[x][y].is_voronoi) {
          skeleton.data[idx] = 0;
        }
        // 其他自由区设成 -1，表示不显示或透明
        else {
          skeleton.data[idx] = -1;
        }
      }
    }

    skeleton_pub_->publish(skeleton);
  }

  static nav_msgs::msg::Path downsamplePath(const nav_msgs::msg::Path& input_path, int step = 2)
  {
    nav_msgs::msg::Path output_path;
    output_path.header = input_path.header;

    if (input_path.poses.size() <= 2 || step <= 1) {
      output_path.poses = input_path.poses;
      return output_path;
    }

    output_path.poses.push_back(input_path.poses.front());
    for (size_t i = step; i + 1 < input_path.poses.size(); i += step) {
      output_path.poses.push_back(input_path.poses[i]);
    }
    output_path.poses.push_back(input_path.poses.back());

    return output_path;
  }

  void publishStopCmd()
  {
    geometry_msgs::msg::Twist stop_cmd;
    stop_cmd.linear.x = 0.0;
    stop_cmd.linear.y = 0.0;
    stop_cmd.linear.z = 0.0;
    stop_cmd.angular.x = 0.0;
    stop_cmd.angular.y = 0.0;
    stop_cmd.angular.z = 0.0;

    // 连续发几次，避免底盘/仿真继续沿用上一帧速度
    for (int i = 0; i < 5; ++i) {
      cmd_vel_pub_->publish(stop_cmd);
      //cout<<"wocaonimade!!!!!!!!"<<endl;
    }
  }
  
  

  bool makePlanFromMap(
  const nav_msgs::msg::OccupancyGrid & map,
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  nav_msgs::msg::Path & plan)
  {
    plan.header.frame_id = map.header.frame_id;
    plan.header.stamp = this->now();
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

    GetStartAndEndConfigurations(
      start, goal, resolution, origin_x, origin_y,
      &start_x, &start_y, &goal_x, &goal_y);

    if (!isInside(start_x, start_y, size_x, size_y)) {
      RCLCPP_WARN(this->get_logger(), "Start out of map: (%d, %d)", start_x, start_y);
      return false;
    }

    if (!isInside(goal_x, goal_y, size_x, size_y)) {
      RCLCPP_WARN(this->get_logger(), "Goal out of map: (%d, %d)", goal_x, goal_y);
      return false;
    }

    if (!isFreeCell(start_x, start_y, map)) {
      RCLCPP_WARN(this->get_logger(), "Start is occupied or unknown.");
      return false;
    }

    if (!isFreeCell(goal_x, goal_y, map)) {
      RCLCPP_WARN(this->get_logger(), "Goal is occupied or unknown.");
      return false;
    }

    std::vector<std::vector<VoronoiData>> gvd_map =
      buildVoronoiDiagramFromOccupancyGrid(map);

    if (gvd_map.empty()) {
      RCLCPP_WARN(this->get_logger(), "Failed to build Voronoi diagram from /combined_grid.");
      return false;
    }

    publishVoronoiSkeleton(gvd_map, map);

    GridPoint S{start_x, start_y};
    GridPoint G{goal_x, goal_y};

    GridPoint Vs, Vg;
    GridPath path_s, path_g, path_v;

    if (!findNearestReachableVoronoiPoint(S, gvd_map, map, Vs, path_s)) {
      RCLCPP_WARN(this->get_logger(), "Cannot connect start to Voronoi skeleton.");
      return false;
    }

    if (!findNearestReachableVoronoiPoint(G, gvd_map, map, Vg, path_g)) {
      RCLCPP_WARN(this->get_logger(), "Cannot connect goal to Voronoi skeleton.");
      return false;
    }

    double sg_dist = std::hypot(
      static_cast<double>(goal_x - start_x),
      static_cast<double>(goal_y - start_y));

    if (sg_dist < 6.0 && lineOfSightFree(start_x, start_y, goal_x, goal_y, map)) {
      GridPath direct_path;
      direct_path.push_back(S);
      direct_path.push_back(G);
      PopulateGridPath(direct_path, plan.header, resolution, origin_x, origin_y, plan);
      if (!plan.poses.empty()) {
        plan.poses.back() = goal;
      }
      return !plan.poses.empty();
    }

    if (!searchVoronoiOnly(Vs, Vg, gvd_map, path_v)) {
      RCLCPP_WARN(this->get_logger(), "Cannot find Voronoi trunk path from start skeleton to goal skeleton.");
      return false;
    }

    std::reverse(path_g.begin(), path_g.end());

    GridPath full_path;
    appendPathNoDuplicate(full_path, path_s);
    appendPathNoDuplicate(full_path, path_v);
    appendPathNoDuplicate(full_path, path_g);

    if (full_path.empty()) {
      RCLCPP_WARN(this->get_logger(), "Merged final path is empty.");
      return false;
    }

    PopulateGridPath(full_path, plan.header, resolution, origin_x, origin_y, plan);

    if (!plan.poses.empty()) {
      plan.poses.back() = goal;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Voronoi 3-stage plan success: start_connector=%zu, trunk=%zu, goal_connector=%zu, total=%zu",
      path_s.size(), path_v.size(), path_g.size(), full_path.size());

    return true;
  }

  std::vector<std::vector<VoronoiData>> buildVoronoiDiagramFromOccupancyGrid(
  const nav_msgs::msg::OccupancyGrid & grid)
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

    const double INF = std::numeric_limits<double>::infinity();

    // 最近障碍距离
    std::vector<std::vector<double>> dist_map(
      w, std::vector<double>(h, INF));

    // 最近障碍源
    std::vector<std::vector<SeedInfo>> seed_map(
      w, std::vector<SeedInfo>(h, {-1, -1}));

    // 多源 Dijkstra 小根堆
    std::priority_queue<
      QueueNode,
      std::vector<QueueNode>,
      std::greater<QueueNode>> open;

    // 8邻域
    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    // 1) 所有障碍点作为源点
    for (int x = 0; x < w; ++x) {
      for (int y = 0; y < h; ++y) {
        const int idx = x + y * w;
        const int8_t v = grid.data[idx];

        if (isObstacle(v)) {
          dist_map[x][y] = 0.0;
          seed_map[x][y] = {x, y};
          open.push({0.0, x, y, x, y});
        }
      }
    }

    if (open.empty()) {
      RCLCPP_WARN(this->get_logger(), "No obstacle cell found in /combined_grid.");
      return {};
    }

    // 2) 多源 Dijkstra：为每个自由格找最近障碍源
    while (!open.empty()) {
      QueueNode cur = open.top();
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

        // 注意：这里不是累加 step，而是直接算“邻居点到该 seed 的欧氏距离”
        // 用 Dijkstra 的扩展顺序保证最近 seed 归属更稳定
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

    // 3) 先填 dist，默认不是骨架
    for (int x = 0; x < w; ++x) {
      for (int y = 0; y < h; ++y) {
        gvd_map[x][y].dist = dist_map[x][y] * resolution;
        gvd_map[x][y].is_voronoi = false;
      }
    }

    // 安全距离阈值：离障碍太近的点，不参与骨架
    // 你也可以改成 robot_radius_ + resolution
    const double min_clearance = std::max(robot_radius_, resolution * 1.5);

    // 4) 粗骨架提取：
    // 若某点周围出现“多个不同最近障碍源”，且自身离障碍足够远，则判成骨架候选点
    std::vector<std::vector<uint8_t>> candidate(w, std::vector<uint8_t>(h, 0));

    for (int x = 0; x < w; ++x) {
      for (int y = 0; y < h; ++y) {
        const int idx = x + y * w;
        const int8_t v = grid.data[idx];

        if (isObstacle(v)) {
          continue;
        }

        if (gvd_map[x][y].dist < min_clearance) {
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
          if (isObstacle(grid.data[nidx])) {
            continue;
          }

          if (gvd_map[nx][ny].dist < min_clearance) {
            continue;
          }

          const auto neigh_seed = seed_map[nx][ny];
          if (neigh_seed.ox < 0 || neigh_seed.oy < 0) {
            continue;
          }

          valid_neighbors++;

          if (neigh_seed.ox != center_seed.ox || neigh_seed.oy != center_seed.oy) {
            different_seed_neighbors++;
          }
        }

        // 阈值比你原来更严一点，减少厚骨架和毛刺
        if (valid_neighbors >= 2 && different_seed_neighbors >= 2) {
          candidate[x][y] = 1;
        }
      }
    }

    // 5) 轻量瘦身 / 去毛刺：
    // 反复删除“候选骨架中度数<=1”的短刺点，但保留主干
    auto countCandidateNeighbors = [&](int x, int y, const std::vector<std::vector<uint8_t>> & img) {
      int c = 0;
      for (int k = 0; k < 8; ++k) {
        int nx = x + dx[k];
        int ny = y + dy[k];
        if (!isInside(nx, ny, w, h)) {
          continue;
        }
        if (img[nx][ny]) {
          c++;
        }
      }
      return c;
    };

    // 迭代几轮去掉孤立点和很短的毛刺
    for (int iter = 0; iter < 6; ++iter) {
      std::vector<std::pair<int, int>> to_remove;

      for (int x = 0; x < w; ++x) {
        for (int y = 0; y < h; ++y) {
          if (!candidate[x][y]) {
            continue;
          }

          int deg = countCandidateNeighbors(x, y, candidate);

          // 删除孤立点和端点附近的小刺
          if (deg == 0) {
            to_remove.push_back({x, y});
          } else if (deg == 1) {
            // 仅当该点 clearance 不够“大”时才删，避免主干末端被删光
            if (gvd_map[x][y].dist < (min_clearance + 2.0 * resolution)) {
              to_remove.push_back({x, y});
            }
          }
        }
      }

      if (to_remove.empty()) {
        break;
      }

      for (const auto & p : to_remove) {
        candidate[p.first][p.second] = 0;
      }
    }

    // 6) 写回 gvd_map
    for (int x = 0; x < w; ++x) {
      for (int y = 0; y < h; ++y) {
        gvd_map[x][y].is_voronoi = (candidate[x][y] != 0);
      }
    }

    return gvd_map;
  }

  bool isObstacle(int8_t v) const
  {
    if (v < 0) {
      return unknown_is_obstacle_;
    }
    return static_cast<int>(v) >= occ_threshold_;
  }

  static bool isInside(int x, int y, int w, int h)
  {
    return x >= 0 && y >= 0 && x < w && y < h;
  }

  void GetStartAndEndConfigurations(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    double resolution,
    double origin_x,
    double origin_y,
    int * start_x,
    int * start_y,
    int * end_x,
    int * end_y)
  {
    *start_x = ContXY2Disc(start.pose.position.x - origin_x, resolution);
    *start_y = ContXY2Disc(start.pose.position.y - origin_y, resolution);

    *end_x = ContXY2Disc(goal.pose.position.x - origin_x, resolution);
    *end_y = ContXY2Disc(goal.pose.position.y - origin_y, resolution);
  }

  void PopulateVoronoiPath(
    const std::vector<std::pair<int, int>> & searched_result,
    const std_msgs::msg::Header & header,
    double resolution,
    double origin_x,
    double origin_y,
    nav_msgs::msg::Path & plan)
  {
    plan.poses.clear();

    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = header;

    for (const auto & pose : searched_result) {
      pose_stamped.pose.position.x = DiscXY2Cont(pose.first, resolution) + origin_x;
      pose_stamped.pose.position.y = DiscXY2Cont(pose.second, resolution) + origin_y;
      pose_stamped.pose.position.z = 0.0;

      tf2::Quaternion quaternion;
      quaternion.setRPY(0.0, 0.0, 0.0);
      pose_stamped.pose.orientation = tf2::toMsg(quaternion);

      plan.poses.push_back(pose_stamped);
    }
  }

private:
  std::mutex data_mutex_;

  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  nav_msgs::msg::Odometry::SharedPtr odom_;
  geometry_msgs::msg::PoseStamped last_goal_;

  bool has_map_ {false};
  bool has_odom_ {false};
  bool has_goal_ {false};
  bool goal_reached_ {false};
  bool planner_inited_ {false};

  double robot_radius_ {0.20};
  int occ_threshold_ {50};
  bool unknown_is_obstacle_ {true};
  bool publish_debug_path2_ {true};
  double goal_tolerance_ {0.2};

  rclcpp::TimerBase::SharedPtr plan_timer_;

  bool need_replan_ {false};
  bool map_dirty_ {false};

  // 控制定时规划频率
  double plan_period_ms_ {500.0};   // 500ms = 2Hz，虚拟机建议先这样

  // 可选：限制机器人移动很小就不必重规划
  double replan_min_move_ {0.15};   // 单位 m
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

  std::unique_ptr<Voronoi> voronoi_planner_;
};

}  // namespace nav2_voronoi_planner

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_voronoi_planner::VoronoiNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}