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

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

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

    RCLCPP_INFO(this->get_logger(), "VoronoiNode started.");
    RCLCPP_INFO(this->get_logger(), "Subscribed: /combined_grid /goal_pose /odom");
    RCLCPP_INFO(this->get_logger(), "Publishing: /path /path2");
  }

private:
  struct ObstacleSeed
  {
    int ox;
    int oy;
  };

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    map_ = msg;
    has_map_ = true;

    voronoi_planner_->Init(
      static_cast<int>(map_->info.width),
      static_cast<int>(map_->info.height),
      robot_radius_);

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 3000,
      "Received combined_grid: %u x %u, resolution=%.3f",
      map_->info.width, map_->info.height, map_->info.resolution);

    if (has_goal_) {
      tryPlan();
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
    has_goal_ = true;

    tryPlan();
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

  void tryPlan()
  {
    if (!has_map_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No /combined_grid received yet.");
      return;
    }

    if (!has_odom_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No /odom received yet.");
      return;
    }

    if (!has_goal_) {
      return;
    }

    double dx = odom_->pose.pose.position.x - last_goal_.pose.position.x;
    double dy = odom_->pose.pose.position.y - last_goal_.pose.position.y;
    double distance = std::hypot(dx, dy);

    if (distance <= goal_tolerance_) {
      return;
    }

    geometry_msgs::msg::PoseStamped start;
    start.header.frame_id = map_->header.frame_id;
    start.header.stamp = this->now();
    start.pose = odom_->pose.pose;

    geometry_msgs::msg::PoseStamped goal = last_goal_;
    if (goal.header.frame_id.empty()) {
      goal.header.frame_id = map_->header.frame_id;
    }

    nav_msgs::msg::Path plan;
    if (!makePlan(start, goal, plan)) {
      RCLCPP_WARN(this->get_logger(), "Voronoi replanning failed.");
      return;
    }

    path_pub_->publish(plan);

    if (publish_debug_path2_) {
      nav_msgs::msg::Path smooth = smoothPathBSpline(plan, static_cast<int>(plan.poses.size()), 3);
      nav_msgs::msg::Path path2 = downsamplePath(smooth, 2);
      path2_pub_->publish(path2);
    }

    RCLCPP_INFO(this->get_logger(), "Published Voronoi path, size = %zu", plan.poses.size());
  }

  bool makePlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    nav_msgs::msg::Path & plan)
  {
    if (!has_map_) {
      return false;
    }

    plan.header.frame_id = map_->header.frame_id;
    plan.header.stamp = this->now();
    plan.poses.clear();

    const double resolution = map_->info.resolution;
    const double origin_x = map_->info.origin.position.x;
    const double origin_y = map_->info.origin.position.y;
    const unsigned int size_x = map_->info.width;
    const unsigned int size_y = map_->info.height;

    int start_x = 0;
    int start_y = 0;
    int end_x = 0;
    int end_y = 0;

    GetStartAndEndConfigurations(
      start, goal, resolution, origin_x, origin_y,
      &start_x, &start_y, &end_x, &end_y);

    if (!isInside(start_x, start_y, static_cast<int>(size_x), static_cast<int>(size_y))) {
      RCLCPP_WARN(this->get_logger(), "Start out of map: (%d, %d)", start_x, start_y);
      return false;
    }

    if (!isInside(end_x, end_y, static_cast<int>(size_x), static_cast<int>(size_y))) {
      RCLCPP_WARN(this->get_logger(), "Goal out of map: (%d, %d)", end_x, end_y);
      return false;
    }

    std::vector<std::vector<VoronoiData>> gvd_map =
      buildVoronoiDiagramFromOccupancyGrid(*map_);

    if (gvd_map.empty()) {
      RCLCPP_WARN(this->get_logger(), "Failed to build Voronoi diagram from /combined_grid.");
      return false;
    }

    std::vector<std::pair<int, int>> path;
    if (!voronoi_planner_->Search(start_x, start_y, end_x, end_y, std::move(gvd_map), &path)) {
      RCLCPP_WARN(this->get_logger(), "Voronoi::Search failed.");
      return false;
    }

    if (path.empty()) {
      RCLCPP_WARN(this->get_logger(), "Voronoi path is empty.");
      return false;
    }

    PopulateVoronoiPath(path, plan.header, resolution, origin_x, origin_y, plan);

    // 保证目标点被加进去
    plan.poses.push_back(goal);

    // 兼容你原插件里的额外标记点
    geometry_msgs::msg::PoseStamped mark_pose;
    mark_pose.header = plan.header;
    mark_pose.pose = goal.pose;
    plan.poses.push_back(mark_pose);

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

    // 记录每个栅格最近障碍源
    std::vector<std::vector<float>> dist_map(
      w, std::vector<float>(h, std::numeric_limits<float>::infinity()));
    std::vector<std::vector<ObstacleSeed>> seed_map(
      w, std::vector<ObstacleSeed>(h, {-1, -1}));

    std::queue<std::pair<int, int>> q;

    // 初始化：把所有障碍作为 BFS 源点
    for (int x = 0; x < w; ++x) {
      for (int y = 0; y < h; ++y) {
        const int8_t v = grid.data[x + y * w];
        if (isObstacle(v)) {
          dist_map[x][y] = 0.0f;
          seed_map[x][y] = {x, y};
          q.push({x, y});
        }
      }
    }

    // 如果没有障碍物，无法构造有效 Voronoi 图
    if (q.empty()) {
      RCLCPP_WARN(this->get_logger(), "No obstacle cell found in /combined_grid.");
      return {};
    }

    // 8邻域传播最近障碍种子
    const int dx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
    const int dy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

    while (!q.empty()) {
      auto [cx, cy] = q.front();
      q.pop();

      for (int k = 0; k < 8; ++k) {
        const int nx = cx + dx[k];
        const int ny = cy + dy[k];
        if (!isInside(nx, ny, w, h)) {
          continue;
        }

        const auto & seed = seed_map[cx][cy];
        if (seed.ox < 0 || seed.oy < 0) {
          continue;
        }

        float nd = std::hypot(
          static_cast<float>(nx - seed.ox),
          static_cast<float>(ny - seed.oy));

        if (nd < dist_map[nx][ny]) {
          dist_map[nx][ny] = nd;
          seed_map[nx][ny] = seed;
          q.push({nx, ny});
        }
      }
    }

    // 依据邻居是否对应不同最近障碍，粗略提取 Voronoi 骨架
    for (int x = 0; x < w; ++x) {
      for (int y = 0; y < h; ++y) {
        gvd_map[x][y].dist = static_cast<double>(dist_map[x][y]) * resolution;
        gvd_map[x][y].is_voronoi = false;

        const int8_t v = grid.data[x + y * w];
        if (isObstacle(v)) {
          gvd_map[x][y].is_voronoi = false;
          continue;
        }

        const auto center_seed = seed_map[x][y];
        if (center_seed.ox < 0 || center_seed.oy < 0) {
          continue;
        }

        int different_seed_neighbors = 0;
        for (int k = 0; k < 8; ++k) {
          const int nx = x + dx[k];
          const int ny = y + dy[k];
          if (!isInside(nx, ny, w, h)) {
            continue;
          }

          const auto neigh_seed = seed_map[nx][ny];
          if (neigh_seed.ox < 0 || neigh_seed.oy < 0) {
            continue;
          }

          if (neigh_seed.ox != center_seed.ox || neigh_seed.oy != center_seed.oy) {
            different_seed_neighbors++;
          }
        }

        // 一个简单、稳一点的经验阈值
        if (different_seed_neighbors >= 2) {
          gvd_map[x][y].is_voronoi = true;
        }
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

  double robot_radius_ {0.20};
  int occ_threshold_ {50};
  bool unknown_is_obstacle_ {true};
  bool publish_debug_path2_ {true};
  double goal_tolerance_ {0.2};

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path2_pub_;

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