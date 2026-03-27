// 路径转换与后处理工具：
// 包括栅格路径转 Path、B 样条平滑和路径降采样。
#include "nav2_voronoi_planner/voronoi_path_utils.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "nav2_voronoi_planner/util.hpp"

namespace nav2_voronoi_planner
{
namespace
{

double deBoorCox(int i, int k, double t, const std::vector<double> & knots)
{
  if (k == 0) {
    if ((knots[i] <= t && t < knots[i + 1]) ||
      (t == knots.back() && knots[i] <= t && t <= knots[i + 1]))
    {
      return 1.0;
    }
    return 0.0;
  }

  const double denom1 = knots[i + k] - knots[i];
  const double denom2 = knots[i + k + 1] - knots[i + 1];

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

std::vector<double> makeClampedUniformKnots(int n_ctrl, int degree)
{
  const int knot_count = n_ctrl + degree + 1;
  std::vector<double> knots(knot_count, 0.0);

  const int interior = knot_count - 2 * (degree + 1);
  for (int i = 0; i <= degree; ++i) {
    knots[i] = 0.0;
    knots[knot_count - 1 - i] = 1.0;
  }

  for (int j = 0; j < interior; ++j) {
    knots[degree + 1 + j] = static_cast<double>(j + 1) / (interior + 1);
  }

  return knots;
}

}  // namespace

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
      const double dx = static_cast<double>(searched_result[i + 1].x - searched_result[i].x);
      const double dy = static_cast<double>(searched_result[i + 1].y - searched_result[i].y);
      yaw = std::atan2(dy, dx);
    } else if (i > 0) {
      const double dx = static_cast<double>(searched_result[i].x - searched_result[i - 1].x);
      const double dy = static_cast<double>(searched_result[i].y - searched_result[i - 1].y);
      yaw = std::atan2(dy, dx);
    }

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    pose_stamped.pose.orientation = tf2::toMsg(quaternion);

    plan.poses.push_back(pose_stamped);
  }
}

nav_msgs::msg::Path smoothPathBSpline(
  const nav_msgs::msg::Path & input_path,
  int output_points,
  int degree)
{
  nav_msgs::msg::Path output_path;
  output_path.header = input_path.header;

  const auto & poses = input_path.poses;
  const int point_count = static_cast<int>(poses.size());

  if (point_count < 3) {
    output_path.poses = poses;
    return output_path;
  }

  degree = std::min(degree, point_count - 1);
  if (degree < 1) {
    output_path.poses = poses;
    return output_path;
  }

  if (output_points <= 0) {
    output_points = point_count;
  }
  output_points = std::max(output_points, 2);

  std::vector<double> ctrl_x(point_count);
  std::vector<double> ctrl_y(point_count);
  for (int i = 0; i < point_count; ++i) {
    ctrl_x[i] = poses[i].pose.position.x;
    ctrl_y[i] = poses[i].pose.position.y;
  }

  const std::vector<double> knots = makeClampedUniformKnots(point_count, degree);

  output_path.poses.clear();
  output_path.poses.reserve(output_points);

  for (int sample = 0; sample < output_points; ++sample) {
    const double t =
      (output_points == 1) ? 0.0 : static_cast<double>(sample) / (output_points - 1);

    double x = 0.0;
    double y = 0.0;
    for (int i = 0; i < point_count; ++i) {
      const double basis = deBoorCox(i, degree, t, knots);
      x += basis * ctrl_x[i];
      y += basis * ctrl_y[i];
    }

    geometry_msgs::msg::PoseStamped pose;
    pose.header = input_path.header;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = 0.0;

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, 0.0);
    pose.pose.orientation = tf2::toMsg(quaternion);

    output_path.poses.push_back(pose);
  }

  output_path.poses.front() = poses.front();
  output_path.poses.back() = poses.back();

  for (size_t i = 0; i + 1 < output_path.poses.size(); ++i) {
    const double dx =
      output_path.poses[i + 1].pose.position.x - output_path.poses[i].pose.position.x;
    const double dy =
      output_path.poses[i + 1].pose.position.y - output_path.poses[i].pose.position.y;
    const double yaw = std::atan2(dy, dx);

    tf2::Quaternion quaternion;
    quaternion.setRPY(0.0, 0.0, yaw);
    output_path.poses[i].pose.orientation = tf2::toMsg(quaternion);
  }

  if (output_path.poses.size() >= 2) {
    output_path.poses.back().pose.orientation =
      output_path.poses[output_path.poses.size() - 2].pose.orientation;
  }

  return output_path;
}

nav_msgs::msg::Path downsamplePath(
  const nav_msgs::msg::Path & input_path,
  int step)
{
  nav_msgs::msg::Path output_path;
  output_path.header = input_path.header;

  if (input_path.poses.size() <= 2 || step <= 1) {
    output_path.poses = input_path.poses;
    return output_path;
  }

  output_path.poses.push_back(input_path.poses.front());
  for (size_t i = static_cast<size_t>(step); i + 1 < input_path.poses.size(); i += step) {
    output_path.poses.push_back(input_path.poses[i]);
  }
  output_path.poses.push_back(input_path.poses.back());

  return output_path;
}

}  // namespace nav2_voronoi_planner
