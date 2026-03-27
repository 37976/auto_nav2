// 通用路径工具的声明，以及轻量级栅格路径数据类型定义。
#pragma once

#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/header.hpp"

namespace nav2_voronoi_planner
{

struct GridPoint
{
  int x;
  int y;

  bool operator==(const GridPoint & other) const
  {
    return x == other.x && y == other.y;
  }
};

using GridPath = std::vector<GridPoint>;

void PopulateGridPath(
  const GridPath & searched_result,
  const std_msgs::msg::Header & header,
  double resolution,
  double origin_x,
  double origin_y,
  nav_msgs::msg::Path & plan);

nav_msgs::msg::Path smoothPathBSpline(
  const nav_msgs::msg::Path & input_path,
  int output_points = -1,
  int degree = 3);

nav_msgs::msg::Path downsamplePath(
  const nav_msgs::msg::Path & input_path,
  int step = 2);

}  // namespace nav2_voronoi_planner
