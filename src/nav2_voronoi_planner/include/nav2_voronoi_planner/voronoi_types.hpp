// 各个规划组件之间共享的 Voronoi 相关基础数据类型。
#pragma once

namespace nav2_voronoi_planner
{

struct VoronoiData
{
  bool is_voronoi = false;
  double dist = 0.0;
};

}  // namespace nav2_voronoi_planner
