// 可执行程序入口：初始化 ROS，运行 VoronoiNode，最后关闭。
#include "nav2_voronoi_planner/voronoi_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_voronoi_planner::VoronoiNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
