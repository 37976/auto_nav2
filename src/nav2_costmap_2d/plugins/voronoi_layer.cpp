/******************************************************************************
 * Copyright (c) 2022, NKU Mobile & Flying Robotics Lab
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 'AS IS'
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *****************************************************************************/

#include "nav2_costmap_2d/voronoi_layer.hpp"

#include <chrono>
#include <stdexcept>
#include <vector>

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::VoronoiLayer, nav2_costmap_2d::Layer)

using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::FREE_SPACE;
using rcl_interfaces::msg::ParameterType;

namespace nav2_costmap_2d
{

void VoronoiLayer::onInitialize()
{
  current_ = true;
  enabled_ = true;

  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("VoronoiLayer: node is invalid");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(getFullName("enabled"), enabled_);
}

void VoronoiLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * /*min_x*/, double * /*min_y*/,
  double * /*max_x*/, double * /*max_y*/)
{
  // VoronoiLayer does not change the bounds; it operates on the full map.
}

bool VoronoiLayer::outlineMap(const Costmap2D & master_grid, uint8_t value)
{
  uint8_t * char_map = master_grid.getCharMap();
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();
  if (char_map == nullptr) {
    return false;
  }

  // Top row
  uint8_t * pc = char_map;
  for (unsigned int i = 0U; i < size_x; ++i) {
    *pc++ = value;
  }
  // Bottom row
  pc = char_map + (size_y - 1U) * size_x;
  for (unsigned int i = 0U; i < size_x; ++i) {
    *pc++ = value;
  }
  // Left column
  pc = char_map;
  for (unsigned int i = 0U; i < size_y; ++i, pc += size_x) {
    *pc = value;
  }
  // Right column
  pc = char_map + size_x - 1U;
  for (unsigned int i = 0U; i < size_y; ++i, pc += size_x) {
    *pc = value;
  }
  return true;
}

void VoronoiLayer::updateDynamicVoronoi(const Costmap2D & master_grid)
{
  const unsigned int size_x = master_grid.getSizeInCellsX();
  const unsigned int size_y = master_grid.getSizeInCellsY();

  if (last_size_x_ != size_x || last_size_y_ != size_y) {
    voronoi_.initializeEmpty(
      static_cast<int>(size_x),
      static_cast<int>(size_y));
    last_size_x_ = size_x;
    last_size_y_ = size_y;
  }

  std::vector<IntPoint> new_free_cells;
  std::vector<IntPoint> new_occupied_cells;

  for (int j = 0; j < static_cast<int>(size_y); ++j) {
    for (int i = 0; i < static_cast<int>(size_x); ++i) {
      if (voronoi_.isOccupied(i, j) &&
        master_grid.getCost(i, j) == FREE_SPACE)
      {
        new_free_cells.emplace_back(i, j);
      }
      if (!voronoi_.isOccupied(i, j) &&
        master_grid.getCost(i, j) == LETHAL_OBSTACLE)
      {
        new_occupied_cells.emplace_back(i, j);
      }
    }
  }

  for (const IntPoint & cell : new_free_cells) {
    voronoi_.clearCell(cell.x, cell.y);
  }
  for (const IntPoint & cell : new_occupied_cells) {
    voronoi_.occupyCell(cell.x, cell.y);
  }
}

void VoronoiLayer::updateCosts(
  Costmap2D & master_grid,
  int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
{
  if (!enabled_) {
    RCLCPP_WARN(logger_, "VoronoiLayer is disabled.");
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);

  if (!outlineMap(master_grid, LETHAL_OBSTACLE)) {
    RCLCPP_ERROR(logger_, "VoronoiLayer: Failed to outline map.");
    return;
  }

  updateDynamicVoronoi(master_grid);

  const auto start_timestamp = std::chrono::steady_clock::now();
  voronoi_.update();
  voronoi_.prune();
  const auto end_timestamp = std::chrono::steady_clock::now();

  const std::chrono::duration<double, std::milli> diff =
    end_timestamp - start_timestamp;
  RCLCPP_DEBUG(
    logger_,
    "VoronoiLayer: update+prune took %.3f ms", diff.count());
}

}  // namespace nav2_costmap_2d
