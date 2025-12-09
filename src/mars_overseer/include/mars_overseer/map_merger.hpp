/**
 * @file map_merger.hpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @author Marcus Hurt (mhurt@umd.edu)
 * @brief Utility class for merging multiple occupancy grid maps into a single
 * global map.
 *
 * @copyright MIT License
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef MAP_MERGER_HPP_
#define MAP_MERGER_HPP_

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

class MapMerger {
 public:
  /**
   * @brief Construct a new Map Merger object
   *
   * @param max_map_age_sec Maximum allowed age (in seconds) for a map to be
   * considered valid during merging.
   */
  explicit MapMerger(double max_map_age_sec = 10.0);

  /**
   * @brief Create an empty (unknown) global occupancy grid with specified world
   * bounds.
   *
   * @param min_x Minimum X coordinate of the map (meters).
   * @param min_y Minimum Y coordinate of the map (meters).
   * @param max_x Maximum X coordinate of the map (meters).
   * @param max_y Maximum Y coordinate of the map (meters).
   * @param resolution Grid resolution (meters per cell).
   * @return nav_msgs::msg::OccupancyGrid - A new OccupancyGrid representing the
   * global map size.
   */
  nav_msgs::msg::OccupancyGrid initialize_global_map(double min_x, double min_y,
                                                     double max_x, double max_y,
                                                     double resolution) const;

  /**
   * @brief Check whether a map is recent enough to be merged.
   *
   * @param map The occupancy grid to evaluate.
   * @param current_time The current time, provided by the node handling merges.
   * @return true - if the map is recent (not older than max_map_age_sec_).
   * @return false - if the map is stale.
   */
  bool is_map_recent(const nav_msgs::msg::OccupancyGrid& map,
                     const rclcpp::Time& current_time) const;

  /**
   * @brief Merge multiple local maps into a global occupancy grid.
   *
   * @param global_map The global occupancy grid to modify in-place.
   * @param local_maps A vector of local occupancy grids from different robots.
   */
  void merge_maps(
      nav_msgs::msg::OccupancyGrid& global_map,
      const std::vector<nav_msgs::msg::OccupancyGrid>& local_maps) const;

 private:
  // Overlay a local occupancy grid on the global map with blending factor alpha
  void overlay_map(nav_msgs::msg::OccupancyGrid& global,
                   const nav_msgs::msg::OccupancyGrid& local,
                   double alpha) const;

  // Maximum allowed map age used in map recency checks
  double max_map_age_sec_;
};

#endif