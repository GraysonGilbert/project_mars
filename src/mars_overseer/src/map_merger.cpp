/**
 * @file map_merger.cpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @author Marcus Hurt (mhurt@umd.edu)
 * @brief Implementation for map_merging functionality
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

#include "mars_overseer/map_merger.hpp"

MapMerger::MapMerger(double max_map_age_sec)
    : max_map_age_sec_(max_map_age_sec) {}

nav_msgs::msg::OccupancyGrid MapMerger::initialize_global_map(
    double min_x, double min_y, double max_x, double max_y,
    double resolution) const {
  nav_msgs::msg::OccupancyGrid global_map;

  // Set map origin pose
  global_map.info.origin.position.x = min_x;
  global_map.info.origin.position.y = min_y;
  global_map.info.origin.position.z = 0.0;
  global_map.info.origin.orientation.w = 1.0;  // identity quaternion

  global_map.info.resolution = resolution;

  // Compute number of grid cells along each dimension
  global_map.info.width =
      static_cast<uint32_t>(std::ceil((max_x - min_x) / resolution));
  global_map.info.height =
      static_cast<uint32_t>(std::ceil((max_y - min_y) / resolution));

  // Initialize all cells to -1 (unknown)
  global_map.data.resize(global_map.info.width * global_map.info.height, -1);

  // Set a consistent frame ID for the merged global map
  global_map.header.frame_id = "global_map";

  return global_map;
}

bool MapMerger::is_map_recent(const nav_msgs::msg::OccupancyGrid& map,
                              const rclcpp::Time& current_time) const {
  // Extract timestamp from map header.
  rclcpp::Time map_time = map.header.stamp;

  // Check if map age is less than max_map_age_sec_
  return (current_time - map_time).seconds() <= max_map_age_sec_;
}

void MapMerger::merge_maps(
    nav_msgs::msg::OccupancyGrid& global_map,
    const std::vector<nav_msgs::msg::OccupancyGrid>& local_maps) const {
  if (local_maps.empty()) {
    return;
  }

  const double EPS = 1e-4;  // tolerance for resolution comparison

  // All local maps should match the global resolution
  for (const auto& local : local_maps) {
    if (std::abs(global_map.info.resolution - local.info.resolution) > EPS) {
      throw std::runtime_error("Occupancy grid resolutions do not match.");
    }
  }

  // Find newest stamp among local maps for relative recency
  rclcpp::Time newest_stamp = local_maps.front().header.stamp;
  for (const auto& local : local_maps) {
    rclcpp::Time local_stamp(local.header.stamp);
    if (local_stamp > newest_stamp) {
      newest_stamp = local_stamp;
    }
  }

  // Characteristic time scale for recency decay
  const double tau =
      std::max(1e-3, max_map_age_sec_);  // avoid division by zero

  // Blend each local map into the global, weighting more recent ones higher
  for (const auto& local : local_maps) {
    // Age relative to newest map
    double age_sec = (newest_stamp - local.header.stamp).seconds();

    // Recency weight in [0, 1], 1 for newest, decays for older maps
    double recency = std::exp(-age_sec / tau);
    recency = std::clamp(recency, 0.0, 1.0);

    // Map recency -> blend factor alpha
    //   oldest (recency ~ 0) -> alpha ~ 0.2  (more history)
    //   newest (recency ~ 1) -> alpha ~ 0.8  (strong recent preference)
    double alpha = 0.2 + 0.6 * recency;

    overlay_map(global_map, local, alpha);
  }
}

void MapMerger::overlay_map(nav_msgs::msg::OccupancyGrid& global,
                            const nav_msgs::msg::OccupancyGrid& local,
                            double alpha) const {
  // Shared resolution assumption
  const double resolution = global.info.resolution;
  const auto global_width = global.info.width;
  const auto global_height = global.info.height;

  // Origins (bottom-left corner in world coordinates)
  const double gx0 = global.info.origin.position.x;
  const double gy0 = global.info.origin.position.y;

  const double lx0 = local.info.origin.position.x;
  const double ly0 = local.info.origin.position.y;

  // Loop through the local map cells
  for (uint32_t y = 0; y < local.info.height; ++y) {
    for (uint32_t x = 0; x < local.info.width; ++x) {
      const int local_idx = static_cast<int>(y * local.info.width + x);
      const int8_t local_val = local.data[local_idx];

      // Skip unknowns in the local map
      if (local_val < 0) {
        continue;
      }

      // Convert local cell position to world coordinates, then to global map
      // indices
      const double wx = lx0 + static_cast<double>(x) * resolution;
      const double wy = ly0 + static_cast<double>(y) * resolution;

      const int gx = static_cast<int>(std::round((wx - gx0) / resolution));
      const int gy = static_cast<int>(std::round((wy - gy0) / resolution));

      // Ensure the cell lies inside global bounds
      if (gx < 0 || gy < 0 || gx >= static_cast<int>(global_width) ||
          gy >= static_cast<int>(global_height)) {
        continue;
      }

      const int global_idx = gy * static_cast<int>(global_width) + gx;
      int8_t& global_val = global.data[global_idx];

      // If the global cell is unknown, just take the local value.
      if (global_val < 0) {
        global_val = local_val;
        continue;
      }

      // Convert to probabilities in [0, 1]
      const double p_global = static_cast<double>(global_val) / 100.0;
      const double p_local = static_cast<double>(local_val) / 100.0;

      // Blend: newer map (larger alpha) pulls harder toward p_local
      double p_new = (1.0 - alpha) * p_global + alpha * p_local;

      // Clamp and store back as int8 [0, 100]
      p_new = std::clamp(p_new, 0.0, 1.0);
      global_val = static_cast<int8_t>(std::round(100.0 * p_new));
    }
  }
}
