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


MapMerger::MapMerger(double max_map_age_sec) : max_map_age_sec_(max_map_age_sec){}


nav_msgs::msg::OccupancyGrid MapMerger::initialize_global_map(double min_x, double min_y, double max_x, double max_y, double resolution) const{
    
    nav_msgs::msg::OccupancyGrid global_map;

    // Set map origin pose
    global_map.info.origin.position.x = min_x;
    global_map.info.origin.position.y = min_y;
    global_map.info.origin.position.z = 0.0;
    global_map.info.origin.orientation.w = 1.0; // identity quaternion

    global_map.info.resolution = resolution;

    // Compute number of grid cells along each dimension
    global_map.info.width = static_cast<uint32_t>(std::ceil((max_x - min_x) / resolution));
    global_map.info.height = static_cast<uint32_t>(std::ceil((max_y - min_y) / resolution));

    // Initialize all cells to -1 (unknown)
    global_map.data.resize(global_map.info.width * global_map.info.height, -1);

    // Set a consistent frame ID for the merged global map
    global_map.header.frame_id = "global_map";

    return global_map;
}


bool MapMerger::is_map_recent(const nav_msgs::msg::OccupancyGrid& map, const rclcpp::Time& current_time) const {

    // Extract timestamp from map header.
    rclcpp::Time map_time = map.header.stamp;

    // Check if map age is less than max_map_age_sec_
    return (current_time - map_time).seconds() <= max_map_age_sec_;
}


void MapMerger::merge_maps(nav_msgs::msg::OccupancyGrid& global_map, const std::vector<nav_msgs::msg::OccupancyGrid>& local_maps) const{
    
    const double EPS = 1e-4;  // tolerance for resolution comparison

    // Iterate through each local map and overlay it onto the global map
    for (const auto& local : local_maps) {
        
        if (std::abs(global_map.info.resolution - local.info.resolution) > EPS) {
            throw std::runtime_error("Occupancy grid resolutions do not match.");
        }

        overlay_map(global_map, local);
    }
} 

void MapMerger::overlay_map(nav_msgs::msg::OccupancyGrid& global, const nav_msgs::msg::OccupancyGrid& local) const {

    // Shared resolution assumption
    double resolution = global.info.resolution;

    int global_width = global.info.width;
    int global_height = global.info.height;

    // Global map origin
    double gx0 = global.info.origin.position.x;
    double gy0 = global.info.origin.position.y;

    // Local map origin
    double lx0 = local.info.origin.position.x;
    double ly0 = local.info.origin.position.y;

    // Loop through the local map cells
    for (uint32_t y = 0; y < local.info.height; ++y) {
        for (uint32_t x = 0; x < local.info.width; ++x) {
            int local_idx = y * local.info.width + x;
            int8_t value = local.data[local_idx];

            // Skip unknown values
            if (value < 0) continue; 

            // Convert local cell position to world coordinates, then to global map indices
            int gx = static_cast<int>(std::round((lx0 + x * resolution - gx0) / resolution));
            int gy = static_cast<int>(std::round((ly0 + y * resolution - gy0) / resolution));

            // Ensure the cell lies inside global bounds
            if (gx < 0 || gy < 0 || gx >= static_cast<int>(global_width) || gy >= static_cast<int>(global_height))
                continue; 

            int global_idx = gy * global_width + gx;

            // Overwrite or take max occupancy
            global.data[global_idx] = std::max(global.data[global_idx], value);
        }
    }
}