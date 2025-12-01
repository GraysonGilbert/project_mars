/**
 * @file map_merger.cpp
 * @author your name (you@domain.com)
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-11-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include "mars_overseer/map_merger.hpp"
//#include <nav_msgs/msg/detail/occupancy_grid__struct.hpp>
//#include <optional>
//#include <vector>



MapMerger::MapMerger(double max_map_age_sec) : max_map_age_sec_(max_map_age_sec){}


nav_msgs::msg::OccupancyGrid MapMerger::initialize_global_map(double min_x, double min_y, double max_x, double max_y, double resolution) const{
    
    nav_msgs::msg::OccupancyGrid global_map;

    global_map.info.origin.position.x = min_x;
    global_map.info.origin.position.y = min_y;
    global_map.info.origin.position.z = 0.0;

    global_map.info.origin.orientation.w = 1.0;

    global_map.info.resolution = resolution;

    global_map.info.width = static_cast<uint32_t>(std::ceil((max_x - min_x) / resolution));
    global_map.info.height = static_cast<uint32_t>(std::ceil((max_y - min_y) / resolution));

    global_map.data.resize(global_map.info.width * global_map.info.height, -1);

    global_map.header.frame_id = "global_map";

    return global_map; // Stub implementation for initialize global map
}


bool MapMerger::is_map_recent(const nav_msgs::msg::OccupancyGrid& map, const rclcpp::Time& current_time) const {

    rclcpp::Time map_time = map.header.stamp;
    return (current_time - map_time).seconds() <= max_map_age_sec_;
}


void MapMerger::merge_maps(nav_msgs::msg::OccupancyGrid& global_map, const std::vector<nav_msgs::msg::OccupancyGrid>& local_maps) const{

    for (const auto& local : local_maps) {
        overlay_map(global_map, local);
    }
} 

void MapMerger::overlay_map(nav_msgs::msg::OccupancyGrid& global, const nav_msgs::msg::OccupancyGrid& local) const {

    double resolution = global.info.resolution;

    int global_width = global.info.width;
    int global_height = global.info.height;

    double gx0 = global.info.origin.position.x;
    double gy0 = global.info.origin.position.y;

    double lx0 = local.info.origin.position.x;
    double ly0 = local.info.origin.position.y;

    //global.data = global.data;

    for (uint32_t y = 0; y < local.info.height; ++y) {
        for (uint32_t x = 0; x < local.info.width; ++x) {
            int local_idx = y * local.info.width + x;
            int8_t value = local.data[local_idx];

            if (value < 0) continue; // unknown, skip

            // Compute global indices
            int gx = static_cast<int>(std::round((lx0 + x * resolution - gx0) / resolution));
            int gy = static_cast<int>(std::round((ly0 + y * resolution - gy0) / resolution));

            if (gx < 0 || gy < 0 || gx >= static_cast<int>(global_width) || gy >= static_cast<int>(global_height))
                continue; // outside bounds

            int global_idx = gy * global_width + gx;

            // Merge strategy: overwrite or take max occupancy
            global.data[global_idx] = std::max(global.data[global_idx], value);
        }
    }
}