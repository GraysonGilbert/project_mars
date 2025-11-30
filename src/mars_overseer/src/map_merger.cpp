/**
 * @file map_merger.cpp
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

    global_map.info.origin.position.x = 0.0;
    global_map.info.origin.position.y = 0.0;
    global_map.info.origin.position.z = 0.0;

    global_map.info.origin.orientation.w = 1.0;

    global_map.info.resolution = 0.05;

    global_map.info.width = 100;
    global_map.info.height = 100;

    global_map.data.resize(global_map.info.width * global_map.info.height, -1);

    global_map.header.frame_id = "global_map";

    return global_map; // Stub implementation for initialize global map
}


bool MapMerger::is_map_recent(const nav_msgs::msg::OccupancyGrid& map, const rclcpp::Time& current_time) const {

    return false; // Stub implementation for testing
}


void MapMerger::merge_maps(nav_msgs::msg::OccupancyGrid& global_map, const std::vector<nav_msgs::msg::OccupancyGrid>& local_maps) const{

} // stub implmentation

void MapMerger::overlay_map(nav_msgs::msg::OccupancyGrid& global, const nav_msgs::msg::OccupancyGrid& local) const {
    // Stub implementation for overlay maps
}