/**
 * @file map_merger.hpp
 * @author your name (you@domain.com)
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-11-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */


#ifndef MAP_MERGER_HPP_
#define MAP_MERGER_HPP_

#include <vector>
#include <optional>
#include <string>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


class MapMerger {
    public:

        explicit MapMerger(double max_map_age_sec=10.0);


        nav_msgs::msg::OccupancyGrid initialize_global_map(double min_x, double min_y, double max_x, double max_y, double resolution) const;  


        bool is_map_recent(const nav_msgs::msg::OccupancyGrid& map, const rclcpp::Time& current_time) const;


        void merge_maps(nav_msgs::msg::OccupancyGrid& global_map, const std::vector<nav_msgs::msg::OccupancyGrid>& local_maps) const;


                                                   
    private:

        void overlay_map(nav_msgs::msg::OccupancyGrid& global, const nav_msgs::msg::OccupancyGrid& local) const;

        double max_map_age_sec_;
        
};

#endif