/**
 * @file overseer.hpp
 * @author your name (you@domain.com)
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-11-30
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#ifndef OVERSEER_NODE_HPP
#define OVERSEER_NODE_HPP

#include <map>
#include <string>

#include "mars_overseer/map_merger.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2_ros/transform_broadcaster.h"

class OverseerNode : public rclcpp::Node
{
public:
    OverseerNode();

private:
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    void publish_global_map_tf();

    MapMerger map_merger_{10.0};


    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr tf_timer_;

    nav_msgs::msg::OccupancyGrid global_map_;

    std::map<std::string, nav_msgs::msg::OccupancyGrid> local_maps_;
    

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    //std::map<std::string, rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> map_subs_;

};

#endif // OVERSEER_NODE_HPP

