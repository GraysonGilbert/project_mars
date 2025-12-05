/**
 * @file overseer.hpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @author Marcus Hurt (mhurt@umd.edu)
 * @brief Overseer node responsible for collecting local maps from multiple robots
 *        and merging them into a global occupancy grid.
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

    /**
     * @brief Construct a new Overseer Node object
     * 
     * Initializes publishers, subscribers, TF broadcaster, global map state,
     * and the map merger utility.
     */
    OverseerNode();

private:

    // Callback for receiving local robot occupancy grid maps
    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg, const std::string& robot_id);

    // Publishes the TF transform for the global map frame
    void publish_global_map_tf();

    // Helper to get vector of local maps
    std::vector<nav_msgs::msg::OccupancyGrid> get_all_local_maps_vector();

    // Utility class responsible for merging multiple occupancy grids into a single global map
    MapMerger map_merger_{10.0};

    // TF Broadcaster used to publish global map transform
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // Timer that periodically publishes TF transforms
    rclcpp::TimerBase::SharedPtr tf_timer_;

    // Resulting global occupancy grid after merging local robot maps
    nav_msgs::msg::OccupancyGrid global_map_;

    // Storage for each robots local map, keyed by robot ID
    std::map<std::string, nav_msgs::msg::OccupancyGrid> local_maps_;
    
    // Publisher for merged global map
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    // Subscriber for incoming local maps
    std::map<std::string, rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr> map_subs_;

};

#endif // OVERSEER_NODE_HPP

