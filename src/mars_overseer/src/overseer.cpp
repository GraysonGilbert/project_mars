/**
 * @file overseer.cpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @author Marcus Hurt (mhurt@umd.edu)
 * @brief Implementation for Overseer node
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

#include "mars_overseer/overseer.hpp"
#include "mars_overseer/map_merger.hpp"



OverseerNode::OverseerNode() : Node("overseer_node")
{
    // Create TF broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Publish transform at 10 Hz
    tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&OverseerNode::publish_global_map_tf, this)
    );

    // Placeholder vector for two robot simulation
    std::vector<std::string> robot_ids = {"robot_1", "robot_2"};

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();

    // // Publisher for global map
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_map", map_qos);

    // Subscribes to all slam_toolbox map topics being published
    for (const auto& robot_id : robot_ids)
    {
        std::string topic_name = "/" + robot_id + "/map";
        RCLCPP_INFO(this->get_logger(), "Subscribing to %s", topic_name.c_str());

        // Lambda captures the robot_id by value
        map_subs_[robot_id] = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            topic_name, map_qos,
            [this, robot_id](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                this->map_callback(msg, robot_id);
            }
        );
    }

    RCLCPP_INFO(this->get_logger(), "Overseer node started. Subscribing to /map and publishing to /global_map");

    // Brute force way to initialize global map
    double min_x = -10.0;
    double min_y = -10.0;
    double max_x =  10.0;
    double max_y =  10.0;
    double resolution = 0.05;

    global_map_ = map_merger_.initialize_global_map(min_x, min_y, max_x, max_y, resolution);

    RCLCPP_INFO(this->get_logger(), "Global map initialized.");

    // End of Global Map Initialization

}

void OverseerNode::publish_global_map_tf()
{
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "global_map";
    t.child_frame_id = "map";       

    // Identity transform
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_broadcaster_->sendTransform(t);
}

void OverseerNode::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg,
    const std::string& robot_id)
{
    rclcpp::Time now = this->get_clock()->now();

    if (!map_merger_.is_map_recent(*msg, now)) {
        RCLCPP_WARN(this->get_logger(), "Received stale map from %s, ignoring", robot_id.c_str());
        return;
    }

    // Store robot's local map
    local_maps_[robot_id] = *msg;

    // Merge all local maps
    map_merger_.merge_maps(global_map_, get_all_local_maps_vector());

    global_map_.header.stamp = now;
    map_pub_->publish(global_map_);

    RCLCPP_INFO(this->get_logger(), "Merged map published after receiving data from %s", robot_id.c_str());
}

std::vector<nav_msgs::msg::OccupancyGrid> OverseerNode::get_all_local_maps_vector()
{
    std::vector<nav_msgs::msg::OccupancyGrid> maps;
    for (auto& pair : local_maps_)
    {
        maps.push_back(pair.second);
    }
    return maps;
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OverseerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}