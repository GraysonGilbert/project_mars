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

OverseerNode::OverseerNode() : Node("overseer_node") {
  // Create TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publish transform at 10 Hz
  tf_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&OverseerNode::publish_global_map_tf, this));

  // Placeholder vector for two robot simulation
  std::vector<std::string> robot_ids = {"robot_1", "robot_2"};

  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.reliable();
  map_qos.transient_local();

  // // Publisher for global map
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_map",
                                                                  map_qos);

  // Subscribes to all slam_toolbox map topics being published
  for (const auto& robot_id : robot_ids) {
    std::string topic_name = "/" + robot_id + "/map";
    RCLCPP_INFO(this->get_logger(), "Subscribing to %s", topic_name.c_str());

    // Lambda captures the robot_id by value
    map_subs_[robot_id] =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            topic_name, map_qos,
            [this,
             robot_id](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
              this->map_callback(msg, robot_id);
            });
  }

  RCLCPP_INFO(this->get_logger(),
              "Overseer node started. Subscribing to /map and publishing to "
              "/global_map");

  // Brute force way to initialize global map
  double min_x = -10.0;
  double min_y = -10.0;
  double max_x = 10.0;
  double max_y = 10.0;
  double resolution = 0.05;

  global_map_ =
      map_merger_.initialize_global_map(min_x, min_y, max_x, max_y, resolution);

  RCLCPP_INFO(this->get_logger(), "Global map initialized.");

  // End of Global Map Initialization
}

void OverseerNode::publish_global_map_tf() {
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
    const std::string& robot_id) {
  rclcpp::Time now = this->get_clock()->now();

  if (!map_merger_.is_map_recent(*msg, now)) {
    RCLCPP_WARN(this->get_logger(), "Received stale map from %s, ignoring",
                robot_id.c_str());
    return;
  }

  // ---------------------------------------------------------------------
  // 1) Look up transform from local map frame -> global_map
  // ---------------------------------------------------------------------
  const std::string target_frame = "global_map";
  const std::string source_frame = msg->header.frame_id;  // e.g. "robot_1/map"

  geometry_msgs::msg::TransformStamped T_global_local;

  try {
    // small timeout to avoid blocking forever if TF is missing
    T_global_local = tf_buffer_->lookupTransform(target_frame, source_frame,
                                                 msg->header.stamp,
                                                 tf2::durationFromSec(0.1));
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN(this->get_logger(),
                "Overseer: TF lookup failed (%s -> %s) for robot %s: %s",
                target_frame.c_str(), source_frame.c_str(), robot_id.c_str(),
                ex.what());
    return;
  }

  // ---------------------------------------------------------------------
  // 2) Transform the map origin pose into global_map frame
  // ---------------------------------------------------------------------
  geometry_msgs::msg::PoseStamped origin_local;
  geometry_msgs::msg::PoseStamped origin_global;

  origin_local.header.frame_id = source_frame;
  origin_local.header.stamp = msg->header.stamp;
  origin_local.pose = msg->info.origin;  // origin in local map frame

  tf2::doTransform(origin_local, origin_global, T_global_local);

  // ---------------------------------------------------------------------
  // 3) Create a copy of the map with origin expressed in global_map frame
  // ---------------------------------------------------------------------
  nav_msgs::msg::OccupancyGrid transformed = *msg;
  transformed.header.frame_id = target_frame;
  transformed.info.origin = origin_global.pose;

  // Store robot's local map in *global* coordinates
  local_maps_[robot_id] = transformed;

  // ---------------------------------------------------------------------
  // 4) Merge all local maps
  // ---------------------------------------------------------------------
  map_merger_.merge_maps(global_map_, get_all_local_maps_vector());

  global_map_.header.stamp = now;
  map_pub_->publish(global_map_);

  // RCLCPP_INFO(this->get_logger(), "Merged map published after receiving data
  // from %s", robot_id.c_str());
}

std::vector<nav_msgs::msg::OccupancyGrid>
OverseerNode::get_all_local_maps_vector() {
  std::vector<nav_msgs::msg::OccupancyGrid> maps;
  for (auto& pair : local_maps_) {
    maps.push_back(pair.second);
  }
  return maps;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OverseerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}