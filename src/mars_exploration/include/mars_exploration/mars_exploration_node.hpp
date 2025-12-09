/**
 * @file mars_exploration_node.hpp
 * @author Marcus Hurt (mhurt@umd.edu)
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief ROS 2 node wrapper for the MarsExploration core logic (goal-only).
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

#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <string>

#include "mars_exploration.hpp"

class MarsExplorationNode : public rclcpp::Node {
 public:
  explicit MarsExplorationNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

 private:
  // --- Callbacks ------------------------------------------------------------

  void goalPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void controlTimerCallback();

  // --- Helpers --------------------------------------------------------------

  static double quaternionToYaw(const geometry_msgs::msg::Quaternion& q);
  static geometry_msgs::msg::Quaternion yawToQuaternion(double yaw);

  // --- Members --------------------------------------------------------------

  MarsExploration exploration_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

  rclcpp::TimerBase::SharedPtr control_timer_;

  // Parameters;
  double min_goal_distance_{0.3};  // m: do not send goals closer than this
  double reject_retry_delay_sec_{
      3.0};  // s: wait after rejection before trying again
  double control_rate_hz_{0.1};
  std::string goal_topic_{"goal_pose"};
  std::string cmd_vel_topic_{"cmd_vel"};
  std::string map_topic_{"map"};
  std::string global_frame_{"map"};
  std::string base_frame_{"base_link"};
  // For dt computation
  rclcpp::Time last_control_time_;
  bool have_last_control_time_{false};

  // Nav2 action client
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_action_client_;

  // Track whether a Nav2 goal is currently active (manual or exploration)
  bool have_active_nav_goal_{false};

  // Track whether we have ever successfully gotten a map->base_link pose
  bool have_slam_pose_{false};

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  rclcpp::Time last_goal_send_time_;
  rclcpp::Time last_goal_reject_time_;

  double max_goal_duration_sec_{5.0};

  bool map_dirty_{false};
};
