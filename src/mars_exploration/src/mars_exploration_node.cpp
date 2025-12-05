/**
 * @file mars_exploration_node.cpp
 * @author Marcus Hurt (mhurt@umd.edu)
 * @author Grayson Gilbert (ggilbert@umd.edu)
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
#include "mars_exploration/mars_exploration_node.hpp"

#include <chrono>
#include <cmath>
#include <rclcpp/logging.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

using namespace std::chrono_literals;

// ----------------------------------------------------------------------------
// Constructor
// ----------------------------------------------------------------------------

MarsExplorationNode::MarsExplorationNode(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mars_exploration_node", options) {
  // Declare and get parameters
  control_rate_hz_ =
      this->declare_parameter<double>("control_rate_hz", control_rate_hz_);
  goal_topic_ = this->declare_parameter<std::string>("goal_topic", goal_topic_);
  cmd_vel_topic_ =
      this->declare_parameter<std::string>("cmd_vel_topic", cmd_vel_topic_);
  map_topic_ =
      this->declare_parameter<std::string>("map_topic", map_topic_);
  global_frame_ =
      this->declare_parameter<std::string>("global_frame", global_frame_);
  base_frame_ = this->declare_parameter<std::string>("base_frame", base_frame_);
  double min_frontier_distance =
      this->declare_parameter<double>("min_frontier_distance", 0.5);
  max_goal_duration_sec_ =
      this->declare_parameter<double>("max_goal_duration_sec", 60.0);

  exploration_ = MarsExploration(min_frontier_distance);

  // TF buffer + listener
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriptions
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      std::bind(&MarsExplorationNode::goalPoseCallback, this,
                std::placeholders::_1));

  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::SystemDefaultsQoS(),
      std::bind(&MarsExplorationNode::mapCallback, this,
                std::placeholders::_1));

  // Nav2 NavigateToPose action client
  nav2_action_client_ = rclcpp_action::create_client<NavigateToPose>(
      get_node_base_interface(), get_node_graph_interface(),
      get_node_logging_interface(), get_node_waitables_interface(),
      "navigate_to_pose");

  // Control loop timer
  auto period = std::chrono::duration<double>(1.0 / control_rate_hz_);
  control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&MarsExplorationNode::controlTimerCallback, this));

  RCLCPP_INFO(get_logger(), "MarsExplorationNode started with:");
  RCLCPP_INFO(get_logger(), "  goal_topic:     %s", goal_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  cmd_vel_topic:  %s", cmd_vel_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  map_topic:      %s", map_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  global_frame:   %s", global_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  base_frame:     %s", base_frame_.c_str());
  RCLCPP_INFO(get_logger(), "  control_rate_hz: %.2f", control_rate_hz_);
}

// ----------------------------------------------------------------------------
// Callbacks
// ----------------------------------------------------------------------------

void MarsExplorationNode::goalPoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  if (!nav2_action_client_) {
    RCLCPP_ERROR(get_logger(), "Nav2 action client not initialized!");
    return;
  }

  if (!nav2_action_client_->action_server_is_ready()) {
    RCLCPP_ERROR(get_logger(),
                 "Nav2 NavigateToPose action server not available!");
    return;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose = *msg;

  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  send_goal_options.result_callback =
      [this](const GoalHandleNavigateToPose::WrappedResult& result) {
        have_active_nav_goal_ = false;

        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Manual Nav2 goal SUCCEEDED.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Manual Nav2 goal ABORTED.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Manual Nav2 goal CANCELED.");
            break;
          default:
            RCLCPP_WARN(this->get_logger(),
                        "Manual Nav2 goal finished with unknown result code.");
            break;
        }
      };

  nav2_action_client_->async_send_goal(goal_msg, send_goal_options);

  // Manual goal is now active; track its send time for timeout logic
  have_active_nav_goal_ = true;
  last_goal_send_time_ = this->now();

  RCLCPP_INFO(get_logger(), "Sent manual Nav2 goal: x=%.3f, y=%.3f, yaw=%.3f",
              msg->pose.position.x, msg->pose.position.y,
              quaternionToYaw(msg->pose.orientation));
}

void MarsExplorationNode::mapCallback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  exploration_.setMap(*msg);
  map_dirty_ = true;
}

// ----------------------------------------------------------------------------
// Control timer: TF-based pose + frontier goal -> Nav2
// ----------------------------------------------------------------------------

void MarsExplorationNode::controlTimerCallback() {
  // ------------------------------------------------------------
  // 1) Compute dt from last call
  // ------------------------------------------------------------
  const auto now = this->now();
  double dt = 1.0 / control_rate_hz_;
  if (have_last_control_time_) {
    const auto dt_nsec = (now - last_control_time_).nanoseconds();
    if (dt_nsec > 0) {
      dt = static_cast<double>(dt_nsec) * 1e-9;
    }
    if (dt > 1.0) {
      RCLCPP_WARN(
          get_logger(),
          "Large dt=%.3f s detected in control loop; check for timer delays.",
          dt);
    }
  }
  last_control_time_ = now;
  have_last_control_time_ = true;

  // ------------------------------------------------------------
  // 2) Get robot pose from TF and set have_slam_pose_
  // ------------------------------------------------------------
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(global_frame_,      // e.g. "map"
                                     base_frame_,        // e.g. "base_link"
                                     tf2::TimePointZero  // latest available
    );
    have_slam_pose_ = true;
  } catch (const tf2::TransformException& ex) {
    have_slam_pose_ = false;
    // No valid pose yet -> no exploration / Nav2 goals
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Waiting for first SLAM pose before starting exploration... (%s)",
        ex.what());
    return;
  }

  Pose2D robot_pose;
  robot_pose.x = tf.transform.translation.x;
  robot_pose.y = tf.transform.translation.y;
  robot_pose.yaw = quaternionToYaw(tf.transform.rotation);
  exploration_.updatePose(robot_pose);

  // ------------------------------------------------------------
  // 3) If a Nav2 goal is already active, check for timeout
  // ------------------------------------------------------------
  if (have_active_nav_goal_) {
    rclcpp::Duration elapsed = now - last_goal_send_time_;
    if (elapsed.seconds() > max_goal_duration_sec_) {
      RCLCPP_WARN(get_logger(),
                  "Exploration Nav2 goal timed out after %.1f s. "
                  "Canceling goal and selecting a new frontier.",
                  elapsed.seconds());

      // Cancel the active Nav2 goal
      if (nav2_action_client_) {
        nav2_action_client_->async_cancel_all_goals();
      }

      // Mark no active goal and clear exploration goal so a new one is picked
      have_active_nav_goal_ = false;
      exploration_.clearGoal();

      // Treat this like a rejection to get a brief cool-down
      last_goal_reject_time_ = now;
    }

    // Either we just timed out (and will pick a new goal on next tick)
    // or the goal is still running; in both cases, we don't send a new Nav2
    // goal this cycle.
    return;
  }

  // ------------------------------------------------------------
  // 4) If we recently got a REJECTED goal, wait a bit before
  //    trying again (avoids rejection storm during bringup).
  // ------------------------------------------------------------
  if (last_goal_reject_time_.nanoseconds() != 0) {
    rclcpp::Duration since_reject = now - last_goal_reject_time_;
    if (since_reject.seconds() < reject_retry_delay_sec_) {
      // Still in cool-down after rejection
      return;
    }
  }

  // Optionally: also throttle how often we send *any* goal
  const double min_goal_period_sec = 0.2;  // 5 Hz max goal send
  if (last_goal_send_time_.nanoseconds() != 0) {
    rclcpp::Duration since_send = now - last_goal_send_time_;
    if (since_send.seconds() < min_goal_period_sec) {
      return;
    }
  }

  // ------------------------------------------------------------
  // 5) Ensure Nav2 NavigateToPose action server is available
  // ------------------------------------------------------------
  if (!nav2_action_client_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 5000,
                          "Nav2 NavigateToPose action client not initialized; "
                          "cannot send exploration goal.");
    return;
  }

  using namespace std::chrono_literals;
  if (!nav2_action_client_->wait_for_action_server(500ms)) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Nav2 NavigateToPose action server not available yet; will retry.");
    return;
  }

  // ------------------------------------------------------------
  // 6) Choose an exploration goal
  //    - Only search for a new goal if the map is dirty
  // ------------------------------------------------------------
  if (!exploration_.hasGoal()) {
    if (!map_dirty_) {
      // Map hasn't changed since last time and we have no goal -> nothing to do
      return;
    }

    if (!exploration_.setNearestUnmappedCellAsGoal()) {
      // No frontier / unmapped cell found -> exploration done
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "No suitable unmapped cell found. Exploration may be complete.");
      map_dirty_ = false;
      return;
    }

    map_dirty_ = false;

    Pose2D g = exploration_.getGoal();
    RCLCPP_INFO(get_logger(),
                "New exploration goal set to nearest unmapped cell at (%.2f, "
                "%.2f, yaw=%.2f)",
                g.x, g.y, g.yaw);
  }

  Pose2D goal = exploration_.getGoal();

  // ------------------------------------------------------------
  // 7) Avoid sending goals that are too close to current pose
  // ------------------------------------------------------------
  const double dx = goal.x - robot_pose.x;
  const double dy = goal.y - robot_pose.y;
  const double dist = std::sqrt(dx * dx + dy * dy);

  if (dist < min_goal_distance_) {
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Candidate exploration goal is too close to current pose "
        "(dist=%.3f < %.3f). Treating as done here and not sending a Nav2 "
        "goal.",
        dist, min_goal_distance_);
    exploration_.clearGoal();
    return;
  }

  // ------------------------------------------------------------
  // 8) Build NavigateToPose goal message in global_frame_
  // ------------------------------------------------------------
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.stamp = now;
  goal_pose.header.frame_id = global_frame_;

  goal_pose.pose.position.x = goal.x;
  goal_pose.pose.position.y = goal.y;
  goal_pose.pose.position.z = 0.0;
  goal_pose.pose.orientation = yawToQuaternion(goal.yaw);

  NavigateToPose::Goal nav_goal;
  nav_goal.pose = goal_pose;

  using GoalHandleNavigateToPose =
      rclcpp_action::ClientGoalHandle<NavigateToPose>;
  auto send_goal_options =
      rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

  // ------------------------------------------------------------
  // 9) Goal response callback (ACCEPTED / REJECTED)
  // ------------------------------------------------------------
  send_goal_options.goal_response_callback =
      [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle) {
        if (!goal_handle) {
          RCLCPP_WARN(
              this->get_logger(),
              "Exploration Nav2 goal was REJECTED by the action server.");
          have_active_nav_goal_ = false;
          exploration_.clearGoal();
          last_goal_reject_time_ = this->now();
          return;
        }

        RCLCPP_INFO(this->get_logger(),
                    "Exploration Nav2 goal was ACCEPTED by the action server.");
        // keep have_active_nav_goal_ = true until result callback
      };

  // ------------------------------------------------------------
  // 10) Result callback (SUCCEEDED / ABORTED / CANCELED)
  // ------------------------------------------------------------
  send_goal_options.result_callback =
      [this](const GoalHandleNavigateToPose::WrappedResult& result) {
        have_active_nav_goal_ = false;
        exploration_
            .clearGoal();  // Let the next timer tick choose another frontier

        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Exploration Nav2 goal SUCCEEDED.");
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(this->get_logger(), "Exploration Nav2 goal ABORTED.");
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(this->get_logger(), "Exploration Nav2 goal CANCELED.");
            break;
          default:
            RCLCPP_WARN(
                this->get_logger(),
                "Exploration Nav2 goal finished with unknown result code.");
            break;
        }
      };

  // ------------------------------------------------------------
  // 11) Send goal
  // ------------------------------------------------------------
  have_active_nav_goal_ = true;
  last_goal_send_time_ = now;
  nav2_action_client_->async_send_goal(nav_goal, send_goal_options);

  RCLCPP_INFO(get_logger(),
              "Sent Nav2 exploration goal to (%.2f, %.2f, yaw=%.2f), dist from "
              "robot = %.3f m.",
              goal.x, goal.y, goal.yaw, dist);
}

// ----------------------------------------------------------------------------
// Helpers
// ----------------------------------------------------------------------------

double MarsExplorationNode::quaternionToYaw(
    const geometry_msgs::msg::Quaternion& q) {
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::msg::Quaternion MarsExplorationNode::yawToQuaternion(
    double yaw) {
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}
