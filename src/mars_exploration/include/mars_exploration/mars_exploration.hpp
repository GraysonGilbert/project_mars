/**
 * @file mars_exploration.hpp
 * @author Marcus Hurt (mhurt@umd.edu)
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief Core logic for MARS exploration: choosing exploration goals.
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

#include <nav_msgs/msg/occupancy_grid.hpp>

struct Pose2D {
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class MarsExploration {
 public:
  /**
   * @brief Construct a MarsExploration object.
   * @param min_frontier_distance_m Minimum distance (meters) for exploration
   * goals from robot.
   */
  explicit MarsExploration(double min_frontier_distance_m = 0.3)
      : min_frontier_distance_m_(min_frontier_distance_m) {}

  /**
   * @brief Set the occupancy grid map for exploration.
   * @param map The occupancy grid map.
   */
  void setMap(const nav_msgs::msg::OccupancyGrid& map) {
    map_ = map;
    have_map_ = true;
  }

  /**
   * @brief Update the robot's pose.
   * @param pose The robot's current pose (x, y, yaw).
   */
  void updatePose(const Pose2D& pose) {
    robot_pose_ = pose;
    have_pose_ = true;
  }

  /**
   * @brief Check if an exploration goal is available.
   * @return True if a goal is set, false otherwise.
   */
  bool hasGoal() const { return have_goal_; }

  /**
   * @brief Get the current exploration goal.
   * @return Reference to the current goal pose.
   */
  const Pose2D& getGoal() const { return goal_; }

  /**
   * @brief Clear the current exploration goal.
   */
  void clearGoal() { have_goal_ = false; }

  /**
   * @brief Pick the nearest unknown cell at least min_frontier_distance_m_ from
   * robot as a goal.
   * @return True if a goal was set, false otherwise.
   */
  bool setNearestUnmappedCellAsGoal();

  /**
   * @brief Set the minimum frontier distance for exploration goals.
   * @param d Minimum distance in meters.
   */
  void setMinFrontierDistance(double d) { min_frontier_distance_m_ = d; }

 private:
  nav_msgs::msg::OccupancyGrid map_;
  bool have_map_{false};

  Pose2D robot_pose_;
  bool have_pose_{false};

  Pose2D goal_;
  bool have_goal_{false};

  Pose2D last_goal_;
  bool have_last_goal_ = false;
  double same_goal_tolerance_m_ = 0.10;
  int same_goal_repeats_ = 0;
  int max_same_goal_repeats_ = 3;

  bool include_low_confidence_cells_{true};
  int low_confidence_value_{20};

  double min_frontier_distance_m_;  // meters
};
