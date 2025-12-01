// mars_exploration.hpp

#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

struct Pose2D
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

class MarsExploration
{
public:
  explicit MarsExploration(double min_frontier_distance_m = 0.3)
  : min_frontier_distance_m_(min_frontier_distance_m)
  {}

  void setMap(const nav_msgs::msg::OccupancyGrid & map)
  {
    map_ = map;
    have_map_ = true;
  }

  void updatePose(const Pose2D & pose)
  {
    robot_pose_ = pose;
    have_pose_ = true;
  }

  bool hasGoal() const { return have_goal_; }

  const Pose2D & getGoal() const { return goal_; }

  void clearGoal() { have_goal_ = false; }

  /// Pick the nearest unknown cell at least `min_frontier_distance_m_` from robot.
  bool setNearestUnmappedCellAsGoal();

  void setMinFrontierDistance(double d)
  {
    min_frontier_distance_m_ = d;
  }

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
  int    same_goal_repeats_ = 0;
  int    max_same_goal_repeats_ = 3;

  bool include_low_confidence_cells_{true};
  int  low_confidence_value_{20};

  double min_frontier_distance_m_;  // meters
};
