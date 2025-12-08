/**
 * @file mars_exploration.cpp
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
#include "mars_exploration/mars_exploration.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

bool MarsExploration::setNearestUnmappedCellAsGoal(double now_sec) {
  if (!have_map_ || !have_pose_) {
    have_goal_ = false;
    return false;
  }

  pruneFailedGoals(now_sec);

  const auto &info = map_.info;
  const auto &data = map_.data;
  const int width = static_cast<int>(info.width);
  const int height = static_cast<int>(info.height);
  const double res = info.resolution;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;

  const double min_dist2 = min_frontier_distance_m_ * min_frontier_distance_m_;

  // First pass: nearest valid "frontier" cell
  double best_dist2 = std::numeric_limits<double>::infinity();
  Pose2D best_goal;
  bool found = false;

  auto cell_is_valid_frontier = [&](int x, int y, double &wx, double &wy,
                                    double &dist2) -> bool {
    // 0) Skip cells too close to map edges to avoid costmap cropping
    if (x < border_margin_cells_ ||
        y < border_margin_cells_ ||
        x >= width - border_margin_cells_ ||
        y >= height - border_margin_cells_) {
      return false;
    }

    const int idx = y * width + x;
    const int occ = data[idx];

    // --- 1) Decide if this cell is "explorable" (unknown or low-confidence)
    // ---
    bool explorable = false;

    if (occ == -1) {
      // Classic unknown cell
      explorable = true;
    } else if (include_low_confidence_cells_) {
      // OccupancyGrid convention: 0 = definitely free, 100 = definitely
      // occupied, 1..99 = prob Treat 1..low_confidence_value_ as low-confidence
      // / "unknown-ish" space.
      if (occ > 0 && occ <= low_confidence_value_) {
        explorable = true;
      }
    }

    if (!explorable) {
      return false;
    }

    // --- 2) World coordinates of cell center ---
    wx = ox + (x + 0.5) * res;
    wy = oy + (y + 0.5) * res;

    // --- 3) Distance from robot & min distance check ---
    const double dx = wx - robot_pose_.x;
    const double dy = wy - robot_pose_.y;
    dist2 = dx * dx + dy * dy;

    if (dist2 < min_dist2) {
      return false;
    }

    // --- 4) Require adjacency to a known FREE cell and not near a previous failed goal ---
    for (const auto &fg : failed_goals_) {
      const double dxg = wx - fg.pose.x;
      const double dyg = wy - fg.pose.y;
      const double d2g = dxg * dxg + dyg * dyg;
      if (d2g < failed_goal_radius_m2_) {
        return false;
      }
    }

    bool adjacent_to_free = false;
    for (int ny = std::max(0, y - 1); ny <= std::min(height - 1, y + 1); ++ny) {
      for (int nx = std::max(0, x - 1); nx <= std::min(width - 1, x + 1);
           ++nx) {
        if (nx == x && ny == y) {
          continue;
        }
        const int nidx = ny * width + nx;
        if (include_low_confidence_cells_) {
          if (data[nidx] < low_confidence_value_) {
            adjacent_to_free = true;
            break;
          }
        } else {
          if (data[nidx] == 0) {  // definitely free
            adjacent_to_free = true;
            break;
          }
        }
      }
      if (adjacent_to_free) {
        break;
      }
    }

    if (!adjacent_to_free) {
      return false;
    }

    return true;
  };

  // ---------- Pass 1: nearest frontier ----------
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      double wx, wy, dist2;
      if (!cell_is_valid_frontier(x, y, wx, wy, dist2)) {
        continue;
      }

      if (dist2 < best_dist2) {
        best_dist2 = dist2;
        best_goal.x = wx;
        best_goal.y = wy;
        best_goal.yaw = std::atan2(wy - robot_pose_.y, wx - robot_pose_.x);
        found = true;
      }
    }
  }

  if (!found) {
    have_goal_ = false;
    return false;
  }

  // ---------- Check: same as last goal? ----------
  bool same_as_last = false;
  if (have_last_goal_) {
    const double dxg = best_goal.x - last_goal_.x;
    const double dyg = best_goal.y - last_goal_.y;
    const double same_tol2 = same_goal_tolerance_m_ * same_goal_tolerance_m_;
    same_as_last = (dxg * dxg + dyg * dyg) < same_tol2;
    same_goal_repeats_++;
  }

  // ---------- Pass 2: if same goal, pick a more "strategic" one ----------
  if (same_as_last && same_goal_repeats_ >= max_same_goal_repeats_) {
    same_goal_repeats_ = 0;
    double farthest_dist2 = 0.0;
    Pose2D far_goal;
    bool found_far = false;

    for (int y = 0; y < height; ++y) {
      for (int x = 0; x < width; ++x) {
        double wx, wy, dist2;
        if (!cell_is_valid_frontier(x, y, wx, wy, dist2)) {
          continue;
        }

        if (dist2 > farthest_dist2) {
          farthest_dist2 = dist2;
          far_goal.x = wx;
          far_goal.y = wy;
          far_goal.yaw = std::atan2(wy - robot_pose_.y, wx - robot_pose_.x);
          found_far = true;
        }
      }
    }

    if (found_far) {
      best_goal = far_goal;
      best_dist2 = farthest_dist2;
    }
  }

  // ---------- Commit goal ----------
  goal_ = best_goal;
  have_goal_ = true;
  last_goal_ = best_goal;
  have_last_goal_ = true;

  return true;
}

void MarsExploration::markLastGoalFailed(double now_sec) {
  if (!have_last_goal_) return;
  failed_goals_.push_back(FailedGoal{last_goal_, now_sec});
}

void MarsExploration::pruneFailedGoals(double now_sec) {
  failed_goals_.erase(
      std::remove_if(failed_goals_.begin(), failed_goals_.end(),
                     [&](const FailedGoal &fg) {
                       return (now_sec - fg.timestamp_sec) >
                              failed_goal_forget_time_sec_;
                     }),
      failed_goals_.end());
}
