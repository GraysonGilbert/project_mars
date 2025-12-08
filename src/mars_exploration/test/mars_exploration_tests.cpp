/**
 * @file mars_exploration_tests.cpp
 * @author Marcus Hurt (mhurt@umd.edu)
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @brief Unit tests for the MarsExploration core logic.
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

#include <gtest/gtest.h>

#include <nav_msgs/msg/occupancy_grid.hpp>

#include "mars_exploration/mars_exploration.hpp"

// Helper to create a simple OccupancyGrid
nav_msgs::msg::OccupancyGrid makeGrid(int width, int height, int value = -1) {
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = 1.0;
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 0.0;
  grid.data.resize(width * height, value);
  return grid;
}

TEST(MarsExplorationTest, NoMapNoPose_NoGoal) {
  MarsExploration explorer;
  EXPECT_FALSE(explorer.setNearestUnmappedCellAsGoal(0.0));
  EXPECT_FALSE(explorer.hasGoal());
}

TEST(MarsExplorationTest, OnlyMap_NoGoal) {
  MarsExploration explorer;
  auto grid = makeGrid(5, 5);
  explorer.setMap(grid);
  EXPECT_FALSE(explorer.setNearestUnmappedCellAsGoal(0.0));
  EXPECT_FALSE(explorer.hasGoal());
}

TEST(MarsExplorationTest, OnlyPose_NoGoal) {
  MarsExploration explorer;
  Pose2D pose{2.0, 2.0, 0.0};
  explorer.updatePose(pose);
  EXPECT_FALSE(explorer.setNearestUnmappedCellAsGoal(0.0));
  EXPECT_FALSE(explorer.hasGoal());
}

TEST(MarsExplorationTest, SimpleUnknownCell_GoalSet) {
  MarsExploration explorer;
  auto grid = makeGrid(3, 3, 0);  // all free
  grid.data[4] = -1;              // center cell unknown
  explorer.setBorderMarginCells(0);
  explorer.setMap(grid);
  Pose2D pose{0.0, 0.0, 0.0};
  explorer.updatePose(pose);
  explorer.setMinFrontierDistance(0.0);  // allow any distance
  EXPECT_TRUE(explorer.setNearestUnmappedCellAsGoal(0.0));
  EXPECT_TRUE(explorer.hasGoal());
  const auto &goal = explorer.getGoal();
  EXPECT_NEAR(goal.x, 1.5, 1e-6);
  EXPECT_NEAR(goal.y, 1.5, 1e-6);
}

TEST(MarsExplorationTest, NoFrontier_NoGoal) {
  MarsExploration explorer;
  explorer.setBorderMarginCells(0);
  auto grid = makeGrid(3, 3, 100);  // all occupied
  explorer.setMap(grid);
  Pose2D pose{1.0, 1.0, 0.0};
  explorer.updatePose(pose);
  EXPECT_FALSE(explorer.setNearestUnmappedCellAsGoal(0.0));
  EXPECT_FALSE(explorer.hasGoal());
}

TEST(MarsExplorationTest, LowConfidenceCells_GoalSet) {
  MarsExploration explorer;
  explorer.setBorderMarginCells(0);
  auto grid = makeGrid(3, 3, 0);  // all free
  grid.data[2] = 10;              // low confidence
  explorer.setMap(grid);
  Pose2D pose{0.0, 0.0, 0.0};
  explorer.updatePose(pose);
  explorer.setMinFrontierDistance(0.0);
  EXPECT_TRUE(explorer.setNearestUnmappedCellAsGoal(0.0));
  EXPECT_TRUE(explorer.hasGoal());
}

TEST(MarsExplorationTest, ClearGoal) {
  MarsExploration explorer;
  explorer.setBorderMarginCells(0);
  explorer.setBorderMarginCells(0);
  auto grid = makeGrid(5, 5, -1);
  grid.data[0] = 0;   // Top-left free
  grid.data[4] = 0;   // Top-right free
  grid.data[20] = 0;  // Bottom-left free
  explorer.setMap(grid);
  Pose2D pose{0.0, 0.0, 0.0};
  explorer.updatePose(pose);
  explorer.setMinFrontierDistance(0.0);
  EXPECT_TRUE(explorer.setNearestUnmappedCellAsGoal(0.0));
  explorer.clearGoal();
  EXPECT_FALSE(explorer.hasGoal());
  // Reset map to ensure a new frontier is available
  grid.data[24] = 0;  // Bottom-right free
  explorer.setMap(grid);
  EXPECT_TRUE(explorer.setNearestUnmappedCellAsGoal(30.0));
}

// Test changing include_low_confidence_cells_ and low_confidence_value_
TEST(MarsExplorationTest, LowConfidenceSettings) {
  MarsExploration explorer;
  explorer.setBorderMarginCells(0);
  auto grid = makeGrid(3, 3, 0);  // all free
  grid.data[1] = 15;              // low confidence
  explorer.setMap(grid);
  Pose2D pose{0.0, 0.0, 0.0};
  explorer.updatePose(pose);
  explorer.setMinFrontierDistance(0.0);
  // Should find low confidence cell as goal
  EXPECT_TRUE(explorer.setNearestUnmappedCellAsGoal(0.0));
  explorer.clearGoal();
  // Now set low_confidence_value_ lower so cell[1] is not considered low
  // confidence Directly access private member for test (if allowed), otherwise
  // skip For demonstration, assume we can set via a setter (not present in
  // class) explorer.setLowConfidenceValue(5); // If such setter exists Instead,
  // test with a cell value above default threshold
  auto grid2 = makeGrid(3, 3, 0);
  grid2.data[1] = 25;  // above default low_confidence_value_
  explorer.setMap(grid2);
  explorer.updatePose(pose);
  EXPECT_FALSE(explorer.setNearestUnmappedCellAsGoal(0.0));
}

// Test repeated goal selection logic (same goal tolerance and repeats)
TEST(MarsExplorationTest, SameGoalRepeatLogic) {
  MarsExploration explorer;
  explorer.setBorderMarginCells(0);
  auto grid = makeGrid(3, 3, -1);
  grid.data[0] = 0;  // free cell
  grid.data[8] =
      0;  // add a second free cell to allow repeat logic to find a new goal
  explorer.setMap(grid);
  Pose2D pose{0.0, 0.0, 0.0};
  explorer.updatePose(pose);
  explorer.setMinFrontierDistance(0.0);
  // Call setNearestUnmappedCellAsGoal multiple times to trigger repeat logic
  for (int i = 0; i < 5; ++i) {
    explorer.setNearestUnmappedCellAsGoal(0.0);
  }
  // Should still have a goal
  EXPECT_TRUE(explorer.hasGoal());
}

// Edge case: setMap with empty grid
TEST(MarsExplorationTest, EmptyMap) {
  MarsExploration explorer;
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = 0;
  grid.info.height = 0;
  explorer.setMap(grid);
  Pose2D pose{0.0, 0.0, 0.0};
  explorer.updatePose(pose);
  EXPECT_FALSE(explorer.setNearestUnmappedCellAsGoal(0.0));
}

// Edge case: updatePose with extreme values
TEST(MarsExplorationTest, ExtremePose) {
  MarsExploration explorer;
  explorer.setBorderMarginCells(0);
  auto grid = makeGrid(3, 3, -1);
  grid.data[0] = 0;
  grid.data[8] = 0;  // add a second free cell to ensure a frontier exists
  explorer.setMap(grid);
  Pose2D pose{1e6, -1e6, 0.0};
  explorer.updatePose(pose);
  explorer.setMinFrontierDistance(0.0);
  // Should still find a goal, but far away
  EXPECT_TRUE(explorer.setNearestUnmappedCellAsGoal(0.0));
}

// Test setMap with a map update
TEST(MarsExplorationTest, MapUpdateChangesGoal) {
  MarsExploration explorer;
  explorer.setBorderMarginCells(0);
  auto grid1 = makeGrid(3, 3, 0);
  grid1.data[4] = -1;  // center unknown
  grid1.data[0] = 0;   // ensure a free cell adjacent to unknown
  explorer.setMap(grid1);
  Pose2D pose{0.0, 0.0, 0.0};
  explorer.updatePose(pose);
  explorer.setMinFrontierDistance(0.0);
  EXPECT_TRUE(explorer.setNearestUnmappedCellAsGoal(0.0));
  auto goal1 = explorer.getGoal();
  auto grid2 = makeGrid(3, 3, 0);
  grid2.data[8] = -1;  // bottom-right unknown
  grid2.data[7] = 0;   // ensure a free cell adjacent to unknown
  explorer.setMap(grid2);
  EXPECT_TRUE(explorer.setNearestUnmappedCellAsGoal(0.0));
  auto goal2 = explorer.getGoal();
  EXPECT_NE(goal1.x, goal2.x);
  EXPECT_NE(goal1.y, goal2.y);
}

// Test updatePose with a pose update
TEST(MarsExplorationTest, PoseUpdateChangesGoal) {
  MarsExploration explorer;
  explorer.setBorderMarginCells(0);
  auto grid = makeGrid(3, 3, 0);
  grid.data[0] = -1;  // top-left unknown
  grid.data[8] = -1;  // bottom-right unknown
  grid.data[1] = 0;   // free cell adjacent to top-left unknown
  grid.data[7] = 0;   // free cell adjacent to bottom-right unknown
  explorer.setMap(grid);
  Pose2D pose1{0.0, 0.0, 0.0};  // near top-left
  explorer.updatePose(pose1);
  explorer.setMinFrontierDistance(0.0);
  EXPECT_TRUE(explorer.setNearestUnmappedCellAsGoal(0.0));
  auto goal1 = explorer.getGoal();
  Pose2D pose2{2.0, 2.0, 0.0};  // near bottom-right
  explorer.updatePose(pose2);
  EXPECT_TRUE(explorer.setNearestUnmappedCellAsGoal(0.0));
  auto goal2 = explorer.getGoal();
  EXPECT_NE(goal1.x, goal2.x);
  EXPECT_NE(goal1.y, goal2.y);
}

// Test clearGoal when no goal is set
TEST(MarsExplorationTest, ClearGoalNoGoalSet) {
  MarsExploration explorer;
  explorer.clearGoal();
  EXPECT_FALSE(explorer.hasGoal());
}

// Test getGoal when no goal is set
TEST(MarsExplorationTest, GetGoalNoGoalSet) {
  MarsExploration explorer;
  const auto &goal = explorer.getGoal();
  EXPECT_DOUBLE_EQ(goal.x, 0.0);
  EXPECT_DOUBLE_EQ(goal.y, 0.0);
  EXPECT_DOUBLE_EQ(goal.yaw, 0.0);
}

// Test edge cases for min_frontier_distance_m_
TEST(MarsExplorationTest, HighMinFrontierDistance) {
  MarsExploration explorer;
  auto grid = makeGrid(3, 3, 0);
  grid.data[4] = -1;  // center unknown
  explorer.setMap(grid);
  Pose2D pose{0.0, 0.0, 0.0};
  explorer.updatePose(pose);
  explorer.setMinFrontierDistance(100.0);  // very high
  EXPECT_FALSE(explorer.setNearestUnmappedCellAsGoal(0.0));
  EXPECT_FALSE(explorer.hasGoal());
}

// Test behavior with all cells occupied
TEST(MarsExplorationTest, AllOccupiedNoGoal) {
  MarsExploration explorer;
  auto grid = makeGrid(3, 3, 100);  // all occupied
  explorer.setMap(grid);
  Pose2D pose{1.0, 1.0, 0.0};
  explorer.updatePose(pose);
  EXPECT_FALSE(explorer.setNearestUnmappedCellAsGoal(0.0));
  EXPECT_FALSE(explorer.hasGoal());
}

// Test toggling include_low_confidence_cells_
// Note: No setter for include_low_confidence_cells_, so this is a demonstration
// If you add a setter, you can test with it set to false
// For now, test with a cell value above low_confidence_value_
TEST(MarsExplorationTest, LowConfidenceCellsIgnoredIfAboveThreshold) {
  MarsExploration explorer;
  auto grid = makeGrid(3, 3, 0);
  grid.data[1] = 25;  // above default low_confidence_value_
  explorer.setMap(grid);
  Pose2D pose{0.0, 0.0, 0.0};
  explorer.updatePose(pose);
  explorer.setMinFrontierDistance(0.0);
  EXPECT_FALSE(explorer.setNearestUnmappedCellAsGoal(0.0));
}

// Test repeated calls to setNearestUnmappedCellAsGoal after clearing the goal
TEST(MarsExplorationTest, SetGoalAfterClearGoal) {
  MarsExploration explorer;
  explorer.setBorderMarginCells(0);
  auto grid = makeGrid(3, 3, 0);
  grid.data[4] = -1;  // center unknown
  grid.data[0] = 0;   // ensure a free cell adjacent to unknown
  explorer.setMap(grid);
  Pose2D pose{0.0, 0.0, 0.0};
  explorer.updatePose(pose);
  explorer.setMinFrontierDistance(0.0);
  EXPECT_TRUE(explorer.setNearestUnmappedCellAsGoal(0.0));
  explorer.clearGoal();
  EXPECT_FALSE(explorer.hasGoal());
  // Reset map to ensure a new frontier is available
  grid.data[8] = -1;  // add another unknown cell
  grid.data[7] = 0;   // ensure a free cell adjacent to new unknown
  explorer.setMap(grid);
  EXPECT_TRUE(explorer.setNearestUnmappedCellAsGoal(0.0));
  EXPECT_TRUE(explorer.hasGoal());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
