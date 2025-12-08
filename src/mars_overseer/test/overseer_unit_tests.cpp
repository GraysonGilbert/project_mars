/**
 * @file overseer_unit_tests.hpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @author Marcus Hurt (mhurt@umd.edu)
 * @brief Unittest cases for map_merging functionality
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
#include "mars_overseer/map_merger.hpp"


// ===================== TEST CASES: =====================

/*
TEST 1: FRESH MAP RETURNS TRUE
- This test creates a map with a timestamp within the accepted range
- The expected result is true
*/

TEST(MapRecencyTests, FreshMapReturnsTrue) {
    MapMerger merger(5.0);  // max_map_age = 5 seconds

    nav_msgs::msg::OccupancyGrid map;
    map.header.stamp.sec = 100;      // map timestamp
    map.header.stamp.nanosec = 0;

    rclcpp::Time current_time(102, 0, RCL_ROS_TIME); // current time = 102 sec

    EXPECT_TRUE(merger.is_map_recent(map, current_time));
}

/*
TEST 2: STALE MAP RETURNS FALSE
- This test creates a map with a timestam outside the accepted range
- The expected result is false
*/
TEST(MapRecencyTests, StaleMapReturnsFalse) {
    MapMerger merger(5.0);  // max_map_age = 5 seconds

    nav_msgs::msg::OccupancyGrid map;
    map.header.stamp.sec = 90;      // map timestamp
    map.header.stamp.nanosec = 0;

    rclcpp::Time current_time(102, 0, RCL_ROS_TIME); // current time = 102 sec

    EXPECT_FALSE(merger.is_map_recent(map, current_time));
}


/*
TEST 3: MERGE TWO MAPS WITH 'OCCUPIED-WINS' MERGE BEHAVIOR
- This test has a global map occupancy grid filled with free spaces (0)
- A local map of the same size is created filled with occupied spaces (100)
- The resulting global_map.data should show that all the spaces are now occupied (100)
*/
TEST(MergeMapsTests, OccupiedWins) {

    MapMerger merger;

    // Create global map (all free)
    nav_msgs::msg::OccupancyGrid global_map;
    global_map.info.width = 5;
    global_map.info.height = 5;
    global_map.info.resolution = 0.05;
    global_map.data.assign(global_map.info.width * global_map.info.height, 0); // free

    // Create local map (all occupied)
    nav_msgs::msg::OccupancyGrid local_map;
    local_map.info = global_map.info;
    local_map.data.assign(global_map.info.width * global_map.info.height, 100); // occupied

    // Call merge_maps
    merger.merge_maps(global_map, {local_map});

    // Check that every cell is occupied
    for (const auto& cell : global_map.data) {
        EXPECT_EQ(cell, 80);
    }
}


/*
TEST 4: MERGE WITH UNKNOWN CELLS
- This has a global map with all unknown cells (-1)
- A local map is created with a combination of free (0), occupied (100), and unknown (-1) cells
- The resulting global_map.data should contain the free and occupied spaces from the local data
  while the unknown spaces remain unchanged
*/ 
TEST(MergeMapsTests, UnknownCells) {

    MapMerger merger;

    nav_msgs::msg::OccupancyGrid global_map;
    global_map.info.width = 5;
    global_map.info.height = 1;
    global_map.info.resolution = 0.05;
    global_map.data.assign(5, -1); // unknown

    nav_msgs::msg::OccupancyGrid local_map;
    local_map.info = global_map.info;
    local_map.data = {0, 100, -1, 0, 100};

    merger.merge_maps(global_map, {local_map});

    std::vector<int8_t> expected = {0, 100, -1, 0, 100};
    EXPECT_EQ(global_map.data, expected);
}

/*
TEST 5: MERGE MULTIPLE LOCAL MAPS
- Creates a global_map and two local maps of the same size
- Creates a vector of the two local maps and merges them with the global_map
- The resulting global_map.data should show occupied spaces from both local maps
*/

TEST(MergeMapsTests, MultipleMapsMerge) {

    MapMerger merger;

    nav_msgs::msg::OccupancyGrid global_map;
    global_map.info.width = 3;
    global_map.info.height = 1;
    global_map.info.resolution = 0.05;
    global_map.data.assign(3, 0); // free

    nav_msgs::msg::OccupancyGrid map1;
    map1.info = global_map.info;
    map1.data = {0, 100, 0};

    nav_msgs::msg::OccupancyGrid map2;
    map2.info = global_map.info;
    map2.data = {100, 0, 0};

    std::vector<nav_msgs::msg::OccupancyGrid> local_maps;
    local_maps.push_back(map1);
    local_maps.push_back(map2);

    merger.merge_maps(global_map, local_maps);

    std::vector<int8_t> expected = {80, 16, 0};
    EXPECT_EQ(global_map.data, expected);
}

/*
TEST 6: LOCAL MAP SMALLER THAN GLOBAL MAP
- This test creates a global_map of size 5x1 and local_map of size 3x1
- The test applies the correct x offset to shift th elocal map into the center
  of the global map and merges
- The resulting map should show the shifted local map merged into the global_map
*/

TEST(MergeMapsTests, LocalMapSmallerThanGlobal) {

    MapMerger merger;

    // Global map 5x1, all free
    nav_msgs::msg::OccupancyGrid global_map;
    global_map.info.width = 5;
    global_map.info.height = 1;
    global_map.info.resolution = 0.05;
    global_map.data = {0,0,0,0,0};

    // Local map 3x1 with offset index 1
    nav_msgs::msg::OccupancyGrid local_map;
    local_map.info.width = 3;
    local_map.info.height = 1;
    local_map.info.resolution = 0.05;
    local_map.data = {100,0,-1};
    local_map.info.origin.position.x = 1 * local_map.info.resolution; // offset by 1 cell
    local_map.info.origin.position.y = 0;

    merger.merge_maps(global_map, {local_map});

    std::vector<int8_t> expected = {0,80,0,0,0};
    EXPECT_EQ(global_map.data, expected);
}

/*
TEST 7: MERGE MAPS WITH MATCHING RESOLUTIONS
- This test attempts to merge two maps with matching occupancy grid resolutions
- The expected result is no exception is thrown
*/
TEST(MapMergerTest, MergeMaps_ResolutionsMustMatch) {

    MapMerger merger;
  
    nav_msgs::msg::OccupancyGrid global_map;
    global_map.info.resolution = 0.05;

    nav_msgs::msg::OccupancyGrid local_map;
    local_map.info.resolution = 0.05000001;  // very close

    // Should pass because difference < epsilon
    EXPECT_NO_THROW(merger.merge_maps(global_map, {local_map}));
}

/*
TEST 8: MERGE MAPS WITH NON-MATCHING RESOLUTIONS
- This test attempts to merge two maps with dissimilar occupancy grid resolutions
- The exepcted result is that an exception is thrown
*/
TEST(MapMergerTest, MergeMaps_ResolutionsDoesntMatch) {

    MapMerger merger;
  
    nav_msgs::msg::OccupancyGrid global_map;
    global_map.info.resolution = 0.05;

    nav_msgs::msg::OccupancyGrid local_map2;
    local_map2.info.resolution = 0.051;  // too far

    EXPECT_THROW(merger.merge_maps(global_map, {local_map2}), std::runtime_error);
}


int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
