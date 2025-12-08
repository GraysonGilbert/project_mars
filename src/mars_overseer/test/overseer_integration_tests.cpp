/**
 * @file overseer_integration_tests.cpp
 * @author Grayson Gilbert (ggilbert@umd.edu)
 * @author Marcus Hurt (mhurt@umd.edu)
 * @brief Integration test cases for MarsOverseerNode functionality
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

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::String;

////////////////////////////////////////////////
// Define Fixture
////////////////////////////////////////////////
auto Logger = rclcpp::get_logger("");  // create an initial Logger

class MyTestsFixture {
 public:
  MyTestsFixture() {
    // Create the node that performs the test. (aka Integration test node):
    overseerTesterNode =
        rclcpp::Node::make_shared("OverseerIntegrationTestNode");
    Logger =
        overseerTesterNode
            ->get_logger();  // make sure message will appear in rqt_console

    // Declare a parameter for the duration of the test:
    overseerTesterNode->declare_parameter<double>("test_duration");

    // Get the test duration value:
    TEST_DURATION = overseerTesterNode->get_parameter("test_duration")
                        .get_parameter_value()
                        .get<double>();
    RCLCPP_INFO_STREAM(Logger, "Got test_duration =" << TEST_DURATION);
  }

  ~MyTestsFixture() {}

 protected:
  double TEST_DURATION;
  rclcpp::Node::SharedPtr overseerTesterNode;
};

////////////////////////////////////////////////
// Test Case 1
////////////////////////////////////////////////

/*
   In this test case, the node under test (aka Auxiliary test node) is an
   individual robot explorer, which got launched by the launcher.

   We will create a nav_msgs listener that subscribe to robot's /map
   topic within its namespace. This test checks that topic is publishing a
   nav_msgs/msg/OccupancyGrid message.
   */

TEST_CASE_METHOD(MyTestsFixture, "Test slam_tool map subscription robot 1",
                 "[map 1]") {
  bool got_map = false;

  // Create callback looking for nav_msgs/msg/OccupancyGrid messages
  auto map_callback =
      [&got_map](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(Logger, "Received map: [%u x %u], res=%.3f",
                    msg->info.width, msg->info.height, msg->info.resolution);
        got_map = true;
      };

  auto qos = rclcpp::QoS(10).transient_local().reliable();
  auto subscriber_ =
      overseerTesterNode->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/robot_1/map",  // added robot_1 namespace to topic
          qos, map_callback);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(overseerTesterNode);

  // Use ROS time
  auto start_time = overseerTesterNode->now();
  auto timeout = rclcpp::Duration::from_seconds(TEST_DURATION);

  rclcpp::Rate rate(10.0);  // 10 Hz

  while (!got_map && (overseerTesterNode->now() - start_time) < timeout) {
    exec.spin_some();  // Process callbacks
    rate.sleep();
  }

  RCLCPP_INFO_STREAM(
      Logger,
      "duration = " << (overseerTesterNode->now() - start_time).seconds()
                    << " got_map=" << got_map);

  CHECK(got_map);  // Assert that we received at least one map
}

////////////////////////////////////////////////
// Test Case 2
////////////////////////////////////////////////

/*

*/
TEST_CASE_METHOD(MyTestsFixture, "Test slam_tool map subscription robot 2",
                 "[map 2]") {
  bool got_map = false;

  // Create callback looking for nav_msgs/msg/OccupancyGrid messages
  auto map_callback =
      [&got_map](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(Logger, "Received map: [%u x %u], res=%.3f",
                    msg->info.width, msg->info.height, msg->info.resolution);
        got_map = true;
      };

  auto qos = rclcpp::QoS(10).transient_local().reliable();
  auto subscriber_ =
      overseerTesterNode->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/robot_2/map",  // added robot_2 namespace to topic
          qos, map_callback);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(overseerTesterNode);

  // Use ROS time
  auto start_time = overseerTesterNode->now();
  auto timeout = rclcpp::Duration::from_seconds(TEST_DURATION);

  rclcpp::Rate rate(10.0);  // 10 Hz

  while (!got_map && (overseerTesterNode->now() - start_time) < timeout) {
    exec.spin_some();  // Process callbacks
    rate.sleep();
  }

  RCLCPP_INFO_STREAM(
      Logger,
      "duration = " << (overseerTesterNode->now() - start_time).seconds()
                    << " got_map=" << got_map);

  CHECK(got_map);  // Assert that we received at least one map
}

////////////////////////////////////////////////
// Test Case 3
////////////////////////////////////////////////

/*

*/

TEST_CASE_METHOD(MyTestsFixture, "Test global map production", "[global_map]") {
  bool got_global_map = false;

  // Create callback looking for nav_msgs/msg/OccupancyGrid messages
  auto map_callback =
      [&got_global_map](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        RCLCPP_INFO(Logger, "Received map: [%u x %u], res=%.3f",
                    msg->info.width, msg->info.height, msg->info.resolution);
        got_global_map = true;
      };

  auto qos = rclcpp::QoS(10).transient_local().reliable();
  auto subscriber_ =
      overseerTesterNode->create_subscription<nav_msgs::msg::OccupancyGrid>(
          "/global_map", qos, map_callback);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(overseerTesterNode);

  // Use ROS time
  auto start_time = overseerTesterNode->now();
  auto timeout = rclcpp::Duration::from_seconds(TEST_DURATION);

  rclcpp::Rate rate(10.0);  // 10 Hz

  while (!got_global_map &&
         (overseerTesterNode->now() - start_time) < timeout) {
    exec.spin_some();  // Process callbacks
    rate.sleep();
  }

  RCLCPP_INFO_STREAM(
      Logger,
      "duration = " << (overseerTesterNode->now() - start_time).seconds()
                    << " got_global_map =" << got_global_map);

  CHECK(got_global_map);  // Assert that we received at least one map
}
