/**
 * @file overseer_integration_tests.cpp
 * @author your name (you@domain.com)
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2025-11-29
 * 
 * @copyright Copyright (c) 2025
 * 
 */

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::String;


////////////////////////////////////////////////
// Define Fixture
////////////////////////////////////////////////
auto Logger = rclcpp::get_logger (""); // create an initial Logger

class MyTestsFixture {
public:
  MyTestsFixture () 
  {
    // Create the node that performs the test. (aka Integration test node):
    overseerTesterNode = rclcpp::Node::make_shared ("OverseerIntegrationTestNode");
    Logger = overseerTesterNode->get_logger(); // make sure message will appear in rqt_console

    // Declare a parameter for the duration of the test: 
    overseerTesterNode->declare_parameter<double> ("test_duration");

    // Get the test duration value:
    TEST_DURATION = overseerTesterNode->get_parameter("test_duration").get_parameter_value().get<double>();
    RCLCPP_INFO_STREAM (Logger, "Got test_duration =" << TEST_DURATION);
  }

  ~MyTestsFixture ()
  {
  }

protected:
  double                  TEST_DURATION;
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

TEST_CASE_METHOD (MyTestsFixture, "Test slam_tool map subscription", "[map]") {

  bool got_map = false;

  // Create callback looking for nav_msgs/msg/OccupancyGrid messages
  auto map_callback = 
    [&got_map](const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        RCLCPP_INFO(Logger, "Received map: [%u x %u], res=%.3f",
                    msg->info.width,
                    msg->info.height,
                    msg->info.resolution);
        got_map = true;
    };

  auto qos = rclcpp::QoS(10).transient_local().reliable();
  auto subscriber_ = overseerTesterNode->create_subscription<nav_msgs::msg::OccupancyGrid> (
    "/robot_1/map", // added robot_1 namespace to topic
    qos,
    map_callback);


  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(overseerTesterNode);

 // Use ROS time
  auto start_time = overseerTesterNode->now();
  auto timeout = rclcpp::Duration::from_seconds(TEST_DURATION);

  rclcpp::Rate rate(10.0);  // 10 Hz

  while (!got_map && (overseerTesterNode->now() - start_time) < timeout) {
    exec.spin_some();   // Process callbacks
    rate.sleep();
  }

  RCLCPP_INFO_STREAM(Logger,
                     "duration = "
                     << (overseerTesterNode->now() - start_time).seconds()
                     << " got_map=" << got_map);

  CHECK(got_map);   // Assert that we received at least one map
}
