// test/test_slam_map_topic.cpp
#include <catch_ros2/catch_ros2.hpp>
#include <rclcpp/rclcpp.hpp>

TEST_CASE("slam node publishes /robot_0/map", "[integration][slam]") {
  auto node = std::make_shared<rclcpp::Node>("slam_map_test");

  // Give graph time to populate after bringup launches slam_toolbox
  using namespace std::chrono_literals;
  rclcpp::sleep_for(2s);

  auto topics = node->get_topic_names_and_types();
  bool found = false;
  for (const auto& t : topics) {
    if (t.first == "/robot_0/map") {
      found = true;
      break;
    }
  }

  REQUIRE(found);  // This will intentionally FAIL until slam nodes are wired up
}
