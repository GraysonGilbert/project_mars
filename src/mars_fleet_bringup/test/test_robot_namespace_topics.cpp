#include <catch2/catch.hpp>
#include <rclcpp/rclcpp.hpp>

TEST_CASE("robot namespace must contain scan, odom, map topics",
          "[integration][robot_namespace]") {
  auto node = std::make_shared<rclcpp::Node>("robot_namespace_topics_test");

  // TODO in future: parameterize namespace, e.g. via node parameter or
  // environment
  const std::string ns = "/robot_0";

  // Give the graph some time to populate once bringup is running
  using namespace std::chrono_literals;
  rclcpp::sleep_for(2s);

  auto topics_and_types = node->get_topic_names_and_types();

  bool has_scan = false;
  bool has_odom = false;
  bool has_map = false;

  for (const auto& pair : topics_and_types) {
    const auto& topic_name = pair.first;
    if (topic_name == ns + "/scan") {
      has_scan = true;
    } else if (topic_name == ns + "/odom") {
      has_odom = true;
    } else if (topic_name == ns + "/map") {
      has_map = true;
    }
  }

  REQUIRE(has_scan);  // Expected /robot_0/scan topic
  REQUIRE(has_odom);  // Expected /robot_0/odom topic
  REQUIRE(has_map);   // Expected /robot_0/map topic
}
