/**
 * @file mars_exploration_node_tests.cpp
 * @author Marcus Hurt (mhurt@umd.edu)
 * @author Grayson Guilbert (ggilbert@umd.edu)
 * @brief Integration-style tests for MarsExplorationNode using catch_ros2.
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

#include <tf2_ros/static_transform_broadcaster.h>

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <thread>

#include "mars_exploration/mars_exploration_node.hpp"

using String = std_msgs::msg::String;
using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;
using Catch::Approx;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using OccupancyGrid = nav_msgs::msg::OccupancyGrid;

class MarsExplorationNodeTestFixture {
 public:
  MarsExplorationNodeTestFixture() {
    // Node used by the tests themselves (parameter source, extra subscriptions,
    // etc.).
    test_node_ =
        rclcpp::Node::make_shared("mars_exploration_node_test_fixture");

    // Allow the launch file to parameterize how long the test is allowed to
    // run. If the parameter is not set, fall back to a small default.
    test_node_->declare_parameter<double>("test_duration", 2.0);
    test_duration_ = test_node_->get_parameter("test_duration")
                         .get_parameter_value()
                         .get<double>();
  }

  std::shared_ptr<rclcpp::Node> test_node_;
  double test_duration_;
};

TEST_CASE_METHOD(MarsExplorationNodeTestFixture,
                 "mars_exploration_node startup", "[integration][startup]") {
  auto node = std::make_shared<MarsExplorationNode>();

  REQUIRE(node != nullptr);

  // The node should declare its core configuration parameters.
  CHECK(node->has_parameter("goal_topic"));
  CHECK(node->has_parameter("cmd_vel_topic"));
  CHECK(node->has_parameter("global_frame"));
  CHECK(node->has_parameter("base_frame"));
  CHECK(node->has_parameter("control_rate_hz"));
}

TEST_CASE_METHOD(MarsExplorationNodeTestFixture,
                 "mars_exploration_node supports subscriptions and publishers",
                 "[integration][ros2_api]") {
  auto node = std::make_shared<MarsExplorationNode>();

  // Create a pub/sub pair that goes through the rclcpp graph.
  auto pub = node->create_publisher<String>("test_topic", rclcpp::QoS(10));

  auto received = std::make_shared<bool>(false);

  auto sub = test_node_->create_subscription<String>(
      "test_topic", rclcpp::QoS(10),
      [received](String::ConstSharedPtr /*msg*/) { *received = true; });

  (void)sub;

  REQUIRE(pub != nullptr);

  // Wire both nodes into a small executor and exercise a single publish.
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.add_node(test_node_);

  String msg;
  msg.data = "hello from mars_exploration_node_tests";
  pub->publish(msg);

  auto timeout =
      std::chrono::milliseconds(static_cast<int>(test_duration_ * 1000.0));

  auto start = std::chrono::steady_clock::now();
  while (!*received && (std::chrono::steady_clock::now() - start) < timeout) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  CHECK(*received);
}

TEST_CASE("mars_exploration_node default parameters", "[params][defaults]") {
  auto node = std::make_shared<MarsExplorationNode>();

  std::string goal_topic;
  REQUIRE(node->get_parameter("goal_topic", goal_topic));
  CHECK(goal_topic == "/goal_pose");

  std::string cmd_vel_topic;
  REQUIRE(node->get_parameter("cmd_vel_topic", cmd_vel_topic));
  CHECK(cmd_vel_topic == "/cmd_vel");

  std::string global_frame;
  REQUIRE(node->get_parameter("global_frame", global_frame));
  CHECK(global_frame == "map");

  std::string base_frame;
  REQUIRE(node->get_parameter("base_frame", base_frame));
  CHECK(base_frame == "base_link");

  double control_rate_hz{};
  REQUIRE(node->get_parameter("control_rate_hz", control_rate_hz));
  CHECK(control_rate_hz == Approx(20.0));
}

TEST_CASE("mars_exploration_node respects parameter overrides",
          "[params][overrides]") {
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      rclcpp::Parameter("goal_topic", "/alt_goal"),
      rclcpp::Parameter("cmd_vel_topic", "/alt_cmd_vel"),
      rclcpp::Parameter("global_frame", "odom"),
      rclcpp::Parameter("base_frame", "base_footprint"),
      rclcpp::Parameter("control_rate_hz", 5.0),
  });

  auto node = std::make_shared<MarsExplorationNode>(opts);

  std::string goal_topic;
  REQUIRE(node->get_parameter("goal_topic", goal_topic));
  CHECK(goal_topic == "/alt_goal");

  std::string global_frame;
  REQUIRE(node->get_parameter("global_frame", global_frame));
  CHECK(global_frame == "odom");

  double control_rate_hz{};
  REQUIRE(node->get_parameter("control_rate_hz", control_rate_hz));
  CHECK(control_rate_hz == Approx(5.0));
}

struct NavigateToPoseTestFixture {
  NavigateToPoseTestFixture() {
    test_node = rclcpp::Node::make_shared("navigate_to_pose_test_fixture");

    // Simple fake server that just captures the last goal pose.
    using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    auto handle_goal = [this](const rclcpp_action::GoalUUID &,
                              std::shared_ptr<const NavigateToPose::Goal> goal)
        -> rclcpp_action::GoalResponse {
      last_goal_pose = goal->pose;
      got_goal = true;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel = [](std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [](std::shared_ptr<GoalHandle>) {
      // No-op; we don't need to send a result for this test.
    };

    server = rclcpp_action::create_server<NavigateToPose>(
        test_node, "navigate_to_pose", handle_goal, handle_cancel,
        handle_accepted);
  }

  std::shared_ptr<rclcpp::Node> test_node;
  rclcpp_action::Server<NavigateToPose>::SharedPtr server;

  bool got_goal{false};
  NavigateToPose::Goal::_pose_type last_goal_pose;
};

TEST_CASE_METHOD(NavigateToPoseTestFixture,
                 "goal pose publishes NavigateToPose goal",
                 "[integration][nav2]") {
  auto mars_node = std::make_shared<MarsExplorationNode>();

  std::string goal_topic;
  REQUIRE(mars_node->get_parameter("goal_topic", goal_topic));

  auto goal_pub = test_node->create_publisher<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(mars_node);
  exec.add_node(test_node);

  // Build a simple goal pose
  geometry_msgs::msg::PoseStamped goal_msg;
  goal_msg.header.frame_id = "map";
  goal_msg.pose.position.x = 1.0;
  goal_msg.pose.position.y = 2.0;
  goal_msg.pose.orientation.w = 1.0;

  // Let the graph settle a bit
  auto start = std::chrono::steady_clock::now();
  auto timeout = 5s;

  while ((std::chrono::steady_clock::now() - start) < 500ms) {
    exec.spin_some();
  }

  // Publish and spin until the fake server sees a goal
  goal_pub->publish(goal_msg);

  start = std::chrono::steady_clock::now();
  while (!got_goal && (std::chrono::steady_clock::now() - start) < timeout) {
    exec.spin_some();
    std::this_thread::sleep_for(10ms);
  }

  REQUIRE(got_goal);
  CHECK(last_goal_pose.pose.position.x == Approx(1.0));
  CHECK(last_goal_pose.pose.position.y == Approx(2.0));
  CHECK(last_goal_pose.header.frame_id == "map");
}

class MarsExplorationNavFixture {
 public:
  MarsExplorationNavFixture()
      : test_node_(rclcpp::Node::make_shared("mars_exploration_nav_fixture")),
        tf_broadcaster_(test_node_),
        got_goal_(false),
        goal_count_(0) {
    using GoalHandle = rclcpp_action::ServerGoalHandle<NavigateToPose>;

    auto handle_goal = [this](const rclcpp_action::GoalUUID &,
                              std::shared_ptr<const NavigateToPose::Goal> goal)
        -> rclcpp_action::GoalResponse {
      last_goal_pose_ = goal->pose;
      got_goal_ = true;
      goal_count_++;
      return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel = [](std::shared_ptr<GoalHandle>) {
      return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [](std::shared_ptr<GoalHandle>) {
      // No-op for these tests
    };

    nav_server_ = rclcpp_action::create_server<NavigateToPose>(
        test_node_, "navigate_to_pose", handle_goal, handle_cancel,
        handle_accepted);
  }

  // Broadcast a simple static transform map -> base_link
  void publish_static_tf() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = test_node_->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";
    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;

    tf_broadcaster_.sendTransform(t);
  }

  // Create a map publisher on the given topic (or reuse if already created)
  void ensure_map_publisher(const std::string &map_topic) {
    if (!map_pub_) {
      map_pub_ = test_node_->create_publisher<OccupancyGrid>(map_topic, 1);
    }
  }

  // Publish a very simple map once
  void publish_simple_map() {
    if (!map_pub_) {
      // Default to /map if nothing was configured yet
      map_pub_ = test_node_->create_publisher<OccupancyGrid>("/map", 1);
    }

    OccupancyGrid grid;
    grid.header.stamp = test_node_->now();
    grid.header.frame_id = "map";
    grid.info.resolution = 1.0;
    grid.info.width = 10;
    grid.info.height = 10;
    grid.info.origin.position.x = 0.0;
    grid.info.origin.position.y = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.assign(grid.info.width * grid.info.height, 0);

    map_pub_->publish(grid);
  }

  // Spin the executor for a duration with small sleeps
  void spin_for(rclcpp::executors::SingleThreadedExecutor &exec,
                std::chrono::nanoseconds duration) {
    auto start = std::chrono::steady_clock::now();
    while ((std::chrono::steady_clock::now() - start) < duration) {
      exec.spin_some();
      std::this_thread::sleep_for(10ms);
    }
  }

  std::shared_ptr<rclcpp::Node> test_node_;

  rclcpp_action::Server<NavigateToPose>::SharedPtr nav_server_;
  tf2_ros::StaticTransformBroadcaster tf_broadcaster_;

  rclcpp::Publisher<OccupancyGrid>::SharedPtr map_pub_;

  bool got_goal_{false};
  int goal_count_{0};
  NavigateToPose::Goal::_pose_type last_goal_pose_;
};

TEST_CASE_METHOD(MarsExplorationNavFixture,
                 "manual goal is sent even without TF",
                 "[integration][nav2][manual_goal]") {
  auto mars_node = std::make_shared<MarsExplorationNode>();

  std::string goal_topic = "/goal_pose";
  (void)mars_node->get_parameter("goal_topic", goal_topic);

  std::string map_topic = "/map";
  (void)mars_node->get_parameter("map_topic", map_topic);

  ensure_map_publisher(map_topic);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(mars_node);
  exec.add_node(test_node_);

  spin_for(exec, 500ms);

  publish_simple_map();
  spin_for(exec, 500ms);

  auto goal_pub = test_node_->create_publisher<PoseStamped>(goal_topic, 10);

  PoseStamped goal_msg;
  goal_msg.header.frame_id = "map";
  goal_msg.pose.position.x = 1.0;
  goal_msg.pose.position.y = 2.0;
  goal_msg.pose.orientation.w = 1.0;

  goal_pub->publish(goal_msg);

  spin_for(exec, 3s);

  CHECK(got_goal_);
}

TEST_CASE_METHOD(MarsExplorationNavFixture,
                 "with TF and map, NavigateToPose goal is sent",
                 "[integration][tf][map][nav2]") {
  auto mars_node = std::make_shared<MarsExplorationNode>();

  std::string goal_topic = "/goal_pose";
  (void)mars_node->get_parameter("goal_topic", goal_topic);

  std::string map_topic = "/map";
  (void)mars_node->get_parameter("map_topic", map_topic);

  ensure_map_publisher(map_topic);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(mars_node);
  exec.add_node(test_node_);

  // Broadcast the needed static transform
  publish_static_tf();

  // Let TF + subscriptions settle
  spin_for(exec, 500ms);

  // Publish a simple map
  publish_simple_map();
  spin_for(exec, 500ms);

  // Goal publisher
  auto goal_pub = test_node_->create_publisher<PoseStamped>(goal_topic, 10);

  PoseStamped goal_msg;
  goal_msg.header.frame_id = "map";
  goal_msg.pose.position.x = 3.0;
  goal_msg.pose.position.y = 4.0;
  goal_msg.pose.orientation.w = 1.0;

  goal_pub->publish(goal_msg);

  // Wait for the NavigateToPose goal to show up on our fake server
  spin_for(exec, 5s);

  REQUIRE(got_goal_);
  CHECK(last_goal_pose_.pose.position.x == Approx(3.0));
  CHECK(last_goal_pose_.pose.position.y == Approx(4.0));
  CHECK(last_goal_pose_.header.frame_id == "map");
}

TEST_CASE_METHOD(
    MarsExplorationNavFixture,
    "max_goal_duration_sec does not re-send manual goals automatically",
    "[integration][timing][nav2]") {
  // Configure a small max_goal_duration_sec so the test runs quickly.
  rclcpp::NodeOptions opts;
  opts.parameter_overrides({
      rclcpp::Parameter("max_goal_duration_sec", 1.0),
  });

  auto mars_node = std::make_shared<MarsExplorationNode>(opts);

  std::string goal_topic = "/goal_pose";
  (void)mars_node->get_parameter("goal_topic", goal_topic);

  std::string map_topic = "/map";
  (void)mars_node->get_parameter("map_topic", map_topic);

  ensure_map_publisher(map_topic);

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(mars_node);
  exec.add_node(test_node_);

  // TF + map available so the node is fully "live"
  publish_static_tf();
  spin_for(exec, 500ms);

  publish_simple_map();
  spin_for(exec, 500ms);

  auto goal_pub = test_node_->create_publisher<PoseStamped>(goal_topic, 10);

  PoseStamped goal_msg;
  goal_msg.header.frame_id = "map";
  goal_msg.pose.position.x = 0.0;
  goal_msg.pose.position.y = 0.0;
  goal_msg.pose.orientation.w = 1.0;

  // Send an initial manual goal
  goal_pub->publish(goal_msg);

  // Wait a bit: we should see a single NavigateToPose goal quickly.
  spin_for(exec, 2s);
  REQUIRE(goal_count_ >= 1);
  int previous_goal_count = goal_count_;

  // Now wait well beyond max_goal_duration_sec.
  // We expect the node to time out / cancel internally, but NOT to
  // re-send additional goals on its own.
  spin_for(exec, 4s);

  CHECK(goal_count_ == previous_goal_count);
}
