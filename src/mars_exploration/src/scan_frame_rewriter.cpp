// mars_exploration/src/scan_frame_rewriter.cpp

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class ScanFrameRewriter : public rclcpp::Node {
 public:
  ScanFrameRewriter() : Node("scan_frame_rewriter") {
    // Declare and get parameters
    input_topic_ = declare_parameter<std::string>("input_topic", "scan");
    output_topic_ =
        declare_parameter<std::string>("output_topic", "scan_corrected");
    new_frame_id_ = declare_parameter<std::string>("new_frame_id", "LDS-01");

    RCLCPP_INFO(this->get_logger(),
                "ScanFrameRewriter starting with:\n"
                "  input_topic:  %s\n"
                "  output_topic: %s\n"
                "  new_frame_id: %s",
                input_topic_.c_str(), output_topic_.c_str(),
                new_frame_id_.c_str());

    // Sensor data QoS is appropriate for LaserScan
    auto qos = rclcpp::SensorDataQoS();

    sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        input_topic_, qos,
        std::bind(&ScanFrameRewriter::scan_callback, this,
                  std::placeholders::_1));

    pub_ =
        this->create_publisher<sensor_msgs::msg::LaserScan>(output_topic_, qos);
  }

 private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg_in) {
    // Copy the message and rewrite frame_id
    auto msg_out = *msg_in;
    msg_out.header.frame_id = new_frame_id_;
    pub_->publish(msg_out);
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string new_frame_id_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScanFrameRewriter>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
