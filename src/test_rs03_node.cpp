#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"

// Include your RS03 header
#include "motor_protocols/rs03.h"

using namespace std::chrono_literals;

class RS03Tester : public rclcpp::Node
{
public:
  RS03Tester()
  : Node("rs03_tester"), timer_count_(0)
  {
    // Create a publisher for can_msgs::msg::Frame
    publisher_ = this->create_publisher<can_msgs::msg::Frame>("to_can_bus", 10);

    // Create a timer to call timerCallback periodically
    timer_ = this->create_wall_timer(500ms, std::bind(&RS03Tester::timerCallback, this));
  }

private:
  void timerCallback()
  {
    // Create an instance of RS03
    RS03 rs03;

    // Call encodeTorqueCommand with some example motor_id and 0.3 N.m torque
    auto frame = rs03.encodeTorqueCommand(0x01 /* motor_id */, 0.3 /* torque */);

    // Log for demonstration
    RCLCPP_INFO(this->get_logger(), "Publishing CAN frame with ID: %u", frame.id);

    // Publish the frame
    publisher_->publish(frame);

    // Just an example: stop after 10 publishes
    timer_count_++;
    // if (timer_count_ >= 10) {
    //   RCLCPP_INFO(this->get_logger(), "Test completed, shutting down.");
    //   rclcpp::shutdown();
    // }
  }

  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  int timer_count_;
};

int main(int argc, char** argv)
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Spin the node
  rclcpp::spin(std::make_shared<RS03Tester>());

  // Shutdown and exit
  rclcpp::shutdown();
  return 0;
}
