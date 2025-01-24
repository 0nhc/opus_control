#include <chrono>
#include <memory>

// ROS 2
#include "rclcpp/rclcpp.hpp"
#include "can_msgs/msg/frame.hpp"

// TF2 and Geometry
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>

// Include your RS03 header
#include "motor_protocols/rs03.h"

using namespace std::chrono_literals;

class RS03Tester : public rclcpp::Node
{
public:
  RS03Tester()
  : Node("rs03_tester")
  {
    // Create a publisher for can_msgs::msg::Frame
    publisher_ = this->create_publisher<can_msgs::msg::Frame>("to_can_bus", 10);

    // Create a timer to call timerCallback periodically
    timer_ = this->create_wall_timer(10ms, std::bind(&RS03Tester::timerCallback, this));

    // Create a subscriber to receive can_msgs::msg::Frame
    subscriber_ = this->create_subscription<can_msgs::msg::Frame>(
      "from_can_bus", 10, std::bind(&RS03Tester::frameCallback, this, std::placeholders::_1));

    // Create a TF broadcaster to publish joint position from frame /motor to frame /output_shaft
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  }

  RS03 rs03;

private:
  void timerCallback()
  {
    // Example: send enable command
    auto frame = rs03.encodeEnableCommand(0x01);
    RCLCPP_INFO(this->get_logger(), "Publishing CAN frame with ID: %u", frame.id);
    publisher_->publish(frame);

    // Example: send torque command
    frame = rs03.encodeTorqueCommand(0x01, 0.45);
    RCLCPP_INFO(this->get_logger(), "Publishing CAN frame with ID: %u", frame.id);
    publisher_->publish(frame);

    // If we have received at least one frame, decode the position and broadcast a TF
    if(init_)
    {
      float position = -rs03.decodePositionFeedback(received_frame_);
      RCLCPP_INFO(this->get_logger(), "Received CAN frame with ID: %u, Position: %f", 
                  received_frame_.id, position);

      // Prepare and send the transform
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = this->now();
      t.header.frame_id = "motor";
      t.child_frame_id = "output_shaft";

      // Set translation (if needed, modify for actual offsets)
      t.transform.translation.x = 0.0;
      t.transform.translation.y = 0.0;
      t.transform.translation.z = 0.25;

      // Convert the position to a rotation about Z (assuming position is in radians)
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, position);  // Roll=0, Pitch=0, Yaw=position
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      // Broadcast the transform
      tf_broadcaster_->sendTransform(t);
    }
  }

  void frameCallback(const can_msgs::msg::Frame& msg)
  {
    received_frame_ = msg;
    // Once the first frame is received, mark init_ = true
    if(!init_) {
      init_ = true;
    }
  }

  // Publishers and subscribers
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr subscriber_;

  // TF broadcaster
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Storage for the last received frame and a flag to indicate initialization
  can_msgs::msg::Frame received_frame_;
  bool init_ = false;
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
