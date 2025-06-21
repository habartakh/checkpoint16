#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;

enum MotionType { // To be executed by the rosbot
  FORWARD,
  BACKWARD,
  LEFT,
  RIGHT,
  CLOCKWISE,
  CONTER_CLOCKWISE,
  STOP
};

// Overload the ++ operator for enum to increment the motion types
// source: https://cplusplus.com/forum/beginner/41790/
inline MotionType &operator++(MotionType &eDOW, int) {
  const int i = static_cast<int>(eDOW);
  eDOW = static_cast<MotionType>((i + 1) % 7);
  return eDOW;
}

class WheelVelocityPub : public rclcpp::Node {
public:
  WheelVelocityPub() : Node("wheel_velocity_pub") {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&WheelVelocityPub::timer_callback, this));

    // Initialize the motion type
    motion_type = FORWARD;

    RCLCPP_INFO(this->get_logger(),
                "Initialized wheel velocities publisher node");
  }

private:
  // The publisher callback is being calles every 0.5 seconds
  // We want to publish each motion message for 3 seconds
  // So we publish each motion 3 / 0.5 = 6 times inside the callback
  void timer_callback() {

    set_wheel_speeds();

    if (current_motion_index < 7) {
      // Publish each wheel vel message for  3 seconds
      if (same_msg_index < 6) {

        publisher_->publish(wheel_speed_msg);
        same_msg_index++;
      } else {
        // Reset the same_msg_index for the following motion
        same_msg_index = 0;
        // Then update the motion index to execute the next one
        ++current_motion_index;
        motion_type++;
      }
    } else {
      timer_->cancel();
    }
  }

  // Set the correct wheel speends depending on the motion wanted
  void set_wheel_speeds() {

    switch (motion_type) {

    case FORWARD:
      wheel_speed_msg.data = {1, 1, 1, 1};
      RCLCPP_INFO_ONCE(this->get_logger(), "Move Forward");
      break;

    case BACKWARD:
      wheel_speed_msg.data = {-1, -1, -1, -1};
      RCLCPP_INFO_ONCE(this->get_logger(), "Move Backward");
      break;

    case LEFT:
      wheel_speed_msg.data = {-1, 1, -1, 1};
      RCLCPP_INFO_ONCE(this->get_logger(), "Move left");
      break;

    case RIGHT:
      wheel_speed_msg.data = {1, -1, 1, -1};
      RCLCPP_INFO_ONCE(this->get_logger(), "Move right");
      break;

    case CLOCKWISE:
      wheel_speed_msg.data = {1, -1, -1, 1};
      RCLCPP_INFO_ONCE(this->get_logger(), "Turn clockwise");
      break;

    case CONTER_CLOCKWISE:
      wheel_speed_msg.data = {-1, 1, 1, -1};
      RCLCPP_INFO_ONCE(this->get_logger(), "Turn counter-clockwise");
      break;

    case STOP:
      wheel_speed_msg.data = {0, 0, 0, 0};
      RCLCPP_INFO_ONCE(this->get_logger(), "Stop");
      break;
    }
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;

  enum MotionType motion_type;
  std_msgs::msg::Float32MultiArray wheel_speed_msg;
  // Each motion's index is between 0 & 7
  int current_motion_index = 0;
  int same_msg_index = 0;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocityPub>());
  rclcpp::shutdown();
  return 0;
}