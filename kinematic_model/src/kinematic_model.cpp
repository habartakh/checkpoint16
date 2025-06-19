#include <memory>

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include <Eigen/Dense>

using std::placeholders::_1;
using namespace std::chrono_literals;

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel() : Node("kinematic_model") {
    wheel_vel_sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10,
        std::bind(&KinematicModel::wheel_vel_sub_callback, this, _1));

    twist_pub =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    twist_timer = this->create_wall_timer(
        500ms, std::bind(&KinematicModel::twist_timer_callback, this));
  }

private:
  void wheel_vel_sub_callback(
      const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    current_wheel_velocities = msg->data;
  }

  void twist_timer_callback() {
    // Publish the Twist message to move the robot
    wheelvel_to_twist();
    
    // std::cout << "current_twist.angular.z : " << current_twist.angular.z
    //           << std::endl;

    // std::cout << " current_twist.linear.x : " << current_twist.linear.x
    //           << std::endl;

    // std::cout << " current_twist.linear.y :" << current_twist.linear.y
    //           << std::endl;

    twist_pub->publish(current_twist);
  }

  // Implement the kinematic model of an holonomic robot
  // This function computes the twist values from the given wheel speeds
  void wheelvel_to_twist() {
    Eigen::MatrixXd H(4, 3);
    H << -l - w, 1, -1, l + w, 1, 1, l + w, 1, -1, -l - w, 1, 1;

    Eigen::MatrixXd H_inv = H.completeOrthogonalDecomposition().pseudoInverse();

    Eigen::MatrixXd u(4, 1);
    u << current_wheel_velocities[0], current_wheel_velocities[1],
        current_wheel_velocities[2], current_wheel_velocities[3];

    Eigen::MatrixXd twist(3, 1);
    twist = H_inv * u;
    twist = r * twist;

    current_twist.angular.z = twist(0, 0);
    current_twist.linear.x = twist(1, 0);
    current_twist.linear.y = twist(2, 0);
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      wheel_vel_sub;
  rclcpp::TimerBase::SharedPtr twist_timer;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

  std::vector<float> current_wheel_velocities;
  geometry_msgs::msg::Twist current_twist;

  double l = 0.170 / 2;   // half of the wheel base distance
  double r = 0.100 / 2;   // the radius of the wheels
  double w = 0.26969 / 2; // half of track width
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicModel>());
  rclcpp::shutdown();
  return 0;
}