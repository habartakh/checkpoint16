#include <cmath>
#include <memory>

#include "geometry_msgs/msg/detail/twist__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <Eigen/Dense>

using std::placeholders::_1;
using namespace std::chrono_literals;

struct WayPoint {
  double dphi; // change in the orientation angle of the robot
  double dx;   // change in the x-coordinate of the robot's position
  double dy;   // change in the y-coordinate of the robot's position

  WayPoint(double a, double b, double c) : dphi(a), dx(b), dy(c) {}
};

class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory() : Node("eight_trajectory") {

    // Initialize the waypoints making the trajectory drawn by the robot
    waypoints_traj_init();

    odom_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions options1;
    options1.callback_group = odom_callback_group_;

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odometry/filtered", 10,
        std::bind(&EightTrajectory::odom_sub_callback, this, _1), options1);

    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    wheel_vel_pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "wheel_speed", 10);
    twist_timer = this->create_wall_timer(
        100ms, std::bind(&EightTrajectory::wheel_vel_timer_callback, this),
        timer_callback_group_);
  }

private:
  // This callback computes the current yaw 'phi' of the robot
  void odom_sub_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // Convert odometry orientation quaternion to euler angles
    tf2::Quaternion quat_tf;
    geometry_msgs::msg::Quaternion quat_msg = msg->pose.pose.orientation;
    tf2::fromMsg(quat_msg, quat_tf);
    double roll{}, pitch{};
    tf2::Matrix3x3 m(quat_tf);
    m.getRPY(roll, pitch, phi);

    // RCLCPP_INFO(this->get_logger(), "Received Yaw: %f radians", phi);
  }

  void wheel_vel_timer_callback() {
    if (current_motion_index >= waypoints_traj.size()) {
      // Trajectory complete
      wheel_speed_msg.data = {0, 0, 0, 0};
      wheel_vel_pub->publish(wheel_speed_msg);
      twist_timer->cancel();
      return;
    }

    same_motion_duration = 30;

    if (current_state == MOVING) {
      // Get current waypoint
      WayPoint current_waypoint = waypoints_traj.at(current_motion_index);

      // Compute twist for current waypoint
      auto twist = velocity_to_twist(current_waypoint.dphi, current_waypoint.dx,
                                     current_waypoint.dy);
      twist_to_wheels(twist);

      // Publish the wheel command
      wheel_vel_pub->publish(wheel_speed_msg);
      same_motion_index++;

      if (same_motion_index >=
          same_motion_duration) { // 5 seconds (if timer = 100ms)
        current_state = STOPPING;
        same_motion_index = 0;
        std::cout << ">>> Transitioning to STOPPING state <<<" << std::endl;
      }
    }

    else if (current_state == STOPPING) {
      std_msgs::msg::Float32MultiArray stop_message;
      stop_message.data = {0, 0, 0, 0};
      wheel_vel_pub->publish(stop_message);

      stop_motion_index++;
      if (stop_motion_index >= 30) { // 2 seconds (if timer = 100ms)
        stop_motion_index = 0;
        current_motion_index++;
        current_state = MOVING;
        std::cout << ">>> Proceeding to next waypoint <<<" << std::endl;
      }
    }
  }

  void wheel_vel_timer_callback_old() {

    if (current_motion_index < waypoints_traj.size()) {

      // Set the current waypoint
      WayPoint current_waypoint = waypoints_traj.at(current_motion_index);

      /* std::cout << "current_waypoint : " << current_waypoint.dphi << " ,"
                 << current_waypoint.dx << " ," << current_waypoint.dy
                 << std::endl;*/

      // Get the necessary wheel velocities to go to that waypoint
      auto twist = velocity_to_twist(current_waypoint.dphi, current_waypoint.dx,
                                     current_waypoint.dy);
      twist_to_wheels(twist);

      // Publish the path to the same waypoint for 10 * 0.5 = 3 seconds
      if (same_motion_index < 50) {

        // std::cout << "Same motion index : " << same_motion_index <<
        // std::endl;

        wheel_vel_pub->publish(wheel_speed_msg);
        same_motion_index++;
      }

      else {
        // Stop for a moment

        if (stop_motion_index < 20) {
          std_msgs::msg::Float32MultiArray stop_message;
          stop_message.data = {0, 0, 0, 0};
          wheel_vel_pub->publish(stop_message);

          stop_motion_index++;
          std::cout << "Stopping for 2 seconds" << std::endl;
        }

        else {
          // Reset the same_motion_index for the following motion
          same_motion_index = 0;
          stop_motion_index = 0;
          // Then update the motion to execute next
          ++current_motion_index;
          RCLCPP_INFO(this->get_logger(),
                      "######################################");
          std::cout << "Changed Waypoint!!" << std::endl;
          RCLCPP_INFO(this->get_logger(),
                      "######################################");
        }
      }
    }

    else {
      wheel_speed_msg.data = {0, 0, 0, 0};
      wheel_vel_pub->publish(wheel_speed_msg);
      twist_timer->cancel();
    }
  }

  void waypoints_traj_init() {

    waypoints_traj.push_back(WayPoint(0.0, 1, -1));
    waypoints_traj.push_back(WayPoint(0.0, 1, 1));
    waypoints_traj.push_back(WayPoint(0.0, 1, 1));

    waypoints_traj.push_back(WayPoint(-1.5708, 1, -1));
    waypoints_traj.push_back(WayPoint(-1.5708, -1, -1));

    waypoints_traj.push_back(WayPoint(0.0, -1, 1));
    waypoints_traj.push_back(WayPoint(0.0, -1, 1));
    waypoints_traj.push_back(WayPoint(0.0, -1, -1));
  }

  Eigen::MatrixXd velocity_to_twist(double dphi, double dx, double dy) {

    Eigen::Matrix3d R;
    R << 1, 0, 0, 0, std::cos(phi), std::sin(phi), 0, -std::sin(phi),
        std::cos(phi);

    Eigen::MatrixXd dq(3, 1);
    dq << dphi, dx, dy;

    Eigen::MatrixXd twist = R * dq;

    return twist;
  }

  void twist_to_wheels(Eigen::MatrixXd &twist) {
    Eigen::MatrixXd H(4, 3);
    H << -l - w, 1, -1, l + w, 1, 1, l + w, 1, -1, -l - w, 1, 1;

    H = (1 / r) * H;

    Eigen::MatrixXd u = H * twist;
    wheel_speed_msg.data = {
        // data consists of floats
        static_cast<float>(u(0, 0)), static_cast<float>(u(1, 0)),
        static_cast<float>(u(2, 0)), static_cast<float>(u(3, 0))};
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::TimerBase::SharedPtr twist_timer;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_vel_pub;

  // Current yaw of the robot
  double phi = 0.0;

  // list of all the waypoints w that make up the figure-eight path
  std::vector<WayPoint> waypoints_traj;

  // robot geometric parameters
  double l = 0.170 / 2;   // half of the wheel base distance
  double r = 0.100 / 2;   // the radius of the wheels
  double w = 0.26969 / 2; // half of track width

  // Messsage containing the wheel speeds to be published
  std_msgs::msg::Float32MultiArray wheel_speed_msg;

  int current_motion_index = 0;
  int same_motion_index = 0;
  int stop_motion_index = 0;
  int same_motion_duration = 0; // in seconds

  enum MotionState { MOVING, STOPPING };
  MotionState current_state = MOVING;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<EightTrajectory> eight_traj_node =
      std::make_shared<EightTrajectory>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(eight_traj_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}