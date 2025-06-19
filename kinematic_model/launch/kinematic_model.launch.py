from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    wheel_vel_pub_node = Node(
            package='wheel_velocities_publisher',
            executable='wheel_velocities_publisher',
            output='screen',
            name='wheel_velocities_publisher',
            )

    kinematic_model_node = Node(
            package='kinematic_model',
            executable='kinematic_model',
            output='screen',
            name='kinematic_model',
            )

    return LaunchDescription([
        wheel_vel_pub_node,
        kinematic_model_node,
    ])