from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    
    eight_traj_node = Node(
            package='eight_trajectory',
            executable='eight_trajectory',
            output='screen',
            name='eight_trajectory',
            )

    kinematic_model_node = Node(
            package='kinematic_model',
            executable='kinematic_model',
            output='screen',
            name='kinematic_model',
            )

    return LaunchDescription([
        eight_traj_node,
        kinematic_model_node,
    ])