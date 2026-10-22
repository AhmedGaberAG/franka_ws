"""Launch the Franka Panda control GUI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_manager = LaunchConfiguration('controller_manager')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the simulation clock.'),
        DeclareLaunchArgument(
            'controller_manager', default_value='/controller_manager',
            description='Fully qualified controller manager node name.'),
        Node(
            package='franka_utils',
            executable='franka_gui.py',
            name='franka_gui',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'controller_manager': controller_manager,
            }],
        ),
    ])
