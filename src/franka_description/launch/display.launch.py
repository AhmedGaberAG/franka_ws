"""Visualise the Panda in RViz.

Loads ``urdf/panda.urdf.xacro`` through ``robot_state_publisher`` and drives
the joints with ``joint_state_publisher`` so the model can be inspected
without a simulator.

Examples
--------
    ros2 launch franka_description display.launch.py
    ros2 launch franka_description display.launch.py hand:=false
    ros2 launch franka_description display.launch.py gui:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    share = get_package_share_directory('franka_description')

    xacro_file = os.path.join(share, 'urdf', 'panda.urdf.xacro')
    rviz_file = os.path.join(share, 'rviz', 'panda.rviz')

    prefix = LaunchConfiguration('prefix')
    hand = LaunchConfiguration('hand')
    with_sc = LaunchConfiguration('with_sc')
    use_sim_time = LaunchConfiguration('use_sim_time')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' prefix:=', prefix,
            ' hand:=', hand,
            ' with_sc:=', with_sc,
        ]),
        value_type=str,
    )

    declared_arguments = [
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Prefix added to every link and joint name.',
        ),
        DeclareLaunchArgument(
            'hand',
            default_value='true',
            description='Attach the Franka hand to the flange.',
        ),
        DeclareLaunchArgument(
            'with_sc',
            default_value='true',
            description='Emit the capsule self-collision companion links.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use the simulation clock.',
        ),
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Run the joint_state_publisher GUI.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz.',
        ),
    ]

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(gui),
        output='screen',
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(gui),
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_file],
        output='screen',
    )

    return LaunchDescription(
        declared_arguments
        + [
            robot_state_publisher,
            joint_state_publisher_gui,
            joint_state_publisher,
            rviz_node,
        ]
    )
