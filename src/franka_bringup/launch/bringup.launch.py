"""Bring up the simulated Panda, its controllers and MoveIt 2.

Composes the Gazebo, ros2_control and MoveIt launch files so the whole stack
starts in one command.  The expected order is::

    ros2 launch franka_bringup bringup.launch.py
    ros2 launch franka_bringup bringup.launch.py use_gazebo_gui:=false rviz:=false
    ros2 launch franka_bringup bringup.launch.py world_name:=warehouse
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    description_share = get_package_share_directory('franka_description')
    controller_share = get_package_share_directory('franka_controller')
    moveit_share = get_package_share_directory('franka_moveit')

    world_name = LaunchConfiguration('world_name')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    rviz = LaunchConfiguration('rviz')
    hand = LaunchConfiguration('hand')
    with_sc = LaunchConfiguration('with_sc')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declared_arguments = [
        DeclareLaunchArgument(
            'world_name',
            default_value='warehouse',
            description='World name; loads worlds/<world_name>.world.',
        ),
        DeclareLaunchArgument(
            'use_gazebo_gui',
            default_value='true',
            description='Run the Gazebo client (false = server only).',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Run RViz with the MoveIt MotionPlanning display.'),
        DeclareLaunchArgument('hand', default_value='true',
                              description='Attach and control the Franka hand.'),
        DeclareLaunchArgument(
            'with_sc',
            default_value='false',
            description='Emit the capsule self-collision companion links.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo simulation clock.'),
    ]

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world_name': world_name,
            'use_gazebo_gui': use_gazebo_gui,
            'use_sim_time': use_sim_time,
            'rviz': 'false',
            'hand': hand,
            'with_sc': with_sc,
        }.items(),
    )
    controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, 'launch', 'controller.launch.py')),
        launch_arguments={'hand': hand}.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'moveit.launch.py')),
        launch_arguments={
            'rviz': rviz,
            'hand': hand,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    return LaunchDescription(
        declared_arguments + [gazebo, controllers, moveit]
    )
