"""Spawn the Panda in Gazebo Sim.

Starts Gazebo Sim, publishes the robot description, spawns the model and loads
the ros2_control controllers from ``franka_controller``.

Examples
--------
    ros2 launch franka_description gazebo.launch.py
    ros2 launch franka_description gazebo.launch.py gui:=false rviz:=true
    ROS_DISTRO=iron ros2 launch franka_description gazebo.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    share = get_package_share_directory('franka_description')
    controller_share = get_package_share_directory('franka_controller')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(share, 'urdf', 'panda.urdf.xacro')
    rviz_file = os.path.join(share, 'rviz', 'panda.rviz')

    # Gazebo turns package:// into model:// URIs, which it resolves from the
    # parent of the package share directory.
    share_root = os.path.dirname(share)
    resource_path = os.pathsep.join(
        filter(None, [share_root, os.environ.get('GZ_SIM_RESOURCE_PATH', '')])
    )
    set_resource_path = [
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
    ]

    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    is_ignition = 'True' if ros_distro == 'humble' else 'False'
    gz_version = {'humble': '6', 'iron': '7', 'jazzy': '8'}.get(ros_distro, '9')
    physics_engine = (
        '' if ros_distro == 'humble'
        else '--physics-engine gz-physics-bullet-featherstone-plugin'
    )

    world = LaunchConfiguration('world')
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    hand = LaunchConfiguration('hand')
    with_sc = LaunchConfiguration('with_sc')

    declared_arguments = [
        DeclareLaunchArgument(
            'world',
            default_value='empty.sdf',
            description='World file, resolved by Gazebo or given as a full path.',
        ),
        DeclareLaunchArgument('gui', default_value='true',
                              description='Run the Gazebo client.'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description='Run RViz alongside Gazebo.'),
        DeclareLaunchArgument('hand', default_value='true',
                              description='Attach the Franka hand to the flange.'),
        DeclareLaunchArgument(
            'with_sc',
            default_value='false',
            description='Emit the capsule self-collision companion links.',
        ),
    ]

    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' hand:=', hand,
            ' with_sc:=', with_sc,
            ' is_sim:=true',
            ' is_ignition:=', is_ignition,
            ' use_world:=true',
        ]),
        value_type=str,
    )

    def gazebo(extra_args: str, condition) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim_share, 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={
                'gz_args': ['-r -v 4 ', physics_engine, ' ', extra_args, world],
                'gz_version': gz_version,
            }.items(),
            condition=condition,
        )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
        output='screen',
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'panda'],
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(controller_share, 'launch', 'controller.launch.py')
        ),
        launch_arguments={'hand': hand}.items(),
    )

    load_controllers = RegisterEventHandler(
        OnProcessExit(target_action=spawn_entity, on_exit=controller_launch),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    gazebo_gui = gazebo('', IfCondition(gui))
    gazebo_headless = gazebo('-s ', UnlessCondition(gui))

    return LaunchDescription(
        declared_arguments
        + set_resource_path
        + [
            gazebo_gui,
            gazebo_headless,
            clock_bridge,
            robot_state_publisher,
            spawn_entity,
            load_controllers,
            rviz_node,
        ]
    )
