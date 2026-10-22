"""Spawn the Panda in a Gazebo Sim world.

Starts Gazebo with ``worlds/<world_name>.world``, publishes the robot
description, spawns the model and starts the bridges/helpers the world needs.
The controllers are loaded separately with
``ros2 launch franka_controller controller.launch.py``.

Examples
--------
    ros2 launch franka_description gazebo.launch.py
    ros2 launch franka_description gazebo.launch.py world_name:=warehouse
    ros2 launch franka_description gazebo.launch.py use_gazebo_gui:=false
    ROS_DISTRO=iron ros2 launch franka_description gazebo.launch.py
"""

import os
from os import pathsep
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Gazebo flavour per ROS distro.  Humble ships Ignition Fortress (v6); Iron and
# newer ship gz-sim, where the DART engine needs an explicit Bullet fallback.
GZ_VERSION = {'humble': '6', 'iron': '7', 'jazzy': '8'}
PHYSICS_ENGINE = '--physics-engine gz-physics-bullet-featherstone-plugin'


def _world_path(share: str, world_name) -> PathJoinSubstitution:
    """Resolve ``worlds/<world_name>.world`` inside the package share."""
    return PathJoinSubstitution([
        share,
        'worlds',
        PythonExpression(["'", world_name, "' + '.world'"]),
    ])


def _gz_sim_launch(share, world_path, gz_version, physics_engine, headless,
                   condition):
    """Include ``ros_gz_sim``'s ``gz_sim.launch.py`` with the world loaded.

    ``headless`` adds ``-s`` so only the server runs (no Gazebo GUI).
    """
    server_only = '-s ' if headless else ''
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(share, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': [
                '-r -v 4 ', server_only, physics_engine, ' ', world_path,
            ],
            'gz_version': gz_version,
        }.items(),
        condition=condition,
    )


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    share = get_package_share_directory('franka_description')
    ros_gz_sim_share = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(share, 'urdf', 'panda.urdf.xacro')
    rviz_file = os.path.join(share, 'rviz', 'panda.rviz')

    world_name = LaunchConfiguration('world_name')
    use_gazebo_gui = LaunchConfiguration('use_gazebo_gui')
    use_sim_time = LaunchConfiguration('use_sim_time')
    hand = LaunchConfiguration('hand')
    with_sc = LaunchConfiguration('with_sc')
    rviz = LaunchConfiguration('rviz')

    declared_arguments = [
        DeclareLaunchArgument(
            'world_name',
            default_value='warehouse',
            description='World name; loads worlds/<world_name>.world.'),
        DeclareLaunchArgument(
            'use_gazebo_gui',
            default_value='true',
            description='Start the Gazebo GUI (false = server only).'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo simulation clock.'),
        DeclareLaunchArgument(
            'hand',
            default_value='true',
            description='Attach the Franka hand to the flange.'),
        DeclareLaunchArgument(
            'with_sc',
            default_value='false',
            description='Emit the capsule self-collision companion links.'),
        DeclareLaunchArgument(
            'rviz',
            default_value='false',
            description='Run RViz alongside Gazebo.'),
    ]

    world_path = _world_path(share, world_name)

    # package:// resolves from the share parent; model:// from the models dir.
    resource_path = str(Path(share).parent.resolve())
    resource_path += pathsep + os.path.join(share, 'models')

    ros_distro = os.environ.get('ROS_DISTRO', 'humble')
    is_ignition = 'True' if ros_distro == 'humble' else 'False'
    gz_version = GZ_VERSION.get(ros_distro, '9')
    physics_engine = '' if ros_distro == 'humble' else PHYSICS_ENGINE

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

    # Bridges and helpers tied to the world: simulation clock, the pose stream
    # used by the conveyor controller, and the conveyor pause command.
    pose_topic = PythonExpression(
        ["'/world/' + '", world_name, "' + '/dynamic_pose/info'"])
    pose_bridge_arg = PythonExpression([
        "'/world/' + '", world_name,
        "' + '/dynamic_pose/info@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'",
    ])

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock'],
    )
    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[pose_bridge_arg],
    )
    pause_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=['/conveyor/pause@std_msgs/msg/Bool]ignition.msgs.Boolean'],
    )
    conveyor_control = Node(
        package='franka_description',
        executable='conveyor_control.py',
        output='screen',
        parameters=[{'pose_topic': pose_topic, 'use_sim_time': use_sim_time}],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'panda'],
    )
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    gazebo_gui = _gz_sim_launch(
        ros_gz_sim_share, world_path, gz_version, physics_engine,
        headless=False, condition=IfCondition(use_gazebo_gui))
    gazebo_headless = _gz_sim_launch(
        ros_gz_sim_share, world_path, gz_version, physics_engine,
        headless=True, condition=UnlessCondition(use_gazebo_gui))

    return LaunchDescription(
        declared_arguments
        + [
            SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', resource_path),
            SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
            gazebo_gui,
            gazebo_headless,
            clock_bridge,
            pose_bridge,
            pause_bridge,
            conveyor_control,
            robot_state_publisher,
            spawn_entity,
            rviz_node,
        ]
    )
