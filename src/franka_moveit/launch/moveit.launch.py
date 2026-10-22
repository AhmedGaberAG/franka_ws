"""Plan and execute Panda motions with MoveIt 2.

Starts ``move_group`` and, optionally, RViz with the MotionPlanning display.
The robot description, TF and controllers are expected to be running already,
for example the Gazebo launch from ``franka_description`` followed by
``controller.launch.py`` from ``franka_controller``.

The semantic description and the controller mapping assume the default
(empty) prefix.

Examples
--------
    ros2 launch franka_moveit moveit.launch.py
    ros2 launch franka_moveit moveit.launch.py rviz:=false
    ros2 launch franka_moveit moveit.launch.py hand:=false
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(path: str) -> dict:
    """Load a YAML file into a parameter dictionary."""
    with open(path, encoding='utf-8') as file:
        return yaml.safe_load(file)


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    share = get_package_share_directory('franka_moveit')
    description_share = get_package_share_directory('franka_description')

    xacro_file = os.path.join(description_share, 'urdf', 'panda.urdf.xacro')
    srdf_file = os.path.join(share, 'config', 'panda.srdf')
    rviz_file = os.path.join(share, 'config', 'moveit.rviz')

    prefix = LaunchConfiguration('prefix')
    hand = LaunchConfiguration('hand')
    rviz = LaunchConfiguration('rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declared_arguments = [
        DeclareLaunchArgument(
            'prefix',
            default_value='',
            description='Link name prefix; the SRDF assumes the default empty prefix.',
        ),
        DeclareLaunchArgument('hand', default_value='true',
                              description='Plan with the Franka hand attached.'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz with the MotionPlanning display.'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use the Gazebo simulation clock.',
        ),
    ]

    # The self-collision capsules are a Gazebo aid; MoveIt checks the meshes.
    robot_description = ParameterValue(
        Command([
            'xacro ', xacro_file,
            ' prefix:=', prefix,
            ' hand:=', hand,
            ' with_sc:=false',
            ' is_sim:=false',
            ' use_world:=true',
        ]),
        value_type=str,
    )

    with open(srdf_file, encoding='utf-8') as file:
        robot_description_semantic = file.read()

    # The controller manager reads its settings at the top level of the node.
    controller_parameters = load_yaml(
        os.path.join(share, 'config', 'moveit_controllers.yaml'))

    moveit_parameters = [{
        'robot_description': robot_description,
        'robot_description_semantic': robot_description_semantic,
        'robot_description_kinematics': load_yaml(
            os.path.join(share, 'config', 'kinematics.yaml')),
        'robot_description_planning': load_yaml(
            os.path.join(share, 'config', 'joint_limits.yaml')),
        **controller_parameters,
        'planning_pipelines': ['ompl'],
        'ompl': load_yaml(os.path.join(share, 'config', 'ompl_planning.yaml')),
        'use_sim_time': use_sim_time,
        # The controllers are already active; MoveIt must not switch them.
        'moveit_manage_controllers': False,
        'trajectory_execution': {
            'allowed_execution_duration_scaling': 1.2,
            'allowed_goal_duration_margin': 0.5,
            'allowed_start_tolerance': 0.05,
        },
    }]

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=moveit_parameters,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        condition=IfCondition(rviz),
        arguments=['-d', rviz_file],
        parameters=moveit_parameters,
        output='screen',
    )

    return LaunchDescription(declared_arguments + [move_group, rviz_node])
