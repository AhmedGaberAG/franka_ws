"""Load and activate the Panda ros2_control controllers.

Expects a running controller manager, usually started by the Gazebo launch
file of ``franka_description``.  Controller names and joint names are defined
in ``config/controllers.yaml`` and assume the default (empty) prefix.

Examples
--------
    ros2 launch franka_controller controller.launch.py
    ros2 launch franka_controller controller.launch.py hand:=false
    ros2 launch franka_controller controller.launch.py controller_manager:=/panda/controller_manager
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Build the launch description."""
    controller_manager = LaunchConfiguration('controller_manager')
    timeout = LaunchConfiguration('timeout')
    hand = LaunchConfiguration('hand')

    declared_arguments = [
        DeclareLaunchArgument(
            'controller_manager',
            default_value='/controller_manager',
            description='Fully qualified name of the controller manager node.',
        ),
        DeclareLaunchArgument(
            'timeout',
            default_value='60.0',
            description='Seconds to wait for the controller manager to appear.',
        ),
        DeclareLaunchArgument(
            'hand',
            default_value='true',
            description='Also load the gripper controller.',
        ),
    ]

    def spawner(controller: str, condition=None) -> Node:
        return Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager', controller_manager,
                '--controller-manager-timeout', timeout,
            ],
            condition=condition,
        )

    return LaunchDescription(
        declared_arguments
        + [
            spawner('joint_state_broadcaster'),
            spawner('panda_arm_controller'),
            spawner('panda_hand_controller', IfCondition(hand)),
        ]
    )
