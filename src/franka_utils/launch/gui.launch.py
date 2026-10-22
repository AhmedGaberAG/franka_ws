from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "controller_manager",
            default_value="/controller_manager",
            description="Fully qualified controller manager node name.",
        ),
        DeclareLaunchArgument(
            "arm_action",
            default_value="/panda_arm_controller/follow_joint_trajectory",
            description="Arm FollowJointTrajectory action name.",
        ),
        DeclareLaunchArgument(
            "gripper_action",
            default_value="/panda_hand_controller/gripper_cmd",
            description="GripperCommand action name.",
        ),
        DeclareLaunchArgument(
            "move_action",
            default_value="/move_action",
            description="MoveIt MoveGroup action name.",
        ),
        DeclareLaunchArgument(
            "joint_states_topic",
            default_value="/joint_states",
            description="JointState topic name.",
        ),
        Node(
            package="franka_utils",
            executable="franka_gui",
            name="franka_pyside6_gui",
            output="screen",
            parameters=[{
                "controller_manager": LaunchConfiguration("controller_manager"),
                "arm_action": LaunchConfiguration("arm_action"),
                "gripper_action": LaunchConfiguration("gripper_action"),
                "move_action": LaunchConfiguration("move_action"),
                "joint_states_topic": LaunchConfiguration("joint_states_topic"),
            }],
        ),
    ])