"""ROS 2 node interface for Franka Panda control."""

import math
from functools import partial
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints, JointConstraint, MotionPlanRequest, OrientationConstraint,
    PositionConstraint, RobotState,
)
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from .constants import ARM_JOINTS, FINGER_JOINT, GRIPPER_MIN, GRIPPER_MAX


class FrankaRosNode(Node):
    """ROS interface only. It does not touch any Qt widgets."""

    def __init__(self, log_callback):
        super().__init__("franka_pyside6_gui")
        self._log = log_callback
        self.declare_parameter("controller_manager", "/controller_manager")
        self.declare_parameter("arm_action", "/panda_arm_controller/follow_joint_trajectory")
        self.declare_parameter("gripper_action", "/panda_hand_controller/gripper_cmd")
        self.declare_parameter("move_action", "/move_action")
        self.declare_parameter("joint_states_topic", "/joint_states")

        self.cm_name = str(self.get_parameter("controller_manager").value).rstrip("/")
        self.arm_action_name = self.get_parameter("arm_action").value
        self.gripper_action_name = self.get_parameter("gripper_action").value
        self.move_action_name = self.get_parameter("move_action").value

        self.arm_client = ActionClient(self, FollowJointTrajectory, self.arm_action_name)
        self.gripper_client = ActionClient(self, GripperCommand, self.gripper_action_name)
        self.move_client = ActionClient(self, MoveGroup, self.move_action_name)
        self.cm_client = self.create_client(ListControllers, f"{self.cm_name}/list_controllers")
        self.create_subscription(
            JointState, self.get_parameter("joint_states_topic").value,
            self._joint_state_cb, QoSProfile(depth=10)
        )
        self.latest_positions = {}
        self.arm_handle = None
        self.gripper_handle = None
        self.move_handle = None

    def _joint_state_cb(self, msg):
        self.latest_positions = dict(zip(msg.name, msg.position))

    def send_arm(self, positions, duration=3.0):
        if len(positions) != 7:
            self._log("ERROR: exactly 7 joint values are required.")
            return
        for value, (name, lo, hi) in zip(positions, ARM_JOINTS):
            if not math.isfinite(value) or value < lo or value > hi:
                self._log(f"Rejected: {name}={value:.4f} is outside [{lo:.4f}, {hi:.4f}] rad.")
                return
        if not self.arm_client.server_is_ready():
            self._log(f"Arm action unavailable: {self.arm_action_name}")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [j[0] for j in ARM_JOINTS]
        point = JointTrajectoryPoint()
        point.positions = list(map(float, positions))
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration - int(duration)) * 1e9)
        )
        goal.trajectory.points = [point]
        self._log(f"Sending joint trajectory ({duration:.1f} s)…")
        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(self._arm_goal_response)

    def _arm_goal_response(self, future):
        try:
            handle = future.result()
            self.arm_handle = handle
            if not handle.accepted:
                self._log("Arm goal rejected by controller.")
                return
            self._log("Arm goal accepted.")
            handle.get_result_async().add_done_callback(self._arm_result)
        except Exception as exc:
            self._log(f"Arm goal error: {exc}")

    def _arm_result(self, future):
        try:
            wrapped = future.result()
            self._log(f"Arm result: status={wrapped.status}, controller_error={wrapped.result.error_code}")
        except Exception as exc:
            self._log(f"Arm result error: {exc}")

    def send_gripper(self, position, effort=10.0):
        if not GRIPPER_MIN <= position <= GRIPPER_MAX:
            self._log("Rejected: gripper position must be between 0.000 and 0.040 m.")
            return
        if not self.gripper_client.server_is_ready():
            self._log(f"Gripper action unavailable: {self.gripper_action_name}")
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(effort)
        self._log(f"Sending gripper goal: {position:.3f} m, max effort {effort:.1f} N.")
        self.gripper_client.send_goal_async(goal).add_done_callback(self._gripper_goal_response)

    def _gripper_goal_response(self, future):
        try:
            self.gripper_handle = future.result()
            if not self.gripper_handle.accepted:
                self._log("Gripper goal rejected.")
                return
            self._log("Gripper goal accepted.")
            self.gripper_handle.get_result_async().add_done_callback(self._gripper_result)
        except Exception as exc:
            self._log(f"Gripper goal error: {exc}")

    def _gripper_result(self, future):
        try:
            wrapped = future.result()
            result = wrapped.result
            self._log(f"Gripper result: pos={result.position:.3f} m, reached={result.reached_goal}, stalled={result.stalled}")
        except Exception as exc:
            self._log(f"Gripper result error: {exc}")

    def cancel_arm(self):
        if self.arm_handle is None:
            self._log("No active arm goal.")
        else:
            self.arm_handle.cancel_goal_async()
            self._log("Arm cancellation requested.")

    def cancel_move(self):
        if self.move_handle is None:
            self._log("No active MoveIt goal.")
        else:
            self.move_handle.cancel_goal_async()
            self._log("MoveIt cancellation requested.")

    def check_action_servers(self):
        for label, client, name in [
            ("Arm", self.arm_client, self.arm_action_name),
            ("Gripper", self.gripper_client, self.gripper_action_name),
            ("MoveGroup", self.move_client, self.move_action_name),
        ]:
            self._log(f"{label}: {'available' if client.server_is_ready() else 'not available'} ({name})")

    def refresh_controllers(self):
        if not self.cm_client.service_is_ready():
            self._log(f"Controller manager service unavailable: {self.cm_name}/list_controllers")
            return
        future = self.cm_client.call_async(ListControllers.Request())
        future.add_done_callback(self._controllers_response)

    def _controllers_response(self, future):
        try:
            result = future.result()
            data = [(c.name, c.state, c.type) for c in result.controller]
            self._log(f"Controller status refreshed ({len(data)} controllers).")
            return data
        except Exception as exc:
            self._log(f"Controller status error: {exc}")
            return None

    def moveit_joint_goal(self, positions, plan_only, group_name, velocity, acceleration):
        for value, (name, lo, hi) in zip(positions, ARM_JOINTS):
            if not math.isfinite(value) or not lo <= value <= hi:
                self._log(f"MoveIt goal rejected: {name} outside joint limits.")
                return
        if not self.move_client.server_is_ready():
            self._log(f"MoveGroup unavailable: {self.move_action_name}")
            return
        request = MotionPlanRequest()
        request.group_name = group_name
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = float(velocity)
        request.max_acceleration_scaling_factor = float(acceleration)
        constraints = Constraints()
        for (name, _, _), value in zip(ARM_JOINTS, positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        request.goal_constraints = [constraints]
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = bool(plan_only)
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        self._log(f"MoveIt: {'planning only' if plan_only else 'planning + execution'} ({group_name})…")
        self.move_client.send_goal_async(goal).add_done_callback(
            partial(self._move_goal_response, plan_only=plan_only)
        )

    def moveit_pose_goal(self, xyz, quat_wxyz, frame, plan_only, velocity, acceleration):
        if not self.move_client.server_is_ready():
            self._log(f"MoveGroup unavailable: {self.move_action_name}")
            return
        if not frame.strip():
            self._log("Pose goal rejected: frame cannot be empty.")
            return
        qnorm = math.sqrt(sum(q*q for q in quat_wxyz))
        if qnorm < 1e-6:
            self._log("Pose goal rejected: quaternion must not be all zeros.")
            return
        q = [v / qnorm for v in quat_wxyz]
        request = MotionPlanRequest()
        request.group_name = "panda_arm"
        request.num_planning_attempts = 5
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor = float(velocity)
        request.max_acceleration_scaling_factor = float(acceleration)
        constraints = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = frame
        pc.link_name = "panda_link8"
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.015]
        pc.constraint_region.primitives.append(sphere)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = q
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0
        constraints.position_constraints.append(pc)
        oc = OrientationConstraint()
        oc.header.frame_id = frame
        oc.link_name = "panda_link8"
        oc.orientation = pose.orientation
        oc.absolute_x_axis_tolerance = 0.12
        oc.absolute_y_axis_tolerance = 0.12
        oc.absolute_z_axis_tolerance = 0.12
        oc.weight = 1.0
        constraints.orientation_constraints.append(oc)
        request.goal_constraints = [constraints]
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = bool(plan_only)
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        self._log(f"MoveIt pose goal: frame={frame}, plan_only={plan_only}…")
        self.move_client.send_goal_async(goal).add_done_callback(
            partial(self._move_goal_response, plan_only=plan_only)
        )

    def _move_goal_response(self, future, plan_only):
        try:
            handle = future.result()
            self.move_handle = handle
            if not handle.accepted:
                self._log("MoveIt goal rejected.")
                return
            self._log("MoveIt goal accepted.")
            handle.get_result_async().add_done_callback(partial(self._move_result, plan_only=plan_only))
        except Exception as exc:
            self._log(f"MoveIt response error: {exc}")

    def _move_result(self, future, plan_only):
        try:
            wrapped = future.result()
            code = wrapped.result.error_code.val
            self._log(f"MoveIt finished: error_code={code}, plan_only={plan_only} (1=SUCCESS).")
        except Exception as exc:
            self._log(f"MoveIt result error: {exc}")