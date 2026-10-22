#!/usr/bin/env python3
"""Tkinter GUI to control the Franka Panda arm and gripper.

Drives the real controllers (joint_trajectory_controller + GripperActionController)
and can plan/execute with MoveIt via the MoveGroup action.
"""

import math
import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory, GripperCommand
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PositionConstraint,
    RobotState,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header


# Joint limits from franka_description/urdf/properties.xacro
ARM_JOINTS = [
    ('panda_joint1', -2.8973, 2.8973),
    ('panda_joint2', -1.7628, 1.7628),
    ('panda_joint3', -2.8973, 2.8973),
    ('panda_joint4', -3.0718, -0.0698),
    ('panda_joint5', -2.8973, 2.8973),
    ('panda_joint6', -0.0175, 3.7525),
    ('panda_joint7', -2.8973, 2.8973),
]

ARM_PRESETS = {
    'Home (all zero)': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    'Ready': [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    'Extended': [0.0, 0.0, 0.0, 0.0, 0.0, 1.571, 0.785],
    'Transport': [0.0, -0.5599, 0.0, -2.97, 0.0, 0.0, 0.785],
}

FINGER_JOINT = 'panda_finger_joint1'
GRIPPER_MIN = 0.0
GRIPPER_MAX = 0.04


class FrankaGuiNode(Node):
    def __init__(self):
        super().__init__('franka_gui')
        self.declare_parameter('controller_manager', '/controller_manager')
        self.declare_parameter('arm_action', '/panda_arm_controller/follow_joint_trajectory')
        self.declare_parameter('gripper_action', '/panda_hand_controller/gripper_cmd')
        self.declare_parameter('move_action', '/move_action')
        self.declare_parameter('joint_states_topic', '/joint_states')

        cm = self.get_parameter('controller_manager').value
        self._cm_name = cm
        self._arm_action_name = self.get_parameter('arm_action').value
        self._gripper_action_name = self.get_parameter('gripper_action').value
        self._move_action_name = self.get_parameter('move_action').value

        self.arm_client = ActionClient(self, FollowJointTrajectory, self._arm_action_name)
        self.grip_client = ActionClient(self, GripperCommand, self._gripper_action_name)
        self.move_client = ActionClient(self, MoveGroup, self._move_action_name)
        self.cm_client = self.create_client(ListControllers, f'{cm}/list_controllers')

        self.joint_state_lock = threading.Lock()
        self.latest_joint_state = None

        qos = QoSProfile(depth=10)
        self.create_subscription(JointState, self.get_parameter('joint_states_topic').value,
                                 self._joint_state_cb, qos)

        self._arm_goal_handle = None
        self._grip_goal_handle = None
        self._move_goal_handle = None

    def _joint_state_cb(self, msg: JointState):
        with self.joint_state_lock:
            self.latest_joint_state = msg

    def get_joint_positions(self):
        with self.joint_state_lock:
            msg = self.latest_joint_state
        if msg is None:
            return {}
        return {n: p for n, p in zip(msg.name, msg.position)}

    def get_controller_status(self):
        if not self.cm_client.service_is_ready():
            return None
        req = ListControllers.Request()
        future = self.cm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if future.result() is None:
            return None
        return future.result()


class FrankaTkGUI:
    def __init__(self, root, node):
        self.root = root
        self.node = node
        self.root.title('Franka Panda Control')
        self.root.geometry('880x820')
        self.root.protocol('WM_DELETE_WINDOW', self.on_close)

        self.sliders = []
        self.slider_vars = []
        self.joint_labels = []
        self.live_labels = []
        self.streaming = tk.BooleanVar(value=False)
        self.stream_job = None

        self._build_arm_tab()
        self._build_gripper_tab()
        self._build_moveit_tab()
        self._build_status_tab()

        self.status_text = tk.Text(self.root, height=8, wrap=tk.WORD)
        self.status_text.pack(fill=tk.X, padx=4, pady=2)
        self._log('Franka GUI started. Waiting for /joint_states...')

        self.update_ui()

    def _log(self, msg):
        ts = time.strftime('%H:%M:%S')
        self.status_text.insert(tk.END, f'[{ts}] {msg}\n')
        self.status_text.see(tk.END)
        self.root.update_idletasks()

    def _build_arm_tab(self):
        frame = ttk.LabelFrame(self.root, text='Arm Joint Control (panda_arm_controller)')
        frame.pack(fill=tk.X, padx=4, pady=2)

        top = ttk.Frame(frame)
        top.pack(fill=tk.X, padx=4)
        ttk.Label(top, text='Preset:').pack(side=tk.LEFT)
        self.preset_var = tk.StringVar(value='Ready')
        preset_cb = ttk.Combobox(top, textvariable=self.preset_var,
                                 values=list(ARM_PRESETS.keys()), state='readonly', width=18)
        preset_cb.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text='Apply Preset', command=self.apply_preset).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text='Read Current', command=self.read_current).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text='Stop / Cancel Goal', command=self.cancel_arm).pack(side=tk.LEFT, padx=2)
        ttk.Checkbutton(top, text='Live Stream (while dragging)', variable=self.streaming).pack(side=tk.LEFT, padx=8)

        for i, (name, lo, hi) in enumerate(ARM_JOINTS):
            row = ttk.Frame(frame)
            row.pack(fill=tk.X, padx=4, pady=1)
            ttk.Label(row, text=name, width=14).pack(side=tk.LEFT)
            var = tk.DoubleVar(value=0.0)
            self.slider_vars.append(var)
            slider = ttk.Scale(row, from_=lo, to=hi, orient=tk.HORIZONTAL, variable=var,
                               length=520, command=self._on_slider_change)
            slider.pack(side=tk.LEFT, padx=4)
            self.sliders.append(slider)
            lbl = ttk.Label(row, text=f'{lo:.3f} / {var.get():.4f} / {hi:.3f}', width=24)
            lbl.pack(side=tk.LEFT)
            self.joint_labels.append(lbl)
            live = ttk.Label(row, text='live: ---', width=12)
            live.pack(side=tk.LEFT)
            self.live_labels.append(live)

        btns = ttk.Frame(frame)
        btns.pack(fill=tk.X, padx=4, pady=4)
        ttk.Button(btns, text='Send Trajectory Goal', command=self.send_arm_goal).pack(side=tk.LEFT, padx=2)
        ttk.Label(btns, text='Duration (s):').pack(side=tk.LEFT, padx=(12, 2))
        self.arm_dur = tk.DoubleVar(value=3.0)
        ttk.Spinbox(btns, from_=0.5, to=10.0, increment=0.5, textvariable=self.arm_dur, width=6).pack(side=tk.LEFT)

    def _build_gripper_tab(self):
        frame = ttk.LabelFrame(self.root, text='Gripper Control (panda_hand_controller)')
        frame.pack(fill=tk.X, padx=4, pady=2)
        row = ttk.Frame(frame)
        row.pack(fill=tk.X, padx=4, pady=2)
        ttk.Label(row, text=FINGER_JOINT).pack(side=tk.LEFT)
        self.grip_var = tk.DoubleVar(value=0.035)
        grip = ttk.Scale(row, from_=GRIPPER_MIN, to=GRIPPER_MAX, orient=tk.HORIZONTAL,
                         variable=self.grip_var, length=520)
        grip.pack(side=tk.LEFT, padx=4)
        self.grip_lbl = ttk.Label(row, text=f'{self.grip_var.get():.4f} m')
        self.grip_lbl.pack(side=tk.LEFT)
        self.grip_var.trace_add('write', self._update_grip_lbl)
        self.grip_live = ttk.Label(row, text='live: ---')
        self.grip_live.pack(side=tk.LEFT, padx=8)
        btns = ttk.Frame(frame)
        btns.pack(fill=tk.X, padx=4, pady=2)
        ttk.Button(btns, text='Open (0.035 m)', command=lambda: self.set_gripper(0.035)).pack(side=tk.LEFT, padx=2)
        ttk.Button(btns, text='Close (0.000 m)', command=lambda: self.set_gripper(0.000)).pack(side=tk.LEFT, padx=2)
        ttk.Button(btns, text='Send Gripper Goal', command=self.send_gripper_goal).pack(side=tk.LEFT, padx=2)
        ttk.Label(btns, text='Max Effort (N):').pack(side=tk.LEFT, padx=(12, 2))
        self.grip_effort = tk.DoubleVar(value=10.0)
        ttk.Spinbox(btns, from_=0.0, to=50.0, increment=1.0, textvariable=self.grip_effort, width=6).pack(side=tk.LEFT)

    def _build_moveit_tab(self):
        frame = ttk.LabelFrame(self.root, text='MoveIt Planning & Execution')
        frame.pack(fill=tk.X, padx=4, pady=2)
        top = ttk.Frame(frame)
        top.pack(fill=tk.X, padx=4)
        ttk.Label(top, text='Planning Group:').pack(side=tk.LEFT)
        self.group_var = tk.StringVar(value='panda_arm')
        ttk.Combobox(top, textvariable=self.group_var,
                     values=['panda_arm', 'panda_hand', 'panda_arm_hand'],
                     state='readonly', width=14).pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text='Check MoveGroup', command=self.check_movegroup).pack(side=tk.LEFT, padx=2)
        ttk.Button(top, text='Cancel Move Goal', command=self.cancel_move).pack(side=tk.LEFT, padx=2)

        mid = ttk.LabelFrame(frame, text='Plan to Current Arm Sliders (Joint Goal)')
        mid.pack(fill=tk.X, padx=4, pady=2)
        mbtns = ttk.Frame(mid)
        mbtns.pack(fill=tk.X, padx=4)
        ttk.Button(mbtns, text='Plan to Sliders', command=self.plan_to_sliders).pack(side=tk.LEFT, padx=2)
        ttk.Button(mbtns, text='Plan & Execute to Sliders', command=self.plan_execute_to_sliders).pack(side=tk.LEFT, padx=2)
        ttk.Label(mbtns, text='Velocity scaling:').pack(side=tk.LEFT, padx=(12, 2))
        self.vel_scale = tk.DoubleVar(value=0.3)
        ttk.Spinbox(mbtns, from_=0.05, to=1.0, increment=0.05, textvariable=self.vel_scale, width=5).pack(side=tk.LEFT)
        ttk.Label(mbtns, text='Accel scaling:').pack(side=tk.LEFT, padx=(8, 2))
        self.acc_scale = tk.DoubleVar(value=0.3)
        ttk.Spinbox(mbtns, from_=0.05, to=1.0, increment=0.05, textvariable=self.acc_scale, width=5).pack(side=tk.LEFT)

        cart = ttk.LabelFrame(frame, text='Cartesian Pose Goal (pose in planning frame, approximate)')
        cart.pack(fill=tk.X, padx=4, pady=2)
        c1 = ttk.Frame(cart)
        c1.pack(fill=tk.X, padx=4)
        ttk.Label(c1, text='Frame:').pack(side=tk.LEFT)
        self.cart_frame = tk.StringVar(value='panda_link0')
        ttk.Entry(c1, textvariable=self.cart_frame, width=12).pack(side=tk.LEFT)
        for ax in ['x', 'y', 'z']:
            ttk.Label(c1, text=ax.upper()).pack(side=tk.LEFT, padx=(8, 1))
            setattr(self, f'cart_{ax}', tk.DoubleVar(value=0.5 if ax == 'x' else 0.0))
            ttk.Spinbox(c1, from_=-1.0, to=1.0, increment=0.01, textvariable=getattr(self, f'cart_{ax}'), width=7).pack(side=tk.LEFT)
        c2 = ttk.Frame(cart)
        c2.pack(fill=tk.X, padx=4)
        ttk.Label(c2, text='Quat w,x,y,z:').pack(side=tk.LEFT)
        self.cart_qw = tk.DoubleVar(value=1.0)
        self.cart_qx = tk.DoubleVar(value=0.0)
        self.cart_qy = tk.DoubleVar(value=0.0)
        self.cart_qz = tk.DoubleVar(value=0.0)
        for n, v in [('w', self.cart_qw), ('x', self.cart_qx), ('y', self.cart_qy), ('z', self.cart_qz)]:
            ttk.Label(c2, text=n).pack(side=tk.LEFT, padx=(4, 1))
            ttk.Spinbox(c2, from_=-1.0, to=1.0, increment=0.01, textvariable=v, width=6).pack(side=tk.LEFT)
        c3 = ttk.Frame(cart)
        c3.pack(fill=tk.X, padx=4)
        ttk.Button(c3, text='Plan to Pose (Position+Orientation Constraint)', command=self.plan_to_pose).pack(side=tk.LEFT, padx=2)
        ttk.Button(c3, text='Plan & Execute to Pose', command=self.plan_execute_to_pose).pack(side=tk.LEFT, padx=2)

    def _build_status_tab(self):
        frame = ttk.LabelFrame(self.root, text='Controllers & System')
        frame.pack(fill=tk.X, padx=4, pady=2)
        btns = ttk.Frame(frame)
        btns.pack(fill=tk.X, padx=4)
        ttk.Button(btns, text='Refresh Controller Status', command=self.refresh_controllers).pack(side=tk.LEFT, padx=2)
        ttk.Button(btns, text='Clear Log', command=self.clear_log).pack(side=tk.LEFT, padx=2)

    def _update_grip_lbl(self, *args):
        self.grip_lbl.config(text=f'{self.grip_var.get():.4f} m')

    def _on_slider_change(self, _val):
        for i, var in enumerate(self.slider_vars):
            self.joint_labels[i].config(text=f'{ARM_JOINTS[i][1]:.3f} / {var.get():.4f} / {ARM_JOINTS[i][2]:.3f}')
        if self.streaming.get():
            self.send_arm_goal(stream=True)

    def apply_preset(self):
        name = self.preset_var.get()
        vals = ARM_PRESETS.get(name, [])
        for v, val in zip(self.slider_vars, vals):
            v.set(val)
        self._log(f'Applied preset: {name}')

    def read_current(self):
        jpos = self.node.get_joint_positions()
        count = 0
        for i, (name, _, _) in enumerate(ARM_JOINTS):
            if name in jpos:
                lo, hi = ARM_JOINTS[i][1], ARM_JOINTS[i][2]
                v = jpos[name]
                v = max(lo, min(hi, v))
                self.slider_vars[i].set(v)
                count += 1
        self._log(f'Read {count} arm joint positions from /joint_states')

    def set_gripper(self, val):
        self.grip_var.set(val)
        self.send_gripper_goal()

    def clear_log(self):
        self.status_text.delete('1.0', tk.END)

    def update_ui(self):
        jpos = self.node.get_joint_positions()
        for i, (name, _, _) in enumerate(ARM_JOINTS):
            if name in jpos:
                self.live_labels[i].config(text=f'live: {jpos[name]:.4f}')
        if FINGER_JOINT in jpos:
            self.grip_live.config(text=f'live: {jpos[FINGER_JOINT]:.4f}')
        self.root.after(200, self.update_ui)

    def refresh_controllers(self):
        res = self.node.get_controller_status()
        if res is None:
            self._log('Controller manager not reachable')
            return
        for c in res.controller:
            self._log(f'{c.name}: {c.state} [{c.type}]')

    def cancel_arm(self):
        if self.node._arm_goal_handle is not None:
            self.node._arm_goal_handle.cancel_goal_async()
            self._log('Canceling arm trajectory goal...')
        else:
            self._log('No active arm goal to cancel')

    def cancel_move(self):
        if self.node._move_goal_handle is not None:
            self.node._move_goal_handle.cancel_goal_async()
            self._log('Canceling MoveGroup goal...')
        else:
            self._log('No active MoveGroup goal to cancel')

    def check_movegroup(self):
        if not self.node.move_client.wait_for_server(timeout_sec=2.0):
            self._log(f'MoveGroup not available: {self.node._move_action_name}')
            return
        self._log(f'MoveGroup reachable: {self.node._move_action_name}')

    def send_arm_goal(self, stream=False):
        if not self.node.arm_client.wait_for_server(timeout_sec=2.0):
            self._log('Arm action server not available')
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [n for n, _, _ in ARM_JOINTS]
        from trajectory_msgs.msg import JointTrajectoryPoint
        from builtin_interfaces.msg import Duration
        pt = JointTrajectoryPoint()
        pt.positions = [float(v.get()) for v in self.slider_vars]
        pt.velocities = [0.0] * 7
        dur = 0.8 if stream else float(self.arm_dur.get())
        pt.time_from_start = Duration(sec=int(dur), nanosec=int((dur - int(dur)) * 1e9))
        goal.trajectory.points = [pt]

        self._log(f'Sending arm goal ({len(pt.positions)} joints, {dur:.2f}s)...')
        future = self.node.arm_client.send_goal_async(goal)
        future.add_done_callback(self._arm_goal_sent)

    def _arm_goal_sent(self, future):
        handle = future.result()
        self.node._arm_goal_handle = handle
        if not handle.accepted:
            self._log('Arm goal rejected')
            return
        self._log('Arm goal accepted')
        result_future = handle.get_result_async()
        result_future.add_done_callback(self._arm_result)

    def _arm_result(self, future):
        try:
            res = future.result()
            self._log(f'Arm result: status={res.status}, error_code={res.result.error_code}')
        except Exception as e:
            self._log(f'Arm result error: {e}')

    def send_gripper_goal(self):
        if not self.node.grip_client.wait_for_server(timeout_sec=2.0):
            self._log('Gripper action server not available')
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(self.grip_var.get())
        goal.command.max_effort = float(self.grip_effort.get())
        self._log(f'Sending gripper goal: pos={goal.command.position:.4f} m, effort={goal.command.max_effort:.1f}')
        future = self.node.grip_client.send_goal_async(goal)
        future.add_done_callback(self._grip_goal_sent)

    def _grip_goal_sent(self, future):
        handle = future.result()
        self.node._grip_goal_handle = handle
        if not handle.accepted:
            self._log('Gripper goal rejected')
            return
        self._log('Gripper goal accepted')
        res_f = handle.get_result_async()
        res_f.add_done_callback(self._grip_result)

    def _grip_result(self, future):
        try:
            res = future.result()
            r = res.result
            self._log(f'Gripper result: pos={r.position:.4f}, reached={r.reached_goal}, stalled={r.stalled}')
        except Exception as e:
            self._log(f'Gripper result error: {e}')

    def _build_move_group_request(self, joint_goal=False, pose_goal=False):
        req = MotionPlanRequest()
        req.group_name = self.group_var.get()
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = float(self.vel_scale.get())
        req.max_acceleration_scaling_factor = float(self.acc_scale.get())

        if joint_goal and req.group_name in ['panda_arm', 'panda_arm_hand']:
            rs = RobotState()
            rs.joint_state.name = [n for n, _, _ in ARM_JOINTS]
            rs.joint_state.position = [float(v.get()) for v in self.slider_vars]
            req.start_state = RobotState()
            req.goal_constraints = []
            gc = Constraints()
            for name, pos in zip(rs.joint_state.name, rs.joint_state.position):
                jc = JointConstraint()
                jc.joint_name = name
                jc.position = float(pos)
                jc.tolerance_above = 0.01
                jc.tolerance_below = 0.01
                jc.weight = 1.0
                gc.joint_constraints.append(jc)
            req.goal_constraints.append(gc)
            return req

        if pose_goal and req.group_name == 'panda_arm':
            req.goal_constraints = []
            gc = Constraints()
            pc = PositionConstraint()
            pc.header.frame_id = self.cart_frame.get()
            pc.link_name = 'panda_link8'
            prim = SolidPrimitive()
            prim.type = SolidPrimitive.SPHERE
            prim.dimensions = [0.01]
            pc.constraint_region.primitives.append(prim)
            pv = Pose()
            pv.position.x = float(self.cart_x.get())
            pv.position.y = float(self.cart_y.get())
            pv.position.z = float(self.cart_z.get())
            pc.constraint_region.primitive_poses.append(pv)
            pc.weight = 1.0
            gc.position_constraints.append(pc)

            oc = OrientationConstraint()
            oc.header.frame_id = self.cart_frame.get()
            oc.link_name = 'panda_link8'
            oc.orientation.w = float(self.cart_qw.get())
            oc.orientation.x = float(self.cart_qx.get())
            oc.orientation.y = float(self.cart_qy.get())
            oc.orientation.z = float(self.cart_qz.get())
            oc.absolute_x_axis_tolerance = 0.1
            oc.absolute_y_axis_tolerance = 0.1
            oc.absolute_z_axis_tolerance = 0.1
            oc.weight = 1.0
            gc.orientation_constraints.append(oc)
            req.goal_constraints.append(gc)
            return req
        return req

    def _send_move(self, plan_only=True):
        if not self.node.move_client.wait_for_server(timeout_sec=3.0):
            self._log(f'MoveGroup not available: {self.node._move_action_name} (is move_group running?)')
            return
        goal = MoveGroup.Goal()
        goal.request = self._build_move_group_request(joint_goal=True)
        goal.planning_options.plan_only = plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        self._log(f'MoveIt: plan_only={plan_only}, group={goal.request.group_name}...')
        future = self.node.move_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._move_goal_sent(f, plan_only))

    def plan_to_sliders(self):
        self._send_move(plan_only=True)

    def plan_execute_to_sliders(self):
        self._send_move(plan_only=False)

    def plan_to_pose(self):
        if not self.node.move_client.wait_for_server(timeout_sec=3.0):
            self._log(f'MoveGroup not available: {self.node._move_action_name}')
            return
        goal = MoveGroup.Goal()
        goal.request = self._build_move_group_request(pose_goal=True)
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        self._log(f'MoveIt pose plan: group={goal.request.group_name}, frame={self.cart_frame.get()}...')
        future = self.node.move_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._move_goal_sent(f, True))

    def plan_execute_to_pose(self):
        if not self.node.move_client.wait_for_server(timeout_sec=3.0):
            self._log(f'MoveGroup not available: {self.node._move_action_name}')
            return
        goal = MoveGroup.Goal()
        goal.request = self._build_move_group_request(pose_goal=True)
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 5
        self._log(f'MoveIt pose plan+execute: group={goal.request.group_name}...')
        future = self.node.move_client.send_goal_async(goal)
        future.add_done_callback(lambda f: self._move_goal_sent(f, False))

    def _move_goal_sent(self, future, plan_only):
        handle = future.result()
        self.node._move_goal_handle = handle
        if not handle.accepted:
            self._log('MoveGroup goal rejected')
            return
        self._log('MoveGroup goal accepted, planning/executing...')
        res_f = handle.get_result_async()
        res_f.add_done_callback(lambda f: self._move_result(f, plan_only))

    def _move_result(self, future, plan_only):
        try:
            res = future.result()
            mr = res.result
            err = mr.error_code.val
            self._log(f'MoveIt done: error_code={err}, plan_only={plan_only}')
            if err == 1:
                self._log('MoveIt: SUCCESS')
            else:
                self._log(f'MoveIt error_code {err} (1=SUCCESS)')
        except Exception as e:
            self._log(f'MoveIt result error: {e}')

    def on_close(self):
        try:
            if self.stream_job is not None:
                self.root.after_cancel(self.stream_job)
        except Exception:
            pass
        self.root.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = FrankaGuiNode()
    root = tk.Tk()
    gui = FrankaTkGUI(root, node)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
