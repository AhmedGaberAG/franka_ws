"""Main window UI for the Franka Panda control GUI."""

from PySide6.QtCore import QTimer, Slot, Qt
from PySide6.QtWidgets import (
    QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QSlider, QDoubleSpinBox, QComboBox, QGroupBox, QTabWidget,
    QPlainTextEdit, QFrame, QMessageBox, QFormLayout, QLineEdit, QLayout
)

from .constants import ARM_JOINTS, PRESETS, FINGER_JOINT, GRIPPER_MIN, GRIPPER_MAX
from .widgets import JointRow


class MainWindow(QMainWindow):
    def __init__(self, node, log_callback, get_joint_positions_callback):
        super().__init__()
        self.node = node
        self._log = log_callback
        self._get_joint_positions = get_joint_positions_callback
        self.setWindowTitle("Franka Panda | Control Studio")
        self.resize(1180, 850)
        self.setMinimumSize(980, 720)
        self.joint_rows = []
        self.latest_joint_positions = {}
        self._build_ui()

        self.refresh_timer = QTimer(self)
        self.refresh_timer.timeout.connect(self.request_controller_refresh)
        self.refresh_timer.start(5000)
        QTimer.singleShot(700, self.request_controller_refresh)
        QTimer.singleShot(1200, self.check_action_servers)

    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        outer = QVBoxLayout(central)
        outer.setContentsMargins(18, 16, 18, 16)
        outer.setSpacing(12)

        # Header
        header = QFrame()
        header.setObjectName("Header")
        h = QHBoxLayout(header)
        h.setContentsMargins(18, 12, 18, 12)
        title_box = QVBoxLayout()
        title = QLabel("FRANKA  /  PANDA")
        title.setObjectName("Title")
        subtitle = QLabel("CONTROL STUDIO     •     ROS 2 HUMBLE     •     MOVEIT")
        subtitle.setObjectName("Subtitle")
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        h.addLayout(title_box)
        h.addStretch()
        self.connection_badge = QLabel("●  ROS ACTIVE")
        self.connection_badge.setStyleSheet("color:#63e6a5; font-weight:700;")
        h.addWidget(self.connection_badge)
        outer.addWidget(header)

        # Tabs
        self.tabs = QTabWidget()
        outer.addWidget(self.tabs, 1)

        # --- Arm Control Tab ---
        control_tab = QWidget()
        self.tabs.addTab(control_tab, "  Arm Control  ")
        root = QVBoxLayout(control_tab)
        root.setSpacing(12)

        arm_box = QGroupBox("ARM JOINTS   /   radians")
        arm_layout = QVBoxLayout(arm_box)
        preset_line = QHBoxLayout()
        preset_line.addWidget(QLabel("Preset"))
        self.preset_combo = QComboBox()
        self.preset_combo.addItems(PRESETS.keys())
        preset_line.addWidget(self.preset_combo)
        apply_btn = QPushButton("Apply to sliders")
        apply_btn.clicked.connect(self.apply_preset)
        preset_line.addWidget(apply_btn)
        read_btn = QPushButton("Read current")
        read_btn.clicked.connect(self.read_current)
        preset_line.addWidget(read_btn)
        preset_line.addStretch()
        arm_layout.addLayout(preset_line)

        for i, (name, low, high) in enumerate(ARM_JOINTS):
            row = JointRow(i, name, low, high)
            self.joint_rows.append(row)
            arm_layout.addWidget(row)

        root.addWidget(arm_box)

        actions = QHBoxLayout()
        self.duration = QDoubleSpinBox()
        self.duration.setRange(0.5, 30.0)
        self.duration.setSingleStep(0.5)
        self.duration.setValue(3.0)
        self.duration.setSuffix(" s")
        actions.addWidget(QLabel("Move duration"))
        actions.addWidget(self.duration)
        send_btn = QPushButton("Send Joint Goal")
        send_btn.setObjectName("Primary")
        send_btn.clicked.connect(self.send_joint_goal)
        actions.addWidget(send_btn, 2)
        plan_btn = QPushButton("MoveIt: Plan")
        plan_btn.clicked.connect(lambda: self.send_moveit(True))
        actions.addWidget(plan_btn)
        execute_btn = QPushButton("Plan + Execute")
        execute_btn.setObjectName("Primary")
        execute_btn.clicked.connect(lambda: self.send_moveit(False))
        actions.addWidget(execute_btn)
        stop_btn = QPushButton("Stop / Cancel")
        stop_btn.setObjectName("Danger")
        stop_btn.clicked.connect(self.cancel_all)
        actions.addWidget(stop_btn)
        root.addLayout(actions)

        lower = QHBoxLayout()

        # Gripper
        grip_box = QGroupBox("GRIPPER")
        grip_layout = QVBoxLayout(grip_box)
        self.grip_value = QDoubleSpinBox()
        self.grip_value.setRange(GRIPPER_MIN, GRIPPER_MAX)
        self.grip_value.setDecimals(3)
        self.grip_value.setSingleStep(0.001)
        self.grip_value.setValue(0.035)
        self.grip_value.setSuffix(" m")
        grip_layout.addWidget(QLabel("Finger opening"))
        grip_layout.addWidget(self.grip_value)
        grip_buttons = QHBoxLayout()
        open_btn = QPushButton("Open")
        open_btn.setObjectName("Success")
        open_btn.clicked.connect(lambda: self.send_gripper(GRIPPER_MAX))
        close_btn = QPushButton("Close")
        close_btn.clicked.connect(lambda: self.send_gripper(GRIPPER_MIN))
        send_grip = QPushButton("Send")
        send_grip.setObjectName("Primary")
        send_grip.clicked.connect(lambda: self.send_gripper(self.grip_value.value()))
        grip_buttons.addWidget(open_btn)
        grip_buttons.addWidget(close_btn)
        grip_buttons.addWidget(send_grip)
        grip_layout.addLayout(grip_buttons)
        effort_row = QHBoxLayout()
        effort_row.addWidget(QLabel("Max effort"))
        self.grip_effort = QDoubleSpinBox()
        self.grip_effort.setRange(0.0, 50.0)
        self.grip_effort.setValue(10.0)
        self.grip_effort.setSuffix(" N")
        effort_row.addWidget(self.grip_effort)
        grip_layout.addLayout(effort_row)
        lower.addWidget(grip_box, 1)

        # MoveIt Settings
        move_box = QGroupBox("MOVEIT SETTINGS")
        move_layout = QFormLayout(move_box)
        self.group_combo = QComboBox()
        self.group_combo.addItems(["panda_arm"])
        move_layout.addRow("Planning group", self.group_combo)
        self.velocity = QDoubleSpinBox()
        self.velocity.setRange(0.05, 1.0)
        self.velocity.setSingleStep(0.05)
        self.velocity.setValue(0.25)
        move_layout.addRow("Velocity scale", self.velocity)
        self.acceleration = QDoubleSpinBox()
        self.acceleration.setRange(0.05, 1.0)
        self.acceleration.setSingleStep(0.05)
        self.acceleration.setValue(0.25)
        move_layout.addRow("Acceleration scale", self.acceleration)
        self.action_status = QLabel("Waiting for action-server check…")
        self.action_status.setWordWrap(True)
        self.action_status.setStyleSheet("color:#8fa8ca;")
        move_layout.addRow("Actions", self.action_status)
        lower.addWidget(move_box, 1)
        root.addLayout(lower)

        # --- Cartesian Goal Tab ---
        pose_tab = QWidget()
        self.tabs.addTab(pose_tab, "  Cartesian Goal  ")
        pose_layout = QVBoxLayout(pose_tab)
        pose_box = QGroupBox("POSE TARGET   /   panda_link8")
        form = QFormLayout(pose_box)
        self.pose_frame = QLineEdit("panda_link0")
        form.addRow("Reference frame", self.pose_frame)
        self.pose_xyz = []
        xyz_row = QHBoxLayout()
        for axis, default in [("X", 0.4), ("Y", 0.0), ("Z", 0.4)]:
            spin = QDoubleSpinBox()
            spin.setRange(-1.0, 1.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.01)
            spin.setValue(default)
            spin.setPrefix(axis + " ")
            self.pose_xyz.append(spin)
            xyz_row.addWidget(spin)
        form.addRow("Position (m)", xyz_row)
        self.pose_quat = []
        quat_row = QHBoxLayout()
        for label, default in [("w", 1.0), ("x", 0.0), ("y", 0.0), ("z", 0.0)]:
            spin = QDoubleSpinBox()
            spin.setRange(-1.0, 1.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.01)
            spin.setValue(default)
            spin.setPrefix(label + " ")
            self.pose_quat.append(spin)
            quat_row.addWidget(spin)
        form.addRow("Quaternion", quat_row)
        pose_layout.addWidget(pose_box)
        pose_note = QLabel("Cartesian planning uses MoveIt constraints. Verify the reference frame, target reachability, and collision scene before executing.")
        pose_note.setWordWrap(True)
        pose_note.setStyleSheet("color:#91a7c6; padding:8px;")
        pose_layout.addWidget(pose_note)
        pose_buttons = QHBoxLayout()
        pose_plan = QPushButton("Plan to Pose")
        pose_plan.clicked.connect(lambda: self.send_pose_goal(True))
        pose_exec = QPushButton("Plan + Execute to Pose")
        pose_exec.setObjectName("Primary")
        pose_exec.clicked.connect(lambda: self.send_pose_goal(False))
        pose_buttons.addWidget(pose_plan)
        pose_buttons.addWidget(pose_exec)
        pose_buttons.addStretch()
        pose_layout.addLayout(pose_buttons)
        pose_layout.addStretch()

        # --- System Status Tab ---
        status_tab = QWidget()
        self.tabs.addTab(status_tab, "  System Status  ")
        status_layout = QVBoxLayout(status_tab)
        status_actions = QHBoxLayout()
        refresh = QPushButton("Refresh controllers")
        refresh.clicked.connect(self.request_controller_refresh)
        check = QPushButton("Check action servers")
        check.clicked.connect(self.check_action_servers)
        status_actions.addWidget(refresh)
        status_actions.addWidget(check)
        status_actions.addStretch()
        status_layout.addLayout(status_actions)
        self.controller_list = QPlainTextEdit()
        self.controller_list.setReadOnly(True)
        self.controller_list.setPlaceholderText("Controller status will appear here.")
        status_layout.addWidget(self.controller_list, 1)

        # --- Log Box ---
        log_box = QGroupBox("EVENT LOG")
        log_layout = QVBoxLayout(log_box)
        log_buttons = QHBoxLayout()
        clear = QPushButton("Clear log")
        clear.clicked.connect(lambda: self.log_view.clear())
        log_buttons.addStretch()
        log_buttons.addWidget(clear)
        log_layout.addLayout(log_buttons)
        self.log_view = QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.setMaximumBlockCount(500)
        self.log_view.setMinimumHeight(95)
        log_layout.addWidget(self.log_view)
        outer.addWidget(log_box)

    @Slot(str)
    def append_log(self, message):
        from datetime import datetime
        self.log_view.appendPlainText(f"[{datetime.now().strftime('%H:%M:%S')}] {message}")

    @Slot(dict)
    def on_joint_state(self, positions):
        self.latest_joint_positions = dict(positions)
        available = sum(1 for name, _, _ in ARM_JOINTS if name in positions)
        if available == 7:
            self.connection_badge.setText("●  JOINT STATES OK")
            self.connection_badge.setStyleSheet("color:#63e6a5; font-weight:700;")
        else:
            self.connection_badge.setText(f"●  JOINT STATES {available}/7")
            self.connection_badge.setStyleSheet("color:#ffca72; font-weight:700;")

    @Slot(object)
    def on_controller_status(self, controllers):
        if controllers is None:
            self.controller_list.setPlainText("Controller manager unavailable.\nCheck controller_manager parameter and ros2_control bringup.")
            return
        self.controller_list.setPlainText("\n".join(
            f"{name}\n  state: {state}\n  type:  {kind}" for name, state, kind in controllers
        ) or "No controllers returned.")

    def positions(self):
        return [row.value() for row in self.joint_rows]

    def apply_preset(self):
        name = self.preset_combo.currentText()
        for row, value in zip(self.joint_rows, PRESETS[name]):
            row.set_position(value)
        self.append_log(f"Preset applied to sliders: {name}. Press Send Goal to move the robot.")

    def read_current(self):
        positions = self._get_joint_positions()
        count = 0
        for row, (name, low, high) in zip(self.joint_rows, ARM_JOINTS):
            if name in positions:
                row.set_position(max(low, min(high, positions[name])))
                count += 1
        self.append_log(f"Read {count}/7 joint positions from /joint_states.")

    def send_joint_goal(self):
        if QMessageBox.question(
            self, "Confirm motion", "Send this joint target to the arm controller?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        ) != QMessageBox.Yes:
            return
        self.node.send_arm(self.positions(), self.duration.value())

    def send_moveit(self, plan_only):
        self.node.moveit_joint_goal(
            self.positions(), plan_only, self.group_combo.currentText(),
            self.velocity.value(), self.acceleration.value()
        )

    def send_gripper(self, position):
        self.grip_value.setValue(position)
        self.node.send_gripper(position, self.grip_effort.value())

    def cancel_all(self):
        self.node.cancel_arm()
        self.node.cancel_move()

    def request_controller_refresh(self):
        data = self.node.refresh_controllers()
        if data is not None:
            self.on_controller_status(data)

    def check_action_servers(self):
        self.node.check_action_servers()

    def send_pose_goal(self, plan_only):
        xyz = [v.value() for v in self.pose_xyz]
        quat = [v.value() for v in self.pose_quat]
        if QMessageBox.question(
            self, "Confirm Cartesian motion",
            "Submit this Cartesian pose target to MoveIt?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        ) != QMessageBox.Yes:
            return
        self.node.moveit_pose_goal(
            xyz, quat, self.pose_frame.text(), plan_only,
            self.velocity.value(), self.acceleration.value()
        )

    def closeEvent(self, event):
        self.refresh_timer.stop()
        event.accept()