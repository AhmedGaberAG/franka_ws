"""Constants for Franka Panda control GUI."""

# Joint limits from franka_description/urdf/properties.xacro
ARM_JOINTS = [
    ("panda_joint1", -2.8973, 2.8973),
    ("panda_joint2", -1.7628, 1.7628),
    ("panda_joint3", -2.8973, 2.8973),
    ("panda_joint4", -3.0718, -0.0698),
    ("panda_joint5", -2.8973, 2.8973),
    ("panda_joint6", -0.0175, 3.7525),
    ("panda_joint7", -2.8973, 2.8973),
]

# Safe, commonly used configurations. Every value is within the limits above.
PRESETS = {
    "Ready": [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785],
    "Transport": [0.0, -0.5599, 0.0, -2.97, 0.0, 0.0, 0.785],
    "Home (safe)": [0.0, -0.3, 0.0, -1.8, 0.0, 1.5, 0.0],
    "Extended (caution)": [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.785],
}

FINGER_JOINT = "panda_finger_joint1"
GRIPPER_MIN, GRIPPER_MAX = 0.0, 0.04