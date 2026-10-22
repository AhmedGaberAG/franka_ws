# Franka Panda — ROS 2 Simulation & Manipulation

Simulation and manipulation stack for the **Franka Emika Panda** 7-DoF arm, built with
**ROS 2 (Humble)**, **URDF/Xacro**, **Gazebo**, **ros2_control** and **MoveIt 2**.

| Franka Emika Panda | Model in RViz |
| :---: | :---: |
| ![Franka Emika Panda](media/panda_real.jpg) | ![Panda URDF in RViz](media/panda_rviz.png) |

The photo of the real robot is by *Ims*, [Wikimedia Commons](https://commons.wikimedia.org/wiki/File:Franka_Emika2.jpg),
licensed under [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/).
The RViz screenshot shows the generated model at the ready pose.

---

## Quick start

```bash
cd ~/franka_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install      # first time: rosdep install --from-paths src --ignore-src -r -y
source install/setup.bash

# Whole stack: Gazebo + controllers + MoveIt (RViz)
ros2 launch franka_bringup bringup.launch.py

# Headless (no Gazebo GUI, no RViz)
ros2 launch franka_bringup bringup.launch.py use_gazebo_gui:=false rviz:=false
```

| Goal | Command |
| :--- | :--- |
| Inspect the model (no simulator) | `ros2 launch franka_description display.launch.py` |
| Gazebo only | `ros2 launch franka_description gazebo.launch.py` |
| Load controllers | `ros2 launch franka_controller controller.launch.py` |
| MoveIt planning only | `ros2 launch franka_moveit moveit.launch.py` |
| Full stack | `ros2 launch franka_bringup bringup.launch.py` |
| Pause / resume belt | `ros2 topic pub --once /conveyor/pause std_msgs/msg/Bool '{data: true}'` |

Handy diagnostics:

```bash
ros2 control list_controllers
ros2 topic echo /joint_states
ros2 topic list | grep detection_camera
ros2 action list | grep -E "move_action|gripper_cmd"
```

---

## Environment

Verified on the development machine:

| Component | Version |
| :--- | :--- |
| OS | Ubuntu 22.04 (Jammy) |
| ROS 2 | Humble Hawksbill (`/opt/ros/humble`) |
| Gazebo | Sim 6.18 (Ignition Fortress); `ros_gz_sim`, `ign_ros2_control` |
| ros2_control | 2.54.3 |
| MoveIt 2 | 2.5.10 |
| Local packages | `franka_description`, `franka_controller`, `franka_moveit`, `franka_bringup` |

### Dependencies

```bash
sudo apt update
sudo apt install \
    ros-humble-moveit \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-controllers \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ign-ros2-control \
    ros-humble-xacro
```

> The workspace ships its own `franka_description`, `franka_controller` and
> `franka_moveit`. Do **not** install `ros-humble-franka-description` alongside
> `franka_description` — both provide the same ament resource name and the
> overlay would shadow one of them. Only the Panda meshes come from Franka
> Robotics (Apache-2.0); the URDF/Xacro, kinematics, controllers and launch
> files are reimplemented here.

---

## Workspace layout

```
franka_ws/
├── media/                                  # README images
└── src/
    ├── franka_bringup/launch/bringup.launch.py   # Gazebo + controllers + MoveIt
    ├── franka_controller/
    │   ├── config/controllers.yaml               # controller types and parameters
    │   └── launch/controller.launch.py           # spawners for an existing manager
    ├── franka_description/
    │   ├── urdf/                                 # Xacro macros, ros2_control, Gazebo
    │   ├── launch/                               # display + gazebo launch files
    │   ├── scripts/conveyor_control.py           # belt stop/resume logic
    │   ├── meshes/                               # visual and collision meshes
    │   ├── models/                               # conveyor, cubes, bins, camera, shelves
    │   ├── worlds/warehouse.world                # warehouse + conveyor cell
    │   └── rviz/panda.rviz
    └── franka_moveit/
        ├── config/                               # SRDF, KDL, limits, controllers, OMPL
        └── launch/moveit.launch.py               # move_group + MotionPlanning
```

The packages are layered: the description knows nothing about controllers, the
controllers know nothing about planning, and `franka_bringup` only composes
them. The same description can drive Gazebo, MoveIt or real hardware.

**Conventions** — packages `franka_<domain>`; namespace `/franka`; frames
`panda_link0`…`panda_link8`, `panda_hand`, `panda_hand_tcp`, `panda_grasp_frame`;
controllers `panda_arm_controller` (joint trajectory) and `panda_hand_controller`
(gripper).

---

## Launch files

| File | Starts | Key arguments |
| :--- | :--- | :--- |
| `franka_description/display.launch.py` | RViz + `robot_state_publisher` (no sim) | `prefix`, `hand`, `with_sc`, `gui`, `rviz`, `use_sim_time` |
| `franka_description/gazebo.launch.py` | Gazebo world, bridges, spawns the Panda | `world_name`, `use_gazebo_gui`, `use_sim_time`, `rviz`, `hand`, `with_sc` |
| `franka_controller/controller.launch.py` | Controller spawners | `controller_manager`, `timeout`, `hand` |
| `franka_moveit/moveit.launch.py` | `move_group` + MotionPlanning | `rviz`, `hand`, `use_sim_time` |
| `franka_bringup/bringup.launch.py` | All of the above | `world_name`, `use_gazebo_gui`, `rviz`, `hand`, `with_sc`, `use_sim_time` |

`gazebo.launch.py` loads `worlds/<world_name>.world` and detects the Gazebo
flavour from `ROS_DISTRO` (Ignition on Humble; gz with the Bullet Featherstone
engine on Iron+). It starts no controllers — run `controller.launch.py`
afterwards in a second terminal. `moveit.launch.py` also starts no robot, TF or
controllers; it connects to the running stack with `moveit_manage_controllers:
false` so the spawners stay in charge.

---

## The warehouse world

The default `warehouse` world is a walled warehouse containing:

- a **conveyor cell**: a 3.6 m belt, seven shuffled green/red/blue cubes that
  advance to the pick station, three colour-matched bins, and an overhead RGB
  detection camera;
- warehouse set dressing: columns, four shelving units, barrels, buckets and a
  pallet jack.

The Panda spawns at the origin in front of the belt.

### Conveyor cell

The belt is static geometry; cube motion is driven by a `TrajectoryFollower`
plugin in each cube model because the Fortress DART engine here lacks the
contact-surface customisation `TrackController` needs. All cube followers share
one pause topic, `/conveyor/pause`. The `conveyor_control` node watches the
Gazebo pose stream and:

- **pauses** the belt when the frontmost on-belt cube reaches `x ≈ 0.7 m`;
- **resumes** once that cube is lifted or removed from the belt.

So the feed stops at the pick station and advances the next cube only after the
front one is picked. To command the belt manually (e.g. before grasping):

```bash
ros2 topic pub --once /conveyor/pause std_msgs/msg/Bool '{data: true}'   # stop
ros2 topic pub --once /conveyor/pause std_msgs/msg/Bool '{data: false}'  # run
```

Camera topics (raw Gazebo transport):

```bash
ign topic -l | grep detection_camera
# /detection_camera/image
# /detection_camera/camera_info
```

---

## Control the robot

Send a joint trajectory to the arm:

```bash
ros2 action send_goal /panda_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], points: [{positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], time_from_start: {sec: 3}}]}}"
```

Open or close the gripper (position in metres):

```bash
ros2 action send_goal /panda_hand_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.04, max_effort: 10.0}}"
```

### Plan and execute with MoveIt

With all three launches running, use the **MotionPlanning** panel in RViz:

1. Select the planning group `panda_arm`.
2. Choose a goal state (`ready`, `extended`, `transport`) or drag the marker.
3. Press **Plan**, inspect the trajectory, then **Plan & Execute**.

The SRDF also defines the `panda_hand` and `panda_arm_hand` groups; the gripper
opens/closes through `panda_hand_controller`. Collision checking uses the STL
meshes; no octomap sensors are configured, so move_group logs
`No 3D sensor plugin(s) defined` once — this is expected.

---

## Development workflow

1. Branch off `main` with a descriptive name (`feature/moveit-config`).
2. Iterate with `colcon build --symlink-install --packages-select <pkg>`.
3. Lint: `ament_uncrustify`, `ament_clang_format`, `ament_cpplint`.
4. Commit small, logically scoped changes; build + smoke test before pushing.

---

## Troubleshooting

| Symptom | Likely cause / fix |
| :--- | :--- |
| `Package 'franka_...' not found` | source `/opt/ros/humble/setup.bash` **and** `install/setup.bash` |
| Gazebo blank / mesh errors | source the workspace; `package://` resolves from the ament index |
| Gazebo rejects the model | check `gz sim` output — usually an invalid inertia or missing plugin |
| `No controllers are currently loaded` | `gazebo.launch.py` starts none; run `controller.launch.py` |
| Spawner can't reach `/controller_manager` | model not spawned yet — start Gazebo first or raise `timeout:=` |
| RViz shows a broken robot | `robot_state_publisher` not publishing TF; check with `ros2 run tf2_tools view_frames` |
| move_group has no controllers | start `controller.launch.py`; action names must match `moveit_controllers.yaml` |
| `model://` assets not found | source the workspace; `gazebo.launch.py` sets `GZ_SIM_RESOURCE_PATH` |
| `colcon build` fails on a stale package | `rm -rf build install log` and rebuild |
| `No 3D sensor plugin(s) defined` | expected — no octomap/camera plugin configured |
| `libEGL warning: egl: failed to create dri2 screen` | harmless in headless / no-GPU environments |

---

## References

- [Franka Emika Panda documentation](https://frankaemika.github.io/docs/)
- [MoveIt 2 tutorials](https://moveit.picknik.ai/main/index.html)
- [ros2_control — controller manager](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)
- [Gazebo Sim / ros_gz](https://gazebosim.org/docs)
- [Franka ROS 2 (franka_ros2)](https://github.com/frankarobotics/franka_ros2)

---

## License

The code, URDF/Xacro files, launch files and documentation in this repository
are released under the [MIT License](LICENSE).

Third-party assets keep their original licenses:

- **Meshes** — `src/franka_description/meshes/` are the Franka Emika Panda
  meshes, Copyright 2023 Franka Robotics GmbH, from
  [frankarobotics/franka_ros2](https://github.com/frankarobotics/franka_ros2),
  licensed under [Apache 2.0](src/franka_description/meshes/LICENSE). See
  [NOTICE](NOTICE) and [`meshes/NOTICE`](src/franka_description/meshes/NOTICE).
- **Preview photo** — `media/panda_real.jpg` by *Ims*,
  [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/).
- **Warehouse props** — the `aws_robomaker_warehouse_*` shelf, pallet-jack and
  bucket models under `src/franka_description/models/` come from the
  [OpenRobotics industrial-warehouse world](https://fuel.gazebosim.org/1.0/OpenRobotics/worlds/industrial-warehouse)
  on Gazebo Fuel, [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/);
  redistributed unmodified apart from removed thumbnails and metadata. The
  warehouse world, conveyor, cubes, bins and camera are original assets.
