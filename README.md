# Franka Panda — ROS 2 Simulation & Manipulation

Simulation and manipulation stack for the **Franka Emika Panda** 7-DoF arm, built with
**ROS 2 (Humble)**, **URDF/Xacro**, **Gazebo**, **ros2_control** and **MoveIt 2**.

The workspace is developed **incrementally**: each stage below is implemented, built and
verified before the next one is started.

## Preview

| Franka Emika Panda | Model in RViz |
| :---: | :---: |
| ![Franka Emika Panda](media/panda_real.jpg) | ![Panda URDF in RViz](media/panda_rviz.png) |

The photo of the real robot is by *Ims*, [Wikimedia Commons](https://commons.wikimedia.org/wiki/File:Franka_Emika2.jpg),
licensed under [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/).
The RViz screenshot shows the generated model at the ready pose.

---

## 1. Status

| Stage | Description | State |
| :--- | :--- | :--- |
| 0 | Workspace scaffolding, README, `.gitignore` | **Done** |
| 1 | Robot description (URDF/Xacro + meshes) | **Done** |
| 2 | Gazebo Sim + `ros2_control` hardware interface | **Done** |
| 3 | Controllers (joint trajectory, gripper) | **Done** |
| 4 | RViz / `robot_state_publisher` visualisation | **Done** |
| 5 | MoveIt 2 configuration (SRDF, kinematics, planners, MoveIt Setup Assistant) | Planned |
| 6 | Manipulation task nodes (e.g. pick & place) | Planned |
| 7 | Integration tests + CI | Planned |

---

## 2. Environment

Verified on the development machine:

| Component | Version | Notes |
| :--- | :--- | :--- |
| OS | Ubuntu 22.04 (Jammy) | Linux 6.8 |
| ROS 2 | Humble Hawksbill | `/opt/ros/humble` |
| Python | 3.10.12 | |
| Gazebo | Sim 6.18 (Ignition Fortress) | `ros_gz_sim`, `ign_ros2_control` |
| ros2_control | 2.54.3 | `controller_manager`, `ros2_controllers` |
| MoveIt 2 | 2.5.10 | `ros-humble-moveit` (via apt) |
| Robot description | local `franka_description` 0.1.0 | original URDF/Xacro, Panda meshes (Apache-2.0) |
| Controllers | local `franka_controller` 0.1.0 | `controllers.yaml` and controller spawners |

### 2.1 Dependencies to install

```bash
sudo apt update
sudo apt install \
    ros-humble-moveit \
    ros-humble-moveit-ros-perception \
    ros-humble-moveit-ros-visualization \
    ros-humble-moveit-ros-planning \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-configs-utils \
    ros-humble-moveit-resources-panda-moveit-config \
    ros-humble-joint-state-publisher-gui \
    ros-humble-ros2-controllers \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ign-ros2-control \
    ros-humble-xacro
```

> The workspace ships its own `franka_description` and `franka_controller`
> packages.  Do **not** install `ros-humble-franka-description` alongside them:
> both provide the same ament resource name and the overlay would shadow one of
> them.  Only the Panda meshes come from Franka Robotics (Apache-2.0); the
> URDF/Xacro, kinematics, controllers and launch files are reimplemented here,
> which keeps the model drop-in compatible with upstream while remaining easy
> to extend (payloads, camera mounts, additional grippers).

---

## 3. Workspace layout

```
franka_ws/
├── LICENSE
├── README.md
├── media/                      # README images
└── src/
    ├── franka_controller/
    │   ├── config/controllers.yaml      # controller types and parameters
    │   └── launch/controller.launch.py  # spawners for an existing manager
    └── franka_description/
        ├── urdf/                        # Xacro macros, ros2_control, Gazebo
        ├── launch/                      # RViz and Gazebo launch files
        ├── meshes/                      # visual and collision meshes
        └── rviz/panda.rviz              # RViz layout
```

The description and the controllers are separate packages: the URDF knows
nothing about how controllers are started, so the same description can drive
Gazebo, MoveIt or real hardware.  Planned: `franka_moveit` (Stage 5) and
`franka_manipulation` (Stage 6).

**Naming conventions**

- Packages: `franka_<domain>`
- Namespaces: `/franka`
- Frames: `world` (optional fixed base), `panda_link0` … `panda_link8`, `panda_hand`, `panda_hand_tcp`, `panda_grasp_frame`
- Controllers: `panda_arm_controller` (joint trajectory), `panda_hand_controller` (gripper)

---

## 4. Build & run

### 4.1 Build

```bash
cd ~/franka_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # first time only
colcon build --symlink-install
source install/setup.bash
```

### 4.2 Run

```bash
source install/setup.bash

# Stage 1 – inspect the model without a simulator
ros2 launch franka_description visualize_panda.launch.py

# Stage 2 – spawn the model in Gazebo Sim
ros2 launch franka_description gazebo.launch.py

# Stage 3 – load the controllers into the running controller manager
ros2 launch franka_controller controller.launch.py

# Stage 5+ – motion planning
ros2 launch franka_moveit moveit.launch.py
```

`visualize_panda.launch.py` accepts `prefix`, `hand`, `with_sc`, `gui`, `rviz`
and `use_sim_time`, e.g. `hand:=false gui:=false`.

`gazebo.launch.py` accepts `world`, `gui`, `rviz`, `hand` and `with_sc`.
The Gazebo flavour is detected from `ROS_DISTRO`: Ignition on Humble, and gz
with the Bullet Featherstone engine on Iron and newer.  It does not load any
controllers: run `controller.launch.py` in a second terminal afterwards.

`controller.launch.py` accepts `controller_manager`, `timeout` and `hand`.

### 4.3 Control the robot

With the controllers active, send a joint trajectory to the arm:

```bash
ros2 action send_goal /panda_arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{trajectory: {joint_names: [panda_joint1, panda_joint2, panda_joint3, panda_joint4, panda_joint5, panda_joint6, panda_joint7], points: [{positions: [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], time_from_start: {sec: 3}}]}}"
```

and open or close the gripper (position in metres):

```bash
ros2 action send_goal /panda_hand_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.04, max_effort: 10.0}}"
```

---

## 5. Development workflow

1. Branch off `main` with a descriptive name (`feature/moveit-config`, `fix/gripper-joint-limits`).
2. Build with `colcon build --symlink-install --packages-select <pkg>` while iterating.
3. Lint: `ament_uncrustify`, `ament_clang_format`, `ament_cpplint` (added in Stage 7).
4. Commit small, logically scoped changes; run a build + smoke test before pushing.

---

## 6. Troubleshooting

| Symptom | Likely cause / fix |
| :--- | :--- |
| `Package 'franka_...' not found` | `source /opt/ros/humble/setup.bash` **and** `source install/setup.bash` |
| Gazebo opens blank / errors on meshes | make sure the package is sourced, `package://` URIs are resolved from the ament index |
| Gazebo Sim rejects the model | check `gz sim` output; the usual cause is an invalid inertia tensor or a missing plugin |
| `ros2_control` controller fails to activate | `controller_manager` did not load the hardware plugin — check the `<ros2_control>` tag and `robot_description` remapping |
| `No controllers are currently loaded` | `gazebo.launch.py` does not start any; run `ros2 launch franka_controller controller.launch.py` |
| Spawner cannot contact `/controller_manager` | the model has not been spawned yet — start Gazebo first or raise `timeout:=` |
| RViz shows a broken robot | `robot_state_publisher` is not publishing TF; run `ros2 run tf2_tools view_frames` to inspect |
| MoveIt says "no kinematics solver" | `kinematics.yaml` is missing or the group name does not match the SRDF group |
| `colcon build` fails on a stale package | `rm -rf build install log` and rebuild |

Useful diagnostics:

```bash
ros2 pkg list | grep franka
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic list
ros2 topic echo /joint_states
```

---

## 7. References

- [Franka Emika Panda documentation](https://frankaemika.github.io/docs/)
- [MoveIt 2 tutorials](https://moveit.picknik.ai/main/index.html)
- [ros2_control — controller manager](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)
- [Gazebo Sim / ros_gz](https://gazebosim.org/docs)
- [Franka ROS 2 (franka_ros2)](https://github.com/frankarobotics/franka_ros2)

---

## 8. License

The code, URDF/Xacro files, launch files and documentation in this repository
are released under the [MIT License](LICENSE).

Third-party assets keep their original licenses:

- **Meshes** — `src/franka_description/meshes/` contains the Franka Emika Panda
  visual and collision meshes, Copyright 2023 Franka Robotics GmbH, obtained
  from [frankarobotics/franka_ros2](https://github.com/frankarobotics/franka_ros2)
  and licensed under the [Apache License 2.0](src/franka_description/meshes/LICENSE).
  The files are redistributed unmodified; see [NOTICE](NOTICE) and
  [`meshes/NOTICE`](src/franka_description/meshes/NOTICE).
- **Preview photo** — `media/panda_real.jpg` by *Ims*, licensed under
  [CC BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/).
