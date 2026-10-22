# Franka Panda — ROS 2 Simulation & Manipulation

Simulation and manipulation stack for the **Franka Emika Panda** 7-DoF arm, built with
**ROS 2 (Humble)**, **URDF/Xacro**, **Gazebo**, **ros2_control** and **MoveIt 2**.

The workspace is developed **incrementally**: each stage below is implemented, built and
verified before the next one is started.

---

## 1. Status

| Stage | Description | State |
| :--- | :--- | :--- |
| 0 | Workspace scaffolding, README, `.gitignore` | **Done** |
| 1 | Robot description (URDF/Xacro + meshes) | Planned |
| 2 | Gazebo simulation + `ros2_control` hardware interface | Planned |
| 3 | Controllers (joint trajectory, gripper, effort) | Planned |
| 4 | RViz / `robot_state_publisher` visualisation | Planned |
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
| Gazebo | 11.10.2 (Gazebo Classic) | `gazebo_ros`, `gazebo_plugins` |
| ros2_control | 2.54.3 | `controller_manager`, `ros2_controllers` |
| MoveIt 2 | 2.5.10 | `ros-humble-moveit` (via apt) |
| Robot description | local `franka_description` 0.1.0 | meshes and kinematics derived from upstream 1.0.1 |

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
    ros-humble-xacro
```

> The workspace ships its own `franka_description` package.  Do **not** install
> `ros-humble-franka-description` alongside it: both provide the same ament
> resource name and the overlay would shadow one of them.  The local package
> reuses the official Franka meshes and kinematics so it stays drop-in
> compatible with upstream, while remaining easy to extend (payloads, camera
> mounts, additional grippers).

---

## 3. Workspace layout

```
franka_ws/
├── .gitignore
├── README.md
└── src/
    ├── franka_description/     # Stage 1 – URDF/Xacro, meshes, launch, RViz
    ├── franka_gazebo/          # Stage 2 – Gazebo world, ros2_control tags
    ├── franka_control/         # Stage 3 – controller configs + spawners
    ├── franka_moveit/          # Stage 5 – SRDF, kinematics, OMPL, MoveIt configs
    ├── franka_manipulation/    # Stage 6 – pick & place / task nodes
    └── franka_bringup/         # Stage 4 – launch files, RViz configs
```

**Naming conventions**

- Packages: `franka_<domain>`
- Namespaces: `/franka`
- Frames: `base_link`, `panda_link0` … `panda_link8`, `panda_hand`, `panda_grasp_frame`
- Controllers: `arm_controller` (joint trajectory), `gripper_controller` (position)

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

# Stage 2+ – spawned in Gazebo
ros2 launch franka_bringup gazebo.launch.py

# Stage 5+ – motion planning
ros2 launch franka_moveit moveit.launch.py
```

`visualize_panda.launch.py` accepts `prefix`, `hand`, `with_sc`, `gui` and
`rviz`, e.g. `hand:=false gui:=false`.  The remaining launch files are
placeholders that arrive with their stages.

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
| Gazebo opens blank / errors on meshes | `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$HOME/franka_ws/install/franka_description/share` |
| `ignition`/`gz` errors | Project targets **Gazebo Classic 11** (`gazebo_ros`). Do not mix with `ros_gz_*` APIs. |
| `ros2_control` controller fails to activate | `controller_manager` did not load the hardware plugin — check the `<ros2_control>` tag and `robot_description` remapping |
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
- [ros2_control — Gazebo Classic integration](https://control.ros.org/humble/doc/ros2_control/controller_manager/doc/userdoc.html)
- [Franka description (franka_description)](https://github.com/frankaemika/franka_ros)

---

## 8. License

Released under the [MIT License](LICENSE).
