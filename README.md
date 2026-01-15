# Robotics (ROS 2) Portfolio — Topics, Services, Actions + UR3 (Gazebo / ros2_control)

This repository is a **ROS 2 workspace** (`src/` contains multiple packages). It is both a learning repo and a portfolio
showing how I design and implement ROS 2 systems end‑to‑end:

- ROS 2 communication patterns: **Topics / Services / Actions**
- **Custom ROS interfaces** (`msg`, `srv`, `action`)
- A multi-node control system using **turtlesim** (assignment-style mini system)
- **UR3** robot description (**URDF/Xacro**) + simulation in **RViz & Gazebo**
- **ros2_control** configuration and **joint controllers** (position and trajectory)
- Basic software engineering hygiene: clean package structure, launch files, and tests

---

## Quick start

### Prerequisites
- Ubuntu 22.04 + **ROS 2 Humble**
- `colcon`, `rosdep`
- For UR3 simulation: Gazebo + ROS integration, `ros2_control` stack

### Build the workspace
From the repository root:

```bash
source /opt/ros/humble/setup.bash
# Optional but recommended:
# rosdep install --from-paths src -y --ignore-src

colcon build
source install/setup.bash
```

---

## Repository layout

```text
.
├── bag1/                      # Example rosbag2 recording (db3 + metadata)
└── src/                       # ROS 2 packages (this is a ROS 2 workspace)
    ├── custom_interfaces/      # Custom msg/srv/action definitions used across the repo
    ├── lab03_topics/           # Topic-based examples (pub/sub)
    ├── lab04_services/         # Service-based examples (client/server)
    ├── lab05_actions/          # Action-based examples (server/client)
    ├── turtle_controller_assignment/  # Assignment-style multi-node turtlesim system + launch + tests
    ├── ur3_description/        # UR3 URDF/Xacro + Gazebo world + ros2_control config + RViz config
    └── ur3_training/           # UR3 joint position & trajectory control nodes + launch
```

> Tip: each package has its own `README.md` (and the turtlesim assignment has a `Readme.md`) with extra details.

---

## What to look at first (high-signal demos)

1) **UR3 simulation + control**
- `src/ur3_description/` (URDF/Xacro + Gazebo/RViz launches + ros2_control YAML)
- `src/ur3_training/` (interactive joint position & trajectory controllers)

2) **Multi-node turtlesim system**
- `src/turtle_controller_assignment/` (launch orchestration, services, actions, tests)

3) **Core ROS 2 patterns**
- `src/lab03_topics/`, `src/lab04_services/`, `src/lab05_actions/`
- `src/custom_interfaces/` (interfaces used by the above)

---

## Packages (what they do, and which skills they demonstrate)

### `custom_interfaces`
Custom ROS interfaces used by other packages:
- `msg/Sphere.msg`, `msg/Time.msg`
- `srv/CircleDuration.srv`
- `srv/FindClosestTurtle.srv`
- `action/MoveTurtle.action`

**Skills demonstrated:** interface design, reusable ROS packages, cross-package integration.

---

### `lab03_topics` (Topics / pub-sub)
Python (`rclpy`) nodes showing basic topic communication:
- `talker.py`, `listener.py`
- `twist_relay.py` (working with `geometry_msgs/Twist`)
- `print_pose.py`, `move_distance.py` (turtlesim control via topics)
- `time_publisher.py` / `time_listener.py` (custom msg)
- `sphere_publisher.py` / `sphere_listener.py` (custom msg)

Run example:

```bash
ros2 run lab03_topics talker
# In another terminal:
ros2 run lab03_topics listener
```

**Skills demonstrated:** pub/sub, message types (standard + custom), ROS node structure in Python.

---

### `lab04_services` (Services / client-server)
Service servers and clients:
- `service_member_function.py`, `service_client.py`
- `move_turtle_arc_server.py`, `move_turtle_arc_client.py`

Run example (with turtlesim):

```bash
ros2 run turtlesim turtlesim_node
# In another terminal:
ros2 run lab04_services move_turtle_arc_server
# In another terminal:
ros2 run lab04_services move_turtle_arc_client
```

**Skills demonstrated:** service design, synchronous requests, turtlesim control via services.

---

### `lab05_actions` (Actions / goals + feedback + result)
Action server/client examples:
- `fibonacci_action_server.py`, `fibonacci_action_client.py`
- `move_turtle_action_server.py`, `move_turtle_action_client.py` (goal + feedback loop)

Run example (with turtlesim):

```bash
ros2 run turtlesim turtlesim_node
# In another terminal:
ros2 run lab05_actions move_turtle_action_server
# In another terminal:
ros2 run lab05_actions move_turtle_action_client
```

**Skills demonstrated:** actions, goal handling, feedback/result patterns, cancellation/timeouts (where implemented).

---

### `turtle_controller_assignment` (multi-node turtlesim system)
Assignment-style ROS system built around turtlesim:
- name management, spawning, monitoring
- closest-target service
- collection orchestration with an action server + client
- launch files:
  - `launch/assignment_launch.py`
  - `launch/assignment_launch_2s.py`
- tests in `test/`

Run:

```bash
ros2 launch turtle_controller_assignment assignment_launch.py
```

**Skills demonstrated:** multi-node system design, orchestration via launch, services + actions integration, testing.

---

### `ur3_description` (UR3 URDF/Xacro + Gazebo + ros2_control)
Contains:
- UR3 model (`urdf/*.xacro`) + meshes
- controller configuration (`config/ur3_controllers.yaml`)
- launch:
  - `launch/ur3_rviz.launch.py`
  - `launch/ur3_gazebo.launch.py`
- example assets: `Gazebo.png`, `RViz_rqt.png`

Run:

```bash
ros2 launch ur3_description ur3_rviz.launch.py
# or
ros2 launch ur3_description ur3_gazebo.launch.py
```

Example joint command (forward position controller):

```bash
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray \
"{data: [0.5, -0.5, 0.3, 0.0, 0.0, 0.0]}"
```

**Skills demonstrated:** robot description (URDF/Xacro), simulation setup, ros2_control configuration, controller tuning basics.

---

### `ur3_training` (UR3 control: position vs trajectory)
Interactive controllers:
- `joint_position_control.py` (position control)
- `joint_trajectory_control.py` (trajectory control)

Launch:

```bash
ros2 launch ur3_training ur3_training.launch.py
```

Run a controller node:

```bash
ros2 run ur3_training joint_position_control
# or
ros2 run ur3_training joint_trajectory_control
```

**Skills demonstrated:** integrating with ros2_control controllers, publishing commands, basic motion workflows.

---

## Rosbag (`bag1/`)
This repo includes an example `rosbag2` recording.

```bash
ros2 bag info bag1
ros2 bag play bag1
```

**Skills demonstrated:** recording/replaying data for debugging and reproducible experiments.

---

## Skills checklist (portfolio summary)

- ✅ ROS 2 fundamentals: topics / services / actions (Python `rclpy`)
- ✅ Custom interfaces (msg/srv/action) and reuse across packages
- ✅ Launch orchestration for multi-node systems
- ✅ Turtlesim used as a controlled testbed (integration + behaviour)
- ✅ UR3 description in URDF/Xacro, RViz/Gazebo simulation
- ✅ ros2_control controllers (position and trajectory workflows)
- ✅ Basic testing and packaging practices

---

## Author
Vladyslav Rastvorov

