# Assignment 2: UR3 Manipulation & Training

## Package Overview
This package contains the UR3 robot description, controllers, simulation environment, and training setup for COMP9069 Robotics & Autonomous Systems Assignment 2.

## Tasks Completion Status

### Task 1: UR3 Description [10%]
**Status**: Complete

Completed:
- Created CMake package `ur3_description`
- Copied Assignment2.zip contents into package
- Added install directives to CMakeLists.txt
- Fixed URDF configuration for RViz visualization
- Robot successfully displays in RViz via `ur3_rviz.launch.py`
- Added joint state broadcaster controller
- Added forward command controller
- Added joint trajectory controller
- Configured controller limits (±2π for all joints except elbow: ±π)
- Created Gazebo launch file with controllers
- Spawned table at (0, 0, 0)
- Spawned cube at (0, 0.5, 1)
- Spawned robot at (0, 0, 1)
- Verified controller functionality via terminal commands

### Task 2: UR3 Manipulation [20%]
**Status**: Not Started

To Do:
- Create package `ur3_training`
- Create node for interactive joint movement (forward command controller)
- Create node for interactive joint movement (joint trajectory controller)
- Implement user input loop for joint selection and movement
- Monitor movement completion
- Print final joint position, error, and end effector pose

### Task 3: UR3 Training [70%]
**Status**: Not Started

To Do:
- Create custom Gymnasium environment
- Implement action space (27 discrete actions for 3 joints)
- Implement observation space (joint positions, end effector xy, cube xy)
- Normalize observation space
- Implement reward function (-1 per step, +100 for goal)
- Implement episode termination/truncation logic
- Implement environment reset with random cube positioning
- Configure Gazebo bridge for `/world/empty/set_pose` service
- Train robot with Q-learning
- Train robot with DQN

## Build Instructions

```bash
cd ~/ros2_ws
colcon build --packages-select ur3_description
source install/setup.bash
```

## Usage

### View Robot in RViz
```bash
ros2 launch ur3_description ur3_rviz.launch.py
```

### Launch Gazebo Simulation
```bash
ros2 launch ur3_description ur3_gazebo.launch.py
```

### Control Robot Joints (via terminal)
```bash
# Move to specific joint positions
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.5, -0.5, 0.3, 0.0, 0.0, 0.0]}"

# Return to zero position
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}"

# Bend elbow
ros2 topic pub --once /forward_position_controller/commands std_msgs/msg/Float64MultiArray "{data: [0.0, -1.0, 1.5, 0.0, 0.0, 0.0]}"
```

## Package Structure

```
ur3_description/
├── config/              # Controller and bridge configurations
├── launch/              # Launch files for RViz and Gazebo
├── meshes/              # Visual and collision meshes
│   ├── visual/
│   └── collision/
├── models/              # Gazebo models (table, cube)
├── rviz/                # RViz configuration
├── urdf/                # Robot URDF/xacro files
├── CMakeLists.txt
├── package.xml
└── README.md
```

## Dependencies

- ROS 2 Humble
- Gazebo
- ros2_control
- ros_gz (ROS-Gazebo bridge)
- joint_state_publisher_gui
- robot_state_publisher

## Notes

### Task 1 Bug Fix
Fixed issue where robot was not visible in RViz:
- Changed launch file to use `ur3.urdf.xacro` instead of `ur3_description.urdf.xacro`
- Fixed robot name attribute in URDF from `$(arg name)` to `ur3` (resolved xacro argument ordering issue)

## Author
Vlad - COMP9069 MSc AI
