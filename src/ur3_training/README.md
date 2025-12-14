# UR3 Training Package

## Package Overview
This package implements two different control approaches for the UR3 robotic arm:
1. Direct joint position control using forward command controller
2. Smooth trajectory control using joint trajectory controller

Both controllers allow interactive joint manipulation with real-time feedback.

## Package Structure

```
ur3_training/
├── launch/
│   └── ur3_training.launch.py  # Launch file that starts Gazebo simulation with configurable controller
├── ur3_training/
│   ├── __init__.py
│   ├── joint_position_control.py    # Node for forward position controller
│   └── joint_trajectory_control.py  # Node for joint trajectory controller
├── test/
│   ├── test_copyright.py
│   ├── test_flake8.py
│   ├── test_pep257.py
│   └── test_ur3_control.py          # Unit tests for control nodes
├── package.xml
├── setup.py
└── README.md
```

## Prerequisites

The `ur3_description` package must be present in the same workspace, as it provides:
- UR3 robot models and simulation environment
- Controller configurations for forward_position_controller and joint_trajectory_controller

## Build Instructions

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ur3_training
source install/setup.bash
```

## Quick Start

### Launch with Default Controller (Forward Position)

First, launch the simulation in one terminal:
```bash
ros2 launch ur3_training ur3_training.launch.py
```

Then in another terminal, source the environment and run the controller:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ur3_training joint_position_control
```

### Launch with Trajectory Controller

First, launch the simulation with trajectory controller in one terminal:
```bash
ros2 launch ur3_training ur3_training.launch.py controller:=joint_trajectory_controller
```

Then in another terminal, source the environment and run the controller:
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ur3_training joint_trajectory_control
```

## Controller Switching Guide

Since both controllers manage the same joints, only one can be active at a time.

### Method 1: Using Launch Parameters (Recommended)

Stop any running launch files, then launch with desired controller:

```bash
# For forward position controller
ros2 launch ur3_training ur3_training.launch.py controller:=forward_position_controller

# For joint trajectory controller
ros2 launch ur3_training ur3_training.launch.py controller:=joint_trajectory_controller
```

### Method 2: Manual Controller Switching

If simulation is already running:

```bash
# Switch to forward position controller
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller

# Switch to joint trajectory controller
ros2 control switch_controllers --deactivate forward_position_controller --activate joint_trajectory_controller
```

### Method 3: Load and Activate Controllers Manually

If controllers are not loaded:

```bash
# Load controllers (they will be inactive)
ros2 control load_controller forward_position_controller
ros2 control load_controller joint_trajectory_controller

# Activate one controller and deactivate the other
ros2 control switch_controllers --deactivate joint_trajectory_controller --activate forward_position_controller
```

## Using the Control Nodes

Both nodes operate identically:

1. View current joint positions
2. Select joint to move (1-6) or exit (0)
3. Enter movement amount in radians (positive/negative)
4. System executes movement and reports:
   - Final joint position
   - Position error
   - End-effector pose (x,y,z position and orientation)

## Node Details

### Joint Position Control
- Direct joint control via `/forward_position_controller/commands`
- Immediate response to user inputs
- Uses `Float64MultiArray` messages

### Joint Trajectory Control
- Smooth motion via `/joint_trajectory_controller/follow_joint_trajectory`
- Acceleration/deceleration profiles
- Uses ROS 2 action interface

## Troubleshooting

1. **Controller conflicts**: Only one controller active at once
2. **"Controller not found"**: Ensure simulation is running and controllers are loaded
3. **No response to inputs**: Check controller activation status
4. **TF errors**: Verify robot_state_publisher is running

## Dependencies

- ROS 2 Humble
- ur3_description package
- ros2_control
- tf2_ros
- rclpy