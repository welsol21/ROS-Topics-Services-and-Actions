# COMP9069 Robotics & Autonomous Systems - Assignment 1

## Package Overview
This ROS 2 package contains nodes for the COMP9069 Assignment 1 tasks using turtlesim, including centralized name management and pen control demonstration.

---

## Package Structure
```
turtle_controller_assignment/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── turtle_controller_assignment
└── turtle_controller_assignment/
    ├── __init__.py
    ├── turtle_spawn_client.py    # Task 1
    ├── turtle_name_manager.py    # Name Management Service
    └── pen_control_client.py     # Task 2
```

---

## Task Implementation Status

### ✅ Task 1: Turtle Spawn Client [5%]
- **Node Name:** `turtle_spawn_client`
- **File:** `turtle_spawn_client.py`
- **Functionality:** Client for the spawn service to spawn turtles at given coordinates
- **Key Features:**
  - Requests unique names from Turtle Name Manager service
  - Guarantees unique turtle names without conflicts
  - Handles service availability and errors gracefully
- **Usage:** `ros2 run turtle_controller_assignment turtle_spawn_client`
- **Status:** ✅ **COMPLETED AND TESTED**

---

### ✅ Task 2: Pen Control Client [5%]
- **Node Name:** `pen_control_client`
- **File:** `pen_control_client.py`
- **Functionality:** Demonstrates the use of the `SetPen` service by changing pen color, width, and toggling drawing while moving the turtle.
- **Key Features:**
  - Controls pen color (R, G, B), width, and on/off state
  - Demonstrates movement with various pen configurations
  - Publishes velocity commands to `/turtle1/cmd_vel`
  - Gracefully handles service availability
- **Demonstration Includes:**
  1. White pen (default)
  2. Red pen
  3. Green pen
  4. Pen off (no drawing)
  5. Blue pen (resumes drawing)
- **Usage:**
  ```bash
  ros2 run turtle_controller_assignment pen_control_client
  ```
- **Status:** ✅ **COMPLETED AND TESTED**

---

### 🔧 Turtle Name Management System
- **Node Name:** `turtle_name_manager`
- **File:** `turtle_name_manager.py`
- **Functionality:** Centralized service for generating unique turtle names
- **Service:** `generate_unique_name` (std_srvs/Trigger)
- **Purpose:** Prevents naming conflicts across all tasks

---

### ⏳ Task 3: Auto Spawner Node [15%]
- **Status:** Not implemented
- **Node Name:** `auto_turtle_spawner`

### ⏳ Task 4: Closest Turtle Service Server [15%]
- **Status:** Not implemented
- **Node Name:** `closest_turtle_server`

### ⏳ Task 5: Turtle Collection Action Server [25%]
- **Status:** Not implemented
- **Node Name:** `turtle_collection_server`

### ⏳ Task 6: Action Client [15%]
- **Status:** Not implemented
- **Node Name:** `turtle_collection_client`

### ⏳ Task 7: Launch File [20%]
- **Status:** Not implemented
- **File Name:** `assignment_launch.py`

---

## Architecture Overview

### Centralized Name Management
```
┌──────────────────┐    ┌──────────────────────┐    ┌──────────────┐
│  Turtle Spawn    │ →  │  Turtle Name Manager │ →  │  turtlesim   │
│  Client (Task 1) │    │  (Service Server)    │    │   Node       │
└──────────────────┘    └──────────────────────┘    └──────────────┘
           │
           ▼
Generates unique names:
turtle_1, turtle_2, turtle_3...
```

---

## Current Progress
- ✅ Package structure created  
- ✅ Centralized Turtle Name Manager implemented  
- ✅ Task 1 completed with unique name generation  
- ✅ Task 2 completed — Pen Control Client implemented and tested  
- ⏳ Remaining tasks pending implementation  

---

## Building and Running

### 1. Build the package
```bash
cd ~/ros2_ws
colcon build --packages-select turtle_controller_assignment
source install/setup.bash
```

### 2. Test Task 1 (Spawn Client with Name Management)
**Terminal 1 – Start turtlesim:**
```bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2 – Start name manager:**
```bash
ros2 run turtle_controller_assignment turtle_name_manager
```

**Terminal 3 – Run spawn client (multiple times):**
```bash
ros2 run turtle_controller_assignment turtle_spawn_client
```

### 3. Test Task 2 (Pen Control Client)
**Terminal 1 – Start turtlesim:**
```bash
ros2 run turtlesim turtlesim_node
```

**Terminal 2 – Run Pen Control Client:**
```bash
ros2 run turtle_controller_assignment pen_control_client
```

**Expected Output:**
- Turtle moves in various paths with different pen colors and widths.  
- Pen turns off temporarily (no drawing), then resumes with new color.  
- Console prints step-by-step demo messages.

---

## Dependencies
- rclpy  
- geometry_msgs  
- turtlesim  
- std_srvs  
- custom interfaces (to be created for later tasks)

---

## Next Implementation Steps
1. ~~Implement Task 1 with name management~~  
2. ~~Implement Task 2 (pen control client)~~  
3. Implement Task 3 (auto-spawner using name manager)  
4. Implement Task 4 (closest turtle service)  
5. Implement Task 5 (collection action server)  
6. Implement Task 6 (action client with cancellation)  
7. Create Task 7 (comprehensive launch file)

---

*Last updated: Task 2 completed — Pen Control Client implemented and tested*
