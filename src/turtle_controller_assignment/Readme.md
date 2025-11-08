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
    ├── turtle_spawn_client.py       # Task 1
    ├── turtle_name_manager.py       # Name Management Service
    ├── pen_control_client.py        # Task 2
    ├── auto_turtle_spawner.py       # Task 3
    ├── turtle_monitor_service.py    # Monitoring Service
    ├── closest_turtle_service.py    # Task 4 - Service Server
    └── closest_turtle_client.py     # Task 4 - Test Client
```

---

## Task Implementation Status

### Task 1: Turtle Spawn Client [5%]
- **Node Name:** `turtle_spawn_client`
- **File:** `turtle_spawn_client.py`
- **Functionality:** Client for the spawn service to spawn turtles at given coordinates
- **Key Features:**
  - Requests unique names from Turtle Name Manager service
  - Guarantees unique turtle names without conflicts
  - Handles service availability and errors gracefully
- **Usage:** `ros2 run turtle_controller_assignment turtle_spawn_client`

---

### Task 2: Pen Control Client [5%]
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

---

### Turtle Name Management System
- **Node Name:** `turtle_name_manager`
- **File:** `turtle_name_manager.py`
- **Functionality:** Centralized service for generating unique turtle names
- **Service:** `generate_unique_name` (std_srvs/Trigger)
- **Purpose:** Prevents naming conflicts across all tasks

---

### Turtle Monitor Service
- **Node Name:** `turtle_monitor_service`
- **File:** `turtle_monitor_service.py`
- **Functionality:** Monitors active and removed turtles by scanning pose topics
- **Service:** `/monitor_turtles` (std_srvs/Trigger)
- **Features:**
  - Automatically discovers new turtles by scanning topics every 2 seconds
  - Tracks turtle health based on pose message timestamps
  - Returns status in format: `"ACTIVE:a,b;REMOVED:x,y"`
  - Removes disappeared turtles from tracking
- **Purpose:** Provides centralized turtle status for auto spawner

---

### Task 3: Auto Turtle Spawner [30%]
- **Node Name:** `auto_turtle_spawner`
- **File:** `auto_turtle_spawner.py`
- **Functionality:** Automatically spawns turtles at random coordinates every 5 seconds and sets them moving on circular paths with pen turned off
- **Key Features:**
  - Spawns turtles every 5 seconds at random safe coordinates
  - Maintains at most 10 additional turtles (excluding turtle1)
  - Uses exact coordinate formula from assignment:
    * `r ~ U(1,5)`, `φ ~ U(0,2π)`
    * `x' ~ U(-5.5+r, 5.5-r)`, `y' ~ U(-5.5+r, 5.5-r)`
    * `x = 5.5 + x' + r*cos(φ)`, `y = 5.5 + y' + r*sin(φ)`
    * `θ = φ ± π/2`
  - Circular motion with velocities: `vx = 1.0`, `ωz = ±1/r`
  - Automatically turns pen off for all spawned turtles
  - Monitors and replaces removed turtles
  - Integrates with Turtle Name Manager for unique names
  - Integrates with Turtle Monitor Service for turtle tracking
- **Required Services:**
  - `/turtle_name_manager/generate_unique_name` (std_srvs/Trigger)
  - `/monitor_turtles` (std_srvs/Trigger)
  - `/spawn` (turtlesim/srv/Spawn)
  - `/<turtle_name>/set_pen` (turtlesim/srv/SetPen)
- **Usage:**
  ```bash
  # Terminal 1 - turtlesim
  ros2 run turtlesim turtlesim_node
  
  # Terminal 2 - name manager
  ros2 run turtle_controller_assignment turtle_name_manager
  
  # Terminal 3 - monitor service
  ros2 run turtle_controller_assignment turtle_monitor_service
  
  # Terminal 4 - auto spawner
  ros2 run turtle_controller_assignment auto_turtle_spawner
  ```
- **Status:** COMPLETED AND TESTED

### Task 4: Closest Turtle Service Server [15%]
- **Node Name:** `closest_turtle_service`
- **File:** `closest_turtle_service.py`
- **Functionality:** Service server that determines the closest turtle to turtle1
- **Service:** `/find_closest_turtle` (custom_interfaces/srv/FindClosestTurtle)
- **Key Features:**
  - Dynamically discovers and tracks all turtles by scanning pose topics
  - Subscribes to pose updates from all detected turtles
  - Calculates Euclidean distance from turtle1 to all other turtles
  - Returns closest turtle name and distance when service is called
  - Automatically removes disappeared turtles from tracking
- **Custom Service Definition:**
  ```
  # Request: empty (just trigger)
  ---
  # Response:
  string closest_turtle_name
  float64 distance
  bool success
  string message
  ```
- **Algorithm:**
  - Distance calculation: `d = sqrt((x2-x1)² + (y2-y1)²)`
  - Compares all active turtles (excluding turtle1)
  - Returns turtle with minimum distance
- **Test Client:** `closest_turtle_client` (calls service every 3 seconds)
- **Usage:**
  ```bash
  # Terminal 1 - turtlesim
  ros2 run turtlesim turtlesim_node
  
  # Terminal 2 - name manager
  ros2 run turtle_controller_assignment turtle_name_manager
  
  # Terminal 3 - monitor service
  ros2 run turtle_controller_assignment turtle_monitor_service
  
  # Terminal 4 - auto spawner (to create turtles)
  ros2 run turtle_controller_assignment auto_turtle_spawner
  
  # Terminal 5 - closest turtle service
  ros2 run turtle_controller_assignment closest_turtle_service
  
  # Terminal 6 - test client (optional)
  ros2 run turtle_controller_assignment closest_turtle_client
  
  # Or call manually:
  ros2 service call /find_closest_turtle custom_interfaces/srv/FindClosestTurtle
  ```
- **Status:** COMPLETED AND TESTED

### Task 5: Turtle Collection Action Server [30%]
- **Node Name:** `turtle_collection_server`
- **File:** `turtle_collection_server.py`
- **Functionality:** Action server that moves `turtle1` to collect all other turtles using proportional control and the closest-turtle service.
- **Action:** `collect_turtles` (`custom_interfaces/action/MoveTurtle`)
- **Key Features:**
  - Determines the current target by calling `/find_closest_turtle` (`FindClosestTurtle`)
  - Proportional control: `vx = 2 * dr`, `ωz = 4 * dφ`
  - Removes target when within `0.5` distance using `/kill`
  - Publishes feedback with remaining distance
  - Supports goal cancellation: stops `turtle1` and resets the simulation via `/reset`
  - Pauses auto spawner during collection via `/spawner/disable`, re-enables via `/spawner/enable`
  - Subscribes to pose topics for `turtle1` and all spawned turtles
- **Required Services:**
  - `/find_closest_turtle` (custom_interfaces/srv/FindClosestTurtle)
  - `/kill` (turtlesim/srv/Kill)
  - `/reset` (std_srvs/srv/Empty)
  - `/spawner/enable` and `/spawner/disable` (std_srvs/srv/Trigger)
    - Enable: resumes auto-spawning of new turtles
    - Disable: pauses auto-spawning; existing turtles continue moving (targets remain dynamic)
- **Topics:**
  - `/turtle1/pose` (turtlesim/msg/Pose)
  - `/<turtle>/pose` for other turtles
  - `/turtle1/cmd_vel` (geometry_msgs/msg/Twist)
- **Usage:**
  ```bash
  # Terminal 1 - turtlesim
  ros2 run turtlesim turtlesim_node

  # Terminal 2 - name manager
  ros2 run turtle_controller_assignment turtle_name_manager

  # Terminal 3 - monitor service
  ros2 run turtle_controller_assignment turtle_monitor_service

  # Terminal 4 - auto spawner (wait until 10 bots)
  ros2 run turtle_controller_assignment auto_turtle_spawner

  # Terminal 5 - closest turtle service
  ros2 run turtle_controller_assignment closest_turtle_service

  # Terminal 6 - collection action server
  ros2 run turtle_controller_assignment turtle_collection_server

  # Terminal 7 - action client
  ros2 run turtle_controller_assignment turtle_collection_client
  ```

### Task 6: Action Client [10%]
- **Status:** Not implemented
- **Node Name:** `turtle_collection_client`

### Task 7: Launch File [5%]
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
- Package structure created  
- Centralized Turtle Name Manager implemented  
- Task 1 completed with unique name generation  
- Task 2 completed - Pen Control Client implemented and tested  
- Task 3 completed - Auto Turtle Spawner with monitoring system  
- Task 4 completed - Closest Turtle Service Server implemented and tested  
- Remaining tasks pending implementation  

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
3. ~~Implement Task 3 (auto-spawner with monitoring system)~~  
4. ~~Implement Task 4 (closest turtle service)~~  
5. Implement Task 5 (collection action server)  
6. Implement Task 6 (action client with cancellation)  
7. Create Task 7 (comprehensive launch file)

---

*Last updated: Task 4 completed — Closest Turtle Service Server implemented and tested*
