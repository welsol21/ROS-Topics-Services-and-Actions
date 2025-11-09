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
├── launch/
│   └── assignment_launch.py        # Task 7 (Unified launch)
└── turtle_controller_assignment/
    ├── __init__.py
    ├── turtle_spawn_client.py       # Task 1 (manual spawner)
    ├── turtle_name_manager.py       # Name Management Service
    ├── pen_control_client.py        # Task 2 (pen control)
    ├── auto_turtle_spawner.py       # Task 3 (auto spawner)
    ├── turtle_monitor_service.py    # Monitoring Service
    ├── closest_turtle_service.py    # Task 4 (service server)
    ├── closest_turtle_client.py     # Task 4 (test client)
    ├── turtle_collection_server.py  # Task 5 (action server)
    └── turtle_collection_client.py  # Task 6 (action client)
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
- **Required Services:**
  - `/spawn` (turtlesim/srv/Spawn)
  - `/turtle_name_manager/generate_unique_name` (std_srvs/Trigger)
- **Launch Order:**
  1. ros2 run turtlesim turtlesim_node
  2. ros2 run turtle_controller_assignment turtle_name_manager
  3. ros2 run turtle_controller_assignment turtle_spawn_client

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
- **Required Services:**
  - `/turtle1/set_pen` (turtlesim/srv/SetPen)
- **Launch Order:**
  1. ros2 run turtlesim turtlesim_node
  2. ros2 run turtle_controller_assignment pen_control_client
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

- **Launch Order:**
  1. ros2 run turtlesim turtlesim_node
  2. ros2 run turtle_controller_assignment turtle_name_manager
  3. ros2 run turtle_controller_assignment turtle_monitor_service
  4. ros2 run turtle_controller_assignment auto_turtle_spawner



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
- **Required Services:**
  - `/find_closest_turtle` server (this node)
  - Prerequisites: `/monitor_turtles` (std_srvs/Trigger) and pose topics discovered by monitor
- **Launch Order:**
  1. ros2 run turtlesim turtlesim_node
  2. ros2 run turtle_controller_assignment turtle_name_manager
  3. ros2 run turtle_controller_assignment turtle_monitor_service
  4. ros2 run turtle_controller_assignment auto_turtle_spawner
  5. ros2 run turtle_controller_assignment closest_turtle_service

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
- **Launch Order:**
  1. ros2 run turtlesim turtlesim_node
  2. ros2 run turtle_controller_assignment turtle_name_manager
  3. ros2 run turtle_controller_assignment turtle_monitor_service
  4. ros2 run turtle_controller_assignment closest_turtle_service
  5. ros2 run turtle_controller_assignment turtle_collection_server
  6. ros2 run turtle_controller_assignment auto_turtle_spawner
  7. ros2 run turtle_controller_assignment turtle_collection_client
### Task 6: Action Client [10%]
- **Status:** Implemented
- **Node Name:** `turtle_collection_client`
- **Functionality:** Action client for `collect_turtles` (`custom_interfaces/action/MoveTurtle`). Starts collection when there is at least 1 active bot, prints feedback (distance to target), and enforces the cancellation rule.
- **Key Features:**
  - Displays action feedback: remaining distance to the current target.
  - Starts collection when bot count ≥ 1.
  - Real-time cancellation rule: cancels the current goal if bots left to collect ≥ 10 at any time during the action.
  - After cancel: logs failure, server resets simulation, client becomes ready for the next collection.
- **Verification of real-time cancellation:**
  - Use the parameterized launch to adjust the spawner interval. To quickly exceed the bot threshold and observe client-side cancellation, run:
    `ros2 launch turtle_controller_assignment assignment_launch.py spawn_interval:=2.0`
  - To restore normal operation, launch without the parameter (default 5.0s).
- **Required Services:**
  - `/collect_turtles` action server (`turtle_collection_server`)
  - `/find_closest_turtle` service (for server’s target selection)
- **Launch Order:**
  1. ros2 run turtlesim turtlesim_node
  2. ros2 run turtle_controller_assignment turtle_name_manager
  3. ros2 run turtle_controller_assignment turtle_monitor_service
  4. ros2 run turtle_controller_assignment closest_turtle_service
  5. ros2 run turtle_controller_assignment turtle_collection_server
  6. ros2 run turtle_controller_assignment auto_turtle_spawner
  7. ros2 run turtle_controller_assignment turtle_collection_client
### Task 7: Launch File [5%]
- **Status:** Implemented
- **File Name:** `assignment_launch.py`
- **Description:** Single parameterized launch that starts turtlesim, services, action server, spawner, and client in strict event-driven order (each waits for prerequisites). Supports configurable spawn interval.
- **Launch Order:**
  1. Default: `ros2 launch turtle_controller_assignment assignment_launch.py`
  2. Custom interval: `ros2 launch turtle_controller_assignment assignment_launch.py spawn_interval:=2.0`

---

## Build the package
```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select turtle_controller_assignment
source install/setup.bash
```

## Dependencies
- rclpy  
- geometry_msgs  
- turtlesim  
- std_srvs  
- custom_interfaces (srv: FindClosestTurtle; action: MoveTurtle)

---

