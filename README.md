# ARDK Lifecycle & Mode Management System

**"Appliance Robot Development Kit"**

The point of this project is to create a readymade, easy to use system that allows individuals to accelerate the arbitrary setup of ROS 2 robots, to allow them to focus on their own unique applications or additions.

Ideally, this system should be able to solve the "boring" parts of ROS2 integration, such as in this case, switching and orchestrating between different stacks on the fly, without the need to use ROS 2 CLI tools or create your own scripts to do so.

This repository contains the hardened lifecycle management system for ROS2 robots. It solves a problem I faced, switching between `MAPPING` (SLAM_Toolbox) and `NAVIGATION` (Nav2) actively and frequently.

---

##  Key Features

### 1. Resident Nav2 Architecture
- **Problem**: Launching Nav2 from scratch takes 30-45 seconds on RPi and consumes massive CPU/DDS resources.
- **Solution**: The `nav2_bringup` process remains resident in memory. We use `LifecycleServiceClients` to purely toggle nodes between `ACTIVE` and `INACTIVE` states.
- **Result**: Transition time reduced to **< 5 seconds** (once costmaps are initialized).

### 2. Strict "No Partial Startups"
- **Atomic Rollback**: If ANY step of a transition fails (e.g., Map Load timeout, Service unavailability), the system **automatically** rolls back to `IDLE`.
- **Zero Zombie Processes**: Strict `process_group` signaling ensures all subprocesses (SLAM, Nav2 container) are cleanly killed on exit.

### 3. Smooth & Safe Transitions
- **Event-Driven Gates**: Active checks of real conditions before transition:
    - `wait_for_topic('/map')`: Ensures SLAM is actually publishing.
    - `wait_for_tf_chain(...)`: Ensures the full `map -> odom -> base_link` tree is valid before handing control to Nav2.
- **Deterministic Wait**: Instead of arbitrary timeouts, the system waits indefinitely for Lifecycle Managers to confirm state transitions (with periodic progress logs), robustly handling variable boot times on edge hardware.
- **Motion Guard**: Every mode transition automatically forces `cmd_vel` to zero to prevent runaway motion.

### 4. Robustness for Edge Hardware
- **Retry Logic**: Critical services (like `load_map`) utilize a 3-try backoff mechanism to handle disk/DDS latency on SD cards.
- **State Tracking**: Intelligent tracking prevents redundant `shutdown` calls, avoiding "hanging" issues common with ROS 2 Lifecycle Managers on slow DDS networks.

---

## System Architecture

### Core Components

1.  **`state_manager` (Node)**: The central orchestrator.
    -   Manages the high-level Finite State Machine (`IDLE`, `MAPPING`, `NAVIGATION`).
    -   Holds persistent connections to Lifecycle Managers (`/lifecycle_manager_navigation/manage_nodes`).
    -   Publishes `/ardk_status` for observability.

2.  **`ardk_api` (FastAPI Service)**:
    -   A dedicated, non-blocking HTTP bridge to control the robot from web apps or external scripts.
    -   Uses `asyncio` to bridge ROS 2 futures to HTTP responses.

3.  **`ardk` (Python Package)**:
    -   **`runners.nav_runner`**: Handles interaction with the Resident Nav2 stack (Composition enabled).
    -   **`runners.slam_runner`**: Manages SLAM Toolbox lifecycle.
    -   **`core.readiness`**: Contains the logic for "Readiness Gates" (Topic/TF checks).
    -   **`core.process_group`**: Safe signal handling.

---

## 🛠️ Usage Guide

### 1. Launch the System
```bash
ros2 launch ardk_lifecycle system.launch.py
```

### 2. Start the API Server
```bash
ros2 run ardk_api server
```
*Runs on `http://0.0.0.0:8000`*

### 3. API Control (Recommended)

**Switch to Mapping:**
```bash
curl -X POST "http://localhost:8000/ardk/mode" -d '{"mode": 1}'
```

**Save Map:**
```bash
curl -X POST "http://localhost:8000/slam/save" -d '{"name": "/home/gio/ARDK_2/maps/my_map"}'
```

**Switch to Navigation (Nav2):**
```bash
# Switches mode to NAV and loads the specified map
curl -X POST "http://localhost:8000/ardk/mode" -d '{"mode": 2, "map_path": "/home/gio/ARDK_2/maps/my_map.yaml"}'
```

**Hot-Swap Map (While in Nav):**
```bash
curl -X POST "http://localhost:8000/nav/map" -d '{"path": "/home/gio/ARDK_2/maps/another_map.yaml"}'
```

### 4. ROS 2 Service Control (Alternative)
You can still use standard ROS 2 CLI:
```bash
ros2 service call /set_mode ardk_lifecycle/srv/SetMode "{target_mode: 1}"
```

### 5. Monitor Status
```bash
ros2 topic echo /ardk_status
```
Example Output:
```yaml
mode: 2 (NAVIGATION)
map_source: "map_server"
tf_authority: "amcl"
motion_authority: "nav2"
```

## Finer Usage

Nav2 and SLAM Toolbox are both controlled through their own APIs, and can be used independently of the state manager to get desired functionalities. You can for example read the /ardk_status to check the mode to ensure the right stack is operational, and then use its corresponding APIs (I.E. map saving, Pose Routing etc.) 

---

## Verification

The system includes comprehensive verification tools:

1.  **Lifecycle Cycle Test**:
    ```bash
    python3 src/ardk_lifecycle/tests/cycle_test.py
    ```

2.  **Full API Integration Test**:
    ```bash
    bash src/ardk_api/test_integration.sh
    ```

**Status:**
- [x] **Zero Partial Startups**: Verified via stress testing.
- [x] **Deterministic Startup**: Verified wait >80s gracefully.
- [x] **API End-to-End**: Verified full Curl->Lifecycle->Stack flow.
- [x] **RPi 5 Verified**: Tuned for real hardware constraints.

## Future WIP

The system is currently in a proof of concept stage and is not yet ready for production use. Future work includes:

- Building a ROS 2 package for the state manager
- Adding support for more operation modes and modular configurations to add your own modes
- Adding rosbridge support to allow remote triggers of mode changes
- adding helpers for Nav2 and SLAM Toolbox native APIs to allow for more fine-grained control
