# ARDK Lifecycle & Mode Management System

**"Appliance Robot Development Kit"**

The ARDK Lifecycle system is a hardened, production-ready orchestration layer designed to simplify the complex task of managing robot states in ROS 2. It provides a robust, fail-safe mechanism for switching between Mapping (SLAM) and Navigation modes without the fragility often associated with custom scripts or manual CLI commands.

This project allows developers to treat the robot's navigation stack as a reliable "black box" appliance, focusing on their unique application logic rather than the low-level intricacies of ROS 2 lifecycle management.

---

##  Key Capabilities

### 1. Instant Mode Switching
- **Fast Transitions**: Switches between Mapping and Navigation modes in seconds.
- **Optimized Performance**: Drastically reduces CPU and memory overhead compared to standard launch-and-kill approaches.

### 2. High Reliability & Fault Tolerance
- **Atomic Operations**: The system ensures the robot is never left in an undefined or broken state. If a transition cannot complete successfully, it automatically reverts to a safe idle state.
- **Clean Shutdowns**: Guarantees no lingering processes or "zombie" nodes, ensuring long-term system stability on edge hardware.

### 3. Safety-First Design
- **Verified Readiness**: Transitions only occur when the system has verified that all necessary data (Sensor Transforms, Map Topics, etc.) is valid and available.
- **Hardware Optimized**: Tuned specifically for the constraints of edge compute devices like the Raspberry Pi, handling variable boot times and resource constraints gracefully.

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

**Switch to Navigation:**
```bash
# Switches mode to NAV and loads the specified map
curl -X POST "http://localhost:8000/ardk/mode" -d '{"mode": 2, "map_path": "/home/gio/ARDK_2/maps/my_map.yaml"}'
```

**Hot-Swap Map (While in Nav):**
```bash
curl -X POST "http://localhost:8000/nav/map" -d '{"path": "/home/gio/ARDK_2/maps/another_map.yaml"}'
```

---

## Verification

The system includes comprehensive verification tools to ensure stability on your hardware:

1.  **Lifecycle Cycle Test**:
    ```bash
    python3 src/ardk_lifecycle/tests/cycle_test.py
    ```

2.  **Full API Integration Test**:
    ```bash
    bash src/ardk_api/test_integration.sh
    ```

**Verified Platforms:**
- Raspberry Pi 5 / Ubuntu 24.04 / ROS 2 Jazzy
