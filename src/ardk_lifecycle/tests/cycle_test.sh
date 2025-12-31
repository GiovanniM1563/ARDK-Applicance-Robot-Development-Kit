#!/bin/bash
source /home/gio/ARDK_2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "=== CYCLE TEST START ==="
echo "1. IDLE -> MAPPING"
ros2 service call /set_mode ardk_lifecycle/srv/SetMode "{target_mode: 1}"
sleep 5

echo "2. MAPPING -> NAVIGATION"
ros2 service call /set_mode ardk_lifecycle/srv/SetMode "{target_mode: 2}"
echo "Waiting 10s for Nav to settle..."
sleep 10

echo "3. NAVIGATION -> IDLE"
ros2 service call /set_mode ardk_lifecycle/srv/SetMode "{target_mode: 0}"
sleep 5

echo "=== CYCLE TEST END ==="
