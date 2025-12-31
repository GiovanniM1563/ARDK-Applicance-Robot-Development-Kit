#!/bin/bash
set -e

# Setup environment
source /home/gio/ARDK_2/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

echo "=== 1. Starting ARDK Lifecycle System ==="
ros2 launch ardk_lifecycle system.launch.py > /tmp/ardk_sys.log 2>&1 &
SYS_PID=$!
sleep 15  # Wait for IDLE state

echo "=== 2. Starting ARDK API Server ==="
ros2 run ardk_api server > /tmp/ardk_api.log 2>&1 &
API_PID=$!
sleep 5 # Wait for uvicorn

echo "=== 3. Switching to MAPPING (via API) ==="
curl -X POST "http://localhost:8000/ardk/mode" -H "Content-Type: application/json" -d '{"mode": 1}'
echo ""
sleep 10 # Wait for SLAM to settle

echo "=== 4. Saving Map (via API) ==="
# Map name "api_test_map"
curl -X POST "http://localhost:8000/slam/save" -H "Content-Type: application/json" -d '{"name": "/home/gio/ARDK_2/maps/api_test_map"}'
echo ""
sleep 2

echo "=== 5. Switching to NAVIGATION (via API) ==="
# We pass empty map path to let it use default or we can pass the one we just saved
# User step 5 says "switch to nav2", then step 6 "send map".
# So lets switch to Nav2 (mode 2) WITHOUT explicitly specifying map in mode call (defaults apply, or we pass it).
# Let's pass the map we just saved to ensure cleanliness.
curl -X POST "http://localhost:8000/ardk/mode" -H "Content-Type: application/json" -d '{"mode": 2, "map_path": "/home/gio/ARDK_2/maps/api_test_map.yaml"}'
echo ""
sleep 20 # Wait for Loc -> Wait(Map) -> Nav to complete

echo "=== 6. Hot-Swap Map (Verify API /nav/map) ==="
# User asked "using api, send map to nav2"
curl -X POST "http://localhost:8000/nav/map" -H "Content-Type: application/json" -d '{"path": "/home/gio/ARDK_2/maps/ardk_smoke_map.yaml"}'
echo ""
sleep 5

echo "=== 7. Check Stack Status ==="
echo "Checking /ardk_status topic..."
timeout 2s ros2 topic echo /ardk_status --once || true

echo "=== 8. Return to IDLE ==="
curl -X POST "http://localhost:8000/ardk/mode" -H "Content-Type: application/json" -d '{"mode": 0}'
echo ""
sleep 5

echo "=== Cleanup ==="
kill $API_PID
kill $SYS_PID
pkill -f state_manager
pkill -f nav2
pkill -f slam_toolbox

echo "TEST COMPLETE."
