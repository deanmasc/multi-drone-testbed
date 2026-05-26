#!/usr/bin/env bash
# Run the simple one-drone hardware flight test.
# Make sure setup.sh has been run first.

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Source ROS2 and workspaces
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/ros2_ws/install/setup.bash" 2>/dev/null || true

# Copy our drone config into the Crazyswarm2 package location
CRAZYSW_CONFIG="/opt/ros/jazzy/share/crazyflie/config"
if [ -d "$CRAZYSW_CONFIG" ]; then
    echo "Copying drone config to Crazyswarm2..."
    sudo cp "$SCRIPT_DIR/config/crazyflies.yaml" "$CRAZYSW_CONFIG/crazyflies.yaml"
fi

echo ""
echo "=== Hardware Flight Test ==="
echo "IMPORTANT: Before running, ensure:"
echo "  1. Crazyradio dongle is plugged in"
echo "  2. Drone is powered on (blinking blue/yellow)"
echo "  3. VICON is running and tracking the drone body"
echo "  4. config/crazyflies.yaml has correct URI and VICON body name"
echo "  5. config/motion_capture.yaml has correct VICON PC IP"
echo ""
read -p "Press Enter to launch Crazyswarm2 + run flight test, or Ctrl+C to abort..."

# Terminal 1: Launch Crazyswarm2 in background
echo "Starting Crazyswarm2..."
ros2 launch crazyflie launch.py &
CRAZYSW_PID=$!
sleep 5  # wait for it to initialise

# Run the flight script
echo "Running flight test..."
python3 "$SCRIPT_DIR/scripts/simple_flight.py"

# Cleanup
kill $CRAZYSW_PID 2>/dev/null || true
echo "Done."
