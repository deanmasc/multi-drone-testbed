"""Basic test flight -- takeoff, hover, land.

Run this first to verify the drone connects and flies before
using the full testbed.

Prerequisites:
  1. Crazyswarm2 running:  ros2 launch crazyflie launch.py
  2. Crazyradio dongle plugged in
  3. Drone powered on and blinking

Usage:
    python3 scripts/test_flight.py
"""

from crazyflie_py import Crazyswarm

TAKEOFF_HEIGHT = 0.5    # metres
HOVER_DURATION = 5.0    # seconds


def main():
    print("Connecting to Crazyswarm...")
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    if not allcfs.crazyflies:
        print("ERROR: No drones found. Check URI in crazyflies.yaml and radio dongle.")
        return

    print(f"Found {len(allcfs.crazyflies)} drone(s). Taking off...")

    allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=2.5)
    timeHelper.sleep(3.0)

    print(f"Hovering for {HOVER_DURATION} seconds... (Ctrl+C to land early)")

    try:
        timeHelper.sleep(HOVER_DURATION)
    except KeyboardInterrupt:
        print("Interrupted -- landing now")

    print("Landing...")
    allcfs.land(targetHeight=0.05, duration=2.5)
    timeHelper.sleep(3.0)

    print("Done.")


if __name__ == '__main__':
    main()
