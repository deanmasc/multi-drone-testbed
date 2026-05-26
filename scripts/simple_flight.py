"""Simple one-drone hardware flight test using Crazyswarm2.

Takes off to 0.5 m, hovers, moves up to 0.8 m, back down to 0.5 m, then lands.

Prerequisites:
  1. Crazyswarm2 running:
       ros2 launch crazyflie launch.py
  2. VICON running and publishing poses
  3. Crazyradio dongle plugged in
  4. config/crazyflies.yaml has correct URI and VICON body name

Usage:
    python3 scripts/simple_flight.py
"""

from crazyflie_py import Crazyswarm

TAKEOFF_HEIGHT = 0.5   # metres
HIGH_HEIGHT    = 0.8   # metres (top of up-down demo)
HOVER_TIME     = 3.0   # seconds hovering at each height


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    allcfs = swarm.allcfs

    print("Taking off...")
    allcfs.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=2.5)
    timeHelper.sleep(3.0)

    print(f"Hovering at {TAKEOFF_HEIGHT}m...")
    timeHelper.sleep(HOVER_TIME)

    print(f"Moving up to {HIGH_HEIGHT}m...")
    for cf in allcfs.crazyflies:
        cf.goTo([0, 0, HIGH_HEIGHT], 0, 2.0)
    timeHelper.sleep(2.5)

    print(f"Moving back down to {TAKEOFF_HEIGHT}m...")
    for cf in allcfs.crazyflies:
        cf.goTo([0, 0, TAKEOFF_HEIGHT], 0, 2.0)
    timeHelper.sleep(2.5)

    print("Landing...")
    allcfs.land(targetHeight=0.04, duration=2.5)
    timeHelper.sleep(3.0)

    print("Done.")


if __name__ == '__main__':
    main()
