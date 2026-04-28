"""Basic test flight -- takeoff, hover, land.

Uses cflib directly -- no ROS2 or Crazyswarm2 required.
Works on Mac, Windows, or Linux.

Prerequisites:
  1. pip install cflib
  2. Crazyradio dongle plugged in
  3. Drone powered on and blinking

Usage:
    python3 scripts/test_flight.py
"""

import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.high_level_commander import HighLevelCommander

# --- Set your drone URI here ---
URI = 'radio://0/80/2M/E7E7E7E701'  # update after running scan

TAKEOFF_HEIGHT = 0.5   # metres
HOVER_DURATION = 5.0   # seconds


def main():
    cflib.crtp.init_drivers()

    print("Scanning for drones...")
    found = cflib.crtp.scan_interfaces()
    if not found:
        print("ERROR: No drones found. Check dongle and drone power.")
        return

    print(f"Found: {found}")
    uri = found[0][0]
    print(f"Connecting to {uri}...")

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        commander = HighLevelCommander(scf.cf)

        print(f"Taking off to {TAKEOFF_HEIGHT}m...")
        commander.takeoff(TAKEOFF_HEIGHT, 2.5)
        time.sleep(3.0)

        print(f"Hovering for {HOVER_DURATION}s... (Ctrl+C to land early)")
        try:
            time.sleep(HOVER_DURATION)
        except KeyboardInterrupt:
            print("Interrupted -- landing now")

        print("Landing...")
        commander.land(0.0, 2.5)
        time.sleep(3.0)

    print("Done.")


if __name__ == '__main__':
    main()
