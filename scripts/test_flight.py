"""Basic test flight for old Crazyflie firmware.

Uses raw commander -- no Kalman filter, no MotionCommander required.
Compatible with legacy firmware.

Prerequisites:
  1. pip install cflib
  2. Crazyradio dongle plugged in
  3. Drone powered on and blinking on a flat surface

Usage:
    python3 scripts/test_flight.py
"""

import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

THRUST_TAKEOFF = 40000   # thrust to climb (tune up/down if needed)
THRUST_HOVER   = 34000   # thrust to hover
THRUST_LAND    = 34000   # thrust to descend slowly

TAKEOFF_TIME   = 1     # seconds climbing
HOVER_TIME     = 0     # seconds hovering
LAND_TIME      = 11.5     # seconds descending


def main():
    cflib.crtp.init_drivers()

    print("Scanning for drones...")
    found = cflib.crtp.scan_interfaces()
    if not found:
        print("ERROR: No drones found.")
        return

    print("Found drones:")
    for i, f in enumerate(found):
        print(f"  [{i}] {f[0]}")

    uri = found[0][0] if len(found) == 1 else found[int(input("Select drone number: "))][0]
    print(f"Connecting to {uri}...")

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # Unlock motors with zero thrust first
        print("Unlocking...")
        for _ in range(100):
            cf.commander.send_setpoint(0, 0, 0, 0)
            time.sleep(0.01)

        try:
            # Climb
            print(f"Taking off ({TAKEOFF_TIME}s)...")
            end = time.time() + TAKEOFF_TIME
            while time.time() < end:
                cf.commander.send_setpoint(0, 0, 0, THRUST_TAKEOFF)
                time.sleep(0.02)

            # Hover
            print(f"Hovering ({HOVER_TIME}s)...")
            end = time.time() + HOVER_TIME
            while time.time() < end:
                cf.commander.send_setpoint(0, 0, 0, THRUST_HOVER)
                time.sleep(0.02)

        except KeyboardInterrupt:
            print("Interrupted -- landing now")

        # Land
        print(f"Landing ({LAND_TIME}s)...")
        end = time.time() + LAND_TIME
        while time.time() < end:
            cf.commander.send_setpoint(0, 0, 0, THRUST_LAND)
            time.sleep(0.02)

        # Kill motors
        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)
        cf.commander.send_stop_setpoint()

    print("Done.")


if __name__ == '__main__':
    main()
