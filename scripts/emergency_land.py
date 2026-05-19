"""Emergency land -- connects and immediately commands landing."""

import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander


def main():
    cflib.crtp.init_drivers()

    found = cflib.crtp.scan_interfaces()
    if not found:
        print("ERROR: Cannot find drone.")
        return

    uri = found[0][0]
    print(f"Connecting to {uri}...")

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        with MotionCommander(scf, default_height=0.3) as mc:
            print("Moving down...")
            mc.down(0.2)
            time.sleep(2.0)
            print("Landing...")

    print("Done.")


if __name__ == '__main__':
    main()
