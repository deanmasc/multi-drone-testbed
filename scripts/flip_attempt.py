"""Attempt to flip drone upright by spinning one side briefly."""

import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


def main():
    cflib.crtp.init_drivers()

    found = cflib.crtp.scan_interfaces()
    if not found:
        print("ERROR: Cannot find drone.")
        return

    uri = found[0][0]
    print(f"Connecting to {uri}...")

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        print("Sending backward burst to slide off rail...")
        for _ in range(20):
            cf.commander.send_setpoint(0, -20, 0, 40000)
            time.sleep(0.05)

        cf.commander.send_setpoint(0, 0, 0, 0)
        time.sleep(0.1)
        cf.commander.send_stop_setpoint()

    print("Done.")


if __name__ == '__main__':
    main()
