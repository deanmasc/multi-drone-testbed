#!/usr/bin/env python3
"""Find every Crazyflie the dongle can hear, including ones on a non-default address.

The plain scan everyone uses --

    python3 -c "import cflib.crtp; cflib.crtp.init_drivers(); print(cflib.crtp.scan_interfaces())"

-- only looks at the factory-default address E7E7E7E7E7. The moment you separate
two drones by address (which is the right way to run two on one dongle) that scan
stops finding one of them, and a drone you cannot find looks exactly like a drone
that is broken.

This sweeps a list of candidate addresses instead, so a drone whose radio
settings you have lost or half-changed still turns up.

    python3 scripts/find_drones.py
    python3 scripts/find_drones.py --address E7E7E7E7E9 E7E7E7E7EA

Read-only: it never writes anything to a drone.
"""

import argparse
import sys

DEFAULT_ADDRESSES = [
    'E7E7E7E7E7',        # Bitcraze factory default -- drone_1
    'E7E7E7E7E8',        # the recommended second address -- drone_2
    'E7E7E7E7E9',
    'E7E7E7E7E0',
    'E7E7E7E701',        # appeared in an old test script, kept in case it is real
]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--address', nargs='*', default=DEFAULT_ADDRESSES,
                    help='hex radio addresses to try (default: the usual suspects)')
    a = ap.parse_args()

    try:
        import cflib.crtp
    except ImportError:
        sys.exit('cflib is not installed here. Run this on the lab PC.')

    cflib.crtp.init_drivers()

    found = []
    for addr in a.address:
        try:
            n = int(addr, 16)
        except ValueError:
            print(f'  {addr:<12} skipped -- not hex')
            continue
        try:
            hits = cflib.crtp.scan_interfaces(n)
        except Exception as exc:                      # noqa: BLE001
            print(f'  {addr:<12} scan failed: {exc}')
            continue
        if hits:
            for uri, _ in hits:
                # scan_interfaces omits the address for the default case, but
                # crazyflie_cpp needs the full four-part form in crazyflies.yaml.
                # Count the segments after the scheme, not the slashes in the
                # whole string -- "radio://0/20/2M" already contains four.
                tail = uri.split('://', 1)[-1]
                full = uri if tail.count('/') >= 3 else f'{uri}/{addr}'
                print(f'  {addr:<12} FOUND  {full}')
                found.append(full)
        else:
            print(f'  {addr:<12} nothing')

    print()
    if not found:
        print('No drones found on any address tried.')
        print('  - is the Crazyradio plugged in, and the drone powered on?')
        print('  - if you changed an address to something not listed above,')
        print('    pass it with --address')
        sys.exit(1)

    print(f'{len(found)} drone(s) found. Paste the full URI into '
          f'config/crazyflies.yaml.')
    if len(found) > 1:
        print('\nMore than one drone answered. They are told apart only by what')
        print('you see above, so power them on ONE AT A TIME if you need to know')
        print('which physical airframe is which -- that ambiguity has already')
        print('caused one misidentification in this project.')


if __name__ == '__main__':
    main()
