# Multi-Drone Testbed

A testbed for developing and testing distributed control algorithms on real Crazyflie drones, with support for simulation (no hardware needed) and hardware flight via VICON motion capture + Crazyswarm2.

---

## Quick Start — Simulation (Mac or Linux, no hardware needed)

```bash
pip3 install numpy matplotlib pyyaml
python3 run_sim.py --algo LeaderFollower
# Other options: ConsensusFormation, Trochoidal
```

---

## Hardware Setup (Ubuntu 26.04 Linux, one-time)

### 1. Clone the repo

```bash
git clone <repo-url> multi-drone-testbed
cd multi-drone-testbed
```

### 2. Run the setup script

```bash
bash setup.sh
```

This installs ROS2 Humble, Crazyswarm2 (from source), builds the workspace, and sets up Crazyradio USB permissions. Takes ~10–15 minutes.

**After setup, log out and back in** (or reboot) so the USB group change takes effect.

### 3. Configure the drone

Edit **`config/crazyflies.yaml`**:

```yaml
robots:
  cf1:
    uri: radio://0/80/2M/E7E7E7E7E7   # find with: python3 -m cflib.scan
    ...
    motion_capture:
      name: cf1   # VICON body name (must match exactly in VICON Tracker)
```

To find your drone's URI, plug in the Crazyradio and run:
```bash
python3 -m cflib.scan
```

### 4. Configure the VICON connection

Edit **`config/motion_capture.yaml`**:

```yaml
hostname: 192.168.1.100   # IP of the lab PC running VICON Tracker
```

You can find the lab PC's IP by running `ipconfig` on the Windows machine.

### 5. Run the hardware flight test

```bash
source ~/.bashrc
bash run_hardware_test.sh
```

The drone will take off to 0.5 m, hover, move up to 0.8 m, come back down, then land.

---

## Hardware — Pre-flight Checklist

- [ ] Crazyradio USB dongle plugged in
- [ ] Drone powered on (blinking slowly = idle, ready to connect)
- [ ] Drone placed flat on the floor in the VICON capture volume
- [ ] VICON Tracker is running and showing the drone body
- [ ] `config/crazyflies.yaml` URI matches the drone (verify with `python3 -m cflib.scan`)
- [ ] `config/crazyflies.yaml` body name matches VICON exactly (case-sensitive)
- [ ] `config/motion_capture.yaml` IP matches the VICON PC

---

## Repository Structure

```
multi-drone-testbed/
├── run_sim.py                    # Standalone simulation (no ROS2)
├── run_hardware_test.sh          # One-drone hardware flight test
├── setup.sh                      # Ubuntu 22.04 setup script
├── config/
│   ├── crazyflies.yaml           # Drone radio URI + VICON body name
│   └── motion_capture.yaml       # VICON PC IP address
├── scripts/
│   ├── simple_flight.py          # Simple takeoff/hover/land (crazyflie_py)
│   └── test_flight.py            # Raw cflib thrust test (old firmware)
└── ros2_ws/src/drone_testbed/
    ├── drone_testbed/
    │   ├── algorithms/           # Pluggable distributed algorithms
    │   │   ├── base_algorithm.py
    │   │   ├── leader_follower.py
    │   │   ├── consensus.py
    │   │   └── trochoidal.py
    │   ├── crazyflie_node.py     # Hardware drone node (ROS2 + Crazyswarm2)
    │   ├── mocap_state_node.py   # VICON → drone state
    │   └── algorithm_manager.py  # Runs selected distributed algorithm
    ├── config/testbed.yaml       # Simulation + algorithm parameters
    └── launch/
        ├── sim.launch.py         # Full simulation launch
        └── hardware_single.launch.py  # Single drone hardware launch
```

---

## Running a Distributed Algorithm on Hardware

Once the basic flight test works, run the full stack:

```bash
# Terminal 1 — Crazyswarm2
ros2 launch crazyflie launch.py

# Terminal 2 — Our testbed (VICON state + algorithm)
ros2 launch drone_testbed hardware_single.launch.py \
    drone_id:=drone1 cf_name:=cf1 mocap_name:=cf1
```

The algorithm is set in `ros2_ws/src/drone_testbed/config/testbed.yaml` under `algorithm.name`.
