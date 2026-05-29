# Multi-Drone Testbed

A research testbed for developing and evaluating distributed control algorithms across multiple Crazyflie nano-drones. The system supports both software-in-the-loop simulation (no hardware required) and full hardware deployment using VICON motion capture and the Crazyswarm2 ROS2 framework.

---

## Overview

Modern drone deployments increasingly rely on fleets of small, low-cost agents rather than a single large vehicle. Coordinating these fleets presents a fundamental bottleneck: centralised control requires every drone to maintain a constant high-bandwidth uplink to a ground station, which becomes infeasible as fleet size grows. This project investigates **distributed control** as a solution — each drone makes decisions using only local state information and (optionally) data from nearby neighbours, with no reliance on a central coordinator.

The testbed provides:
- A physics-accurate 2D simulation for rapid algorithm iteration
- A pluggable algorithm interface so new strategies can be added without changing the core framework
- A hardware pipeline connecting real Crazyflie 2.x drones through VICON pose estimation to the same algorithm interface
- Three implemented distributed algorithms: Leader-Follower, Consensus Formation, and Trochoidal

---

## System Architecture

```
┌─────────────────────────────────────────────────────┐
│                   Algorithm Layer                    │
│  LeaderFollower │ ConsensusFormation │ Trochoidal   │
│         (implements BaseAlgorithm ABC)               │
└──────────────────────┬──────────────────────────────┘
                       │ ControlOutput [ax, ay]
          ┌────────────┴────────────┐
          │                         │
   ┌──────▼──────┐           ┌──────▼──────┐
   │  Simulation  │           │  Hardware    │
   │  (run_sim.py)│           │  (ROS2)      │
   │  NumPy +     │           │  Crazyswarm2 │
   │  Matplotlib  │           │  + VICON     │
   └─────────────┘           └─────────────┘
```

Both paths share the same algorithm interface. An algorithm developed and tuned in simulation can be deployed to hardware without modification.

### ROS2 Node Graph (Hardware Mode)

```
VICON Tracker ──► mocap_state_node ──► /droneX/state ──► algorithm_manager
                                                               │
                                                    /droneX/cmd_accel
                                                               │
                                               crazyflie_node ──► Crazyflie drone
```

---

## Implemented Algorithms

### 1. Leader-Follower (`LeaderFollower`)

The leader drone tracks a predefined circular path. Each follower drone applies PD control to maintain a fixed offset from the leader's current position.

- **Communication:** Followers read the leader's state (one-hop)
- **Control law:** `a_i = kp * (p_leader + offset_i - p_i) + kd * (v_leader - v_i)`
- **Use case:** Rigid formation maintenance along a known trajectory

### 2. Consensus Formation (`ConsensusFormation`)

Each drone communicates only with its configured neighbours and drives the relative positions toward a desired formation geometry. No drone has global knowledge of the formation.

- **Communication:** Neighbour-to-neighbour only (configurable adjacency graph)
- **Control law:** `a_i = kp * Σ_j w_ij((p_j - p_i) - (d_j - d_i)) + kd * Σ_j w_ij(v_j - v_i)`
- **Use case:** Fully distributed formation control over sparse communication topologies

### 3. Trochoidal (`Trochoidal`)

Each drone independently follows a trochoidal trajectory — the superposition of a slow large orbit and a fast small epicycle — with evenly spaced phase offsets. No inter-drone communication is needed at all.

- **Communication:** None (fully independent)
- **Path:** `p_i(t) = [R·cos(Ωt + φ_i) + r·cos(ωt + φ_i), R·sin(Ωt + φ_i) + r·sin(ωt + φ_i)]`
- **Use case:** Coverage and area-scanning patterns where communication cannot be guaranteed

### Algorithm Parameters

All algorithms are configured via `ros2_ws/src/drone_testbed/config/testbed.yaml`:

```yaml
algorithm:
  name: "Trochoidal"   # LeaderFollower | ConsensusFormation | Trochoidal
  params:
    orbit_radius: 0.8
    orbit_speed: 0.3
    epicycle_radius: 0.2
    epicycle_speed: 1.5
```

---

## Adding a New Algorithm

The plugin system uses a `@register_algorithm` decorator. To add a new algorithm:

1. Create a new file in `ros2_ws/src/drone_testbed/drone_testbed/algorithms/`
2. Subclass `BaseAlgorithm` and implement three methods:

```python
from drone_testbed.algorithms.base_algorithm import BaseAlgorithm
from drone_testbed.algorithms.registry import register_algorithm

@register_algorithm
class MyAlgorithm(BaseAlgorithm):

    def name(self) -> str:
        return "MyAlgorithm"

    def configure(self, params: dict, drone_ids: list) -> None:
        # Called once at startup with params from testbed.yaml
        ...

    def compute_controls(self, states: dict, dt: float) -> dict:
        # Called at control_rate Hz. Return {drone_id: ControlOutput}
        ...
```

3. Import it in `algorithms/__init__.py`

No other changes are required. The algorithm is immediately available in both simulation and hardware modes.

---

## Dynamics Model

The simulation uses a **2D double integrator**:

```
State:    [x, y, vx, vy]
Control:  [ax, ay]

x(t+dt)  = x + vx·dt + 0.5·ax·dt²
vx(t+dt) = vx + ax·dt
```

This is the standard model for quadrotor horizontal motion when altitude is held constant — valid in the near-hover regime where roll/pitch angles are small. The state-space matrices `(A, B)` are also exposed for model-based algorithm design.

---

## Quick Start — Simulation

No ROS2 or hardware required. Works on macOS and Linux.

**Prerequisites:**
```bash
pip3 install numpy matplotlib pyyaml
```

**Run:**
```bash
python3 run_sim.py --algo Trochoidal
# Options: LeaderFollower | ConsensusFormation | Trochoidal
```

The simulation window shows:
- Drone positions (coloured markers) with ID labels
- Velocity vectors
- Position trails
- Live algorithm name and elapsed time

---

## Hardware Setup (Ubuntu 24.04, one-time)

### 1. Clone the repo

```bash
git clone <repo-url> multi-drone-testbed
cd multi-drone-testbed
```

### 2. Run the setup script

```bash
bash setup.sh
```

Installs ROS2 Jazzy, Crazyswarm2 (via apt), builds the `drone_testbed` package, and sets up Crazyradio USB permissions. Takes approximately 10–15 minutes.

After setup completes, **reboot** to apply USB group permissions.

### 3. Configure the drone

Find the drone's radio URI with the Crazyradio plugged in:
```bash
python3 -c "import cflib.crtp; cflib.crtp.init_drivers(); print(cflib.crtp.scan_interfaces())"
```

Edit `config/crazyflies.yaml`:
```yaml
robots:
  cf1:
    uri: radio://0/90/1M        # output from scan above
    ...
    motion_capture:
      name: cf1                 # rigid body name in VICON Tracker (case-sensitive)
```

### 4. Configure VICON

Edit `config/motion_capture.yaml`:
```yaml
hostname: 192.168.11.3    # IP of the lab PC running VICON Tracker
```

Find the lab PC's IP by running `ipconfig` on the Windows machine.

### 5. Pre-flight checklist

- [ ] Crazyradio USB dongle plugged in
- [ ] Drone powered on and blinking (idle state)
- [ ] Drone placed flat inside the VICON capture volume
- [ ] VICON Tracker is open and in **Live** mode
- [ ] Drone rigid body is visible and tracked in VICON
- [ ] Radio URI confirmed with `cflib.scan`
- [ ] VICON body name matches `config/crazyflies.yaml` exactly

### 6. Run the hardware flight test

```bash
source ~/.bashrc
bash run_hardware_test.sh
```

The drone will take off to 0.5 m, hover, move up to 0.8 m, return to 0.5 m, then land.

---

## Running a Distributed Algorithm on Hardware

Once the basic flight test works, launch the full stack:

```bash
# Terminal 1 — Crazyswarm2
ros2 launch crazyflie launch.py

# Terminal 2 — VICON state + algorithm
ros2 launch drone_testbed hardware_single.launch.py \
    drone_id:=drone1 cf_name:=cf1 mocap_name:=cf1
```

The active algorithm is set in `config/testbed.yaml` under `algorithm.name`.

---

## Lab Computer Requirements (Windows, VICON PC)

The VICON PC requires no software changes if it is already running VICON Tracker. Confirm:

1. **VICON Tracker is open** and in Live mode before launching the flight stack
2. **DataStream is enabled** — in Tracker: System → DataStream → enable (port 801)
3. **Firewall allows port 801** inbound — if not, an administrator needs to add the rule
4. **The drone rigid body exists** in Tracker with a known name

---

## Repository Structure

```
multi-drone-testbed/
├── run_sim.py                         # Standalone simulation (no ROS2 needed)
├── run_hardware_test.sh               # One-drone hardware flight test
├── setup.sh                           # Ubuntu 24.04 one-time setup script
├── config/
│   ├── crazyflies.yaml                # Drone radio URI + VICON body name
│   └── motion_capture.yaml            # VICON PC IP address
├── scripts/
│   ├── simple_flight.py               # Crazyswarm2 takeoff/hover/land test
│   └── test_flight.py                 # Raw cflib thrust test (legacy firmware)
└── ros2_ws/src/drone_testbed/
    ├── drone_testbed/
    │   ├── algorithms/
    │   │   ├── base_algorithm.py      # Abstract base class (ABC)
    │   │   ├── registry.py            # @register_algorithm decorator + lookup
    │   │   ├── leader_follower.py     # Leader-Follower implementation
    │   │   ├── consensus.py           # Consensus Formation implementation
    │   │   └── trochoidal.py          # Trochoidal implementation
    │   ├── dynamics/
    │   │   └── double_integrator.py   # 2D physics model (A, B matrices + step)
    │   ├── utils/
    │   │   ├── types.py               # DroneState, ControlOutput dataclasses
    │   │   └── config_loader.py       # YAML config loader
    │   ├── drone_node.py              # Simulated drone (ROS2)
    │   ├── crazyflie_node.py          # Hardware drone node (Crazyswarm2)
    │   ├── mocap_state_node.py        # VICON → drone state publisher
    │   ├── algorithm_manager.py       # Runs selected algorithm at control rate
    │   └── sim_visualizer.py          # Live matplotlib visualiser (ROS2 mode)
    ├── config/testbed.yaml            # Drone config + algorithm selection
    └── launch/
        ├── sim.launch.py              # Full simulation launch
        └── hardware_single.launch.py  # Single drone hardware launch
```

---

## Dependencies

| Dependency | Purpose | Install |
|---|---|---|
| Python 3.10+ | Core language | System |
| NumPy | Physics / linear algebra | `pip3 install numpy` |
| Matplotlib | Simulation visualisation | `pip3 install matplotlib` |
| PyYAML | Config loading | `pip3 install pyyaml` |
| ROS2 Jazzy | Hardware middleware | `setup.sh` |
| Crazyswarm2 | Crazyflie ROS2 interface | `setup.sh` |
| cflib | Direct Crazyflie comms | `pip3 install cflib` |
| VICON Tracker | Motion capture (lab only) | Lab installation |
