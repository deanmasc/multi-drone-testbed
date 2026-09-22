# Expanding/contracting distance formation

Edit `ros2_ws/src/drone_testbed/config/testbed_hexagon_breathing.yaml`.
This config uses the static hybrid's six starting positions, 0.7 m nominal
hexagon, octahedron graph, gains and single anchor. Use the same file in the
headless baseline, visual simulation and hybrid launch.

## Variables to change

| Variable under `algorithm.params` | Default | Meaning and test sequence |
|---|---:|---|
| **`breathing_period`** | **30.0 s** | **Change this first.** Seconds per complete expansion/contraction. Compare 60, 30, 20, 15 in simulation. Smaller values demand faster tracking. |
| **`breathing_amplitude`** | **0.10** | Fractional size swing. 0.10 means ±10%, giving 0.63–0.77 m sides at nominal 0.7 m. Then compare 0.05, 0.10, 0.15 at a fixed period. **0 disables breathing.** Must be in `[0, 1)`. |
| `gain_kp` | 0.45 | Formation correction gain. After the motion sweep, compare 0.15, 0.30, 0.45 at fixed period, amplitude and damping. |
| `gain_kv` | 1.2 | Velocity damping. Then compare 0.8, 1.2, 1.6 with the other settings fixed. |
| `breathing_start_delay` | 15.0 s | Hold nominal size before starting the wave. This is measured from controller activation, after the launch's takeoff hold. |
| `breathing_ramp_duration` | 5.0 s | Smoothly introduce the wave's amplitude; must be positive. Keep fixed across comparisons. |
| `side_length` | 0.7 m | Nominal hexagon size. Keep fixed: it changes both the motion envelope and force demand. |
| `max_accel` | 0.5 m/s² per axis | Acceleration limit. Keep fixed to compare controllers under the same constraint. |

Change one variable per comparison. Keep topology, anchor, initial placement,
control rate, estimation settings and altitude settings fixed. Save a separate
config for each condition. Record at least three full cycles after the ramp;
allow `delay + ramp + 3*period` controller seconds, plus takeoff/launch overhead
on hardware. For a 60 s period, use a longer run than the 150 s example below.
Repeat hardware conditions at least five times and retain aborted runs as such.

## Run

Visual simulation (the title displays the commanded scale):

```bash
python3 run_sim.py --config testbed_hexagon_breathing.yaml
```

Record a headless baseline, without ROS:

```bash
python3 tools/sim_baseline.py \
  --config ros2_ws/src/drone_testbed/config/testbed_hexagon_breathing.yaml \
  --duration 150
```

For ROS/hybrid, rebuild the changed Python package and source its installation
in the lab's ROS shell. The new reference publisher must be installed as well
as the new config:

```bash
cd ros2_ws
colcon build --symlink-install --packages-select drone_testbed
source install/setup.bash
cd ..
./fly.sh testbed_hexagon_breathing --real drone1,drone4 --duration 150 --record
```

The script saves the config and recordings in its run directory. Place the
real drones at the config's starting marks; VICON supplies their actual
positions. The static hybrid's documented clearance estimates are not a
guarantee for this moving experiment. Check each candidate's simulated motion
envelope before flight. Existing acceleration limits and altitude staggering
remain in place. No additional separation controller has been introduced.

## What changed in the controller

Let `A` be amplitude, `T` period, and `tau = max(0, controller_time - delay)`.
The scale is

```text
q = min(1, tau / ramp_duration)
ramp = q*q*(3 - 2*q)
scale = 1 + A*ramp*sin(2*pi*tau/T)
desired_edge_ij = nominal_edge_ij * scale
```

Every ring edge and chord changes together, keeping the target geometrically
consistent. The controller still applies the distance-gradient and velocity
damping law; it does not add a motion feedforward term. Tracking lag is an
experimental result, not something hidden by supplying the solution.

Anchor targets scale about `formation_center` too. In the supplied config,
drone1's anchor moves along x from 0.63 to 0.77 m after the ramp, with y = 0.
This keeps the absolute anchor reference consistent with scaling about the
configured center (a fixed anchor would instead imply a different placement).
The scale schedule restarts on reset/reconfiguration. Existing configs without
breathing parameters use amplitude zero and retain the static control law and
metric columns.

## Measurements

`algorithm_manager` publishes `/distance_formation/reference`, a
`Float64MultiArray` containing `[controller_time_seconds, target_scale]`, each
control tick. This is the reference actually used by that tick. Reset or
algorithm switch invalidates it. A recorder started partway through a flight
uses that reference rather than starting a separate sine wave at recording time.

The existing recorder now compares edge distances, fitted shape and potential
against the current target. Breathing records add these columns:

| Column | Meaning |
|---|---|
| `reference_time` | Controller time, independent of the recorder's `t` |
| `reference_scale` | Requested size / nominal size |
| `actual_scale` | Least-squares scale fitted from all measured graph-edge lengths |
| `reference_age` | Seconds since reference receipt |
| `max_state_age` | Age of the oldest agent state, measured at receipt |
| `reference_valid` | 1 only when the reference matches the configured schedule and reference/state ages are at most 0.5 s |

Missing or stale references/states produce `nan` formation metrics; raw x/y
positions remain available. Samples after an observed abort are also excluded.
These are latest-message samples, not synchronized source-time measurements;
sub-tick lag estimates include sampling/receipt effects. Freshness checks do
not change the algorithm manager's handling of stale control inputs.

After the delay and ramp, the summary reports:

- Edge RMS over the samples, the 95th percentile of instantaneous edge RMS,
  and the worst individual edge error.
- Shape RMS and peak error, allowing translation/rotation/reflection but not
  fitting away scale errors.
- The sample fraction meeting the existing 1 cm edge-RMS and 5 cm shape
  thresholds. This is not a sustained-success or collision-safety criterion.
- Scale response amplitude divided by requested amplitude (ideal = 1).
- Fundamental scale phase lag in seconds (positive means behind the request).
  This requires a continuous full cycle after the ramp. The fit is modulo one
  cycle and summarizes the fundamental, not arbitrary waveform distortion.
- Minimum planar separation over valid samples, including the initial transient.

Plot `reference_scale` and `actual_scale` against `reference_time`, plus
`edge_rms`, `shape_err` and pair separation. Also retain the `--record` logs
for commanded-versus-measured position and onboard-estimate checks.
`/cmd_accel` remains the already-clipped acceleration; pre-clamp demand is not
recorded by this change.

Changing targets can add energy, so the breathing summary does **not** apply
the static "W must decrease" or "formation settled" verdicts. Anchored runs
also do not test centroid invariance. Use `breathing_amplitude: 0` for a matched
static control experiment.

To trim takeoff/landing or select a later cycle, reanalyse with the exact saved
config and recorder-clock bounds:

```bash
python3 tools/metrics_recorder.py --config /absolute/path/to/saved_config.yaml \
  --analyse /path/to/record.txt --from 40 --to 130
```

The absolute config path is honoured, as it is by `fly.sh`; a stale installed
copy will not silently override it. Reanalysis retains the recorded controller
phase. A static record cannot be reinterpreted as a breathing run.

The headless baseline includes the algorithm's acceleration clamp, but not the
ROS drone nodes' speed/boundary clamps or hardware tracking/leash dynamics.
Treat it as an ideal controller baseline, not a flight-envelope guarantee.

## Verification

Run controller/metrics regression tests without ROS:

```bash
python3 -m unittest discover -s ros2_ws/src/drone_testbed/test -p 'test_*.py'
```

The tests cover static-law preservation, valid scale bounds, smooth onset,
scaled anchors, reset, force cancellation, late recording, stale references,
known scale lag/amplitude, and offline reanalysis.
