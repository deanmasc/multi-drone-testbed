# Kuramoto neighbour-phase delay tests

Select `KuramotoPhaseDelay` to introduce discrete communication delay into
neighbour phase coupling. `KuramotoFormation` remains available with its
original behaviour and configurations.

The oscillator uses:

```text
theta_dot_i(t) = omega
  + phase_gain * sum_j w_ij * sin((theta_j(t - D) - offset_j)
                                - (theta_i(t) - offset_i))
  + tracking_phase_gain * min(distance_i / radius, 1)
      * sin(actual_angle_i(t) - theta_i(t))
```

Only `theta_j` in this coupling sum is delayed. Own phase, physical states,
and neighbour phases used for formation position targets remain current.
Published phase telemetry is also current. The acceleration law is unchanged;
`k = velocity_gain = 1`, multiplying `(target_velocity - measured_velocity)`.
The oscillator coupling coefficient stays `phase_gain = 0.25`.

| Config | Delay | Delayed controller steps |
|---|---:|---:|
| `testbed_kuramoto_delay_000ms.yaml` | 0 ms | 0 |
| `testbed_kuramoto_delay_050ms.yaml` | 50 ms | 1 |
| `testbed_kuramoto_delay_100ms.yaml` | 100 ms | 2 |
| `testbed_kuramoto_delay_200ms.yaml` | 200 ms | 4 |

All four use 20 Hz control and simulation (`dt = 0.05 s`), a 0.65 m ring,
`max_accel = 3.5 m/s²`, and no injected mocap noise. The original 10 Hz
config cannot represent a 50 ms delay in whole timesteps. Compare these tests
against the new 0 ms control, which has the same rate and gains.

Change `algorithm.params.phase_delay_ms` to choose a delay in milliseconds.
It must be a nonnegative whole multiple of the controller timestep. Alternatively,
remove that key and set `phase_delay_steps: N` for an integer number of ticks.
Setting both keys is rejected. Controller timestep changes require reset.

History contains phase snapshots from the start of each control step, before
any agent advances. During warm-up, the first available phase is held; reset
clears history and measured phase initialization. This models an additional
phase communication delay in controller ticks, on top of existing hardware
latency; it does not sleep or delay ROS telemetry delivery.

Build from the repository root:

```bash
source /opt/ros/humble/setup.bash
cd ros2_ws
colcon build --packages-select drone_testbed
source install/setup.bash
cd ..
```

For each test, set the same `CFG` in both terminals. Start the recorder first:

```bash
CFG=testbed_kuramoto_delay_050ms.yaml
python3 tools/metrics_recorder.py \
  --config ros2_ws/src/drone_testbed/config/$CFG \
  --hw drone1 --mocap-name drone_1 \
  --note "phase communication delay test; k=1; control_rate=20Hz; velocity_window=10; max_age=0.25"
```

Launch one real drone with three virtual neighbours, placing drone1 at
`(0.65, 0.00)`:

```bash
CFG=testbed_kuramoto_delay_050ms.yaml
ros2 launch drone_testbed hardware_hybrid.launch.py \
  config:=config/$CFG \
  hw_drone:=drone1 cf_name:=drone_1 mocap_name:=drone_1 \
  flight_duration:=180
```

For simulation:

```bash
ros2 launch drone_testbed sim.launch.py config:=config/testbed_kuramoto_delay_050ms.yaml
```

The recorder automatically uses the existing Kuramoto phase and physical
formation metrics. Its header includes the configured delay and gains.
