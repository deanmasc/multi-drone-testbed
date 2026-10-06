# Multi-drone testbed

Crazyflie / ROS 2 / VICON testbed flying five published multi-agent control laws
in simulation and on hardware. FIT4701 final-year project, Monash; supervisor
Hoam Chung. Dev on macOS, flights on the Ubuntu/VICON lab PC.

## Read these before doing anything substantive

| file | what it is |
|---|---|
| `docs/REPORT_BRIEF.md` | **start here.** The claims, the numbers that are safe to quote, the ones that are not, and where everything lives. |
| `docs/PROJECT_AIM.md` | the primary source, ~2300 lines, chronological by session. §1–8 is the framing, §11 onward is per-session findings. |
| `docs/figures/index.html` | 55 figures with prose captions, one tab per sweep. |

**The aim is to explain the sim-to-hardware gap, not minimise it.** A tuned gain
that flies prettily is not a result. Identifying which of a paper's idealisations
is load-bearing on real hardware is.

## The one idea

Every control law here commands an acceleration containing a brake `−k·v` acting
on a velocity measured τ ≈ 0.28 s ago. Below **k·τ ≈ 0.67** nothing oscillates;
above **≈ 0.81** everything does, at a period of 4τ ≈ 1.1 s. `k` is the
coefficient on a drone's *own* velocity and nothing else —
`tools/ladder_common.py:own_velocity_gain()` is the single definition.

## Layout

```
ros2_ws/src/drone_testbed/   flight code; algorithms/ holds one file per law
  config/                    one yaml per experiment — the config decides which law runs
tools/                       analysis; every figure rebuilds from a record
  metrics_recorder.py        also the recorder that runs in the lab
  ladder_common.py           shared: load, window, replay, lag, wobble
  sim_baseline.py            the noiseless reference ("where the law wanted it")
logs/hw/                     flight records — NOT in git, local only
docs/figures/<sweep>/        PNGs + summary.json that the captions read from
```

## Traps that have cost real time

- **The recorder logs the *noisy* position channel.** `x`/`y` come from
  `/<drone>/state`, downstream of the artificial noise injection; only `z` and
  `tilt` come from `/poses`. Differencing position at 10 Hz multiplies injected
  noise ×14, second-differencing ×245. Never quote speed or acceleration on a
  noise rung. Always compare against a null. (`PROJECT_AIM.md` §16a)
- **A record's header is written by the recorder from `src/`**, and is not
  evidence about what the flight controller ran — the controller loads the
  install space. A stale install space silently flew the wrong law on 6 Oct.
  (§16f)
- **Check placement at engage.** Compare drone1's row-0 position against
  `cfg['drones'][0]['initial_position']`. It sat on drone4's mark, 41 cm out,
  from 23 Sep to 30 Sep without anyone noticing. (§16c)
- **Separate the observation from the inferred cause**, and say which is which.
  Two "hardware findings" in this project turned out to be defects in our own
  analysis; both were caught by asking what a null flight would have read.

## Working agreements

- **Never `git commit` or `git push`.** Make the edits and stop; Dean commits.
- No unrequested code changes. Ideas as text are fine; edits only on an explicit
  ask, and then directly — no design-tool runs or clarifying questions first.
- Lab-PC commands must be copy-pasteable from any directory: absolute paths,
  `REPO=~/Desktop/multi-drone-testbed`, detect the ROS distro, `diff` before
  `sudo cp`. Lab PC user `tam`, ROS humble, Crazyswarm2 is apt-installed.
- Short, plain answers. When Dean reaffirms a decision, proceed without
  repeating the warning.
