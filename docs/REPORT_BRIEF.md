# Report brief — orientation for writing up the testbed

**Written for an agent or person starting cold on the write-up.** Everything
here is a pointer plus the one thing you need to know before you follow it.
Last updated 2026-10-07, after the 6 Oct sessions.

Read this first, then `docs/PROJECT_AIM.md` — which is 2300 lines and is the
primary source, not a summary. This file exists so you know *which* part of it
to read and which numbers in it are still safe to quote.

---

## 1. What the project is

FIT4701 final-year project, Monash. Supervisor **Hoam Chung**, who co-authored
the trochoidal consensus paper the testbed implements. A Crazyflie / ROS 2 /
VICON testbed flies five published multi-agent control laws, in simulation and
on hardware, with up to two real aircraft and the rest simulated in the same
fleet ("hybrid" configs).

**The aim is to EXPLAIN the sim-to-hardware gap, not to minimise it.** Which
idealisations in the source papers turn out to be load-bearing on real
hardware, and why. A tuned-down gain that makes the drone fly prettily is not a
result; identifying *which assumption* the hardware violates is. This reframing
is dated 2026-09-01 and is set out in `PROJECT_AIM.md` §1–2. Do not write the
report as an engineering-improvement narrative.

**The success criterion is the theorem's property, not the trajectory**
(`PROJECT_AIM.md` §3). Olfati-Saber promises cohesion, collision avoidance and
velocity matching — not a path. Path deviation is only a failure when the
promised property breaks. This distinction is load-bearing for the whole
discussion chapter and is the easiest thing to get wrong.

**Three things are being critiqued, in three different ways** (§4): the papers'
*applicability* (never their correctness — no drone refutes a proof), our own
implementation choices, and the hardware's limits.

**Gap A / gap B** (§8): THEORY →(A)→ SIMULATION →(B)→ HARDWARE. Gap A is
algorithmic (discretisation, saturation, gain choice); gap B is physical
(delay, noise, motor limits, aerodynamics). Attributing each effect to A or B
is the core analytical move of the report.

---

## 2. The claims, in order of strength

### Primary — the k·τ threshold

Every one of the five control laws commands an acceleration containing a brake
term `−k·v` acting on a velocity measured **τ ≈ 0.28 s ago**. Form the
dimensionless product **k·τ**, where k is the coefficient on the drone's *own*
velocity:

| algorithm | k is | source paper |
|---|---|---|
| Trochoidal consensus | `beta` | Monsingh, Sinha & Chung, *EJC* 76 (2024) 100928, eq. (7) |
| Coverage | `gain_kd` | Cortes, Martinez, Karatas & Bullo, *IEEE TRA* 20(2), 2004 |
| Flocking | `c2_gamma + c2_alpha · Σ bump` | Olfati-Saber, *IEEE TAC* 51(3), 2006, Algorithm 2 |
| Kuramoto formation | `velocity_gain` | **no citation recorded in the repo** |
| Distance formation | `gain_kv` | **no citation recorded in the repo** |

> **Citation gap — resolve before writing.** `PROJECT_AIM.md` §5a cites papers
> for the first three only. The Kuramoto and distance-formation implementations
> have no source anywhere in the repo, the configs, or the code comments. Ask
> Dean for them. Until then write **"five control laws"**, never "five papers",
> and do not describe the 6 Oct ladders as testing anyone's published claim —
> only as testing the same *mechanism*. Everything else about those two results
> stands; it is the attribution that is missing, not the data.

**Below k·τ ≈ 0.67 nothing oscillates (0.5–1.9 cm). Above ≈ 0.81 everything
does (16–23 cm), with a period of 4τ ≈ 1.1 s regardless of which paper the law
came from.** 25 real-drone rungs, five control laws.

The onset band was **narrowed from 0.67–1.01 to 0.67–0.81 on 6 Oct**
(`PROJECT_AIM.md` §16g). Use 0.67–0.81. The older figures and §13/§15 text say
0.67–1.01; they predate the Kuramoto and distance-formation rungs.

**Kuramoto and distance formation are an out-of-sample test**, not another fit:
their rungs were chosen in advance from the other three algorithms' numbers and
nothing was tuned afterwards (§16e). Say so in the report — it is the
difference between a correlation and a prediction.

Headline figure: `docs/figures/key/1_threshold.png`.

### Secondary — and arguably more interesting: metric blindness

**A published metric can only see a sim-to-hardware gap if it is a function of
the quantity the disturbance perturbs.** Four noise ladders now give all three
possible outcomes, which is what makes the claim precise rather than cynical:

| algorithm | did the flight degrade? | did its own metrics report it? |
|---|---|---|
| Coverage | yes, ×5.1 (×4.0 above null) | **no** — 4/4 flat, 2–9% spread |
| Flocking | yes, ×1.95 above null | **no** — 4/4 flat |
| Distance formation | yes, ×3.0 (×2.1 above null) | **yes** — 3/4 move, edge error 1.23→2.20 cm, Lyapunov W ×4.3 |
| Kuramoto | **no** — 13.4→13.3 cm, sits on its null | n/a, nothing to report |

An edge length is a direct function of the positions the noise corrupts, so it
cannot be blind. A Voronoi cost and a phase order parameter are not, so they
are. Kuramoto also shows that noise is **not universally harmful**: a law
holding the fleet to a prescribed trajectory with strong position feedback
averages radial noise out, while trochoidal (unanchored) and coverage (a
diffusion) both move by ×5.

Two independent demonstrations of the blindness half:

- *Under sensor noise* (§16b): coverage's four published metrics (mean and worst
  distance to own Voronoi centroid, locational cost H, fraction of ticks where H
  rose) span **2.0–8.6%** of their own means across a 0→20 mm ladder, with no
  trend, while deviation rises ×5.1 and tilt ×2.0 on channels the injection
  cannot reach. Flocking's four (lattice error, `vel_spread`, `d_min`,
  connectivity) do the same, once `vel_spread` is identified as an artefact.
- *Under delay* (§16e): Kuramoto's order parameter R reads **1.0000 / 0.9909 /
  0.9962** across a ladder that puts the aircraft at 21 cm of oscillation, 29°
  of bank and 80% clipping. Distance formation's edge error, on the *same*
  ladder and the *same* aircraft, reads **27× its own simulation**. R is
  invariant to the radial motion the delay produces; an edge length is not.

### Supporting — the physical-layer levers

A parameter *inside the control law* (hotspot speed, sense range, graph
topology) cannot widen the hardware-vs-simulation gap, because the simulator
runs the same parameter from the same file. Only a physical-layer lever can.
Two have been swept:

- **Sensor noise** on drone1's VICON position (0–20 mm per axis): widens gap B
  on all three algorithms tested (§15k, §16b).
- **Fleet size**, one real aircraft against two (§16d): costs little at 0 mm
  (2.7 → 3.6 cm) and a great deal at 10 mm (12.6 → **36.4 cm**). The *simulated*
  agents also move (4.4 → 18.4 cm) despite carrying no noise, so measurement
  error propagates through the interaction graph.

---

## 3. Numbers you must NOT quote

| where | what | why |
|---|---|---|
| `PROJECT_AIM.md` §15k | "median flown speed 0.035 → 0.117 m/s" | the recorder logs the **noisy** position channel; differencing it at 10 Hz multiplies the injected noise ×14. See §16a. |
| any pre-6-Oct text | commanded/achieved **acceleration** on a noise rung | second-differencing multiplies the injection ×245 |
| flocking noise ladder | `vel_spread` as evidence of degradation | it reads the least-squares velocity, where the fit has already multiplied position noise ×11 |
| flocking noise ladder | the **ripple** column as an oscillation result | only ×1.20 above its sensor-only null — essentially all artefact |
| `testbed_fig4.yaml` | "drone1–drone4 is 0.354 m, the only safe pair" | true of the configured marks; the drone was not on its mark 23 Sep – 30 Sep, and as flown the pair's designed separation was 15 cm median / 6 cm min |
| §13, §15f | the onset band "0.67–1.01" | narrowed to **0.67–0.81** on 6 Oct (§16g) |
| any distance-formation record before a rebuild | the **header's** breathing parameters | the header is written by the recorder from `src/`; the controller was running the install space and never breathed (§16f) |

**The general rule this corpus learned twice:** before claiming a hardware
effect on a noise rung, ask what a *null* flight would have read — the control
trajectory re-measured through that rung's noise. `add_nulls()` in
`tools/plot_noise_ladder_algos.py`. Report the gap above the null, never the
raw bar. Tilt (from `/poses`) is the one motion witness the injection cannot
reach.

---

## 4. Limitations that must appear in the report

1. **Almost no error bars.** Every cell is a single flight except two, reflown
   7 Oct: coverage 2 mm (10.3 and 7.9 cm — the two span 2.4 cm, wider than the
   gap to the 5 mm rung, so that ladder's middle is not resolved) and flocking
   0 mm (5.5 and 5.2 cm — highly repeatable, and still above its own 2 mm
   rung, so that inversion is real rather than a bad run). Figure:
   `docs/figures/repeatability/1_repeats.png`. State this once, clearly.
2. **The placement fault** (§16c): 23 Sep – 30 Sep the real drone1 was placed on
   drone4's floor mark, 41–48 cm from its own. Corrected 6 Oct (0.6 and 2.1 cm).
   Deviation results survive because every comparison is against that run's own
   replay from its *actual* start, but "the designed trochoidal pattern" in the
   23 Sep figures means the pattern that start produces, not the paper's fig. 4.
3. **The breathing experiment has not been run** (§16f) — a stale install space
   meant the controller flew the static shape. Needs a clean rebuild and a
   reflight.
4. **The two-real-drone flights are vertically separated** (1.41 m and 0.75 m)
   because otherwise the pair collides. Downwash is a confound in that row.
5. **Two instrumentation defects were found in our own analysis**, not in the
   hardware: the coverage-cost clock (§15d) and the noisy position channel
   (§16a). Both were caught by a null check. This belongs in the method
   section as a strength, not buried as an erratum.
6. **`s1` (flocking, k·τ 1.32) shows only 2.9 cm** of ripple despite 83%
   clipping — flocking's oscillation lives in the lattice, not in one drone's
   path, so the per-drone band-passed radius is the wrong instrument for it.

---

## 5. Where everything is

```
docs/PROJECT_AIM.md        the primary source, 2300 lines, chronological by session
docs/REPORT_BRIEF.md       this file
docs/LAB_PLAN_2026-10-06.md the 6-7 Oct session plan; Blocks D and E not yet flown
docs/DISTANCE_BREATHING.md breathing design notes (max_accel figure in it is stale)
docs/figures/index.html    55 figures, one tab per sweep, prose captions — START HERE
docs/figures/index_standalone.html  same, images inlined, sendable
logs/hw/                   66 flight records, NOT in git (.gitignore excludes logs/)
tools/                     analysis; every figure is reproducible from a record
ros2_ws/src/drone_testbed/ the flight code and the configs
```

Figure directories, each with its own `summary.json` that the captions read
from, so prose and plots cannot drift apart:

| directory | sweep | date |
|---|---|---|
| `key/` | the threshold, all five algorithms | — |
| `trochoidal_ladder/` | β = 1…14 | 15 Sep |
| `coverage_ladder/`, `flocking_ladder/` | time rescale | 16 Sep |
| `coverage_hotspot/`, `flocking_topology/` | in-law levers (null for gap B) | 22–23 Sep |
| `noise_ladder/` | trochoidal, 0–10 mm | 23 Sep |
| `noise_ladder_flocking/`, `noise_ladder_coverage/` | 0–10 / 0–20 mm | 29 Sep |
| `fleet_size/` | 1 real vs 2 real × 0/10 mm | 23 Sep + 6 Oct |
| `kuramoto_ladder/`, `distance_ladder/` | own-velocity gain ladder | 6 Oct |
| `flocking_sense/` | **null result, kept deliberately** | — |

Rebuild everything:

```bash
python3 tools/plot_trochoidal_ladder.py
python3 tools/plot_ladder.py --algo coverage|flocking|kuramoto|distance
python3 tools/plot_noise_ladder.py
python3 tools/plot_noise_ladder_algos.py --algo flocking|coverage
python3 tools/plot_levers.py
python3 tools/plot_fleet_size.py
python3 tools/plot_key.py
python3 tools/make_figure_index.py --embed
```

---

## 6. Vocabulary, used consistently throughout

- **k** — the coefficient on a drone's *own* velocity in its commanded
  acceleration. A gain on a neighbour's velocity, or on a velocity
  *difference*, does not count: the delay only destabilises the loop a drone
  closes on itself. `ladder_common.own_velocity_gain()` is the single
  definition.
- **τ** — the round-trip loop delay, state → command → thrust → motion,
  measured by cross-correlating the replayed command against the achieved
  acceleration. 0.25–0.28 s measured, 0.28 s nominal.
- **Ripple / wobble / oscillation radius** — RMS of the position band-passed to
  0.6–1.5 Hz, which brackets 4τ ≈ 1.1 s.
- **Deviation / design gap** — distance from where the law wanted the drone,
  which is the *same config in the noiseless simulator*, restarted from the
  fleet's actual positions at engage. Because the simulator has no noise and no
  physics, this distance **is** gap B.
- **Null** — the control flight's own trajectory re-measured through a rung's
  noise, 200 draws. The floor a bar must clear to mean anything.
- **Promise** — the property the source theorem actually guarantees (H for
  coverage, lattice error for flocking, order parameter R for Kuramoto, edge
  error for distance formation).
- **Rung labels** — `r*` trochoidal, `c*` coverage, `s*` flocking, `q*`
  Kuramoto, `h*` distance formation.

---

## 7. Open work, if there is lab time left

In priority order (`PROJECT_AIM.md` §16j):

1. **Repeats** — three flights of one config in one session, to put an error bar
   on anything at all.
2. **The breathing experiment**, after `rm -rf build/drone_testbed
   install/drone_testbed && colcon build`.
3. **A rung between k·τ 0.67 and 0.81**, to turn the bracket into a number.
4. **Kuramoto and distance formation under noise** — `*_k04_n10.yaml` exist,
   never flown; would take the metric-blindness result from two algorithms to
   four.
5. **`velocity_window`** — the only physical-layer lever never swept, and it
   sets the ×11 amplification in §16a directly.

Not a flight, but blocking: **the two missing citations** (§2 above).

Blocks D and E of `docs/LAB_PLAN_2026-10-06.md` were not flown.

---

## 8. Working with Dean

- Do not commit or push. Make the edits; he does his own commits.
- No unrequested code changes. Proposals as text are fine.
- Lab-PC commands must be copy-pasteable from any directory: absolute paths,
  distro detection, `diff` before `sudo cp`.
- Short, plain answers. When he reaffirms a decision, proceed without repeating
  the warning.
