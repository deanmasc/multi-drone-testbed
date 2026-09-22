# Project Aim

*Written 2026-09-01, after the supervisor meeting. This document exists because
the team's working understanding of the project ("run algorithms, measure how
far the real flight deviates from the simulated one") was not what the
supervisor was asking for. This is the corrected framing.*

---

## 1. The aim, in one sentence

> We built a testbed that runs published multi-agent control laws on real
> quadrotors, catalogued the simplifications each law requires in order to be
> flyable, predicted from the mathematics which of those simplifications would
> degrade behaviour, and tested those predictions.

The **testbed is the deliverable**. The **finding** is which idealisations in
distributed control theory survive contact with hardware, and why.

## 2. What this is not

It is not "minimise the deviation between simulation and hardware."

Tuning gains until a real flight traces the simulated path produces a number
that is true of our drones, in our lab, on that day, with those batteries.
Nobody else can use it. It is engineering, and it generates no knowledge.

Explaining *why* the deviation has the size and shape it does transfers to
anyone else's hardware. That is the result.

**Deviation is the measurement, not the grade.**

## 3. The idea the whole report rests on

A theorem promises a **property**, not a **trajectory**.

Olfati-Saber never promises a specific path. It promises the group stays
together, does not collide, and matches velocities. Measuring path error
therefore tests something the theorem never claimed.

So the success criterion for each algorithm is: **did the property the theorem
promises survive the simplifications?** If yes, the simplification is
validated, and path deviation is simply the price of flying. Deviation is only
failure when the promised property breaks.

This is what the supervisor meant by *"a simplification that still works is a
success even though it deviates."*

## 4. Whose simplifications are we critiquing?

Three different things, critiqued in three different ways. Getting this wrong is
the easiest way to write a bad report.

**(a) The papers' assumptions — we critique their *applicability*, never their
*correctness*.** The distinction matters and is easy to blur:

- *Mathematical correctness* is not in question and cannot be tested by flying.
  The theorems are proved. No drone refutes a proof.
- *Practical applicability* is absolutely in question, and evaluating it is
  exactly what the testbed is for.

So if we implement a law faithfully and the promised behaviour does not appear,
we **can and should** say the assumptions do not hold up on real hardware, and
show the consequence. That is a legitimate, pointed critique — aimed at the
paper's engineering relevance, not its proof. It is also the main reason the
testbed exists.

Every simplification we made is us violating one of the papers' assumptions. We
discretised (violating continuous time), we clamped acceleration (violating
unbounded control), we differentiate VICON for velocity (violating exact state).
So the assumptions are our **hypotheses under test**:

- Violate an assumption and the promised property survives → that assumption was
  not load-bearing on real hardware. The result is more robust than the paper
  needed to claim.
- Violate it and the property breaks → it *was* load-bearing, and the paper's
  result is conditional in a way it did not emphasise.

**The discipline that makes the critique credible.** When an algorithm fails on
hardware there are three candidate causes, not one:

1. *Our harness* — too slow a control rate, too tight a clamp, too noisy an
   estimator.
2. *Our implementation* — we got the law or the gains wrong.
3. *An assumption no realistic hardware can satisfy* — the actual finding.

Claim (3) only after ruling out (1) and (2). Otherwise a reader hears "we could
not get it working, so the paper must be unrealistic," which is an excuse rather
than a result.

Earn (3) by showing the failure **tracks the assumption**, not merely that
failure occurred. For trochoidal: do not only show the pattern decays — vary the
suspected cause (run at 10, 25 and 50 Hz) and show the decay rate moves as
theory predicts. That demonstrates the mechanism, and makes the claim about the
algorithm rather than about our lab. Same discipline as the `grid_res` finding:
separate the observation from the inferred cause.

**Contingent vs necessary — the line that actually matters.** "Hardware caused
it" does not by itself decide between cause (1) and cause (3), because a
hardware limit can be either. The real question is whether the violation could
be engineered away:

- *Contingent* — better gear, more money or more effort would fix it.
- *Necessary* — no achievable hardware satisfies it.

**Worked examples from this system.** The test question is: *if we had unlimited
budget and a year, could we make this go away?*

Contingent — ours, fixable, so cause (1):

| Simplification | Why it is contingent |
|---|---|
| 25 Hz control rate | Set by our ROS 2 stack, Python, and shared radio bandwidth. A C++ node and more radio budget would run faster. |
| Velocity by differentiating VICON | A Kalman filter fusing VICON position with the onboard IMU would be far cleaner. We simply have not written one. |
| `max_accel` (0.5; 3.5 for trochoidal since 15 Sep) | Chosen for safety in a small flight volume. The airframe can do considerably more. |
| `max_lead` leash, geofence clamping | Our safety scaffolding. It distorts the control law whenever it engages, and it is entirely our choice. |

Necessary — no hardware fixes it, so cause (3):

| Simplification | Why it is necessary |
|---|---|
| Tilt-to-translate | A quadrotor thrusts along its body axis, so it must tilt before it can push sideways, and tilting takes torque and therefore time. Every algorithm says "apply acceleration `u` now"; the vehicle physically cannot. |
| Eigenvalues *exactly* on the imaginary axis | Measure-zero condition. Gains are floats, the loop is discrete, state is estimated. **Our strongest cause-3 claim.** |
| Continuous time | Every digital system samples. We can go faster; we cannot go continuous. |
| Exact state | Every sensor has noise. VICON is excellent, not perfect. |
| Rotor downwash between agents | All three algorithms model agents as independent point masses coupled *only* through the control law. Real rotors push air down and no controller removes that. |

Two caveats worth stating precisely:

- **"Necessary" is relative to a hardware class.** A fully-actuated multirotor
  with tilted rotors *can* translate without leaning. So the correct claim is
  "unachievable on any standard quadrotor", not "unachievable in principle".
  Stating it that way is stronger, because it is exactly right.
- **Latency is not one thing.** Position *update* rate is contingent (VICON
  streams far faster than our 25 Hz loop, so the bottleneck is ours), while
  *actuation* delay is necessary. Same word, opposite verdicts — which is why it
  must be decomposed rather than blamed as a lump. (Measured total, and a first
  split: section 13e.)

Downwash may matter to us specifically: we stagger the two real drones by 0.4 m
in altitude for safety, so one can fly directly beneath the other. At that
separation it is a real effect on a Crazyflie, and no algorithm in the testbed
knows it exists. It would present as random-looking tracking error.

**The scaling test settles it empirically.** Rather than arguing, vary what we
control and watch the failure. Run trochoidal at 10, 25 and 50 Hz:

- Failure disappears as we improve → contingent. Our limitation; say so.
- Failure shrinks but extrapolates to nonzero at the best rate a Crazyflie could
  reach → partly necessary, and now quantified.
- Failure does not improve → structural. The strong claim is earned.

**Aim for a specification, not a verdict.** The most useful form of the result is
a number, e.g. *"the trochoid decays with time constant tau; tau scales with
control rate as measured; holding the pattern for 60 s therefore requires roughly
X Hz, which exceeds what the Crazyradio link sustains across a four-drone
fleet."* That tells the next person what hardware the algorithm needs, which is
worth far more than "it did not work for us."

Section 13g now has a result of exactly this form: a loop delay of ~0.28 s caps
trochoidal's velocity gain at β ≈ 4–6, and so caps how fast this pattern can be
flown.

Note also that these two statements are not in tension — the second earns the
first:

- *"The paper assumes zero actuation delay; real quadrotors have ~X ms; the
  assumption is violated."* (the general claim)
- *"Our measured delay is X ms, of which Y is irreducible and Z is our stack."*
  (the local measurement)

"Whose fault is it" is the wrong frame. The right question is: *what would it
take to satisfy this assumption, and is that reachable?*

**Where the strong version is available.** Trochoidal needs eigenvalues sitting
*exactly* on the imaginary axis — a measure-zero condition requiring infinite
gain precision, zero delay and exact state. Every digital implementation has
quantisation, latency and estimation error. So the defensible strong claim is:
*this requirement cannot be met by any digital implementation, so the trochoid
is necessarily transient on real hardware; the only question is how long it
survives.* That is a structural statement about the algorithm class, not a
report of our difficulties, and it is a genuine contribution.

Coverage will most likely go the other way — assumptions violated, behaviour
survives. That contrast is the result.

**(b) Our harness simplifications — critique these directly.** These are ours,
so we own them and they are fair game. Was 25 Hz necessary or merely convenient?
Is the 0.5 m/s^2 cap protecting the airframe or distorting the algorithm? Would
a proper state estimator instead of differentiated VICON have changed the
outcome, and at what cost? This is the bulk of the discussion chapter, and it is
what the supervisor meant by *"just discuss the simplifications and why they
diverge from the simulated."*

**(c) Our algorithm-level adaptations — critique these hardest.** There is
essentially one: the `-kd*v` PD wrapper in `coverage.py`, invented to run a
first-order control law on a second-order vehicle. We chose it and there were
alternatives, so it deserves the most scrutiny — unlike the harness choices, it
modifies the control law itself.

**In one line:** we critique our own simplifications directly, and we critique
the papers' assumptions on applicability — but only once we have ruled ourselves
out as the cause. The simplifications are the apparatus; the assumptions are the
hypotheses; ruling out (1) and (2) is what turns a failed flight into a finding.

## 5. Where the simplifications actually come from

Important point of fact, because it was a source of confusion:

**The papers do not contain these simplifications. We introduced them.** But
they are almost all in the *harness around* the control law, not in the law
itself. The three control laws are implemented essentially as published:

| Algorithm | Source | Law as implemented |
|---|---|---|
| Flocking | Olfati-Saber, IEEE TAC 51(3), 2006, Algorithm 2 | faithful |
| Trochoidal consensus | Monsingh, Sinha & Chung, EJC 76 (2024) 100928, eq. (7) | faithful |
| Coverage | Cortes, Martinez, Karatas & Bullo, IEEE TRA 20(2), 2004 | adapted — see below |

### 5a. Harness-level simplifications (shared by all three)

These apply to every algorithm, which is what makes the three comparable. This
is the experimental design: same simplifications, different algorithms, see
which algorithms care.

1. **Time is discretised.** The papers are continuous-time ODEs. We run the
   control law at 25 Hz on hardware (`crazyflie_node.py`, `CONTROL_RATE = 25`),
   and 10–20 Hz in the hybrid configs. *Hidden assumption: 25 Hz is close
   enough to continuous.*

2. **Force is saturated.** `max_accel` is 0.5 m/s^2 for coverage and flocking,
   and 3.5 for trochoidal since 15 Sep (until then the launch never passed it
   on, so every drone ran at 0.5 — 11h). On top of that, `crazyflie_node` caps
   the target velocity at 0.7 m/s and applies the `max_lead` leash. No stability
   proof in any of these papers covers a saturated input. *Hidden assumption:
   we never hit the limits hard enough to matter.*

3. **Velocity is estimated, not measured.** The control laws want each agent's
   true velocity. VICON returns position only; `mocap_state` differentiates it
   by least-squares fit. Our velocity is noisy and slightly late. *Hidden
   assumption: an estimated velocity behaves like a true one.*

4. **The plant is substituted, not simplified.** This is the least obvious and
   the most important, and it is easy to state wrongly.

   The double integrator is **not** our simplification — it is the paper's own
   model. Monsingh, Sinha & Chung is titled "...for *double-integrator dynamic
   agents*", and the whole eigenvalue-placement derivation depends on the agents
   being double integrators. We match that exactly.

   What we simplified is the **plant**. A Crazyflie is a quadrotor with an
   onboard attitude loop and a position controller; it is not `v_dot = u`. So
   instead of simplifying the algorithm to fit the drone, we manufactured a
   double integrator in software and made the real drone chase it
   (`crazyflie_node.py:487-488`):

   ```python
   self._desired_vel += acc3 * dt
   self._desired_pos += self._desired_vel * dt
   ```

   Verified signal path:

   ```
   VICON -> mocap_state_node -> /<id>/state   (REAL position; velocity by differentiation)
                                     |
                                     v
                          algorithm_manager -> control law (paper eq. 7)
                                     |
                                     v
                            /<id>/cmd_accel
                                     |
                                     v
                      crazyflie_node:487-488   <- software double integrator
                                     |             (VIRTUAL agent)
                                     v
                            cmdFullState -> firmware -> real drone -> VICON
   ```

   Two consequences worth stating precisely:

   - If tracking were perfect and instantaneous, `p_real = p_virtual` and the
     system collapses to exactly the paper's. It is not, so what we fly is **the
     paper's system in series with the firmware's tracking dynamics** — extra
     phase lag in the loop that the paper's eigenvalue analysis does not include.
   - `algorithm_manager` reads the **real** state (VICON), but the acceleration
     it produces drives the **virtual** one. These are two different signals. The
     `max_lead` leash exists because they can drift apart.

   *Hidden assumption: the tracking lag between virtual and real is negligible.*
   **Measured 15 Sep: it is not.** The real drone accelerates ~0.23 s after the
   command, ~0.28 s round the whole loop (13e).

   This is the simplification with the sharpest predicted split between
   algorithms. Trochoidal places eigenvalues *precisely* on the imaginary axis,
   and unmodelled loop lag moves them off it — so the trochoid should decay or
   spiral rather than close. Coverage drives to a fixed point, so lag only means
   arriving later at the same place. Same simplification, opposite consequences.
   *Outcome (13c, 13g): the split happened, but not as decay. The lag drives
   trochoidal's velocity brake into a ~1.1 s oscillation.*

5. **Motion is planar and altitude is pinned.** All laws run in 2-D; z is held
   at the takeoff height. *Hidden assumption: the vertical loop does not
   interact with the horizontal one.*

6. **Agents are point masses.** No attitude in any law. A real quadrotor must
   tilt to accelerate, so acceleration and attitude are coupled and lagged.

### 5b. Algorithm-level changes

**Coverage — the one genuine change to a control law.** The central result of
Cortes et al. is for *single-integrator* agents: the vehicle moves directly
toward its Voronoi cell centroid. A Crazyflie under `cmdFullState` is a
double integrator — we command acceleration. `coverage.py` bridges this with a
PD wrapper:

```python
accel = self._kp * (centroid - pos[k]) - self._kd * vel
```

The `-kd*v` damping term is ours; it makes a second-order agent behave like the
first-order one the paper assumes. This is a real, nameable adaptation, and it
sits exactly on the single-integrator / double-integrator axis the supervisor
raised.

**Coverage — the density integral is discretised.** The locational cost
`H(p) = sum_i integral_{V_i} ||q - p_i||^2 phi(q) dq` is a continuous integral;
we evaluate it on a `grid_res x grid_res` sample grid per cell. Measured effect
on the final settled position: 40.3 / 19.1 / 9.3 / 4.6 mm of error at
`grid_res` 18 / 36 / 72 / 144. **This matters:** at the default 36 it is
roughly 19 mm, which would otherwise look like hardware error in a
physical-vs-theory comparison. It is not. It is us.

**Flocking — the obstacle term is omitted.** We implement Algorithm 2
(alpha- and gamma-agents). Algorithm 3's beta-agent obstacle avoidance is not
implemented. The virtual leader (gamma-agent) follows a scripted circle rather
than being a real tracked target.

**Trochoidal — two implementations, deliberately.** `trochoidal.py` hard-codes
the path: each drone is told its exact position at every instant, and the
pattern exists because the paths were designed to fit. `trochoidal_consensus.py`
implements the paper's law, where no drone is told anything and the trochoid is
*emergent* in the closed-loop dynamics. Comparing these two is itself a result:
it isolates how much of the pattern quality comes from the algorithm versus
from being handed the answer.

## 6. Why these three algorithms

*This is the question we could not answer in the meeting. The honest reason we
picked them was "we wanted a range." That is not an answer, because it does not
say a range of what. Here is the axis.*

The three occupy deliberately different corners of the design space:

| | Coverage | Flocking | Trochoidal consensus |
|---|---|---|---|
| Native dynamics | single integrator (1/s) | double integrator (1/s^2) | double integrator (1/s^2) |
| Stability type | asymptotically stable | asymptotically stable | **marginally stable** |
| End behaviour | settles to a static configuration | settles to a moving formation | never settles — persistently dynamic |
| What the theorem promises | convergence to a local minimum of `H` | cohesion, no collisions, matched velocities | trajectory is a trochoid with specific radius and period ratios |
| Gain sensitivity | forgiving | moderate | **knife-edge** — gains must be computed from graph eigenvalues |
| Scalar quality metric | yes: `H` | no | no |

**Coverage is in the set precisely because it is the least dynamic one.** It is
the control condition. Its equilibrium is static, its stability is robust, and
we predict the harness simplifications will *not* bite. Without an algorithm
like that in the set we could not distinguish "our hardware is bad" from "this
algorithm is fragile" — every result would be confounded.

It also earns its place two other ways:

- It is the **only** algorithm with a scalar optimality measure. `H` at the real
  final configuration against `H` at the theoretical optimum is a single
  defensible number: *"hardware achieved X% of theoretical optimum."* Trochoidal
  has no cost function; flocking has no single number. This is what the
  supervisor was asking for with *"what optimality is achieved in real hardware
  and simulated for the coverage algorithm."*
- It is our **single-integrator representative**, so it anchors one end of the
  1/s vs 1/s^2 comparison.

**Trochoidal is the opposite corner and the most fragile thing we have.** A
trochoid exists only because exactly two pairs of closed-loop eigenvalues sit
*precisely* on the imaginary axis, neither growing nor decaying. That is a
knife-edge condition. Discretisation, estimation lag, and the virtual-plant lag
of section 5a.4 all nudge eigenvalues sideways.

**This gives us a prediction to make before flying:**

> Trochoidal will degrade most under the harness simplifications; coverage will
> degrade least; flocking will sit between them. Specifically, the trochoidal
> pattern should decay or spiral rather than close cleanly, because lagged
> velocity estimates and the tracking lag between the virtual and real agent
> perturb marginally-stable poles off the imaginary axis.

Writing that down *first* and then testing it is the shape of the whole report.

**Outcome.** The ordering held: trochoidal degraded most, coverage least,
flocking between. The mechanism did not. Under the 0.5 clamp the trochoid grew
(11c); with the clamp lifted it roughly holds its size, but is swamped by a
~1.1 s oscillation caused by the tracking lag (13c).

### 6a. One suspect already eliminated: control-loop discretisation

An earlier version of this prediction also blamed discretisation at 25 Hz. That
has been **tested in simulation and ruled out**, which is worth recording both
as a result and as a warning.

Discretisation is not latency. Nothing waits for anything in a simulation. What
sampling does is compute the control force once per tick and then *hold* it
constant while the drone keeps moving, so the force being applied is on average
the one that was correct half a step ago. That is the zero-order hold, and it
does behave like a small phase lag — but the mechanism is freezing a value, not
waiting for one.

Measured, isolating the hold by integrating the plant finely and varying only
the control rate:

| Control rate | amplitude lost per 63.5 s cycle |
|---|---|
| 5 Hz | 11.2% |
| 10 Hz | 1.3% |
| **25 Hz** | **0.004%** |
| 50 Hz | 0.058% |

At the rate we actually fly, the hold contributes nothing measurable. Running
`drone_node`'s real dynamics confirms it: no consistent decay at any rate, and a
period ratio of 3.3329–3.3367 against the eigenvalue prediction of 3.3337.

**The warning.** The decay that first prompted this was an artifact of the test
harness used to look for it. That harness integrated with semi-implicit Euler
(`p += v*dt`), which loses energy on an oscillator. `drone_node` uses
`x + v*dt + 0.5*a*dt^2`, the *exact* zero-order-hold solution for a double
integrator, which does not. The apparatus was producing the phenomenon.

This is precisely the failure mode section 4 warns about — a cause inferred from
an observation without first ruling out our own instrument. It is worth keeping
as a worked example, because on hardware the same mistake is much harder to
catch. **If a simulated trochoid appears to decay, check the integrator before
concluding anything.**

What survives: the remaining candidate mechanisms are the ones that are
genuinely *delays* — velocity estimated by differentiating VICON, and the lag
between the virtual double integrator and the real drone chasing it.
Discretisation is off the list. The tracking lag has since been measured
(~0.23 s, 13e) and accounts for the ~1.1 s oscillation (13c); the velocity
estimate's ~45 ms is part of the same loop delay.

## 7. What to measure, per algorithm

Measure the promised property, not the path.

| Algorithm | Collect |
|---|---|
| **Flocking** | spread of pairwise distances against target spacing `d`; smallest inter-agent gap that ever occurred (collision avoidance); spread of velocities across the fleet (does it go to zero?); whether the communication graph stays connected |
| **Coverage** | `H(p)` over time — does it decrease, and where does it stop; final `H` vs theoretical optimum; each agent's distance from its own Voronoi centroid at rest |
| **Trochoidal** | measured period ratio against the eigenvalue prediction (3.3337 in simulation, slow 63.53 s / fast 19.06 s); measured radii against the predicted envelope; does the curve close or drift |

Note that trochoidal has no reference trajectory at all, so "tracking error" is
undefined for it. It can only be validated against the eigenvalue predictions.

**All of this is collected by `tools/metrics_recorder.py`**, which picks the
metric set from the algorithm named in the config. Run it in a third terminal
alongside the usual two, and Ctrl-C it when the flight ends:

```bash
python3 tools/metrics_recorder.py --config <the same config the run uses>
```

Output is `logs/<algorithm>_<YYYYmmdd_HHMMSS>.txt` — provenance header, one row
per sample streamed as it is computed (so an abort keeps the data), and the
derived summary appended at exit. See the ReadMe section "Recording Validation
Metrics" for options.

Two things it computes that are worth knowing about:

- For coverage it runs **Lloyd's iteration offline** from the flight's own
  starting positions and reports `H_flight / H_lloyd`. Comparing against a global
  optimum would be unfair, since the control law only ever claims to find a
  *local* minimum — the honest target is the minimum in the basin the flight
  actually started in.
- For trochoidal, a decaying pattern makes the FFT report radii far below the
  real ones (about half, for decay comparable to the run length), because the
  transform averages amplitude over the window. The tool measures tau first and
  corrects the radii back to the start of the analysis window, reporting both
  figures. Verified accurate to about 1% for realistic decay.
- On hardware, do not quote the recorder's trochoidal summary (11c). Use
  `tools/analyse_trochoidal.py` for one flight and `tools/plot_trochoidal_ladder.py`
  for the speed ladder.

## 8. How to discuss the sim-to-real gap

Three layers, and the gap between each pair has a *different* cause. This is
the structure of the discussion chapter.

```
  THEORY  --gap A-->  SIMULATION  --gap B-->  HARDWARE
```

- **Gap A (theory to simulation)** isolates the *algorithmic* simplifications.
  Simulation has perfect state and no physics, so anything that goes wrong here
  is discretisation, saturation, gain choice, or grid resolution — and nothing
  else. The coverage `grid_res` error above is a pure gap-A effect.

- **Gap B (simulation to hardware)** isolates the *physical* ones: estimation
  noise, latency, motor limits, aerodynamics, the virtual-to-real tracking lag,
  battery sag.

Attributing each observed effect to gap A or gap B is the core analytical move.
It is also what makes the simulation work count as a result in its own right
rather than a rehearsal.

## 9. Open questions and experiments raised by the supervisor

1. **Single vs double integrator for trochoidal (1/s vs 1/s^2).** Run the same
   law commanding velocity versus commanding acceleration, and determine which
   plant model the real Crazyflie's behaviour actually matches. The firmware's
   inner loop means it is a heavily-damped second-order system, so neither
   idealisation is exactly right.
   *Partly answered (13g): on the Crazyflie, the order of the integrator matters
   less than the ~0.3 s lag in front of it.*
2. **Coverage optimality on hardware vs simulation.** Final `H` in both, against
   the theoretical minimum. Must report `grid_res` alongside, since it moves the
   answer by ~19 mm at the default.
3. **Merging different trochoidal patterns mid-flight.** New capability. Note
   this only makes sense in `trochoidal_consensus.py`, since the hard-coded
   version has nothing to merge.
4. **Control-rate scaling test (highest value).** Run each algorithm at 10, 25
   and 50 Hz and measure how the degradation scales. This is what separates a
   contingent limitation from a necessary one, and it is what converts "it did
   not work" into a hardware specification. Do this before writing any
   conclusion that blames an assumption.
   *The simulated half is already done for trochoidal — see section 6a. It shows
   no rate dependence in simulation, so any rate dependence measured on hardware
   is attributable to the physical layer rather than to sampling. That is a
   clean baseline to test against.*
5. **Latency decomposition.** Measure total loop delay and split it into the
   irreducible part (motor spin-up, aerodynamic response, radio time-of-flight)
   and our part (ROS 2 queueing, loop rate, Python). Only the irreducible part
   supports a claim against a paper.
   *Total measured: ~0.23 s from command to acceleration, ~0.28 s round the
   loop, the same in flocking and trochoidal (13e). The split there is an
   estimate from the code's rates; the step test (12, item 4) measures it.*
6. **Algorithm latency comparison.** Per-algorithm compute time per tick, and
   whether any of them threatens the 25 Hz budget. Coverage is the expensive one
   (`O(grid_res^2)` per cell per tick).
7. **Comparison against other algorithms** not yet in the testbed, at least in
   discussion.

## 10. What this means for how we work

- Before flying anything new: **write down the prediction first.** A measured
  result with no prior prediction is much weaker than the same result with one.
- Every parameter that differs from the paper gets recorded as a simplification
  with a stated assumption, in the algorithm's config comments.
- Keep the `grid_res` lesson in mind generally: **check whether a discrepancy is
  ours before attributing it to hardware.** Several will be.

## 11. Hardware findings, 8–9 September 2026

*Analysed 2026-09-15 from the 14 records in the team Drive (folders 8/9/2026 and
9/9/2026); copies in `logs/hw/`. Every number below was computed from the raw
rows over live, pre-geofence windows — **not** from the summary block the recorder
appends, which is wrong for several of these files (see 11g).*

### 11a. What flew, and what is usable

| Day | Flight | Usable? |
|---|---|---|
| 8 Sep | Flocking hybrid ×5 (agents drone1, drone2 real) | test4 (27 s clean) and test2 (16 s). test1: drone1 never took off. test3: 6 s. test5: no data, because the state feed died at ~6 s. |
| 8 Sep | Trochoidal, 2 real (drone1, drone4), rescale 6 | yes, 71 s clean |
| 8 Sep | Trochoidal, 3 real ×3 | no. All three failed during takeoff (11f). |
| 9 Sep | Trochoidal, 2 real, test1 (rescale 10) | yes, 29 s clean |
| 9 Sep | Trochoidal test2, test3 | no. Both failed in the hover *before* the algorithm started. |
| 9 Sep | Trochoidal test4, labelled rescale 14 | yes, but it actually ran rescale 10 (11b) |
| 9 Sep | Kuramoto | no. The algorithm never started. |

In the Drive notes, "drone3" etc. name physical airframes, whereas the logs name
algorithm agents. On fig4 runs the real agents are drone1 = VICON `drone_1` and
drone4 = VICON `drone_2`.

### 11b. The "rescale 14" flight ran rescale-10 gains

The flight logs the positions of the two virtual drones, and they are exact
double integrators. Replaying them through the control law, driven by the
logged real drones, shows which gains actually ran:

| Flight | Header says | Replay error: r6 | r10 | r14 |
|---|---|---|---|---|
| 8 Sep test1 | r6 | **0.2 cm** | 15.8 cm | 23.9 cm |
| 9 Sep test1 | r10 | 16.0 cm | **0.8 cm** | 13.4 cm |
| 9 Sep test4 | r14 | 17.8 cm | **0.6 cm** | 13.5 cm |

Why this happened:
- The recorder prints the params from the yaml passed to *it* (the repo copy).
- The flight loads the *installed* copy under `~/ros2_ws/install`.
- The copy + `colcon build` was not redone after the rescale-14 edit.

**Consequences:** rescale 14 has never flown, and rescale 10 has two repeats. The
recorder header is not evidence of what flew.

### 11c. Trochoidal: speed ladder

Method: a two-mode (eigenvalue) fit over the window from algorithm start to the
first geofence approach. Applied to simulation, the same fit recovers the
designed periods and zero growth exactly.

| Run | Fast period, hw vs sim | Slow period | Growth rate | Size doubles every | Start → geofence |
|---|---|---|---|---|---|
| r6, 2 Sep | 19.42 vs 19.04 s (+2%) | 59.5 s | +0.039 /s | 18 s (0.9 laps) | 53 s |
| r6, 8 Sep | 19.38 vs 19.04 s (+2%) | 64.0 vs 63.5 s | +0.024 /s | 29 s (1.5 laps) | 73 s |
| r10, 9 Sep t1 | 11.9 vs 11.42 s (+4%) | 38.0 vs 38.1 s | +0.064 /s | 11 s (0.9 laps) | 29 s |
| r10, 9 Sep t4 | 11.8 vs 11.42 s (+4%) | ~35 s (short window) | +0.062 /s | 11 s (0.9 laps) | 29 s |

- **The shape survives.** Periods are within 2–4% of the eigenvalue prediction,
  and the period ratio is 3.30 against 3.34.
- **The marginal-stability promise does not survive.** The pattern grows on
  every flight, while simulation holds it for thousands of laps. This is the
  predicted cause-3 result of section 4.
- **Going faster widened the gap in real time.** A 1.67× speed-up made the growth
  rate 1.6–2.7× faster, and the fleet reached the geofence in 29 s instead of
  53–73 s.
- **Per lap, the gap is unchanged so far:** about one lap to double in size at
  both speeds. So up to rescale 10, the divergence scales with the pattern's own
  clock. If per-lap growth gets worse at rescale 14 (peak tilt ~12°), that is
  where the point-mass simplification starts to break. (Caution on
  "~12°": that is the start-up transient. Once running, the designed pattern
  asks for only ~0.3–0.7° of tilt; the grown hardware pattern asks for more.)
- The recorder's own trochoidal summaries (period ratios of 5–47, "decay" time
  constants) are geofence and sub-cycle artefacts. Do not quote them.
- **Update 15 Sep (section 13).** All four runs above flew with the 0.5 clamp
  (11h). With the clamp at 3.5 the growth disappeared (×0.92–1.05 per lap), so
  these growth rates belong to the clipped law. The rescale-14 tilt question is
  answered in 13b: 22–32°, set by the ~1.1 s oscillation, not by the pattern.

### 11d. Flocking: the circling is flocking-specific

The 8 Sep test4 flight (20–47 s) repeats 2 Sep:
- **Held:** min gap 0.39 m, lattice error 0.120 m, spacing/d 0.94, connected 100%.
- **Failed:** velocity spread 0.133 m/s, against 0.000 in simulation.

Two separate motions show up on hardware. RMS ripple:

| Motion | Flocking | Trochoidal | Coverage (settled, 2 Sep) |
|---|---|---|---|
| Slow circling, 2–6 s period (the visible "each drone goes in circles") | real 4.5–12.7 cm, **virtual 4.3–4.8 cm** (sim 1.7 cm) | virtual 0.5–0.8 cm at r6 | ~1 cm to centroid |
| Fast shake, ~1.2 s period | real 3.3–7.0 cm, virtual ~2 cm | real 3.4–3.8 cm, virtual 0.1–0.7 cm | < 1 cm |

(At r10, trochoidal's 2–6 s band also picks up the much larger, faster pattern,
so r6 is the clean comparison.)

**The circling belongs to flocking.** The key observation is that it reaches the
*virtual* drones, which have no physics of their own. So something in the control
law carries it across the fleet.

The candidate mechanism is a hypothesis, not a result:
- Flocking couples agents through their **neighbours' measured velocity**
  (`flocking.py:264`, `other.velocity - state.velocity`). Every real drone's
  velocity is differentiated from VICON, so it is noisy and late, and it is
  broadcast to every neighbour.
- Trochoidal couples only through positions; each agent uses its own velocity only.

This is the supervisor's velocity-differentiation hypothesis, now with a
specific path. Section 12 (item 3) tests it. The fast shake in the table above
is explained in section 13c; the slow circling is not (13f).

### 11e. A separate fast shake on the real drones

**Superseded.** Explained on 15 Sep, and now the main trochoidal finding: see
section 13c. As first observed:
- The ~1.2 s shake is 10–16× faster than trochoidal's own inner loop (12–19 s),
  so it is not the designed motion.
- It was the same at r6 and r10, even though β, the velocity gain, rose 67%.
- It is absent in hover, in settled coverage, and in the one clean segment of
  the 12 Aug square flight (explicit setpoints).
- At the 0.5 clamp it was 3–4 cm. With the clamp at 3.5 it is 15–19 cm.

### 11f. Three drones and other failures

- **All three 3-drone attempts failed at takeoff, before the algorithm ran.**
  - test1: `drone_3` shot ~2.7 m sideways in 1 s.
  - test2: `drone_3` never lifted.
  - test3: the heights of `drone_2` and `drone_3` swap sample-to-sample, so
    VICON Tracker is confusing the two rigid bodies.

  Suspected, unconfirmed: the two marker layouts are too alike.
- **9 Sep test2 and test3 went wrong in the pre-start hover, not under the
  algorithm.**
  - test2: `drone_2` slid ~4.5 m while falling.
  - test3: `drone_1` drifted steadily at ~0.2 m/s while holding a fixed point.

  They say nothing about speed.
- **Kuramoto:** the `active` flag stayed 0 and no phases were published. Its code
  is not on GitHub, so the cause needs the terminal-2 log.

### 11g. Recorder defects found

1. The header shows the params from the recorder's `--config`, not what flew.
   This is what hid 11b.
2. Height columns are keyed by VICON body, so on fig4 runs `z_drone2` is agent
   drone4.
3. The "held"/"SINKING" altitude figures include time spent landed.
4. Frozen tails (the recorder keeps writing after the feed stops): 43–55% of
   flocking tests 1–3. Unfixed.

### 11h. The acceleration clamp never reached the drones

Found 2026-09-15, while preparing the next lab.

- `hardware_hybrid.launch.py` did not pass `max_accel` to `crazyflie_node` or
  `drone_node`, so both clipped every command at their own 0.5 m/s² default.
  `testbed_fig4.yaml`'s 1.8 and 3.5 never reached a drone.
- Evaluating the law on the logged states, the real drones' command exceeded
  0.5 on an axis on **85–96% of ticks**, at rescale 6 and rescale 10 alike. That
  is a lower bound, because the mocap velocity the drones really used is
  noisier than the smoothed estimate used here.
- So every trochoidal hardware flight so far flew the real drones saturated,
  and the growth rates in 11c are for the clipped law.
- Fixed: the launch now passes the config's `max_accel` to both nodes
  (`max_acceleration:=config`, the default), and prints the clamp at startup.
- **Confirmed 15 Sep (13c):** a saturated velocity loop settles into a cycle
  whose size is set by the clamp rather than the gain. Raising the clamp from
  0.5 to 3.5 grew the ~1.2 s shake from 3–4 cm to 15–19 cm, while its period
  stayed at 1.1–1.3 s and did not change with β.

## 12. Next lab plan

*Rewritten after the 15 Sep flights; the findings behind it are in section 13.
Items 1 and 2 of the previous plan (rescale 14 flown properly; tilt and
laps-to-double) were done. Its item 3, the flocking velocity window, was not
flown and is carried over as item 3 below.*

### Before every flight

On the lab PC:

```bash
REPO=~/Desktop/multi-drone-testbed
cd $REPO && git pull
rsync -a $REPO/ros2_ws/src/drone_testbed/ ~/ros2_ws/src/drone_testbed/
cd ~/ros2_ws && colcon build --packages-select drone_testbed && source install/setup.bash
grep -E "^\s+(alpha|beta|kappa|max_accel|gain_\w+):" ~/ros2_ws/install/drone_testbed/share/drone_testbed/config/<the config you will fly>
```

- Terminal 2 must print the acceleration clamp you expect.
- **Put the drones on their own marks:** VICON `drone_1` on (0.235, −0.132),
  `drone_2` on (0.179, 0.219). All three 15 Sep flights had them swapped.

### 1. Trochoidal below the ceiling — the decisive test

The same design, slower. Three configs, made 16 Sep and verified in simulation
(designed periods to 0.1 s, zero growth), from
`tools/design_consensus_gains.py --config config/testbed_fig4.yaml --theta 2.28488 --rescale N`:

| Config | β | β × 0.28 s | Fast lap | Pattern speed | `flight_duration` |
|---|---|---|---|---|---|
| `testbed_fig4_r4.yaml` | 4 | 1.1 — the edge, about where flocking sits | 28.6 s | 0.05 m/s | 200.0 |
| `testbed_fig4_r3.yaml` | 3 | 0.84 | 38.1 s | 0.04 m/s | 250.0 |
| `testbed_fig4_r2.yaml` | 2 | 0.56 | 57.2 s | 0.03 m/s | 250.0 |

`max_accel` stays 3.5, so only the speed differs across the whole ladder. The
design itself needs only 0.04–0.17 m/s² here, so the clamp should never engage
unless the oscillation comes back.

**Status: r3 and r2 flown 16 Sep — section 14. The oscillation collapsed at r3
and is essentially absent at r2, which flew the designed pattern. Fly r4 next to
find the threshold.** These are 3–4 minute flights, so use fresh batteries.

```bash
# T2
ros2 launch drone_testbed hardware_hybrid.launch.py config:=config/testbed_fig4_r3.yaml \
    hw_drone:=drone1,drone4 cf_name:=drone_1,drone_2 mocap_name:=drone_1,drone_2 flight_duration:=250.0
# T3
cd ~/Desktop/multi-drone-testbed && python3 tools/metrics_recorder.py \
    --config ros2_ws/src/drone_testbed/config/testbed_fig4_r3.yaml \
    --hw drone1,drone4 --mocap-name drone_1,drone_2 --note "rescale 3, clamp 3.5"
```

**Prediction.** The ~1.1 s oscillation disappears or shrinks to a few cm, tilt
falls towards the design, and the pattern holds closer to its designed size. If
it oscillates as at β = 6–14, section 13c is wrong.

**Afterwards:** add the record to `RUNS` in `tools/plot_trochoidal_ladder.py`
and rerun it, which redraws every figure with the new rung included.

### 2. Flocking with a weaker velocity gain

`testbed_flocking_hybrid_c2a05.yaml` lowers the velocity-matching gain from 2.0
to 0.5, which takes gain × delay from ~1.1 to ~0.5.

```bash
ros2 launch drone_testbed hardware_hybrid.launch.py config:=config/testbed_flocking_hybrid_c2a05.yaml \
    hw_drone:=drone1,drone2 cf_name:=drone_1,drone_2 mocap_name:=drone_1,drone_2 flight_duration:=90.0
python3 tools/metrics_recorder.py --config ros2_ws/src/drone_testbed/config/testbed_flocking_hybrid_c2a05.yaml \
    --hw drone1,drone2 --mocap-name drone_1,drone_2 --note "c2a 0.5"
```

**Prediction.** The ~1.1 s oscillation shrinks. A second algorithm showing the
same dependence on its velocity gain makes the mechanism general.

### 3. Flocking: velocity window, and real pose timing (carried over)

Same config each time. Change only the VICON velocity fit, 1–2 flights each:

```bash
ros2 launch drone_testbed hardware_hybrid.launch.py config:=config/testbed_flocking_hybrid.yaml \
    hw_drone:=drone1,drone2 cf_name:=drone_1,drone_2 mocap_name:=drone_1,drone_2 \
    flight_duration:=90.0 velocity_window:=20        # then 10 (the default), then 5
python3 tools/metrics_recorder.py --config ros2_ws/src/drone_testbed/config/testbed_flocking_hybrid.yaml \
    --hw drone1,drone2 --mocap-name drone_1,drone_2 --note "velocity_window 20"
```

**Metrics:**
- The recorder's WOBBLE block: circling amplitude and period per drone. The
  **virtual** drones are the decisive ones.
- The theorem's promises: velocity spread, lattice error, min gap.
- **Pose timing:** a new TIMING block in every record. It gives the VICON rate
  and jitter per drone, dropouts over 20 ms, the /state rate, and the lag each
  velocity window costs at the measured rate. Raw receipt times go to
  `<record>_timing.npz`.

**Prediction.** If the lagged, noisy differentiated velocity drives the
circling:
- window 20 (smoother but later) and window 5 (fresher but noisier) should
  both change it;
- if the circling is the same at all three, differentiation is not the driver.

### 4. Step test: measure the delay directly

Hover, jump the target 20 cm, and time the response in VICON. This splits
13e's delay into our stack and the vehicle. It needs a small script, which is
not written yet.

### 5. A simpler graph at the same β (optional)

A two-drone or symmetric-ring trochoid at β = 10. **Config not made yet.** If it
shows the same oscillation, the graph is ruled out as a cause (13h).

### 6. Record the setpoint

Add `crazyflie_node`'s `/<id>/setpoint` to the recorder, so figures 6–7 no longer
rely on a rebuild. Not done yet.

### 7. Lever 3 diagnostic (optional; supervisor first)

Rescale 10 with the brake reading the target's velocity instead of the drone's
(13i). This needs a code change: a launch switch, off by default. It has not
been made. Agree it with the supervisor first, because it changes what the law
is fed.

## 13. Hardware findings, 15 September 2026

*Three trochoidal flights at three speeds, the first with `max_accel` 3.5
actually reaching the drones (11h fixed). Records in `logs/hw/`. Figures in
`docs/figures/trochoidal_ladder/`, made by `tools/plot_trochoidal_ladder.py`,
which also prints every number below. A fourth flight, rescale 10 at clamp 1.8
(`trochoidalconsensus_20260915_144306`), hit the geofence repeatedly and is left
out.*

### 13a. What flew

| | r6 | r10 | r14 |
|---|---|---|---|
| Record `trochoidalconsensus_…` | `20260915_151744` | `20260915_145550` | `20260915_143416` |
| β, the velocity gain | 6 | 10 | 14 |
| Speed the designed pattern asks of the real drones (median) | 0.06 m/s | 0.11 m/s | 0.15 m/s |
| Designed fast lap | 19.0 s | 11.4 s | 8.2 s |
| Gains that flew (replay, the 11b method) | the config's (0.3 cm) | config time scale ×0.98 | ×0.99 |

For r10 and r14 the replay residual is 3–4 cm rather than under 1 cm, so
`analyse_trochoidal.py` prints "NOT the config gains". But the best-fit time
scale is within 2% of the config, whereas the 11b mis-deploy showed as ×0.72.
These are the config gains; the larger residual is unexplained.

**The start positions were swapped on all three flights.** VICON `drone_1`
(agent drone1) sat on drone4's mark (0.18, 0.22), and `drone_2` (agent drone4) on
drone1's (0.24, −0.13). Trochoidal's pattern size depends on where it starts, so
every "designed" figure in this section is simulated from the real start
positions. From there the designed pattern is 20–28% smaller than from the marks.
The swap does not cause the oscillation (13c).

### 13b. Results

| | r6 | r10 | r14 |
|---|---|---|---|
| Pattern size change per fast lap | ×1.05 | ×0.92 | ×0.94 |
| A drone reached the 1.3 m edge | once, at 129 s; the flight carried on | never (124 s) | never (97 s) |
| Pattern size vs design (fast loops filtered out) | 2.4× | 3.2× | 1.9× |
| Real-drone speed, median (design) | 0.83–0.90 m/s (0.06) | 0.75–1.05 (0.11) | 0.82–0.90 (0.15) |
| Tilt, median, from VICON (design, p95) | 24–27° (0.1°) | 22–32° (0.4°) | 24–28° (0.7°) |
| ~1.1 s oscillation: RMS size / period | 17 cm / 1.16–1.28 s | 15–19 cm / 1.11–1.22 s | 15–16 cm / 1.11–1.16 s |
| Command clipped at 3.5 m/s² | 79–92% of ticks | 89–94% | 89–92% |

- **Raising the clamp stopped the divergence.** At 0.5 the pattern grew 1.4–1.9×
  per lap and reached the geofence in 29–73 s (11c). At 3.5 it roughly holds its
  size. The growth in 11c belonged to the clipped law.
- **No speed flies the design.** At every speed:
  - the real drones move 6–15× faster than the pattern asks;
  - they tilt 20–30° where the pattern needs under 1°;
  - the pattern is 2–3× its designed size, and drifts 2–5× slower than designed.
    The mode fit gives periods 2–5× the design's, at R² 0.73–0.88, so the motion
    is no longer a clean two-mode trochoid.
- **Speed made almost no difference.** Every row is flat across the ladder. 13d
  explains why.
- **The tilt is exactly what the flown path needs.** Tilt computed from the path,
  atan(|a|/g), matches VICON's tilt to within 1–2° (figure 5). The drones are not
  tilting erratically. They are flying a far more violent path than the design.
  This also validates that estimate for records before 15 Sep, which have no tilt
  column.

### 13c. The ~1.1 s oscillation: what it is

This is the "fast shake" of 11e.

**The drone follows its commanded position well; the commanded position itself
oscillates.** Figures 6–7 rebuild the setpoint `crazyflie_node` streamed, using
the node's own steps. The drone traces the same loops ~0.3 s behind, 10–13 cm off
once that lag is allowed for. The loops are in the command. The drone does not
add them. (The setpoint is not recorded; 12, item 6.)

**Mechanism.** This is the best explanation of every observation so far, and it
survived the flight designed to break it (section 14).

- Trochoidal's command contains a velocity brake, −β·v. It asks the drone to
  cancel its velocity within 1/β = 0.07–0.17 s.
- The drone takes ~0.3 s to respond (13e).
- So the brake always acts on out-of-date motion: it brakes too late, the drone
  overshoots, the brake reverses, and the cycle repeats.
- For a brake with a pure delay τ, the textbook result is that v′ = −k·v(t−τ) is
  stable only if k·τ < π/2 ≈ 1.57, and at that limit it oscillates with period
  4τ.
- Past the limit, the oscillation grows until the acceleration clamp stops it.
  Its **size is set by the clamp and its period by the delay — neither by β.**

| This predicts | Measured |
|---|---|
| Period ≈ 4 × delay ≈ 1.1–1.2 s | 1.1–1.3 s on every run (figure 4) |
| The same at every β once past the limit | identical at β 6, 10 and 14 |
| Size scales with the clamp | clamp 0.5: 3.4–3.7 cm; clamp 3.5: 15–19 cm |
| Absent without delay | the virtual drones — same law, same flights — do not oscillate |
| A self-sustained cycle, not noise | one sharp spectral peak (figure 3); sensor noise × β would spread across all frequencies |
| Absent without a velocity brake | the 12 Aug square flight (explicit position setpoints) |
| Absent with a weak brake | coverage (13f) |

**The mechanism, step by step.** This is an illustration with round numbers,
not flight data. Setup:
- The drone and its target are both moving at +0.2 m/s.
- Only the brake acts, so it aims for 0 m/s.
- β = 10, worked in 0.1 s steps.
- The drone copies the target's speed from 0.3 s earlier.

| Time | Target's speed | Drone's speed (target's, 0.3 s earlier) | Brake reads the drone, pushes the target by |
|---|---|---|---|
| 0.0 s | +0.2 | +0.2 | −0.2 |
| 0.1 s | 0.0 (job done) | +0.2 | −0.2 (pushes again) |
| 0.2 s | −0.2 | +0.2 | −0.2 |
| 0.3 s | −0.4 | +0.2 | −0.2 |
| 0.4 s | −0.6 | 0.0 | 0 |
| 0.5 s | −0.6 | −0.2 (now going backwards) | +0.2 |
| 0.6 s | −0.4 | −0.4 | +0.4 |
| 0.7 s | 0.0 | −0.6 | +0.6 |
| 0.8 s | +0.6 | −0.6 | +0.6 |
| 0.9 s | +1.2 | −0.4 | +0.4 |
| 1.0 s | +1.6 | 0.0 | 0 |

What happens:
- At 0.1 s the target has already stopped. The drone still shows the old +0.2,
  so the brake keeps pushing for three more steps.
- The target reaches −0.6, and the drone copies it and flies backwards.
- The next swing is larger (+1.6). Each swing is bigger than the last until
  the 3.5 m/s² clamp caps them.
- In x and y together, this back-and-forth is the circles.

It does not matter where along the loop the 0.3 s sits. Part of it is the drone
responding (~0.23 s) and part is the reading being old (~0.05 s). What counts is
the time from a push to the brake seeing its effect.

**Gain × delay, in plain terms.** The brake removes a fraction β of the speed
per second. So during one delay, before it can see the result, it removes a
fraction β·τ.
- Below about 1, it has not finished when it catches up with reality, so the
  motion settles.
- Above about 1, it has already overdone it, so the drone overshoots.
- Above about 1.5, each swing is bigger than the last.

The fraction does not depend on speed, so a 1 cm/s disturbance grows as readily
as a 1 m/s one. Speed matters only once the command hits the clamp. At β = 14
that happens at 0.25 m/s, and from then on the swings stop growing. That is why
the clamp sets the size of the circles.

**Why the brake term dominates the command.** Rebuilt from the logs, the brake
term was 5–14 m/s², against 0.5–6 for the centring and coupling terms.
- The brake is the only term that scales with speed. The circles make the real
  drones move at 0.8–1 m/s, against a designed 0.06–0.15.
- In the design the brake would be 0.4–2 m/s², the same size as the other two
  terms, which balance it.
- So it is circular: the brake is large because the drones move fast, and they
  move fast because the late brake overshoots.
- The design also makes the brake strong relative to the rest: β² ≈ 19.75·α on
  every rung, because the time rescale keeps that ratio fixed.

### 13d. Why going faster did not make it worse

In this ladder β rises with speed, because the time rescale multiplies β by the
speed-up. So faster patterns sit further past the limit:

| | r6 | r10 | r14 |
|---|---|---|---|
| β·τ | 1.4–1.8 | 2.2–3.0 | 3.1–4.2 |

But past the limit the symptom saturates at the clamp. The ladder therefore did
not test "more speed, more error". It tested three points on the far side of a
threshold. The flights that test the explanation are ones below it (12, item 1).

The threshold is fuzzier than the pure-delay formula:
- Flocking, at k·τ ≈ 1.0–1.2, also oscillates at ~1.1 s (13f).
- A real drone's response is not a pure delay.

Expect trouble from roughly k·τ ≈ 1. On this stack that puts the ceiling at
**β ≈ 4–6**. So **this trochoidal design cannot be flown faster than about
0.04–0.06 m/s** without the oscillation.

### 13e. The delay: what is late, and why

The delay is measured by cross-correlating two things for each real drone:
- the command it was sent, which is the control law replayed on the logged
  states;
- the acceleration it achieved, from VICON.

Results:
- **≈ 0.23 s** (0.22–0.26) from the command to the drone accelerating. This uses
  the command rebuilt with the ~50 ms-late velocity the controller actually used.
- **≈ 0.28 s** (0.26–0.31) with the command rebuilt from the true velocity. The
  difference is the velocity estimate's own lag.
- **The same in flocking (0.23–0.26 s) and trochoidal (0.22–0.25 s).** The delay
  belongs to the hardware chain, not to either algorithm.

Where it builds up. The rates come from the code; the split is an estimate, not a
measurement:

| Stage | Adds | Section 4 verdict |
|---|---|---|
| `mocap_state_node` velocity fit (10 samples at 100 Hz) | ~45 ms | contingent |
| `algorithm_manager` at 10 Hz (waiting for the next tick, then holding) | ~50 ms on average | contingent |
| `crazyflie_node` at 25 Hz | ~20 ms | contingent |
| ROS 2 messaging, radio | ~10 ms | contingent |
| Onboard: the Mellinger controller chasing a moving target, which must tilt before it can accelerate | the rest, ~0.15 s | mostly necessary (tilt-to-translate); to be measured |

Roughly half the loop delay is our stack and half is the vehicle. The step test
(12, item 4) splits it properly.

**Is 0.23 s slow?** Yes, for a Crazyflie, which can tilt in well under 0.1 s. On
the estimate above:
- ~0.08 s is our pipeline, before the drone even has the command. The 10 Hz
  algorithm tick is the largest piece.
- ~0.15 s is the onboard controller and tilting.

**Why velocity comes from the last 10 positions.** VICON measures position only,
so velocity has to be derived, and there is a trade-off:
- Differencing the two newest samples (0.01 s apart) turns ~1 mm of VICON jitter
  into ~0.1 m/s of false speed. At β = 14 that is ±1.4 m/s² of random braking.
- A line through the last 10 samples is far smoother, but it describes the
  middle of the window, ~45 ms ago.

Fresh-and-noisy against smooth-and-late is the `velocity_window` knob (12, item
3). Better options exist:
- a Kalman filter on the ROS side;
- the drone's onboard estimate, which fuses VICON with the IMU. Sending it back
  over the radio costs its own delay and bandwidth.

The usual Crazyswarm arrangement runs the fast feedback onboard, using that
estimate, and sends only targets from the PC. Running a fast velocity loop
through ROS, as the testbed does, is the less usual part. The velocity estimate
is about a sixth of the loop delay, so fixing it alone does not remove the
problem.

### 13f. Across algorithms: same delay, and the oscillation follows the velocity gain

| Algorithm (flights) | Gain on each drone's own velocity | Gain × delay | ~1.1 s oscillation, real drones |
|---|---|---|---|
| Coverage (2 Sep ×4, clamp 0.5) | k_d = 1.2 | ≈ 0.3 | 0.3–1.5 cm (one run 2.6 cm)* |
| Flocking (8 Sep tests 2 and 4, clamp 0.5) | c2γ + c2α·Σ bump weights ≈ 4.5–4.7 | 1.0–1.2 | 2.5–4.0 cm, period 1.0–1.3 s |
| Trochoidal r6, r10 (8–9 Sep, clamp 0.5) | β = 6, 10 | 1.4–2.4 | 3.4–3.7 cm, 1.15–1.17 s |
| Trochoidal r6–r14 (15 Sep, clamp 3.5) | β = 6, 10, 14 | 1.4–3.2 | 15–19 cm, 1.11–1.26 s |

Gain × delay uses the 0.22–0.26 s command-to-acceleration delay.

**Superseded in part by section 15f**, which adds the coverage and
flocking ladders of 16 Sep and brackets the threshold between k·τ = 0.67
and 1.01. The claim below that coverage "does not oscillate" holds only
at its original gains: at k_d = 3.6 (k·τ = 1.01) it oscillates at 23 cm.

\*The 2 Sep coverage records hold only each drone's distance to its Voronoi
centroid, not its position. A circle centred on the centroid would not show in
that distance, so this row is weaker evidence than the others. It agrees with
what was seen in the room.

- The algorithm with a gentle brake, coverage, does not oscillate. The two with
  strong velocity gains do, at the same ~1.1 s period and with the same measured
  delay.
- **Flocking's visible "mini circles" are mostly a different motion.** The 2–6 s
  circling of 11d (2–10 cm here) differs from the ~1.1 s oscillation in two ways:
  - it is slower than 4× the delay;
  - it reaches the virtual drones, which the ~1.1 s oscillation does not.

  It fits flocking's velocity-*matching* term, which carries the real drones'
  late, noisy velocity to every neighbour (11d). That is related (velocity
  feedback plus delay) but a separate path, and it is still a hypothesis. The
  velocity-window test (12, item 3) targets it.

### 13g. What this says about the simplification

The paper and our simulation treat each agent as an **ideal double integrator**:
the commanded acceleration happens instantly (sections 5a.4 and 5a.6). The
Crazyflie behaves like a double integrator **plus ~0.3 s of lag**, through the
chain in 13e.
- For coverage that lag is harmless: 5a.4 predicted "arriving later at the same
  place".
- For trochoidal it is decisive. The design needs a strong velocity brake, and a
  strong brake cannot tolerate delay.

This is the section 4 form of a result:
- **The general claim.** Trochoidal consensus assumes zero actuation delay. With
  a loop delay τ, its velocity gain must stay below roughly π/(2τ), which caps
  how fast a pattern can be flown.
- **The local measurement.** Here τ ≈ 0.28 s, about half of it our stack. The
  ceiling is β ≈ 4–6, which means patterns slower than ~0.04–0.06 m/s for this
  design.
- **The prediction in section 6 was half right.** Trochoidal did degrade most and
  coverage least. But the lag did not make the trochoid decay or spiral. It
  produced a separate fast oscillation that swamps the pattern.

### 13h. The supervisor's concern, and the graph

**The "black box" of sending position, velocity and acceleration is where the
delay lives.**
- `crazyflie_node` turns the acceleration into a moving target, and the onboard
  controller chases that target ~0.3 s behind.
- The box also contains limits the simulation does not have:
  - a 0.7 m/s cap on the target's velocity, while the drones actually flew at
    0.75–1.05 m/s;
  - the 0.3 m leash, engaged on 7–16% of ticks.
- The supervisor's suspicion and the mechanism in 13c are the same thing, seen
  from two ends.

**The complex graph is probably not the direct cause.**
- The oscillation comes from each drone's own brake, which involves no neighbour.
  In the logs the brake term was 5–14 m/s², and the graph coupling 0.5–6.
- The graph may matter indirectly, if this graph and rotation angle force a
  larger β for a given pattern speed than a simpler design would.
- A simpler graph at the same β separates the two (12, item 5).

### 13i. How it could be fixed

There are four levers. The project aim (sections 1–2) is to explain the
oscillation, not to tune it away, so the main result needs none of them. They
matter for what the report says the hardware would need.

1. **Lower the velocity gain.**
   - *Flocking:* lower c2α (the `c2a05` config) or c2γ. Olfati-Saber's
     guarantees hold for any positive gains; the flock just matches velocities
     more slowly.
   - *Trochoidal:* β is not a free dial. For a fixed graph and rotation angle,
     the eigenvalue design ties β to the pattern speed (the time rescale keeps
     β²/α fixed). So lowering β means a slower pattern: β = 3 gives ~0.03 m/s and
     a 38 s lap. Another graph or angle might need less β for the same speed;
     `tools/design_consensus_gains.py` can check.
2. **Cut our share of the delay.** Run the algorithm at 25–50 Hz instead of 10,
   and use a better velocity estimate. The law is unchanged and still fed the
   drone's measured velocity, so this is a harness fix.
   - Realistically, the delay drops from ~0.28 s to ~0.22 s, which moves the
     ceiling from β ≈ 4–6 to ≈ 5–7.
   - Removing all of our share (leaving ~0.15 s) would give β ≈ 7–10.
3. **Take the delay out of the brake ("lever 3").** Explained below.
4. **Delay compensation.** Predict the velocity ~0.3 s ahead from a model. This
   is more complex, and only a partial fix.

**Lever 3 in detail.** For each real drone there are two things:
- The **moving target** exists only in `crazyflie_node`, as `_desired_pos` and
  `_desired_vel`, updated every 1/25 s from the command. It is the black line in
  figures 6–7.
- The **drone** chases the target ~0.3 s behind.

"Pushing the target" means only changing those two stored numbers. The
algorithm never pushes the drone directly, either today or under lever 3. Lever
3 changes only which velocity the brake reads:

| Term | Now reads | Lever 3 reads | "Replay the simulation" |
|---|---|---|---|
| −α × position (pull to centre) | real drone position | real drone position | target position |
| −β × velocity (brake) | real drone velocity, ~0.3 s late | target velocity (exact, instant) | target velocity |
| −κ × gaps to neighbours (coupling) | real drone positions | real drone positions | target positions |

Under lever 3 the brake reads and changes the same number, so it never acts on
old information. In the worked example of 13c, the target stops at 0.1 s, the
brake sees that at once and stops pushing, and the drone stops 0.3 s later with
no overshoot. The virtual drones already work this way, which is why they fly
the design.

**What it costs.**
- The brake no longer sees the real drone's speed. If a drone lags, gets
  knocked, or cannot keep up, the brake does not know.
- The law is no longer fed the true state the paper assumes.

**What it is not.** It is not the right-hand column:
- Real positions still feed back, through the centring and coupling terms and
  the leash. So the target does not simply replay the simulation.
- It would also leave the position terms' ~0.3 s lag visible without the fast
  circles on top. That is the slow growth or decay section 6 originally
  predicted.

**Status.** Lever 3 is an adaptation of the law (section 4, category (c)), not a
harness fix. It is legitimate in two roles:
- as a diagnostic: the same gains, with the delay removed from the brake. If the
  circles vanish, the mechanism is shown.
- as a clearly labelled remedy.

It should not become "the algorithm works on hardware". To be discussed with the
supervisor before any code is written. Not implemented.

### 13j. What is measured, and what is inferred

**Measured in the logs:**
- the ~1.1–1.3 s oscillation: 15–19 cm at clamp 3.5, and 3–4 cm at 0.5;
- the drones' acceleration following the command 0.22–0.26 s later (correlation
  0.9–0.97);
- period ≈ 4 × that delay, with both sides measured independently;
- the virtual drones, which do not oscillate;
- the brake term dominating the command;
- the oscillation following the velocity gain across algorithms (the coverage
  evidence is weaker).

**Reconstructed or inferred:**
- The commands and setpoints are not logged. They are rebuilt from 10 Hz
  positions, so the play-by-play of 13c cannot be seen directly.
- The k·τ < π/2 rule is for a pure delay and only approximate for a real drone.
  Treat the delay as ±0.03 s.
- Other contributors inside the black box are not excluded. Examples: the
  0.7 m/s cap on the target velocity, and the 0.3 m leash.
- Tested 16 Sep at β = 3, below the ceiling: the oscillation collapsed as
  predicted (section 14). One flight, one rung — r2 and r4 would pin it down.
- The recorder's own trochoidal summary is still not quotable (11c).

**To turn it into a known result:**
1. Fly trochoidal at β = 2–3 (12, item 1). If it oscillates as at β = 6–14, 13c
   is wrong.
2. Log the commands and setpoints (12, item 6).
3. Run a simulation check (it can be done now; not done yet): the same law with
   ~0.25 s of delay and the clamp added. If it reproduces a ~1.1 s loop of
   15–19 cm at all three rungs, and 3–4 cm at clamp 0.5, the explanation is much
   stronger. It would not be proof.

### 13k. How to write it up

The main result, with the law unmodified and fed the measured velocity:

> Each algorithm's gain on a drone's own velocity, multiplied by the measured
> loop delay (~0.28 s), predicts the ~1.1 s oscillation. Coverage ≈ 0.3: none.
> Flocking ≈ 1.1: small. Trochoidal 1.5–3.5: large. Trochoidal's design ties β to
> pattern speed, so the delay caps this design at about 0.04–0.06 m/s. About
> half the delay is our software (reducible) and half the vehicle (not).

Then, once flown, add the β = 2–3 and `c2a05` confirmations. If lever 3 is used,
present it as a diagnostic and as a labelled adaptation with its cost stated,
never as the published algorithm working on hardware.

### 13l. Figures

In `docs/figures/trochoidal_ladder/`; regenerate with
`python3 tools/plot_trochoidal_ladder.py`.

| File | Shows |
|---|---|
| `1_summary.png` | every metric against speed |
| `2_pattern_size.png` | pattern size over time, in laps |
| `3_shake.png` | the oscillation seen from above, and its single spectral peak |
| `4_delay.png` | the ~0.3 s delay, and period = 4 × delay |
| `5_tilt.png` | VICON tilt against the tilt the flown path needs |
| `6_`, `7_commanded_vs_actual` | the drone against its (rebuilt) setpoint, at the start and mid-flight |
| `8_`, `9_design_vs_actual` | the drone against the designed pattern, at the start and mid-flight |

## 14. Hardware findings, 16 September 2026: below the ceiling

*The flights section 13 called for. Same law, same clamp, same drones, same
marks; only the speed is lower, so β falls from 6 to 3 and then 2, and β × delay
from ~1.7 to ~0.9 and ~0.6. Records
`logs/hw/trochoidalconsensus_20260916_143323.txt` (r3) and `_145915.txt` (r2).
Replay confirms the config gains flew in both (0.1–0.2 cm).*

### 14a. What changed

| | r3 (16 Sep) | r6 | r10 | r14 |
|---|---|---|---|---|
| β × delay | **0.9** | 1.7 | 2.8 | 3.9 |
| Command clipped at 3.5 m/s² | **0% of ticks** | 79–92% | 89–94% | 89–92% |
| Wobble, RMS | **5.8–6.1 cm** | 17 cm | 15–19 cm | 15–16 cm |
| Wobble envelope, median | **1.3–2.2 cm** | 17 cm | 15–20 cm | 16–17 cm |
| Share of the flight above 10 cm | **12–15%** | 96–97% | 89–93% | 86–87% |
| Tilt, median (VICON) | **3.3–4.4°** | 24–27° | 22–32° | 24–28° |
| Real-drone speed (design) | **0.08–0.12 m/s** (0.03) | 0.83–0.90 (0.06) | 0.75–1.05 (0.11) | 0.82–0.90 (0.15) |
| Fast period, hw vs design | **37.4 vs 38.1 s (−2%)** | 2–5× too slow | 2–5× | 2–5× |
| Slow period, hw vs design | **117 vs 127 s (−8%)** | — | — | — |
| Size change per fast lap | ×0.96 | ×1.05 | ×0.92 | ×0.94 |

**The prediction held.** Everything the mechanism said depends on β × delay moved
the way it should, and everything it said does not depend on β stayed put:
- The command **never reached the clamp**, where at β = 6–14 it was clipped
  ~90% of the time.
- The drones' typical wobble fell from ~17 cm to ~2 cm, and the tilt from
  ~25° to ~4°.
- **The delay did not change:** 0.28–0.32 s, the same as every earlier flight.
  It is a property of the hardware, not of the gains.
- **The period of what wobble remains did not change either:** 1.16–1.28 s, still
  ≈ 4 × the delay.

**This is the first flight where the designed pattern actually survived.** The
fast period is within 2% of the design and the slow within 8%, the fleet held
its size (×0.96 per lap, so the slow mode's growth needs 21 laps to double
against about 1 lap on the 0.5-clamp flights), and no drone approached the
geofence in 186 s.

### 14b. What did not go away

The wobble is smaller but not gone, and it is no longer steady:
- The envelope sits at 1–2 cm for most of the flight and bursts to 13–22 cm
  about 12–15% of the time. At β = 6–14 it was above 10 cm essentially always.
- Tilt follows the same shape: median 3.3–4.4°, p95 21°, max 32–35°.

That is what the mechanism predicts just below the threshold. At β × delay ≈ 0.9
the loop is stable but barely damped, so a disturbance rings at ~4 × delay and
dies away instead of growing into a clamped limit cycle.

**A likely source of the disturbances, and a confound to rule out.** VICON
tracking was much worse on 16 Sep than on 15 Sep:

| | 16 Sep (r3) | 15 Sep |
|---|---|---|
| Gaps > 20 ms | **5.5% of samples** | 1.0–1.3% |
| Worst gap | 344 ms | 123 ms |

A 344 ms hole makes the velocity fit jump when tracking returns, and β
multiplies that jump. So some of the bursts may be dropout-driven rather than
intrinsic. The `_timing.npz` for this flight would settle it by lining the
bursts up against the gaps.

### 14c. What this does and does not establish

- **Establishes:** the oscillation follows β × delay, not β or the clamp alone,
  and it collapses below the threshold with everything else held fixed. That is
  the mechanism of 13c tested rather than merely consistent.
- **Does not establish:** where exactly the threshold sits. r3 (0.9) is clean and
  r6 (1.7) is not. r4 (1.1) would place it, and r2 (0.6) would show whether the
  residual bursts keep shrinking.
- **Still open:** the drones move 3–4× faster than the design asks (0.08–0.12
  against 0.03 m/s), so the pattern is flown, but not quietly.

### 14d. Rescale 2: the designed pattern, essentially clean

β = 2, β × delay ≈ 0.6. This is the best flight the testbed has produced.

| | r2 | r3 | r6 |
|---|---|---|---|
| β × delay | 0.6 | 0.9 | 1.7 |
| Wobble, RMS | **1.0–1.2 cm** | 5.8–6.1 cm | 17 cm |
| Wobble envelope, median | **0.6 cm** | 1.3–2.2 cm | 17 cm |
| Time above 8 cm | **0%** in every window | 12–15% | ~100% |
| Tilt, median (design) | **2.2–4.8°** (0.01°) | 3.3–4.4° | 24–27° |
| Speed vs design | **0.04 vs 0.02 m/s (2×)** | 3–4× | 14× |
| Command clipped | 0% | 0% | 79–92% |
| Fast period, hw vs design | **57.05 vs 57.10 s (0%)** | −2% | 2–5× too slow |
| Slow period, hw vs design | **189.5 vs 190.4 s (0%)** | −8% | — |
| Period ratio (design 3.33) | **3.32** | 3.14 | — |
| Mode fit R² | **0.98** | 0.84 | 0.73–0.88 |
| Size change per fast lap | **×1.00** | ×0.96 | ×1.05 |

- **The bursts are gone.** At r3 the wobble rang for the first ~60 s and reached
  22 cm; at r2 it never exceeds 8 cm in any 30 s window, and sits at 0.6 cm.
- **The pattern is the designed one.** Both periods land within a fraction of a
  percent, the ratio matches, the fit is clean (R² 0.98), and the fleet held its
  size exactly (×1.00 per lap) for 185 s.
- **The delay is unchanged again:** 0.29–0.34 s, as on every flight since 2 Sep.
- What remains is the drones flying about twice the designed speed, with a ~1 cm
  residual wobble at the edge of the measurable band, and 2–5° of tilt where the
  design asks for 0.01°.

So the ladder now reads: **sustained oscillation at β = 6–14, a decaying
transient at β = 3, and essentially nothing at β = 2.**

### 14e. A measurement note: where each flight's window ends

`plot_trochoidal_ladder.py` now ends a flight's window at the first *sustained*
tilt over 90° (half a second of it). A tilt that large means VICON lost the body
or the drone has landed. The r2 record ends exactly that way: flips from 201 s, a
jump to y = −3.3 m (outside the room), then frozen rows.

Without the guard, that landing tail counted as flight and produced two
artefacts: a "drone reached the 1.3 m edge at 188 s" verdict, and a 29 cm wobble
burst in the last 30 s. Both are tracking, not flying. The rule requires half a
second because single-row spikes happen (r6 has one at 64.8 s) and would
otherwise throw away most of a good flight.

### 14f. Next

1. **r4** (`testbed_fig4_r4.yaml`, β × delay ≈ 1.1): the edge, and about where
   flocking sits. With r2 clean, r3 ringing and r6 sustained, r4 is what places
   the threshold.
2. **Upload the `_timing.npz`** with the next record, so the r3 bursts can be
   tested against the tracking dropouts.
3. ~~Flocking `c2a05` (12, item 2) remains the second-algorithm test.~~
   *Done differently on 16 Sep: the second-algorithm test was run as a
   time rescale (flocking s2) rather than a lone gain change, and
   coverage was added as a third. See section 15.*
4. Optional, once the ladder is done: lever 3 (12, item 7), with the supervisor's
   agreement.

## 15. Hardware findings, 16 September 2026: the coverage and flocking ladders

*Flown 16 Sep, after the trochoidal rungs of section 14; analysed 20–21 Sep.
This is the section that carries the delay result from one algorithm to three —
including the first deliberate attempt to make a quiet algorithm oscillate.*

### 15a. What flew

Five records, each **one real drone** (VICON `drone_1` = agent `drone1`) flying
with three simulated agents in the same flight. That layout matters: the
simulated agents obey the same law through the same ROS graph but have no radio,
no motors and no airframe, so every flight carries its own delay-free control
group. `max_accel` 3.5 on every run.

| Rung | Config | Record | Gains | k = own-velocity gain | k·τ (τ = 0.28 s) |
|---|---|---|---|---|---|
| c1 | `testbed_coverage_c1.yaml` | `coverage_20260916_161129` | kp 1.0, kd 1.2 | 1.20 | 0.34 |
| c2 | `testbed_coverage_c2.yaml` | `coverage_20260916_161409` | kp 4.0, kd 2.4 | 2.40 | 0.67 |
| c3 | `testbed_coverage_c3.yaml` | `coverage_20260916_161948` | kp 9.0, kd 3.6 | 3.60 | 1.01 |
| s3 | `testbed_flocking_hybrid_s3.yaml` | `flocking_20260916_165909` | c2α 0.667, c2γ 0.467 | 1.39–1.61 | 0.39–0.45 |
| s2 | `testbed_flocking_hybrid_s2.yaml` | `flocking_20260916_165223` | c2α 1.0, c2γ 0.7 | 2.06–2.34 | 0.58–0.66 |
| s1 | `testbed_flocking_hybrid.yaml` | `flocking_20260916_164800` | c2α 2.0, c2γ 1.4 | 4.12–4.75 | 1.15–1.33 |

Every table and figure below is ordered the same way: **closest to what
the theory asks for first, furthest away last** — that is, ascending k·τ.
The rung names cannot carry that order themselves (coverage counts up
with speed, flocking counts down), so the figures label each rung by its
k·τ and put the name in brackets underneath.

Rung names are the rescale factor: **s1 is the unrescaled law** (c = 1), s2 is
half speed, s3 a third — s1 is *not* a reference run, since every rung is scored
against simulation of its own config. Coverage's c1/c2/c3 run the other way,
c1 being the slowest.

Both ladders are **time rescales**, not detunes: position-type gains ×c²,
velocity-type gains ×c, and the moving reference ×c. The trajectory and the
settled geometry are unchanged — verified in simulation to under 2 cm — so the
only thing that moves is k, and therefore k·τ. Coverage needed a *moving*
hotspot (0.3 rad/s on a 0.6 m orbit) to have anything to track, because plain
coverage settles in 1–5 s and then sits still.

Flocking `s3` (k·τ ≈ 0.4) was flown on the same afternoon but its record was
only uploaded on 22 Sep; it is included here and completes the quiet end of the
flocking ladder.

### 15b. Coverage: the quiet algorithm made to ring

| | c1 (k·τ 0.34) | c2 (k·τ 0.67) | c3 (k·τ 1.01) |
|---|---|---|---|
| Wobble RMS, real drone | 0.46 cm | 1.94 cm | **23.32 cm** |
| Own (fleet-common removed) | 0.35 cm | 1.46 cm | 17.82 cm |
| Ripple period | — | — | **1.16 s** |
| Measured τ, real drone | 0.35 s* | 0.27 s* | 0.25 s |
| Command clipped | 0% | 0% | **79%** |
| Median tilt demanded | 0.6° | 1.2° | **33.2°** |
| Median speed | 0.14 m/s | 0.16 m/s | **1.25 m/s** |
| Peak speed | 0.25 m/s | 0.83 m/s | 1.54 m/s |
| Coverage cost H vs simulation | **−3.2%** | **−1.9%** | **+14.5%** |
| Median distance from the designed position | 8 cm | 6 cm | **89 cm** |
| Wobble, the three simulated agents | 0.01–0.02 cm | 0.06–0.16 cm | 2.54–3.18 cm |
| Fleet-common ripple | 0.12 cm | 0.48 cm | 5.59 cm |

\* τ at c1 and c2 is not meaningful: with no wobble there is nothing for the
cross-correlation to time, and the peak correlation is only 0.19 and 0.38. At c3
it is 0.96. A delay is only measurable when something is happening.

Read as a ladder:

- **Below k·τ ≈ 0.7 coverage is as good as its own simulation** — H lands
  within 2–3% of the simulated value, and the hardware is *slightly better* than
  sim at both rungs, which is what a converged run in a slightly different basin
  looks like. Nothing oscillates; nothing clips.
- **At k·τ = 1.01 it breaks, hard.** 23 cm of wobble at 1.16 s, with a measured
  τ of 0.25 s — 4τ = 1.00 s. The drone spends 79% of ticks at the acceleration
  clamp, demands 33° of bank where c1 asked for 0.6°, and flies at 1.25 m/s
  where the law wanted ~0.2.
- **The cost degrades only at the rung that oscillates** (+14.5%), and the
  degradation is small compared to the motion: the fleet keeps covering the
  region *because* the oscillation is roughly symmetric about where the drone
  should be. H is an integral, and an integral forgives a wobble.
- **It leaks into the agents that have no hardware.** The three simulated drones
  pick up 2.5–3.2 cm of ripple at the same period, with a 5.6 cm fleet-common
  component. Voronoi neighbours are coupled through position, so one oscillating
  agent moves everyone's cell boundary. This is the coverage analogue of what
  11d guessed at for flocking, and here it is unambiguous, because those three
  agents are integrated by `drone_node` with no physical layer at all.

This is the first time the project has **produced** the failure rather than
found it. Coverage was the robust control condition; pushing it to k·τ ≈ 1 made
it fail in exactly the way the delay model says it should.

### 15c. Flocking: the rescale closes the gap to simulation

| | s3 (k·τ 0.45) | s2 (k·τ 0.64) | s1 (k·τ 1.32) |
|---|---|---|---|
| Wobble RMS, real drone | **0.46 cm** | 0.77 cm | 2.88 cm |
| Own (fleet-common removed) | 0.37 cm | 0.62 cm | 2.57 cm |
| Ripple period | 1.42 s* | 1.22 s | 1.16 s |
| Measured τ, real drone | 0.34 s (corr 0.72) | 0.33 s (corr 0.80) | 0.31 s (corr 0.90) |
| Command clipped | **0%** | **0%** | **83%** |
| Median tilt demanded | 0.5° | 0.8° | 4.6° |
| Median speed | 0.06 m/s | 0.08 m/s | 0.20 m/s |
| Lattice error vs simulation | **−0.1%** | **−1.2%** | **+40.8%** |
| Median distance from the designed position | **6 cm** | 12 cm | 21 cm |
| Wobble, simulated agents | 0.02–0.05 cm | 0.06–0.13 cm | 0.84–0.96 cm |
| Fleet-common ripple | 0.09 cm | 0.16 cm | 0.69 cm |
| Closest approach d_min | 0.57 m | 0.58 m | 0.46 m |
| Window | 147 s | 119 s | 119 s |

\* At s3 there is barely a ripple left to time: 0.46 cm is the same figure the
quietest coverage rung returned, so the period fit is running on close to the
measurement floor and should not be read as precisely as the s1/s2 entries.

- **s3 is the quiet end, and it arrives where the mechanism says it should.**
  Every column improves monotonically from s1 to s3, and at k·τ 0.45 the real
  drone's wobble (0.46 cm) is indistinguishable from coverage c1's at k·τ 0.34
  (0.46 cm) — two different algorithms, two different papers, the same floor.
  That floor is what the hardware itself contributes once the delay stops
  driving anything, and nothing below k·τ ≈ 0.45 buys any more quiet.
- **The lattice error at s3 is −0.1% of simulation of its own config.** The
  gap the project set out to explain is, at this rung, not measurable.
- **At s2 the sim-to-hardware gap does not shrink, it closes.** The settled
  lattice error is within 1.2% of simulation of the same file, the command never
  clips, and the wobble drops 3.7×. The flock is the same flock — same spacing,
  same diamond, same 0.6 m orbit — flown on a clock half as fast.
- **s1 is the same flight the 8 Sep runs were**, reproduced: 2.88 cm
  against 2.5–4.0 cm then, at 1.16 s against 1.0–1.3 s then, with 83% clipping.
  Two weeks and a different number of real drones later, the same k·τ gives the
  same behaviour.
- **The 41% lattice-error gap at s1 is the cost of the oscillation**, and
  it disappears with it. That is the cleanest statement of the trade the whole
  project is about: *speed bought at the price of the property the theorem
  promises, with the exchange rate set by k·τ.*

### 15d. A defect in the recorder's coverage cost column

**The `H` column of the 16 Sep coverage records is wrong as logged, and the
correction is large.** `CoverageMetrics.row()` integrates the density at the time
it is handed, and `metrics_recorder.py` hands it *its own* elapsed time, which
starts when the recorder is launched. The algorithm's hotspot clock starts when
the algorithm starts — about 12.5 s later, after takeoff. At 0.3 rad/s that is
~3.8 rad, so the logged H scores the fleet against a hotspot most of an orbit
away from the one the drones are chasing.

Scanning the offset confirms it: mean H is minimised at 14 s for both c1 and c3,
against measured algorithm starts of 12.6 s and 12.4 s.

| Rung | H as logged | H on the algorithm's clock | simulation |
|---|---|---|---|
| c1 | 0.366 | 0.160 | 0.165 |
| c2 | 0.524 | 0.120 | 0.122 |
| c3 | 0.583 | 0.128 | 0.112 |

Everything else in those records is unaffected — positions, centroid distances,
wobble, tilt, clipping and delay never use `t`. **Only runs with a moving
density are affected**, which is every coverage run from 16 Sep onward and none
before: the 2 Sep runs used `density: uniform`, where the two clocks cannot
disagree.

The analysis now recomputes H on the algorithm's clock
(`ladder_common.recompute_coverage_H`), and figure 2 of the coverage ladder
plots both series so the artefact is visible rather than hidden. **The recorder
itself is still wrong for future moving-density runs.** The fix is for the
recorder to start its density clock when the fleet starts moving, rather than
when the recorder is launched — not yet implemented, because it changes a file
that every past record was written by.

The general lesson is worth keeping: *a metric that reads a clock is a metric
that can be out of phase with the thing it scores.* The first version of this
section reported gaps of +111%, +328% and +432% and would have concluded that
coverage degrades catastrophically with speed. It does not.

### 15e. The delay, split into its software and physical halves

Because each flight carries simulated agents that share the software path and
have no hardware, the loop delay splits for the first time:

| | real drone | simulated agents, same flight |
|---|---|---|
| Coverage c3 | 0.25 s | 0.16 s |
| Flocking s1 | 0.31 s | 0.15–0.16 s |
| Flocking s2 | 0.33 s | 0.16–0.17 s |
| Flocking 8 Sep (2 real) | 0.28–0.32 s | 0.14–0.19 s |

A simulated agent's 0.15–0.17 s is entirely **software**: the mocap/state
publish path, the 10 Hz algorithm tick, and message passing. The real drone adds
**0.10–0.16 s** on top, which is the radio, the firmware, and the airframe
physically tilting.

This **corrects the estimate in 13e**, which guessed ~90 ms of sensing and
~180 ms of actuation from the rates in the code. Measured, it is closer to the
reverse: the software half is the larger one. Two caveats:

- both columns are measured by the same method (replay the law on logged
  positions, cross-correlate against achieved acceleration), so the *difference*
  is trustworthy even though each absolute number inherits the same
  centred-difference smoothing;
- a simulated agent's dynamics are exact, so its 0.16 s is a floor for the
  software path, not a full accounting of it.

It also matters for the mechanism. A delay made mostly of **dead time** (command
in flight, nothing happening) destabilises a loop more than the same delay made
of a *gradual* response, because a gradual response also attenuates. A toy model
with only 0.05 s of dead time and a 0.11 s tilt lag predicts β = 6 would be
stable — and r6 was emphatically not. The measured split says there is enough
genuine dead time in the software path for that prediction to fail, which is
what the flights show.

### 15f. Where the threshold sits now

Fourteen flights, three algorithms, three papers, one number:

| Algorithm | k·τ | Wobble, real drone | Verdict |
|---|---|---|---|
| Trochoidal r1 | 0.28 | 0.5 cm | quiet |
| Coverage c1 | 0.34 | 0.46 cm | quiet |
| Flocking s3 | 0.45 | 0.46 cm | quiet |
| Trochoidal r2 | 0.56 | 1.0–1.2 cm | quiet |
| Flocking s2 | 0.64 | 0.77 cm | quiet |
| Coverage c2 | 0.67 | 1.94 cm | quiet |
| Trochoidal r3 | 0.84 | 1.3–2.2 cm (bursts to 22) | decaying transient |
| **Coverage c3** | **1.01** | **23.3 cm** | **sustained** |
| **Trochoidal r4** | **1.12** | **16.6 cm** | **sustained** |
| Flocking s1 | 1.32 | 2.88 cm | sustained, mild |
| Trochoidal r5 | 1.40 | 21.1 cm | sustained |
| Trochoidal r6 | 1.68 | 17 cm | sustained |
| Trochoidal r10 | 2.80 | 15–19 cm | sustained |
| Trochoidal r14 | 3.92 | 15–16 cm | sustained |

Trochoidal r4 (22 Sep) is the strongest single confirmation the ladder has
produced. It sits at k·τ 1.12, just past the bracket, and it rings at 16.6 cm —
but it saturates the command only **10% of the time**, against 79–95% for r5,
r6, r10 and r14. Clipping is therefore not what causes the oscillation; it is
only what stops the oscillation growing. That distinction was previously an
inference from the model, and r4 is the flight that separates the two.
Trochoidal r1 anchors the other end: β = 1 is the paper's own published gain
set, k·τ 0.28, and it returns 0.5 cm — the same floor as coverage c1 and
flocking s3.

Flocking s3 does not move the bracket — it lands in the quiet half, where the
mechanism already said it would — but it does put a floor under the table: two
algorithms now return the *same* 0.46 cm at two different k·τ below 0.5, which
is how you can tell the lower rungs have stopped measuring the delay and started
measuring the hardware.

**The threshold is bracketed between 0.67 and 1.01**, tighter than the "fuzzy
≈ 1" of 13c, and coverage c3 is now the binding upper bound: severe oscillation
at exactly 1.0, with trochoidal r4 the next rung up at 1.12. Nothing quiet has
been measured above 0.67 and nothing calm above 1.01, across three laws.

Why flocking s1 at 1.32 wobbles less than coverage c3 at 1.01 is worth stating rather
than smoothing over: **amplitude is set by the clamp and the speed of the
reference, not by how far past the threshold you are.** Flocking's leader crawls
at 0.15 m/s and its commands sit near the clamp at 0.5-scale accelerations;
coverage c3 is chasing a hotspot with kp = 9 and demands 4.7 m/s² against a 3.5
clamp. Past the threshold, k decides *whether*, and the clamp and the reference
decide *how big*.

### 15g. What is measured and what is inferred

Measured directly from the records:

- every wobble, period, tilt, speed, clipping fraction and delay in 15b and 15c;
- the coverage cost on both clocks (15d);
- the real/simulated delay split (15e).

Inferred, and dependent on the model:

- that k·τ is the *right* single number — supported by fourteen flights across three
  laws, but the threshold is bracketed, not resolved;
- that the delay's composition is dead-time-heavy (15e) — consistent with the
  flights, not separately measured. A step-command test from hover would settle
  it: command a fixed sideways acceleration and see whether the response starts
  flat for ~0.1 s or begins rising immediately;
- that the leak into the simulated agents is positional coupling rather than a
  second mechanism. For coverage this is nearly forced (those agents have no
  physics); for flocking the velocity-matching path of 11d remains the
  hypothesis.

### 15h. Figures and how to reproduce

`tools/ladder_common.py` holds the method — replay the repo's own algorithm
class over the logged positions, cross-correlate the command against the
achieved acceleration for τ, band-pass 0.6–1.5 Hz for the wobble, read k out of
the law — so coverage and flocking are measured the *same* way as trochoidal,
not a similar way.

```bash
python3 tools/plot_ladder.py --algo coverage     # -> docs/figures/coverage_ladder/
python3 tools/plot_ladder.py --algo flocking     # -> docs/figures/flocking_ladder/
python3 tools/make_figure_index.py --embed       # -> docs/figures/index.html
```

Six figures per ladder: `1_summary`, `2_expected_vs_actual` (**design vs
actual**: the same config in simulation against where the drone went, plus the
distance between them over time), `3_promise` (the property the theorem
promises), `4_wobble` (the ripple alone), `5_delay` (correlation curves, and
real vs simulated τ), `6_command`.

`tools/plot_key.py` draws the one cross-algorithm figure — every flight's wobble
against its k·τ, and the 4τ test — from the three ladders' `summary.json`, so it
cannot disagree with them. `index.html` leads with those four key figures and
folds the rest into per-algorithm detail; `index_standalone.html` inlines the
images for sending to someone else.

A note on the setpoint: `ladder_common.setpoint()` reconstructs what
crazyflie_node streamed, but for coverage and flocking that setpoint sits on the
node's 0.3 m leash 94–97% of the time (trochoidal: 6–8%), because a law chasing
a moving reference produces a command with a persistent forward bias. The
reconstructed setpoint therefore carries the leash's shape rather than the
law's, and is not plotted.

### 15i. Next

1. **A run-to-run repeat, flown twice in the same session.** Every rung in
   every table is a single flight, so nothing above has an error bar. It has to
   be same-session to mean anything, and the rung worth repeating is one that
   oscillates — c3's 23 cm or r6's 17 cm — since those are the numbers the whole
   argument rests on.
2. **A step-command tilt test** (15e): the one measurement that would separate
   dead time from a gradual actuator response, and the last soft spot in the
   mechanism story.
3. **Fix the recorder's density clock** (15d) before any further moving-hotspot
   run, or every future H column repeats the artefact.
4. Lever 3 (13i) still waits on the supervisor.

### 15j. Two levers that widen the gap without touching k·τ

Every ladder in 15a–15f varies k·τ. These two sweeps (22 Sep) deliberately do
not: within each sweep the gains are byte-identical and only the *task* changes.
The question they answer is whether the delay mechanism is the only thing
separating hardware from theory, or just the one we found first.

Both sweeps are **one session only**. Coverage c2 and flocking s2 were also
flown on 16 Sep with the same configs, and those flights stay in the k·τ ladder,
but they are not used here: a different session means different batteries, a
different VICON calibration and drones on different marks, and mixing sessions
puts a session difference on the same axis as the lever. The cost is that the
flocking sweep is only two points.

Figures: `docs/figures/coverage_hotspot/` and `docs/figures/flocking_sense/`,
built by `tools/plot_levers.py`.

**Coverage — hotspot orbit rate, gains fixed at c2 (k·τ 0.67 throughout)**

| | 0.3 rad/s | 0.6 rad/s | 0.9 rad/s |
|---|---|---|---|
| Oscillation radius, real drone | 0.87 cm | 1.28 cm | 1.36 cm |
| Command clipped | 0% | 0% | 0% |
| Distance from the designed position, median | 4 cm | 8 cm | **13 cm** |
| … 95th percentile | 7 cm | 12 cm | **46 cm** |
| Fleet lag behind the hotspot | 1.15 s = 20° | 1.00 s = 34° | 0.85 s = **44°** |
| … the same lag in simulation | 1.40 s = 24° | 1.25 s = 43° | 1.10 s = 57° |
| Coverage cost H at its own phase | 0.1058 | 0.1117 | 0.1222 |
| … simulation of the same file | 0.1057 | 0.1110 | 0.1209 |
| … gap | +0.1% | +0.6% | +1.1% |

**Flocking — sense range, s2 gains and 0.70 m spacing throughout**

| | 0.78 m | 0.73 m |
|---|---|---|
| k (own-velocity gain) | 1.90 | 1.74 |
| k·τ | 0.53 | 0.49 |
| Oscillation radius, real drone | 0.41 cm | 0.54 cm |
| Settled distance between neighbours | 0.575 m | 0.562 m |
| … in simulation | 0.569 m | 0.550 m |
| … the law asks for | 0.70 m | 0.70 m |
| Graph connected | 100% | 99.3% |
| Distance from the designed position, median | 6 cm | 5 cm |

- **The fleet runs a fixed lag behind a moving reference, not a fixed angle.**
  Coverage trails the hotspot by about a second at every speed; tripling the
  speed turns that same second into 20° → 44° of orbit, and that is what raises
  H and what puts the drone 13 cm off its designed position. Nothing about the
  hardware got worse — the task got harder in exactly the way a constant lag
  predicts.
- **Scored at the phase it actually ran at, hardware matches simulation to
  1% at every speed.** Scored at a fixed phase instead, hardware appears to beat
  simulation by 4–13%, growing with speed — a phase-alignment artefact in the
  same family as the recorder defect of 15d, because `t_start` is only
  recoverable to about a tick and at 0.9 rad/s a quarter of a second is 13° of
  hotspot phase. *Caveat:* hardware's measured lag is a consistent 0.25 s
  shorter than simulation's, which is the same size as that ambiguity, so it
  should not be read as hardware leading simulation.
- **The flock settles well short of the spacing it is told to hold**, at 0.575
  and 0.562 m against 0.70 m, and shorter still as the sense range shrinks.
  Simulation of the same file settles at 0.569 and 0.550 m, so this is the
  control law's own behaviour and not a hardware limit.
- **Shrinking the sense range does not fragment the flock — the second time
  this prediction has been wrong.** 15c predicted the graph would flicker in and
  out of connected; the simulation said it would not; hardware agrees with the
  simulation, staying connected 99–100% of the flight. The flock compresses to
  stay inside the shrinking radius rather than losing edges.
- **Both levers widen the gap from *theory* without widening the gap from
  *simulation*.** That is the point of the two sweeps taken together. k·τ is not
  just one knob among several: it is the only one so far under which the
  physical layer becomes the binding constraint. Everything these two levers
  break, an ideal simulation breaks in the same way and by the same amount.
- **What these sweeps cannot say** is how much of any of it is run-to-run
  scatter, because each rung is still a single flight. That remains item 1 of
  15i, and it has to be answered inside one session to be worth anything.
