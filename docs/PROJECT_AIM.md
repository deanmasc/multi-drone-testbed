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
| `max_accel = 0.5` | Chosen for safety in a small flight volume. The airframe can do considerably more. |
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
  must be decomposed rather than blamed as a lump.

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

2. **Force is saturated.** `max_accel = 0.5` m/s^2 in every algorithm, plus a
   velocity clamp and the `max_lead` leash in `crazyflie_node`. No stability
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

   This is the simplification with the sharpest predicted split between
   algorithms. Trochoidal places eigenvalues *precisely* on the imaginary axis,
   and unmodelled loop lag moves them off it — so the trochoid should decay or
   spiral rather than close. Coverage drives to a fixed point, so lag only means
   arriving later at the same place. Same simplification, opposite consequences.

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
between the virtual double integrator and the real drone chasing it. Both remain
plausible and untested. Discretisation is off the list.

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
