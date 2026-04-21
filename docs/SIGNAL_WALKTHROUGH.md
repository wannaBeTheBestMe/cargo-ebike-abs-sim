# Signal Walkthrough — Phase C ABS Panic Stop

A sequential narration of the Phase C ABS scenario, tracing every signal from
the rider's initial command through the plant, the sensor chain, and back into
the controller. Read this alongside
[`BLOCK_DIAGRAM.md`](BLOCK_DIAGRAM.md) — each section names the block it's
describing so you can spot it on the diagram.

Audience: you already know that this project simulates a cargo e-bike
braking, that there are roughly a dozen blocks split across plant / sensing /
control, and that the goal is to compare ABS against cadence braking. This
walkthrough fills in *how a single simulation step actually flows* — and why
several non-obvious design choices are the way they are.

---

## 0. Where the simulation starts

The only things that exist at `t = 0` are:

- **Scenario parameters** loaded from `configs/default.toml` — masses, radii,
  tire coefficients, actuator constants, controller thresholds.
- **Initial conditions** — `v(0) = 8.33 m/s` (30 km/h), `ω_f(0) = v(0)/R_f`,
  motor current zero, hydraulic clamp zero, ABS FSM in `APPLY`.
- **The rider's intent**, encoded as the linear voltage ramp that
  `HumanBrakeController` will emit. That ramp is the *only* exogenous
  time-varying input to the simulation.

The simulator (`src/ebike_abs/simulator.py`) then walks all 13 blocks in
declared topological order at `dt = 1 × 10⁻⁴ s`, using RK4 for continuous
states and `commit()` calls for discrete state. Everything that follows
happens inside that loop.

---

## 1. `HumanBrakeController` — the rider's open-loop command

**Produces:** `V_pwm_cmd` [V] (algebraic, fresh each step).

```
V_pwm_cmd(t) = V_hold · min(1, t / t_rise)
```

The rider squeezes the electro-hydraulic brake lever with a linear 0 → 6 V
ramp over 150 ms, then holds at 6 V. This is deliberately *open loop* — the
model rider does not react to wheel lockup. That's the whole point of a
baseline: without feedback, a hard stop on dry asphalt reaches a fully locked
front wheel within the first couple of tenths of a second. Phase C then
layers the ABS FSM (or the cadence chop) on top of this profile to see how
much damage it can undo.

On the diagram this block sits at the far left of the purple **Control**
cluster with a single arrow labelled `V_cmd  V_pwm_cmd  [V]  (A)` heading
right.

---

## 2. `ABSController` — gating the rider's command

**Reads:** `V_pwm_cmd` (fresh), and three estimator outputs (`lambda_f_hat`,
`omega_f_hat_dot`, `v_hat`) that are read **lag-1** from the persistent
signals dict — those arrows are dashed red on the diagram.

**Produces:** `V_pwm` [V] plus the `abs_mode` diagnostic.

The FSM has five states:

| State | Output | Exit condition |
|---|---|---|
| `APPLY` | pass `V_pwm_cmd` through | `λ̂_f > λ_on ∧ ω̇̂_f < ω̇_trig` → `DUMP` |
| `DUMP` | force `V = V_dump = 0` | `λ̂_f < λ_off ∧ ω̇̂_f > 0` → `HOLD` |
| `HOLD` | force `V = 0` | dwell timer (50 ms) expires → `REAPPLY` |
| `REAPPLY` | pass `V_pwm_cmd` through | same lockup trigger → `DUMP` |
| `BYPASS` | pass `V_pwm_cmd` through | absorbing — entered when `v̂ < v_cutoff` |

Two details worth knowing:

- The lockup trigger is **conjunctive**: both high slip *and* fast wheel
  deceleration. This rejects slow slip drift from estimator settling.
- Below `v_cutoff ≈ 1.4 m/s` (walking pace), the FSM enters `BYPASS` and
  stops modulating. The sensor chain loses resolution at low speed, and the
  modulation benefit evaporates anyway.

On step 1 the estimator feedback is still zeros, so the FSM quietly stays in
`APPLY` and simply passes `V_pwm_cmd` through.

---

## 3. `MotorActuator` — turning voltage into clamping force

**Reads:** `V_pwm` (fresh from the ABS).
**Produces:** `F_clamp` [N] plus the `i_motor`, `omega_motor` diagnostics.

This is the plant's most complex block — three first-order ODEs cascaded with
one algebraic lead-screw step:

```
L · di/dt       = V_pwm − R · i − K_e · ω_m            (electrical)
J · dω_m/dt     = K_t · i − b_m · ω_m                  (mechanical)
F_piston        = K_t · i / r_lever                    (lead-screw / cam)
τ_hyd · dF/dt   = (A_c / A_m) · F_piston − F_clamp     (hydraulic lag)
```

The three states — `i_motor`, `omega_motor`, `F_clamp` — are stacked into
the RK4 vector. Two saturations matter:

- `V_pwm` is clamped to `±V_max = ±12 V` before the electrical stage. This
  matters during `DUMP`: if `V_dump` is set below `−V_max`, the limiter still
  caps it, so the motor never draws more than its supply allows.
- `F_clamp` is clipped at ≥ 0 on output — a disc brake cannot pull the pads
  away from the rotor, so a sub-zero hydraulic-lag excursion just means "no
  force." A negative state is still allowed internally; the clip lives on
  the output path, so the integrator can't wind up.

**The dominant time constant is `τ_hyd = 30 ms`** — an order of magnitude
slower than the electrical or mechanical stages. That's why the diagram's
Actuation cluster wears a double border (`∫∫∫`) and why the RK4 step was
chosen well below this: `dt = 10⁻⁴ s` is 300× smaller than the dominant
dynamics, so numerical error doesn't sneak in through the lag.

At steady state with 6 V of command, `F_clamp ≈ 6860 N`, well above the
~3100 N needed to lock the front wheel on dry asphalt. That's the whole
reason ABS is interesting here — the actuator chain has authority to stall
the tire; the controller has to prevent it.

---

## 4. `BrakeTorqueComputation` — linear gain

**Reads:** `F_clamp` (fresh).
**Produces:** `T_b` [N·m].

No state, one line of algebra:

```
T_b = μ_pad · F_clamp · r_eff · n_pads
```

With `μ_pad = 0.4`, `r_eff = 0.14 m`, `n_pads = 2`, the gain is 0.112 N·m
per newton of clamping force. Steady-state `F_clamp ≈ 6860 N` ⇒
`T_b ≈ 770 N·m`.

---

## 5. `FrontWheelRotation` — the plant's core integrator

**Reads:** `F_f` (**lag-1**) and `T_b` (**lag-1**).
**Produces:** `omega_f` [rad/s].

This is the Σ node on the diagram — the summing junction that drives
`I_f · ω̇_f`:

```
I_f · dω_f/dt = +F_f · R_f − T_b
```

The `+F_f·R_f` sign is the one non-obvious detail in the whole plant. `F_f`
is the magnitude of the *retarding* tire force on the bike, but the reaction
at the contact patch torques the wheel in the direction of forward rolling
— so it has a `+` sign in the wheel equation, not a `−`. This is the
physically correct convention, and it's what lets an ABS `DUMP` *unlock*
the wheel: when `T_b → 0`, the saturated `F_f · R_f` remains and re-spins
`ω_f` back toward `v/R_f`. See
[`ASSUMPTIONS.md` → FrontWheelRotation §3](../ASSUMPTIONS.md#frontwheelrotation).

A one-sided lock guard in `derivatives` prevents the wheel from spinning
negative under pure brake torque: if `ω_f ≤ 0` and net torque is still
negative, `dω_f/dt` is clamped to zero. A real disc brake cannot reverse
wheel direction.

Both inputs are **lag-1** because their producers (`BrushTireModel` at #9,
`BrakeTorqueComputation` at #10 in the evaluation order) run after this
block. At `dt = 10⁻⁴ s` the phase error is ≪ 1 % of the hydraulic time
constant, so the lag is effectively invisible.

---

## 6. Closing the tire-force loop: `SlipRatioTrue` → `BrushTireModel`

This is where the interesting physics happens. Take the just-integrated
`v` and `omega_f`:

**`SlipRatioTrue`** → `lambda_f_true = (v − ω_f·R_f) / max(v, v_ε)`. The
`v_ε` floor (0.1 m/s) prevents division by zero at standstill. Under steady
rolling, `ω_f·R_f = v` and `λ = 0`. As brake torque bites, `ω_f` drops below
`v/R_f` and `λ` grows toward 1. A fully locked wheel has `λ = 1`.

**`BrushTireModel`** is closed-form Dugoff:

```
λ_s  = λ / (1 − λ)
σ    = μ_peak · N_f · (1 − λ) / (2 · C_x · λ)
f(σ) = 1                if σ ≥ 1
     = σ · (2 − σ)      if σ < 1
F_f  = C_x · λ_s · f(σ)
```

Two regimes: at small `λ` the tire is in the linear zone (`F_f ≈ C_x · λ`),
at large `λ` the Dugoff saturation `f(σ)` kicks in and `F_f → μ_peak · N_f`.
The critical slip where these meet is `λ_crit = μ_peak · N_f / C_x ≈ 0.03–0.04`
— a narrow operating window. Brake torque that nudges `λ` past `λ_crit` drops
the tire into saturation, the force goes flat, and any further torque
imbalance snowballs into lockup.

That narrow window is why ABS matters: the controller has to hold `λ` just
below `λ_crit` for maximum `F_f`, without overshooting. The MVP loses this
race on the first cycle (see §11).

---

## 7. `VehicleTranslation` — the outer integrator

**Reads:** `F_f` (**lag-1** from the tire model).
**Produces:** `v` [m/s] (state), `a_x` [m/s²] (algebraic, `−F_f / m`).

The vehicle equation is trivial:

```
m · dv/dt = −F_f
```

Drag and rolling resistance are off by default (single-line toggles in
`configs/default.toml`). The reason: for a 30 km/h panic stop, drag is a
few tens of newtons and tire force is several thousand — leaving them off
keeps the comparison oracle clean.

`a_x` is emitted algebraically from the same `F_f` that drove the `v`
integration, so it inherits the lag-1. This matters next.

---

## 8. `NormalLoad` — weight transfer

**Reads:** `a_x` (fresh — vehicle ran before this block).
**Produces:** `N_f` [N], clamped to `[0, m·g]`.

Quasi-static moment balance about the rear contact:

```
N_f = (m · g · a − m · a_x · h) / L
```

Under hard braking `a_x < 0`, so `−m · a_x · h > 0` and `N_f` grows above
its static value. Clamping at `m·g` marks the stoppie threshold — the
rear wheel would leave the ground beyond that. The clamp is a *diagnostic*
bound, not a physical model of the stoppie; we flag rather than simulate
the limit.

The freshly-computed `N_f` feeds straight into the tire model's `σ`
computation, raising both `λ_crit` and the saturation limit. Weight
transfer is a gift on a cargo bike: more load on the front means more
braking force available before lockup — if the controller can avoid
lockup.

---

## 9. `RearWheelKinematics` — the kinematic reference

**Reads:** `v`.
**Produces:** `omega_r = v / R_r`.

Algebraic, no state. Because there is no rear brake and the rear tire is
assumed to roll without slip (`λ_r = 0`), `ω_r` is a perfect measurement
of `v / R_r`. This is the reason the ABS can compute a clean `v̂` without
needing inertial sensing — it rides on the front-only-braking assumption.

Relax that assumption (e.g. add a rear brake or a steep grade) and the
whole estimator chain needs a new reference. That's out of scope for this
study.

---

## 10. `HallSensor` — sampling `ω_f` to edges

**Reads:** `omega_f`.
**Produces:** `theta_f`, `hall_edge_count`, `hall_last_edge_dt`,
`hall_last_edge_t`.

A Hall sensor doesn't report continuous speed — it reports magnet crossings.
With `N_hall = 20` magnets per revolution, an edge fires every `2π / 20 =
0.314 rad (18°)` of wheel rotation. The block integrates `max(0, ω_f)` to
get `theta_f`, and in `commit()` it walks any boundary crossings since
the last step, emitting an edge for each.

Edge timestamps are **back-interpolated**: rather than snap the edge to the
current simulator time, the block computes `t_edge = t − (θ − θ_boundary) / ω`,
accurate to `O(dt²)` for a smooth `ω_f`. That tenth-of-a-millisecond
correction compounds: without it, the estimator's `Δt_edge` would be
discretised onto the simulator grid and the MA filter would see stair-step
noise floor.

Jitter and missed-edge probabilities are both zero in Phase C. The hooks
exist for Phase B noise-robustness sweeps.

**Why this block owns continuous state.** `theta_f` is integrated by RK4.
But `edge_count`, `last_edge_t`, and `last_edge_dt` are discrete — they're
advanced inside `commit()` exactly once per simulator step, *never* inside
an RK4 substep. Advancing them four times per step would triple-count
edges. This separation is the single subtlest design choice in the block;
the `step/commit` split in the base `Block` class exists specifically to
make it safe.

---

## 11. `WheelSpeedEstimator` — reconstructing `ω̂_f`

**Reads:** the three `hall_*` outputs.
**Produces:** `omega_f_hat`, `omega_f_hat_dot`, `omega_f_raw`.

Four processing stages on each new edge:

1. **Raw edge rate:** `ω_raw = (2π/N) / Δt_edge`.
2. **First-order LPF** at `f_c = 50 Hz` — knocks down edge-interval noise.
3. **Moving-average** over 4 LPF samples — further smooths the ramp-up
   artefacts when edges first arrive.
4. **Central-difference derivative** over a 3-sample history of MA outputs
   — delivers `ω̇̂_f` with half the lag of a forward-difference estimator.

Between edges, the estimator emits **ZOH** from the last commit. But ZOH
alone breaks on a locked wheel: once `ω_f → 0`, no more edges fire, and
`ω̂_f` would freeze at its pre-lock value — the ABS would never see the
lockup. The block fixes this with an **edge-timeout bound**: whenever the
time since the last edge exceeds `1.5 × last_dt`, it pulls `ω̂_f` down
toward the upper bound `angle_per_edge / dt_since`. The 1.5× slack
prevents false positives during normal inter-edge gaps.

**This is the MVP's primary weakness.** The full chain (18° Hall → LPF →
4-sample MA → central-diff) adds 15–25 ms of lag between a physical
lockup and the estimator's detection of it. On the first panic-stop
cycle, the wheel briefly reaches `ω_f ≈ 0` (peak `|λ_true| ≈ 1`) before
the FSM fires `DUMP`. That's why this implementation does *not* meet
PLAN's peak-slip < 0.30 oracle — see
[`ASSUMPTIONS.md` → ABSController §5](../ASSUMPTIONS.md) for the
Phase D sensor upgrade that would fix it. What the MVP *does* achieve is
the *locked-time* oracle: fraction of time spent at ω ≈ 0 drops from
~72 % (Phase B human-only) to ~12 % (Phase C ABS). The lockup still
happens, but the wheel doesn't stay there.

---

## 12. `SlipRatioEstimated` — closing the feedback

**Reads:** `omega_r`, `omega_f_hat`.
**Produces:** `lambda_f_hat`, `v_hat`.

```
v̂     = ω_r · R_r
λ̂_f   = (v̂ − ω̂_f · R_f) / max(v̂, v_ε)
```

Because `ω_r` is an exact (noise-free) kinematic reference and `ω̂_f`
tracks `ω_f` to within the MA window's settling, `λ̂_f` agrees with
`λ_f,true` to within sub-1 % once the estimator has had a few hundred
milliseconds to converge.

---

## 13. Back to the `ABSController` — the feedback loop closes

The three estimator outputs — `lambda_f_hat`, `omega_f_hat_dot`, `v_hat` —
land in the persistent signals dict at the end of this step. On the *next*
simulator step, the ABS FSM reads them (as lag-1, because it runs at
evaluation order #5 while the producers are at #12 and #13) and checks its
transition conditions.

Once the first lockup trigger fires:

1. `APPLY → DUMP`: `V_pwm = 0`.
2. Motor coasts under `b_m · ω_m` viscous drag; hydraulic lag bleeds
   `F_clamp` down through `τ_hyd = 30 ms`.
3. As `T_b` falls below `F_f · R_f`, the tire force re-spins the wheel —
   `ω_f` accelerates back toward `v / R_f`.
4. Estimator sees `ω̇̂_f > 0` and `λ̂_f` falling back below `λ_off`:
   `DUMP → HOLD`.
5. `HOLD` dwells for 50 ms, letting the wheel fully recover, then
   `HOLD → REAPPLY`.
6. Cycle repeats.

Below `v_cutoff = 1.4 m/s`, the FSM latches into `BYPASS` and lets the
wheel lock for the final few metres of the stop. Modulating at walking
pace gains nothing and the sensor chain is out of resolution anyway.

---

## 14. Putting it all together — the panic-stop narrative

Between `t = 0` and `t ≈ 50 ms`, the rider's ramp is building `F_clamp`
through the hydraulic lag; `T_b` hasn't reached the friction limit yet;
`λ` grows slowly; `F_f` rides up the linear branch of the Dugoff curve
and `v` starts falling.

Around `t ≈ 100 ms`, `T_b` overshoots `F_f · R_f`, `ω_f` decelerates hard,
and `λ` rockets past `λ_crit`. The tire saturates (`f(σ)` collapses), the
front wheel approaches lockup. The estimator is chasing a 15–25 ms
shadow; by the time `λ̂_f > 0.20` and `ω̇̂_f < −100 rad/s²` both hold,
`λ_true` is already near 1.

FSM fires `DUMP`. Hydraulic pressure bleeds. Within 30–60 ms, `ω_f` has
re-synced with `v/R_f`, `λ̂_f` drops below 0.05, `ω̇̂_f` goes positive.
`DUMP → HOLD → REAPPLY`, and the rider's command flows back to the motor.

The cycle repeats — typically four or five times — until `v̂ < 1.4 m/s`,
at which point `BYPASS` takes over and the bike locks out the last metre.

Total stopping distance, peak `|a_x|`, peak `|λ|`, and locked-time fraction
are the four metrics `scripts/run_comparison.py` emits alongside the
side-by-side plots for human / cadence / ABS.

---

## Quick reference: the six lag-1 edges

Every dashed-red arrow on the diagram is one of these. They're safe because
`dt = 10⁻⁴ s` is three orders of magnitude below `τ_hyd = 30 ms`.

| From | To | Signal | Why lag-1 |
|---|---|---|---|
| `BrushTireModel` | `VehicleTranslation` | `F_f` | tire runs at #9, vehicle at #1 |
| `BrushTireModel` | `FrontWheelRotation` | `F_f` | tire runs at #9, wheel at #2 |
| `BrakeTorqueComputation` | `FrontWheelRotation` | `T_b` | brake runs at #10, wheel at #2 |
| `WheelSpeedEstimator` | `ABSController` | `omega_f_hat_dot` | estimator at #12, ABS at #5 |
| `SlipRatioEstimated` | `ABSController` | `lambda_f_hat` | slip_est at #13, ABS at #5 |
| `SlipRatioEstimated` | `ABSController` | `v_hat` | slip_est at #13, ABS at #5 |

The first three break the plant's **F_f / T_b ↔ ω_f ↔ v ↔ ...** algebraic
loop. The last three break the sensor→controller loop without forcing the
estimator to run before the FSM (which would require re-ordering the
simulator's evaluation pass).

---

## What this walkthrough deliberately glosses over

- **Cadence scenario:** identical flow, but `ABSController` is replaced by
  `CadenceController` — a time-driven square-wave chop of `V_pwm_cmd` at
  2 Hz × 50 % duty. No feedback at all; the estimator still runs but no
  one reads it.
- **Phase A / Phase B:** same topology, fewer blocks. Phase A uses
  `PrescribedClamp` directly instead of the motor chain; Phase B adds the
  actuator and sensor chain but gates with `Passthrough` (no ABS / no
  cadence).
- **Noise sweeps:** `HallSensor`'s `jitter_frac` and `missed_prob` are the
  hooks for Phase B robustness sweeps. Phase C runs both at zero.
- **Phase D sensor upgrade:** higher-`N` Hall or a direct wheel-speed
  encoder would cut estimator lag to ≲ 5 ms and meet the peak-slip < 0.30
  oracle. Out of scope for the MVP.

For the exact equations, parameter values, and all sign conventions, the
source of truth is `PLAN.md` plus the per-block sections of
[`ASSUMPTIONS.md`](../ASSUMPTIONS.md).
