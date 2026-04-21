# Control strategies — command laws and modelling-choice justifications

Each strategy is a rule for the motor-actuator input voltage `V_pwm(t)`;
brake torque is then the output of a fixed downstream chain

```
V_pwm  →  motor current i  →  F_clamp  →  T_b = 2·μ_pad·r_eff·F_clamp
```

with a first-order hydraulic lag `τ_hyd = 30 ms`. So the three
controllers differ *only* in how they command `V_pwm`.

## 1. Human baseline — `src/ebike_abs/control/human.py`

**Command law.**

```
V_pwm(t) = V_hold · clip(t / t_rise, 0, 1)
```

A linear ramp from 0 to `V_hold` over `t_rise`, then hold. Purely
open-loop: no function of `λ`, `ω_f`, or `v`. A `Passthrough` block
forwards `V_pwm_cmd → V_pwm` unchanged.

**Why this shape.** Models a panicked rider squeezing the
motor-actuated lever once and holding. The *point* of the baseline is
that it does not react to lockup — that is what makes it a reference
against which ABS and cadence can be measured.

**Parameter choice (`V_hold = 6 V`, `t_rise = 0.15 s`).**

- `V_hold = 6 V` is half of `V_pwm_max = 12 V`. Per the actuator tuning
  (`ASSUMPTIONS.md → MotorActuator §2`), a steady 6 V yields
  `F_clamp^ss ≈ 6860 N`, comfortably above the `≈ 3100 N` front-wheel
  lock threshold `μ_peak·m·g·R_f / (μ_pad·r_eff·n_pads)`. That is
  deliberate: we want the actuator saturated so the rider's input is
  not the bottleneck — the tire is.
- `t_rise = 0.15 s` matches the Phase A prescribed-clamp ramp so Phase A
  and Phase B runs are apples-to-apples; it is also a plausible
  reach-and-squeeze time for a panic stop.

## 2. Cadence baseline — `src/ebike_abs/control/cadence.py`

**Command law.** A square-wave chop of the human command:

```
V_pwm(t) = V_pwm_cmd(t) · 1[ (t mod T) / T < duty ],   T = 1/freq
```

During the *off* half the motor gets 0 V, coasts under its viscous
load `b_m`, and the hydraulic line bleeds down through `τ_hyd` — so
the release is not instantaneous. Still open-loop (no `λ` or `ω_f`
feedback).

**Why this shape.** Models a rider pumping the lever. Keeping it
purely time-driven preserves the apples-to-apples property: ABS is
then the *only* controller that reads the estimator, which isolates
the effect of feedback from the effect of chopping.

**Parameter choice (`freq = 2 Hz`, `duty = 0.5`).**

- `2 Hz` is the fastest sustained pumping rate a human can realistically
  hold on a brake lever for a multi-second stop (fatigue and
  reaction-time limited).
- `50 % duty` gives a clean average command of `0.5 · V_pwm_cmd`; on
  dry asphalt this stretches the stop ~1.5–2× vs human-only (verified
  in `tests/test_cadence.py`). Both values come from `PLAN.md`'s
  parameter table.

## 3. ABS — `src/ebike_abs/control/abs_fsm.py`

**Command law.** A four-state FSM (plus a `BYPASS`) that *gates* the
human command:

| Mode      | `V_pwm` output            |
| --------- | ------------------------- |
| `APPLY`   | `V_pwm_cmd` (passthrough) |
| `DUMP`    | `V_dump` (default 0)      |
| `HOLD`    | `V_dump` (default 0)      |
| `REAPPLY` | `V_pwm_cmd` (passthrough) |
| `BYPASS`  | `V_pwm_cmd` (passthrough) |

Transitions are a function of the estimator outputs `λ̂_f`, `ω̂̇_f`,
`v̂`:

- `APPLY / REAPPLY → DUMP` when `λ̂_f > λ_on` **and** `ω̂̇_f < ω̇_trig`
- `DUMP → HOLD` when `λ̂_f < λ_off` **and** `ω̂̇_f > 0`
- `HOLD → REAPPLY` after `dump_dwell` elapses
- any state → `BYPASS` when `v̂ < v_cutoff` (latched)

Brake torque therefore still comes from the same motor/hydraulic
chain — the ABS does not "release" pressure directly, it pulls
`V_pwm` to 0 and lets `τ_hyd` + `b_m` bleed it off.

**Parameter choices.**

- `λ_on = 0.20, λ_off = 0.05`. The Dugoff tire peaks near
  `λ ≈ 0.10–0.15` for our (`μ_peak = 0.9`, `C_x = 30 000 N/rad`)
  combination, so `λ_on = 0.20` is just past the peak — past enough
  that noise in `λ̂_f` will not false-trigger, not so far that we lose
  a lot of μ before dumping. `λ_off = 0.05` is safely inside the
  linear regime so we only exit DUMP after the wheel has really
  recovered; the gap (`0.20 → 0.05`) is the hysteresis band that
  prevents chatter.
- `ω̇_trig = −100 rad/s²`. A purely slip-based trigger fires on any
  slow drift past `λ_on`, including from estimator lag on a normal
  stop. Requiring a genuine wheel-deceleration event rejects those.
- `v_cutoff = 1.4 m/s (≈ 5 km/h)`. Below walking pace the slip
  estimate is dominated by the `v_epsilon = 0.1 m/s` denominator
  clamp, so `λ̂_f` is unreliable; and modulating at that speed only
  prevents the bike stopping. Latching into BYPASS (rather than
  letting the FSM re-enter APPLY) is a safety guard on that
  reasoning.
- `dump_dwell = 50 ms`. Long enough that the hydraulic line
  re-pressurises smoothly through `τ_hyd = 30 ms` (so REAPPLY is not
  a ringing step), short enough to not leave meaningful μ on the
  table. Roughly 1.5–2 × `τ_hyd` is the standard rule of thumb.
- `V_dump = 0` (option to set negative). We collapse pressure by
  cutting drive and letting the motor back-EMF + `b_m` do the work.
  Driving negative would be faster but requires an H-bridge; leaving
  it as a knob documents the design choice.

**Known limitation baked into the ABS modelling choices.** The
20-magnet Hall + 4-sample MA + LPF estimator chain has ~20 ms
detection lag, so the first lockup cycle still spikes to `|λ| ≈ 1`
before DUMP fires. PLAN's `peak-|λ| < 0.30` oracle is *not* met — the
primary oracle is instead the locked-time fraction (72 % → 12 %).
Fixing the peak-λ bound is a sensor upgrade (higher-res encoder,
sliding-mode estimator), explicitly deferred to Phase D. See
`ASSUMPTIONS.md → ABSController §5`.
