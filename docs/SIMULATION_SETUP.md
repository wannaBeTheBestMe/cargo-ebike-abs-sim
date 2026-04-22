# Simulation setup

All runs exercise the single-track planar plant defined in
`configs/default.toml` on a 30 km/h dry-asphalt panic stop. Parameters are
literature-grounded where a citation was available and flagged **assumed**
where they are chosen for the vehicle class without a specific source; every
row moves with a matching entry in `ASSUMPTIONS.md`.

## Parameters

| block           | parameters                            | value(s)                                           | source                                                                           |
|-----------------|---------------------------------------|----------------------------------------------------|----------------------------------------------------------------------------------|
| Vehicle         | m, L, a (rear→CG), h                  | 120 kg, 1.2 m, 0.7 m, 1.0 m                        | **assumed** — loaded cargo e-bike (`ASSUMPTIONS.md` §Global)                     |
| Front wheel     | R_f, I_f                              | 0.33 m, 0.12 kg·m²                                 | 26″ spec; I_f via thin-ring m_wh ≈ 2 kg, ±30 % (§FrontWheelRotation)             |
| Dugoff tire     | μ_peak, C_x                           | 0.9, 30 000 N/rad                                  | dry-asphalt / bicycle-tire literature (§DugoffTireModel)                          |
| Disc brake      | μ_pad, r_eff, n_pads                  | 0.4, 0.14 m, 2                                     | sintered pad on 160 mm rotor (§BrakeTorqueComputation)                           |
| Motor           | R_m, L_m, K_e = K_t, J_m, b_m         | 0.1 Ω, 1 mH, 0.05 (SI), 10⁻⁴ kg·m², 10⁻² N·m·s/rad | **tuned** so half-scale V_pwm saturates F_clamp ≫ 3.1 kN lock (§MotorActuator 2) |
| Hydraulic       | r_lever, A_caliper / A_master, τ_hyd  | 10⁻³ m, 4×, 30 ms                                  | 20–50 ms typical caliper line (§MotorActuator 1)                                 |
| Hall sensor     | N_hall                                | 20 / rev (18° resolution)                          | **assumed** hub ring (§HallSensor)                                               |
| Speed estimator | LPF f_c, MA window                    | 50 Hz, 4 samples                                   | absorbs 18° quantisation (§WheelSpeedEstimator)                                  |
| Scenario        | v₀, t_end                             | 8.333 m/s (30 km/h), 3 s                           | urban panic-stop envelope (§Scenario defaults)                                   |
| Solver          | scheme, dt                            | fixed-step RK4, 10⁻⁴ s                             | §Global 4 — see *Solver choice* below                                            |

## Initial conditions

The bike is cruising at the moment the rider grabs the lever:

- v(0) = v₀ = 8.333 m/s (30 km/h)
- ω_f(0) = v₀ / R_f (rolling, no pre-slip)
- ω_r(0) = v₀ / R_r
- zero motor current, zero clamp force, zero brake torque

## Solver choice

Fixed-step RK4 at dt = 10⁻⁴ s is chosen because three features of the plant
make variable-step solvers awkward:

1. **Discrete and step-synchronous subsystems.** The ABS FSM is a discrete
   state machine; the Hall emulator emits edges on step boundaries; the
   wheel-speed estimator runs a moving average over those edges. All three
   break adaptive error estimates.
2. **Lag-1 weight-transfer loop.** N_f is closed with a_x read from the
   previous step (`ASSUMPTIONS.md` §Global 5). A small, uniform step keeps
   the phase error negligible against the 30 ms hydraulic lag.
3. **Millisecond-scale lockup transient.** The step has to resolve the
   lockup dynamics regardless of the stop's overall 1–2 s duration.

A full 3 s run is 30 000 RK4 steps — cheap enough that
`scripts/run_comparison.py` sweeps all three controllers (human, cadence,
ABS) in a few seconds.
