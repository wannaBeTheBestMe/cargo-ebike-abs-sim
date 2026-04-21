# Assumptions

Per-block log of modelling choices. Every new choice lands here in the same
commit that introduces it. Each entry: the assumption, a short rationale, and
(where relevant) a bound on the error it introduces.

## Global

1. **Single-track 1D, planar.** Lateral dynamics, steering, and roll are out of scope.
2. **Rigid cargo mass.** Rider + cargo are a rigid body lumped at the CG.
3. **Front-only braking.** Rear wheel is a free rolling speed reference
   ($\lambda_r = 0$, rear rolling resistance ignored per scenario defaults).
4. **Fixed-step RK4 at $dt = 10^{-4}$ s.** Hybrid dynamics (discrete ABS FSM,
   Hall emulation, moving-average filter) make variable-step solvers awkward;
   the chosen step is stable for the 20–50 ms hydraulic lag and the ms-scale
   lockup transient flagged in review.
5. **Weight-transfer algebraic loop is broken by lag-1.** $N_f$ at step $k$ is
   computed from $a_x$ emitted at step $k-1$. At $dt = 10^{-4}$ s the phase
   error is negligible relative to the ~30 ms hydraulic time constant.

## VehicleTranslation

1. Drag and rolling resistance are **off by default**; single-line toggles in
   `configs/default.toml` (`vehicle.drag_enabled`, `vehicle.rr_enabled`).

## FrontWheelRotation

1. Inertia $I_f = 0.12$ kg·m² from a thin-ring approximation ($m_{\text{wheel}}
   \approx 2$ kg at $R = 0.33$ m). Off by < 30 % for a real spoked wheel with
   hub mass; acceptable for a qualitative ABS-vs-cadence study.
2. State is $\omega_f$ directly (not a $\Delta\omega$ sum), per review.
3. **Sign convention.** $F_f \geq 0$ is the magnitude of the tire retarding
   force; its reaction at the contact patch applies a torque $-F_f R_f$ on
   the wheel. The wheel equation is therefore
   $I_f \dot\omega_f = -F_f R_f - T_b$, which is the consistent-sign form of
   PLAN.md §9 / §2 (the plan's "$+ F_f R_f$" uses the opposite convention
   and would accelerate the wheel during braking).

## RearWheelKinematics

1. $\omega_r = v / R_r$ algebraically — rear-wheel inertia reaction is
   ignored. Valid because there is no rear brake and the rear tire rolls
   without slip on our scenarios.

## NormalLoad

1. Quasi-static moment balance about the rear contact:
   $N_f = (m g a + m a_x h) / L$, with $a_x$ read from the previous step
   (see Global §5).
2. $N_f$ clamped to $[0, m g]$. A clamp at $m g$ means the bike is at the
   stoppie threshold; we flag (do not enforce) $N_r \geq 0$ in diagnostics.

## BrushTireModel

1. **Dugoff closed-form** for $F_f(\lambda, N_f)$. Avoids iterative solves and
   matches the piecewise linear/saturated shape to within a few percent.
2. Longitudinal stiffness $C_x = 30\,000$ N/rad (bicycle-tire literature, dry
   asphalt). Single value across operating range.
3. $\mu_{\text{peak}} = 0.9$ (dry asphalt) constant — no velocity or
   temperature dependence in Phase A.

## BrakeTorqueComputation

1. $T_b = \mu_{\text{pad}} F_{\text{clamp}} r_{\text{eff}} n_{\text{pads}}$ —
   constant pad friction, no fade.
2. $\mu_{\text{pad}} = 0.4$, $r_{\text{eff}} = 0.14$ m (160 mm rotor),
   $n_{\text{pads}} = 2$ — typical sintered-pad disc brake.

## Prescribed clamping force (Phase A only)

1. `F_clamp(t)` is a linear ramp 0 → `F_peak` over `t_rise`, then hold. This
   stands in for the full actuator chain during Phase A so that the plant can
   be exercised end-to-end before motor/hydraulic dynamics land.

## MotorActuator

1. **Lumped viscous load.** The mechanical equation is
   $J_m \dot\omega_m = K_t i - b_m \omega_m$, not
   $K_t i - T_{\text{load}}(F_{\text{clamp}})$. Replacing the hydraulic
   back-pressure reaction with a viscous term keeps the motor a clean
   lumped first-order system and avoids a second algebraic loop; the
   hydraulic $\tau_{\text{hyd}} = 30$ ms is designed to absorb any
   dynamics this approximation loses.
2. **Parameter tuning.** $R_m = 0.1\,\Omega$, $K_e = K_t = 0.05$,
   $J_m = 10^{-4}\,\text{kg·m}^2$, $b_m = 10^{-2}\,\text{N·m·s/rad}$,
   $r_{\text{lever}} = 0.001\,\text{m}$,
   $A_{\text{caliper}}/A_{\text{master}} = 4$. Chosen so that a half-scale
   $V_{\text{pwm}} = 6\,\text{V}$ step yields $F_{\text{clamp}}^{\text{ss}}
   \approx 6860\,\text{N}$ — comfortably above the $\sim 3100\,\text{N}$
   wheel-lock threshold on dry asphalt, matching a "panic stop" actuator
   that is always deeply in saturation on hard pulls.
3. **F_clamp output clipped at zero.** A disc brake cannot *pull* the pads
   off the rotor; the continuous state is left unclamped but its exposed
   signal is clipped ≥ 0. Reverse-voltage excursions therefore appear in
   the state trajectory but not in the brake torque.

## Scenario defaults

1. $v_0 = 8.33$ m/s (30 km/h) panic stop on dry asphalt.
2. Initial conditions: $\omega_f(0) = v_0 / R_f$, zero brake torque, zero
   motor current, zero clamp force.

## Phase A end-of-phase checkpoint

1. **Forced-lock oracle met.** Running the Phase A plant with $\omega_f \equiv
   0$ (see `tests/test_integration.py::ForcedLockedWheel`) gives a stopping
   distance within 1 % of $v_0^2 / (2 \mu_{\text{peak}} g)$ ≈ 3.93 m. The
   measured distance from the prescribed-clamp panic-stop is ≈ 3.95 m
   (0.5 % over the oracle), with the wheel locking once the brake ramp
   saturates the tire.
2. **Known minor limitation.** Between RK4 substeps $\omega_f$ can dip a few
   orders of magnitude below zero before the one-sided lock guard kicks in
   on the next evaluation, so the logged $\lambda_f^{\text{true}}$ signal
   briefly reports values slightly above 1 (peak ≈ 1.33 on the default
   scenario). The Dugoff block clamps $\lambda$ back into $[0, 1)$
   internally, so $F_f$ stays bounded at $\mu_{\text{peak}} N_f$. A
   post-RK4 state clip will be added in Phase B if the noise interferes
   with the ABS estimator.
