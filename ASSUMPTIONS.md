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
3. **Sign convention (matches PLAN.md §2).** $F_f \geq 0$ is the magnitude
   of the retarding tire force on the bike; the reaction at the contact
   patch applies a torque $+F_f R_f$ on the wheel — the same direction
   as forward-rolling $\omega_f$. This is the physically correct sign:
   when the tire slides forward (ω·R < v), ground friction spins the
   wheel *up* toward $v/R$. Without it, an ABS DUMP that releases
   $T_b$ could not unlock the wheel.
4. **One-sided lock guard.** Inside ``derivatives`` we clamp
   ``dω_f/dt`` to zero whenever ``ω_f ≤ 0`` and the net torque is still
   negative, so the wheel doesn't spin backward under a pure brake
   torque. Once the net torque goes positive (e.g. DUMP drops $T_b$ to
   zero and the tire force re-spins the wheel) the guard releases and
   ω_f accelerates back toward rolling.

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

## HallSensor

1. **Continuous wheel angle.** Edges are detected by integrating $\omega_f$
   into a continuous state $\theta_f$ and watching for crossings of
   $2\pi/N$. The integrator mirrors the one-sided lock clamp
   (`derivatives` clips to $\max(0, \omega_f)$) so that the small
   sub-zero $\omega_f$ excursions between RK4 substeps (Phase A known
   limitation) do not retrace $\theta$ back across a boundary and emit
   phantom edges.
2. **Edge timestamp by back-interpolation.** When $\theta$ crosses a
   boundary mid-step, the edge time is estimated as
   $t_{\text{edge}} = t - (\theta - \theta_{\text{boundary}})/\omega_f$.
   This is $O(dt^2)$ for smooth $\omega_f$ and well inside the Hall
   resolution at the step sizes we run.
3. **Jitter / missed edges off by Phase B default.** `jitter_frac = 0`
   and `missed_prob = 0` in `configs/default.toml`; both hooks exist for
   Phase B noise sweeps.
4. **Discrete state lives outside the RK4 substeps.** Edge count,
   `last_edge_t`, and `next_edge_angle` update only inside
   `Block.commit`, which the simulator invokes once per step with the
   final signal snapshot. Updating them inside `output` would cause RK4
   to triple-count edges.

## WheelSpeedEstimator

1. **Edge-interval LPF.** First-order low-pass run on the edge-rate
   samples with $\tau = 1/(2\pi f_c)$ and $\alpha = dt_{\text{edge}} /
   (\tau + dt_{\text{edge}})$ — so the effective cutoff stays at
   `lpf_fc` regardless of wheel speed. $f_c = 50$ Hz by default.
2. **Moving-average window 4.** Matches `configs/default.toml`; the
   window absorbs the discrete-rate quantisation noise from the 18°
   Hall resolution.
3. **Central-difference derivative.** $\dot{\hat\omega}_f$ is computed
   from a rolling 3-sample history of MA outputs, resolving review
   point 5 (no forward-Euler derivative). Before the history fills, the
   derivative is held at zero.
4. **ZOH between edges.** `step()` returns the cached MA / derivative
   outputs from the last edge; the estimator only advances inside
   `commit`.

## SlipRatioEstimated

1. $\hat v = \omega_r R_r$ is exact on this bike — rear has $\lambda_r =
   0$ by the front-only-braking assumption, so the rear kinematic
   reference is the cleanest available speed surrogate.
2. Same $v_\epsilon$ denominator clamp as `SlipRatioTrue`, so the
   estimator and the ground truth share the singularity treatment near
   $v \to 0$.
3. **Expected residual at zero physical slip.** With noise off,
   $|\hat\lambda_f - \lambda_f^{\text{true}}|$ is bounded by the
   estimator's LPF/MA lag. On a flat coast-down it settles to $<
   5\times10^{-3}$ within ~15 Hall edges (≈ 200 ms at the 30 km/h
   scenario).

## HumanBrakeController (Phase B baseline)

1. **Linear $V_{\text{pwm}}$ ramp.** The rider is modelled as a linear
   ramp from $0$ to ``V_hold`` over ``t_rise``, then hold. No feedback
   from $\lambda$, $\omega_f$, or $v$ — a baseline-shaped input exists
   precisely so that Phase C can compare ABS and cadence against a
   fixed, open-loop rider profile.
2. **Defaults ``V_hold = 6 V``, ``t_rise = 0.15 s``.** Half of
   ``V_pwm_max`` — by the actuator tuning this saturates
   $F_{\text{clamp}}$ to ≈ 6860 N, well above the 3100 N wheel-lock
   threshold. The ramp shape matches the Phase A prescribed-clamp
   source so the two runs can be compared apples-to-apples.
3. **Panic-stop distance within ≈ 1 % of Phase A.** With the ramp
   saturated the hydraulic lag dominates and the wheel still locks
   quickly; measured Phase B stopping distance ≈ 3.96 m vs Phase A
   ≈ 3.95 m and oracle ≈ 3.93 m.

## Scenario defaults

1. $v_0 = 8.33$ m/s (30 km/h) panic stop on dry asphalt.
2. Initial conditions: $\omega_f(0) = v_0 / R_f$, zero brake torque, zero
   motor current, zero clamp force.

## ABSController (Phase C)

1. **Five-state FSM.** Modes are ``APPLY`` (0), ``DUMP`` (1), ``HOLD``
   (2), ``REAPPLY`` (3), and ``BYPASS`` (4). ``APPLY/REAPPLY/BYPASS``
   pass ``V_pwm_cmd`` through to the motor; ``DUMP/HOLD`` clamp
   ``V_pwm = 0``. The motor then spins down through its own viscous
   load, which drops $F_{\text{clamp}}$ via the lead-screw + hydraulic
   lag — the DUMP is *not* an instantaneous brake release, it's gated
   by $\tau_{\text{hyd}}$.
2. **Trigger thresholds.** ``APPLY → DUMP`` when
   $\hat\lambda_f > \lambda_{\text{on}} = 0.20$ *and*
   $\dot{\hat\omega}_f < \dot\omega_{\text{trig}} = -100\,\text{rad/s}^2$.
   The ω-dot check rejects slow-drift slip from estimator lag.
3. **Dump dwell / hold.** Once in ``DUMP`` the FSM waits for
   $\hat\lambda_f < \lambda_{\text{off}} = 0.05$, then moves to
   ``HOLD`` for ``dump_dwell = 50 ms``, then ``REAPPLY``. The HOLD
   dwell gives the hydraulic line time to re-pressurise smoothly before
   we unclamp the motor command again.
4. **Low-speed bypass.** Below ``v_cutoff = 1.4 m/s`` (≈ 5 km/h) the
   FSM falls to ``BYPASS`` and stops modulating. Slip estimates are
   unreliable near standstill (``v_epsilon`` denominator clamp
   dominates) and modulating there would just prevent the bike
   stopping.
5. **MVP estimator-lag limitation.** With a 20-magnet Hall + 4-sample
   MA + LPF chain, there is ≈ 15–25 ms of detection lag between the
   true wheel-lock event and the FSM's first DUMP. On a $v_0 = 8.33$
   m/s dry-asphalt panic stop this means the wheel does briefly reach
   $\omega_f \approx 0$ on the first cycle (peak
   $|\lambda_f^{\text{true}}| \approx 1$), so PLAN's $< 0.30$ peak-slip
   oracle is *not* met by the MVP. What the MVP *does* achieve is
   cycling behaviour: the locked-time fraction (above ``v_cutoff``)
   drops from ≈ 72 % in Phase B to ≈ 12 % in Phase C, documented in
   ``tests/test_abs.py``. A higher-resolution encoder or a
   sliding-mode estimator would be needed to hit the $< 0.30$ bound;
   that is explicitly deferred to a Phase D iteration.
6. **Dry-asphalt stopping-distance finding.** On dry pavement the
   MVP ABS does *not* shorten the stop vs the locked-wheel slide —
   because $\mu_{\text{sliding}}$ for a tire on dry asphalt is close to
   $\mu_{\text{peak}}$ and every sub-peak moment spent in DUMP/HOLD is
   distance lost. This is the expected outcome per the study's
   motivating question (ABS is a low-μ story), and is asserted only as
   "same order of magnitude" in ``tests/test_abs.py`` rather than
   "shorter".

## CadenceController (Phase C)

1. **Square-wave chop on ``V_pwm_cmd``.** The baseline for a rider
   pumping the brake lever is a 2 Hz, 50 %-duty on/off chop of the
   human command. During the *off* half the motor sees ``V_pwm = 0``
   and the hydraulic line bleeds down through $\tau_{\text{hyd}}$ —
   the DUMP is therefore not instantaneous, same as in
   ``ABSController``.
2. **No closed-loop reaction.** The cadence controller is purely
   time-driven; it does not read $\hat\lambda_f$ or $\hat\omega_f$.
   Matching the human baseline's open-loop shape keeps the comparison
   apples-to-apples (ABS is the only block reading the estimator).
3. **Default 2 Hz, 50 % duty** per PLAN.md parameter table. The
   half-duty means the *average* V_pwm is half the rider's command,
   so on dry asphalt cadence stretches the stop roughly 1.5–2× vs
   Phase B (verified in ``tests/test_cadence.py``).

## Controller chain (Phase B/C)

1. **``V_pwm_cmd`` → ``V_pwm`` indirection.** The rider models
   (``HumanBrakeController``, ``CadenceController``) emit
   ``V_pwm_cmd``; ``Passthrough`` or ``ABSController`` then decides
   whether to pass it through or dump it. Scenario builders swap the
   middle block to compare strategies without touching the rider or
   plant chain.

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

## Phase B end-of-phase checkpoint

1. **Actuator chain closes the loop.** Replacing the prescribed clamp
   with the motor + hydraulic chain driven by a 6 V human ramp
   reproduces Phase A's stopping distance to within ≈ 1 % (≈ 3.96 m vs
   ≈ 3.95 m). The ≈ 30 ms hydraulic lag is visible in the
   $F_{\text{clamp}}(t)$ trace but the tire saturates well before it
   matters for distance.
2. **Estimator lag is bounded.** With noise off, the MA-filtered
   $\hat\omega_f$ settles to within 0.5 % of truth within ≈ 15 Hall
   edges (~200 ms at $v_0 = 30$ km/h) and the estimated slip
   $\hat\lambda_f$ tracks $\lambda_f^{\text{true}}$ to within
   $5\times10^{-3}$ at steady coast-down. This is the tightest the
   MVP's 20-magnet + MA-4 chain can run.
3. **Edge-timeout bound for locked-wheel detection.** Without the
   bound, a locked wheel stops emitting edges and the estimator gets
   stuck at its last fresh reading — the ABS FSM would never fire.
   With the bound (1.5× last-known edge interval slack), the
   estimator tracks $\omega_f \to 0$ without spurious firings at
   constant $\omega_f$.

## Phase C end-of-phase checkpoint

1. **Primary oracle met.** The locked-time fraction above ``v_cutoff``
   drops from ≈ 72 % in the Phase B human-only run to ≈ 12 % in the
   Phase C ABS run, verified in
   ``tests/test_abs.py::test_abs_spends_less_time_locked_than_phase_b``.
   The FSM visits ``DUMP`` and ``APPLY``/``REAPPLY`` states on every
   hard stop and no ``ω_f ≈ 0`` interval lasts longer than
   $2\cdot\text{dump\_dwell} + 5\,\text{ms}$.
2. **PLAN's peak-|λ| < 0.30 oracle not met.** The MVP sensor chain has
   ≈ 20 ms of lag between the true lockup event and the FSM's first
   DUMP, so the first lockup cycle spikes to $|\lambda| \approx 1$
   before recovery. A higher-resolution encoder or a sliding-mode
   estimator would be needed to close the gap. This is a *property of
   the MVP sensor chain*, not the FSM, and is surfaced as the single
   clearest Phase D lever.
3. **Dry-asphalt headline.** On default dry-asphalt
   ($\mu_{\text{peak}} = 0.9$) the stopping-distance ranking is
   human-only (4.71 m) < ABS (7.22 m) < cadence (7.81 m). All three
   controllers stop in the same order of magnitude; the real
   separation is in wheel control (lock fraction 72 % / 12 % / 18 %)
   and time spent at dangerous slip (76 % / 22 % / 25 %). This is
   the expected null result that the motivating study calls out:
   ABS is a *low-μ* win, so the Phase C numbers here set up the
   hypothesis to be tested against a wet/ice scenario by sweeping
   ``tire.mu_peak`` on the same machinery.
