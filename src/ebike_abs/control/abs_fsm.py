"""ABS controller — four-state FSM gating the human V_pwm command.

States (PLAN.md §12(b)):

* ``APPLY``   — pass the human command through.
* ``DUMP``    — force ``V_pwm = 0`` (or ``-V_pwm_max``) to drive the motor
  back-EMF, collapsing hydraulic pressure through ``τ_hyd``.
* ``HOLD``    — hold ``V_pwm = 0`` for a short dwell while the wheel
  recovers. Same actuator output as DUMP, but ignores new lockup
  triggers until the dwell timer expires.
* ``REAPPLY`` — re-engage toward the human command (simple snap-back
  here, matching PLAN.md's "ramp pressure toward human command"). New
  lockup triggers are honoured immediately.

Below ``v_cutoff`` the ABS bypasses and the bike is allowed to lock —
at walking pace the benefit of modulation is nil and the sensor /
estimator chain loses resolution.

Inputs: ``V_pwm_cmd`` (from the human), ``lambda_f_hat``,
``omega_f_hat_dot``, ``v_hat``. Output: gated ``V_pwm``.
Diagnostics: ``abs_mode`` (0=APPLY, 1=DUMP, 2=HOLD, 3=REAPPLY, 4=BYPASS).
"""

from __future__ import annotations

from ..block import Block

APPLY, DUMP, HOLD, REAPPLY, BYPASS = 0, 1, 2, 3, 4


class ABSController(Block):
    name = "abs"
    inputs = ["V_pwm_cmd", "lambda_f_hat", "omega_f_hat_dot", "v_hat"]
    outputs = ["V_pwm", "abs_mode"]

    def __init__(
        self,
        lambda_on: float,
        lambda_off: float,
        omega_dot_trig: float,
        v_cutoff: float,
        dump_dwell: float = 0.050,
        V_dump: float = 0.0,
    ):
        self.lambda_on = float(lambda_on)
        self.lambda_off = float(lambda_off)
        self.omega_dot_trig = float(omega_dot_trig)
        self.v_cutoff = float(v_cutoff)
        self.dump_dwell = float(dump_dwell)
        self.V_dump = float(V_dump)
        # Discrete state.
        self.mode: int = APPLY
        self.mode_entered_t: float = 0.0
        # Cache for the between-commit output: set by ``step``.
        self._last_V: float = 0.0

    def _gate(self, mode: int, V_cmd: float) -> float:
        if mode in (APPLY, REAPPLY, BYPASS):
            return V_cmd
        # DUMP / HOLD: force the dump voltage (default 0, can be set
        # negative to actively drive the motor reverse).
        return self.V_dump

    def step(self, t, u):
        V_cmd = u.get("V_pwm_cmd", 0.0)
        V = self._gate(self.mode, V_cmd)
        self._last_V = V
        return {"V_pwm": V, "abs_mode": float(self.mode)}

    def commit(self, t, signals):
        lam_hat = float(signals.get("lambda_f_hat", 0.0))
        omega_dot = float(signals.get("omega_f_hat_dot", 0.0))
        v_hat = float(signals.get("v_hat", 0.0))
        mode = self.mode

        # Low-speed bypass overrides everything. Once below v_cutoff the
        # ABS stays out of the way for the rest of the stop.
        if v_hat < self.v_cutoff:
            if mode != BYPASS:
                self.mode = BYPASS
                self.mode_entered_t = t
            return

        if mode in (APPLY, REAPPLY):
            if lam_hat > self.lambda_on and omega_dot < self.omega_dot_trig:
                self.mode = DUMP
                self.mode_entered_t = t
        elif mode == DUMP:
            # Wheel has recovered: slip back below threshold and ω̇_f > 0.
            if lam_hat < self.lambda_off and omega_dot > 0.0:
                self.mode = HOLD
                self.mode_entered_t = t
        elif mode == HOLD and (t - self.mode_entered_t) >= self.dump_dwell:
            self.mode = REAPPLY
            self.mode_entered_t = t
