"""Four-stage motor + hydraulic brake actuator.

PLAN.md §10 — three cascaded first-order systems plus a kinematic
torque-to-force step:

    L_m · di/dt        = V_pwm − R_m · i − K_e · ω_m              (electrical)
    J_m · dω_m/dt      = K_t · i − b_m · ω_m                      (mechanical)
    F_piston           = K_t · i / r_lever                         (lead-screw)
    τ_hyd · dF_clamp/dt = (A_caliper / A_master) · F_piston − F_clamp   (hydraulic)

The mechanical equation lumps back-pressure reaction as a viscous load
``b_m · ω_m``; this keeps the motor model lumped first-order without
introducing a second algebraic loop. In steady state the motor spins at
``ω_m = K_t i / b_m`` and the hydraulic lag sets the dominant time
constant of the full chain.

Continuous states: ``[i_motor, omega_motor, F_clamp]``.
Input: ``V_pwm``. Output: ``F_clamp`` (plus ``i_motor``/``omega_motor`` as diagnostics).
"""

from __future__ import annotations

import numpy as np

from ..block import Block


class MotorActuator(Block):
    name = "actuator"
    inputs = ["V_pwm"]
    outputs = ["F_clamp", "i_motor", "omega_motor"]

    def __init__(
        self,
        L_m: float,
        R_m: float,
        K_e: float,
        K_t: float,
        J_m: float,
        b_m: float,
        r_lever: float,
        A_master: float,
        A_caliper: float,
        tau_hyd: float,
        V_pwm_max: float,
    ):
        self.L_m = float(L_m)
        self.R_m = float(R_m)
        self.K_e = float(K_e)
        self.K_t = float(K_t)
        self.J_m = float(J_m)
        self.b_m = float(b_m)
        self.r_lever = float(r_lever)
        self.A_ratio = float(A_caliper) / float(A_master)
        self.tau_hyd = float(tau_hyd)
        self.V_pwm_max = float(V_pwm_max)
        # [i_motor, omega_motor, F_clamp] all start at rest.
        self.state = np.zeros(3, dtype=float)

    def _saturate_V(self, V_pwm: float) -> float:
        return max(-self.V_pwm_max, min(self.V_pwm_max, V_pwm))

    def output(self, t, x, u):
        return {
            "i_motor": float(x[0]),
            "omega_motor": float(x[1]),
            # Clamp F_clamp ≥ 0: a disc brake cannot pull the pads away
            # from the rotor, so a sub-zero hydraulic lag excursion just
            # means "no force".
            "F_clamp": max(0.0, float(x[2])),
        }

    def derivatives(self, t, x, u):
        i_m, omega_m, F_clamp = float(x[0]), float(x[1]), float(x[2])
        V_pwm = self._saturate_V(u.get("V_pwm", 0.0))
        di = (V_pwm - self.R_m * i_m - self.K_e * omega_m) / self.L_m
        T_motor = self.K_t * i_m
        d_omega = (T_motor - self.b_m * omega_m) / self.J_m
        F_piston = T_motor / self.r_lever
        dF_clamp = (self.A_ratio * F_piston - F_clamp) / self.tau_hyd
        return np.array([di, d_omega, dF_clamp])

    def steady_state_F_clamp(self, V_pwm: float) -> float:
        """Analytical steady-state clamping force at a constant ``V_pwm``.

        Used by tests; not called during normal simulation.
        """
        V = self._saturate_V(V_pwm)
        # At steady state: V = R i + K_e ω, K_t i = b_m ω.
        # ⇒ i_ss = V / (R + K_e K_t / b_m)
        i_ss = V / (self.R_m + self.K_e * self.K_t / self.b_m)
        F_piston_ss = self.K_t * i_ss / self.r_lever
        return self.A_ratio * F_piston_ss
