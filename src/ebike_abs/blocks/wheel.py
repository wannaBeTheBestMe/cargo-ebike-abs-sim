"""Front wheel rotational dynamics and rear wheel kinematic reference.

Sign convention (PLAN.md §2):
* ``omega_f`` > 0 : wheel spinning in the forward-rolling direction
  (``ω R = v`` under pure rolling).
* ``F_f`` ≥ 0 : magnitude of the retarding tire force on the bike. The
  reaction on the wheel at the contact patch produces a torque in the
  *same* direction as the forward-rolling ω — the ground friction is
  what re-spins a locked wheel once brake torque is released.
* ``T_b`` ≥ 0 : magnitude of the brake torque; opposes ω.

Wheel equation: ``I_f * dω_f/dt = +F_f * R_f - T_b``. Under steady
braking ``T_b > F_f R_f``, so ω_f decelerates toward lock. In an ABS
DUMP with ``T_b → 0``, the saturated tire force spins the wheel back up
toward the free-rolling condition.
"""

from __future__ import annotations

import numpy as np

from ..block import Block


class FrontWheelRotation(Block):
    name = "wheel_f"
    inputs = ["F_f", "T_b"]
    outputs = ["omega_f"]

    def __init__(self, I_f: float, R_f: float, omega0: float):
        self.I_f = float(I_f)
        self.R_f = float(R_f)
        self.state = np.array([float(omega0)], dtype=float)

    def output(self, t, x, u):
        return {"omega_f": float(x[0])}

    def derivatives(self, t, x, u):
        omega_f = float(x[0])
        F_f = u.get("F_f", 0.0)
        T_b = u.get("T_b", 0.0)
        net_torque = F_f * self.R_f - T_b
        # One-sided wheel lock: if already at/below zero and net torque would
        # drive ω_f further negative, hold it at zero. A real disc brake
        # cannot reverse wheel rotation; the tire friction becomes static.
        if omega_f <= 0.0 and net_torque < 0.0:
            return np.array([0.0])
        return np.array([net_torque / self.I_f])


class RearWheelKinematics(Block):
    name = "wheel_r"
    inputs = ["v"]
    outputs = ["omega_r"]

    def __init__(self, R_r: float):
        self.R_r = float(R_r)

    def step(self, t, u):
        v = u.get("v", 0.0)
        return {"omega_r": v / self.R_r}
