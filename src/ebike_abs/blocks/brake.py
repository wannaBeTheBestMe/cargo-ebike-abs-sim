"""Brake torque from clamping force, plus a prescribed clamp source.

``BrakeTorqueComputation`` is the permanent block for the study:

    T_b = μ_pad · F_clamp · r_eff · n_pads

``PrescribedClamp`` is a Phase-A stand-in for the full motor/hydraulic
actuator chain (Phase B will replace it with ``MotorActuator``). It
linearly ramps the clamping force from 0 to ``F_peak`` over ``t_rise``
seconds, then holds at ``F_peak``.
"""

from __future__ import annotations

from ..block import Block


class PrescribedClamp(Block):
    name = "clamp"
    inputs: list[str] = []
    outputs = ["F_clamp"]

    def __init__(self, F_peak: float, t_rise: float):
        self.F_peak = float(F_peak)
        self.t_rise = float(t_rise)

    def step(self, t, u):
        if t <= 0.0:
            return {"F_clamp": 0.0}
        if t >= self.t_rise:
            return {"F_clamp": self.F_peak}
        return {"F_clamp": self.F_peak * t / self.t_rise}


class BrakeTorqueComputation(Block):
    name = "brake"
    inputs = ["F_clamp"]
    outputs = ["T_b"]

    def __init__(self, mu_pad: float, r_eff: float, n_pads: int):
        self.mu_pad = float(mu_pad)
        self.r_eff = float(r_eff)
        self.n_pads = int(n_pads)

    def step(self, t, u):
        F_clamp = u.get("F_clamp", 0.0)
        return {"T_b": self.mu_pad * F_clamp * self.r_eff * self.n_pads}
