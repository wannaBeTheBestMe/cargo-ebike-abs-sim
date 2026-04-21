"""Longitudinal slip ratio (plant-side, ground truth).

    λ_f = (v − ω_f · R_f) / max(v, v_ε)

For braking with ``v > ω_f · R_f`` the slip ratio is positive. The
``v_epsilon`` denominator clamp prevents a division-by-zero as the vehicle
stops; the sensing-side twin of this block (Phase B) will use the same
guard.
"""

from __future__ import annotations

from ..block import Block


class SlipRatioTrue(Block):
    name = "slip_true"
    inputs = ["v", "omega_f"]
    outputs = ["lambda_f_true"]

    def __init__(self, R_f: float, v_epsilon: float):
        self.R_f = float(R_f)
        self.v_epsilon = float(v_epsilon)

    def step(self, t, u):
        v = u.get("v", 0.0)
        omega_f = u.get("omega_f", 0.0)
        denom = max(v, self.v_epsilon)
        lam = (v - omega_f * self.R_f) / denom
        return {"lambda_f_true": lam}
