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


class SlipRatioEstimated(Block):
    """Controller-side slip ratio computed from sensed wheel speeds.

    ``v̂ = ω_r · R_r`` (exact on this bike — rear wheel has λ_r = 0).
    ``λ̂_f = (v̂ − ω̂_f · R_f) / max(v̂, v_ε)``.

    Consumes ``omega_r`` (rear kinematic reference) and ``omega_f_hat``
    (Hall-derived, LPF+MA filtered). Matches ``SlipRatioTrue`` exactly
    when noise is off and the estimator has settled.
    """

    name = "slip_est"
    inputs = ["omega_r", "omega_f_hat"]
    outputs = ["lambda_f_hat", "v_hat"]

    def __init__(self, R_f: float, R_r: float, v_epsilon: float):
        self.R_f = float(R_f)
        self.R_r = float(R_r)
        self.v_epsilon = float(v_epsilon)

    def step(self, t, u):
        omega_r = u.get("omega_r", 0.0)
        omega_f_hat = u.get("omega_f_hat", 0.0)
        v_hat = omega_r * self.R_r
        denom = max(v_hat, self.v_epsilon)
        lam = (v_hat - omega_f_hat * self.R_f) / denom
        return {"lambda_f_hat": lam, "v_hat": v_hat}
