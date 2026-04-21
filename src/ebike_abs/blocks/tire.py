"""Closed-form Dugoff longitudinal-tire model.

Braking convention (``λ ∈ [0, 1)``):

    λ_s = λ / (1 − λ)                                          (slip-to-stretch)
    σ   = μ_peak · N_f · (1 − λ) / (2 · C_x · λ)
    f(σ) = 1                 if σ ≥ 1
         = σ (2 − σ)         if σ < 1
    F_f = C_x · λ_s · f(σ)

Limits:
* ``λ → 0⁺``:  λ_s → 0,  σ → ∞,  f(σ) = 1,  F_f → 0.
* ``λ → 1⁻``:  λ_s → ∞,  σ → 0,  f(σ) → 0,  F_f → μ_peak · N_f.

``λ_crit = μ_peak N_f / C_x`` is emitted as a diagnostic only — the
smooth ``f(σ)`` saturation replaces the brush-model piecewise switch.
"""

from __future__ import annotations

from ..block import Block


class BrushTireModel(Block):
    name = "tire"
    inputs = ["lambda_f_true", "N_f"]
    outputs = ["F_f", "lambda_crit"]

    # Guards at the numerical limits of λ.
    _LAMBDA_MIN = 1.0e-9
    _LAMBDA_MAX = 1.0 - 1.0e-6

    def __init__(self, mu_peak: float, C_x: float):
        self.mu_peak = float(mu_peak)
        self.C_x = float(C_x)

    def step(self, t, u):
        lam = u.get("lambda_f_true", 0.0)
        N_f = u.get("N_f", 0.0)
        F_f, lam_crit = self._compute(lam, N_f)
        return {"F_f": F_f, "lambda_crit": lam_crit}

    def _compute(self, lam: float, N_f: float) -> tuple[float, float]:
        lam_crit = self.mu_peak * N_f / self.C_x if self.C_x > 0.0 else 0.0
        # Braking only in Phase A; clamp λ into the physically meaningful
        # half-open interval [0, 1).
        lam_eff = min(max(lam, 0.0), self._LAMBDA_MAX)
        if lam_eff <= self._LAMBDA_MIN or N_f <= 0.0:
            return 0.0, lam_crit
        lam_s = lam_eff / (1.0 - lam_eff)
        sigma = self.mu_peak * N_f * (1.0 - lam_eff) / (2.0 * self.C_x * lam_eff)
        f_sigma = 1.0 if sigma >= 1.0 else sigma * (2.0 - sigma)
        F_f = self.C_x * lam_s * f_sigma
        return F_f, lam_crit
