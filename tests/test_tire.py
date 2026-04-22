"""Dugoff tire model: limits, saturation, monotonicity, friction-circle cap."""

from __future__ import annotations

import numpy as np
import pytest

from ebike_abs.blocks.tire import DugoffTireModel


@pytest.fixture
def tire() -> DugoffTireModel:
    return DugoffTireModel(mu_peak=0.9, C_x=30_000.0)


@pytest.fixture
def N_static() -> float:
    # m g a / L at default params → a representative static front load.
    return 120.0 * 9.81 * 0.7 / 1.2


def test_zero_slip_zero_force(tire, N_static):
    out = tire.step(0.0, {"lambda_f_true": 0.0, "N_f": N_static})
    assert out["F_f"] == 0.0


def test_small_slip_approximately_linear(tire, N_static):
    # Well below λ_crit, σ ≫ 1 so f(σ) = 1 and F = C_x · λ_s = C_x · λ/(1-λ).
    # That approaches the C_x · λ brush-model linear region as λ → 0.
    lam = 1e-4
    F = tire.step(0.0, {"lambda_f_true": lam, "N_f": N_static})["F_f"]
    linear = 30_000.0 * lam
    expected = 30_000.0 * lam / (1.0 - lam)
    assert F == pytest.approx(expected, rel=1e-9)
    assert abs(F - linear) / linear < 1e-3  # 0.1 % off the pure-linear form.


def test_force_saturates_near_lambda_one(tire, N_static):
    # λ → 1 should drive F_f to μ_peak · N_f.
    F = tire.step(0.0, {"lambda_f_true": 0.9999, "N_f": N_static})["F_f"]
    peak = 0.9 * N_static
    assert F == pytest.approx(peak, rel=1e-4)


def test_force_never_exceeds_friction_circle(tire, N_static):
    peak = 0.9 * N_static
    for lam in np.linspace(1e-4, 0.9999, 200):
        F = tire.step(0.0, {"lambda_f_true": lam, "N_f": N_static})["F_f"]
        assert F <= peak * (1.0 + 1e-9)


def test_force_is_monotone_nondecreasing_in_lambda(tire, N_static):
    lam_grid = np.linspace(1e-4, 0.9999, 200)
    F = [tire.step(0.0, {"lambda_f_true": float(x), "N_f": N_static})["F_f"] for x in lam_grid]
    # Dugoff is monotone non-decreasing on [0, 1).
    diffs = np.diff(F)
    assert (diffs > -1e-9).all()


def test_lambda_crit_tracks_normal_load(tire):
    low = tire.step(0.0, {"lambda_f_true": 0.0, "N_f": 500.0})["lambda_crit"]
    high = tire.step(0.0, {"lambda_f_true": 0.0, "N_f": 1500.0})["lambda_crit"]
    assert high == pytest.approx(3.0 * low, rel=1e-12)
    assert low == pytest.approx(0.9 * 500.0 / 30_000.0, rel=1e-12)


def test_negative_slip_clamped_to_zero(tire, N_static):
    # Phase A is braking-only; a negative λ (driving) is clamped out.
    F = tire.step(0.0, {"lambda_f_true": -0.1, "N_f": N_static})["F_f"]
    assert F == 0.0


def test_zero_normal_load_gives_zero_force(tire):
    F = tire.step(0.0, {"lambda_f_true": 0.5, "N_f": 0.0})["F_f"]
    assert F == 0.0
