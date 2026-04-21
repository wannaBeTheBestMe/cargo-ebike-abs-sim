"""Phase B end-to-end panic stop integration tests.

Phase B replaces the Phase A prescribed-clamp stand-in with the full
motor + hydraulic actuator chain driven by a human V_pwm ramp. With the
human ramp deep in saturation the wheel still locks for most of the
stop, so the stopping distance should sit near the Phase A run and well
inside the forced-lock oracle.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from ebike_abs.scenarios import (
    build_phase_a_panic_stop,
    build_phase_b_panic_stop,
    load_config,
)


def _cfg() -> dict:
    return load_config(Path(__file__).resolve().parents[1] / "configs" / "default.toml")


def _stopping_distance(rows: list[dict]) -> float:
    ts = np.array([r["t"] for r in rows])
    v = np.clip(np.array([r["v"] for r in rows]), 0.0, None)
    return float(np.trapezoid(v, ts))


def test_phase_b_panic_stop_near_phase_a():
    cfg = _cfg()
    rows_a = build_phase_a_panic_stop(cfg).run(cfg["scenario"]["t_end"])
    rows_b = build_phase_b_panic_stop(cfg).run(cfg["scenario"]["t_end"])
    d_a = _stopping_distance(rows_a)
    d_b = _stopping_distance(rows_b)
    # Phase B adds an electrical + mechanical + hydraulic lag on top of
    # the same-shape V_pwm ramp, so it should stop a touch longer than A
    # but still inside ~10 % of it.
    assert d_b >= d_a * 0.95, f"Phase B stops too short: {d_b:.3f} m vs A {d_a:.3f} m"
    assert d_b <= d_a * 1.15, f"Phase B stops too long: {d_b:.3f} m vs A {d_a:.3f} m"


def test_phase_b_stopping_distance_inside_oracle_bound():
    cfg = _cfg()
    rows = build_phase_b_panic_stop(cfg).run(cfg["scenario"]["t_end"])
    d = _stopping_distance(rows)
    v0 = cfg["scenario"]["v0"]
    mu, g = cfg["tire"]["mu_peak"], cfg["vehicle"]["g"]
    oracle = v0**2 / (2.0 * mu * g)
    # The oracle is the best possible (full-lock from t=0). Any real
    # ramp loses a bit; 1–10 % over is expected.
    assert d >= oracle * 0.98
    assert d <= oracle * 1.20, f"Phase B distance {d:.3f} m vs oracle {oracle:.3f} m"


def test_phase_b_vehicle_comes_to_rest():
    cfg = _cfg()
    rows = build_phase_b_panic_stop(cfg).run(cfg["scenario"]["t_end"])
    vs = np.array([r["v"] for r in rows])
    assert vs[0] == cfg["scenario"]["v0"]
    assert vs[-1] < 0.2


def test_phase_b_actuator_engages_within_hydraulic_tau():
    # With V_pwm ramping up over 0.15 s and τ_hyd = 30 ms, we expect F_clamp
    # to reach ≥ 100 N inside the first 100 ms. A looser sanity bound than
    # the standalone step-response test, but catches wiring errors.
    cfg = _cfg()
    sim = build_phase_b_panic_stop(cfg)
    rows = sim.run(0.2)
    Fc = np.array([r["F_clamp"] for r in rows])
    ts = np.array([r["t"] for r in rows])
    idx = np.where(ts >= 0.10)[0][0]
    assert Fc[idx] > 100.0, f"F_clamp @ 100 ms = {Fc[idx]:.1f} N (expected > 100)"


def test_phase_b_estimator_present_in_log():
    cfg = _cfg()
    rows = build_phase_b_panic_stop(cfg).run(0.1)
    # Sanity: the Hall / estimator / slip-est blocks emit their signals.
    for key in ("lambda_f_hat", "omega_f_hat", "hall_edge_count", "V_pwm"):
        assert key in rows[-1], f"missing signal: {key}"
