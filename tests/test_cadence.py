"""Cadence controller — unit tests + end-to-end sanity on the Phase C
cadence scenario.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from ebike_abs.control.cadence import CadenceController
from ebike_abs.scenarios import (
    build_phase_b_panic_stop,
    build_phase_c_cadence_panic_stop,
    load_config,
)


def _cfg() -> dict:
    return load_config(Path(__file__).resolve().parents[1] / "configs" / "default.toml")


def test_cadence_square_wave_shape():
    # 2 Hz @ 50 % duty → on for the first half of each 0.5 s period,
    # off for the second half. Check a handful of sample times.
    cad = CadenceController(freq=2.0, duty=0.5)
    V_cmd = 6.0
    # On-phase samples:
    for t in (0.0, 0.1, 0.24, 0.50, 0.74, 1.24):
        out = cad.step(t, {"V_pwm_cmd": V_cmd})
        assert out["V_pwm"] == V_cmd, f"t={t} should be on-phase"
        assert out["cadence_on"] == 1.0
    # Off-phase samples:
    for t in (0.25, 0.4, 0.49, 0.75, 0.99, 1.49):
        out = cad.step(t, {"V_pwm_cmd": V_cmd})
        assert out["V_pwm"] == 0.0, f"t={t} should be off-phase"
        assert out["cadence_on"] == 0.0


def test_cadence_respects_varying_command():
    # The chop just gates V_pwm_cmd — it should not hold a stale value
    # through the off→on edge.
    cad = CadenceController(freq=1.0, duty=0.5)  # 1 Hz, on in [0, 0.5)
    assert cad.step(0.1, {"V_pwm_cmd": 3.0})["V_pwm"] == 3.0
    assert cad.step(0.6, {"V_pwm_cmd": 3.0})["V_pwm"] == 0.0
    assert cad.step(1.1, {"V_pwm_cmd": 7.0})["V_pwm"] == 7.0  # new command


def test_cadence_rejects_invalid_params():
    import pytest

    with pytest.raises(ValueError):
        CadenceController(freq=0.0, duty=0.5)
    with pytest.raises(ValueError):
        CadenceController(freq=1.0, duty=1.5)
    with pytest.raises(ValueError):
        CadenceController(freq=1.0, duty=-0.1)


def test_cadence_end_to_end_stops_bike():
    cfg = _cfg()
    sim = build_phase_c_cadence_panic_stop(cfg)
    rows = sim.run(cfg["scenario"]["t_end"])
    vs = np.array([r["v"] for r in rows])
    # The cadence chops brake pressure in half on average, so it should
    # still stop within the run window but take longer than Phase B.
    assert vs[-1] < 0.2


def test_cadence_modulates_clamp_force():
    # The telltale of a real cadence baseline: F_clamp oscillates
    # visibly rather than sitting at the steady-state value a pure
    # human ramp would produce. Pick a mid-stop window where the rider
    # ramp is saturated (t > 0.15 s) and confirm the coefficient of
    # variation on F_clamp is large — i.e. there is real oscillation.
    cfg = _cfg()
    rows = build_phase_c_cadence_panic_stop(cfg).run(cfg["scenario"]["t_end"])
    ts = np.array([r["t"] for r in rows])
    F_clamp = np.array([r["F_clamp"] for r in rows])
    v = np.array([r["v"] for r in rows])
    mask = (ts > 0.2) & (v > 2.0)
    window = F_clamp[mask]
    assert window.size > 0
    assert window.std() / max(window.mean(), 1e-9) > 0.1, (
        f"cadence chop should oscillate F_clamp (CV={window.std() / window.mean():.3f})"
    )


def test_cadence_stops_later_than_phase_b():
    # On dry pavement chopping the brake means you spend time in DUMP
    # where the tire makes no retarding force — the stop should be
    # *longer* than the uninterrupted Phase B run. (This is the whole
    # reason cadence braking is a poor baseline on high-μ.)
    cfg = _cfg()

    def stop_time(rows):
        for r in rows:
            if r["v"] < 0.2:
                return r["t"]
        return rows[-1]["t"]

    t_b = stop_time(build_phase_b_panic_stop(cfg).run(cfg["scenario"]["t_end"]))
    t_c = stop_time(build_phase_c_cadence_panic_stop(cfg).run(cfg["scenario"]["t_end"]))
    assert t_c > t_b, (
        f"cadence expected to take longer on dry asphalt (cadence {t_c:.3f} s vs Phase B {t_b:.3f} s)"
    )
