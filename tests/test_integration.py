"""Phase A end-of-phase checkpoint.

Forced-lock oracle (PLAN.md Verification §2): with ``ω_f ≡ 0`` the wheel
is locked, Dugoff saturates at ``μ_peak · N_f``, weight transfer pushes
``N_f`` to the ``m g`` clamp, and the bike decelerates at ``μ_peak · g``.
Expected stopping distance is ``v_0² / (2 μ_peak g)`` to within 1 %.
"""

from __future__ import annotations

import numpy as np

from ebike_abs.block import Block
from ebike_abs.blocks.normal_load import NormalLoad
from ebike_abs.blocks.slip import SlipRatioTrue
from ebike_abs.blocks.tire import DugoffTireModel
from ebike_abs.blocks.vehicle import VehicleTranslation
from ebike_abs.simulator import Simulator


class ForcedLockedWheel(Block):
    """Test stand-in that holds ``ω_f ≡ 0``; no continuous state."""

    name = "wheel_f"
    inputs: list[str] = []
    outputs = ["omega_f"]

    def step(self, t, u):
        return {"omega_f": 0.0}


def _run_forced_lock(v0: float, mu_peak: float = 0.9):
    m, g = 120.0, 9.81
    a, h, L = 0.7, 1.0, 1.2
    R = 0.33
    dt = 1.0e-4
    blocks = [
        VehicleTranslation(m=m, v0=v0),
        ForcedLockedWheel(),
        NormalLoad(m=m, g=g, a=a, h=h, L=L),
        SlipRatioTrue(R_f=R, v_epsilon=0.01),
        DugoffTireModel(mu_peak=mu_peak, C_x=30_000.0),
    ]
    sim = Simulator(blocks, dt=dt)

    v_prev = sim._persistent_signals["v"]
    distance = 0.0
    t_max = 5.0
    n = int(round(t_max / dt))
    for _ in range(n):
        sig = sim.step()
        v_now = sig["v"]
        distance += 0.5 * (v_prev + v_now) * dt
        v_prev = v_now
        if v_now <= 1.0e-6:
            break
    return distance, sim


def test_forced_lock_stopping_distance_matches_oracle():
    v0 = 8.333
    mu_peak = 0.9
    g = 9.81
    distance, _ = _run_forced_lock(v0, mu_peak)
    expected = v0**2 / (2.0 * mu_peak * g)
    rel_err = abs(distance - expected) / expected
    assert rel_err < 0.01, (
        f"stopping distance {distance:.4f} m vs oracle {expected:.4f} m, "
        f"rel err {rel_err:.4%}"
    )


def test_forced_lock_clamps_N_f_to_mg():
    # With full stoppie, the NormalLoad clamp at m*g is hit within a handful
    # of steps of the start of braking.
    _, sim = _run_forced_lock(v0=8.333)
    # N_f at end-of-run (v=0) has a_x=0, so N_f drops back to static. Use
    # peak N_f from an explicit probe instead.
    # Re-build a quick sim and read N_f after a few steps.
    m, g = 120.0, 9.81
    blocks = [
        VehicleTranslation(m=m, v0=8.333),
        ForcedLockedWheel(),
        NormalLoad(m=m, g=g, a=0.7, h=1.0, L=1.2),
        SlipRatioTrue(R_f=0.33, v_epsilon=0.01),
        DugoffTireModel(mu_peak=0.9, C_x=30_000.0),
    ]
    sim2 = Simulator(blocks, dt=1.0e-4)
    for _ in range(100):  # 10 ms of settling
        sim2.step()
    assert sim2._persistent_signals["N_f"] == m * g


def test_coast_down_through_full_plant_and_tire():
    # Full Phase A plant with a regular (unlocked) wheel at rolling condition:
    # ω_f R = v → λ = 0 → F_f = 0 → v stays at v_0.
    from ebike_abs.blocks.wheel import FrontWheelRotation

    v0 = 8.333
    m, g = 120.0, 9.81
    R = 0.33
    blocks = [
        VehicleTranslation(m=m, v0=v0),
        FrontWheelRotation(I_f=0.12, R_f=R, omega0=v0 / R),
        NormalLoad(m=m, g=g, a=0.7, h=1.0, L=1.2),
        SlipRatioTrue(R_f=R, v_epsilon=0.01),
        DugoffTireModel(mu_peak=0.9, C_x=30_000.0),
    ]
    sim = Simulator(blocks, dt=1.0e-4)
    sim.run(2.0, log=False)
    v_final = sim._persistent_signals["v"]
    assert abs(v_final - v0) < 1.0e-6 * v0
    assert abs(sim._persistent_signals["lambda_f_true"]) < 1.0e-9


def test_panic_stop_scenario_runs_and_stops():
    # Smoke test on the shared scenario builder: v starts at v_0 and falls
    # monotonically to near zero under a ramp-and-hold prescribed clamp.
    from ebike_abs.scenarios import build_phase_a_panic_stop, load_config

    from pathlib import Path

    cfg = load_config(Path(__file__).resolve().parents[1] / "configs" / "default.toml")
    sim = build_phase_a_panic_stop(cfg)
    rows = sim.run(cfg["scenario"]["t_end"])
    vs = np.array([r["v"] for r in rows])
    assert vs[0] == cfg["scenario"]["v0"]
    assert vs[-1] < 0.2
    assert (np.diff(vs) <= 1e-9).all()
