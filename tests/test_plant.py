"""Plant-core sanity checks: coast-down, rear kinematics, slip invariant, N_f."""

from __future__ import annotations

import numpy as np

from ebike_abs.blocks.normal_load import NormalLoad
from ebike_abs.blocks.slip import SlipRatioTrue
from ebike_abs.blocks.vehicle import VehicleTranslation
from ebike_abs.blocks.wheel import FrontWheelRotation, RearWheelKinematics
from ebike_abs.simulator import Simulator


def _build_plant(v0: float) -> Simulator:
    m, g = 120.0, 9.81
    a, h, L = 0.7, 1.0, 1.2
    R = 0.33
    I_f = 0.12
    dt = 1e-4
    v_eps = 0.1
    blocks = [
        VehicleTranslation(m=m, v0=v0),
        FrontWheelRotation(I_f=I_f, R_f=R, omega0=v0 / R),
        RearWheelKinematics(R_r=R),
        NormalLoad(m=m, g=g, a=a, h=h, L=L),
        SlipRatioTrue(R_f=R, v_epsilon=v_eps),
    ]
    return Simulator(blocks, dt=dt)


def test_coast_down_preserves_speed():
    # No brake, no drag, no tire in the loop → v must be constant.
    v0 = 8.333
    sim = _build_plant(v0)
    rows = sim.run(5.0, log=False)
    # run(..., log=False) still advances state; check final persistent signals.
    v_final = sim._persistent_signals["v"]
    omega_final = sim._persistent_signals["omega_f"]
    assert abs(v_final - v0) < 1e-6 * v0
    assert abs(omega_final - v0 / 0.33) < 1e-6 * (v0 / 0.33)
    # Slip must stay exactly zero (ω_f R = v throughout).
    assert abs(sim._persistent_signals["lambda_f_true"]) < 1e-12


def test_rear_kinematics_matches_v_over_R():
    sim = _build_plant(v0=5.0)
    sig = sim._persistent_signals
    assert abs(sig["omega_r"] - 5.0 / 0.33) < 1e-12


def test_normal_load_static_case():
    # At t=0 with zero a_x, N_f should equal m*g*a/L (static front load).
    m, g, a, h, L = 120.0, 9.81, 0.7, 1.0, 1.2
    block = NormalLoad(m=m, g=g, a=a, h=h, L=L)
    out = block.step(0.0, {"a_x": 0.0})
    assert abs(out["N_f"] - m * g * a / L) < 1e-12


def test_normal_load_weight_transfers_forward_on_brake():
    m, g, a, h, L = 120.0, 9.81, 0.7, 1.0, 1.2
    block = NormalLoad(m=m, g=g, a=a, h=h, L=L)
    static = block.step(0.0, {"a_x": 0.0})["N_f"]
    # Braking: a_x < 0 → N_f must increase.
    braking = block.step(0.0, {"a_x": -5.0})["N_f"]
    assert braking > static


def test_normal_load_clamped_at_stoppie():
    # Extreme deceleration should hit the upper clamp at m*g.
    m, g, a, h, L = 120.0, 9.81, 0.7, 1.0, 1.2
    block = NormalLoad(m=m, g=g, a=a, h=h, L=L)
    out = block.step(0.0, {"a_x": -100.0})["N_f"]
    assert abs(out - m * g) < 1e-12


def test_slip_is_zero_at_rolling_condition():
    R, v_eps = 0.33, 0.1
    slip = SlipRatioTrue(R_f=R, v_epsilon=v_eps)
    out = slip.step(0.0, {"v": 5.0, "omega_f": 5.0 / R})
    assert abs(out["lambda_f_true"]) < 1e-12


def test_slip_denominator_clamp_near_stop():
    # Guard against blow-up as v → 0.
    R, v_eps = 0.33, 0.1
    slip = SlipRatioTrue(R_f=R, v_epsilon=v_eps)
    out = slip.step(0.0, {"v": 0.0, "omega_f": 0.0})
    assert np.isfinite(out["lambda_f_true"])
    assert out["lambda_f_true"] == 0.0
