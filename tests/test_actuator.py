"""MotorActuator step response, saturation, and steady-state checks."""

from __future__ import annotations

import numpy as np
import pytest

from ebike_abs.block import Block
from ebike_abs.blocks.actuator import MotorActuator
from ebike_abs.simulator import Simulator


def _params() -> dict:
    # Mirrors the [actuator] section of configs/default.toml.
    return dict(
        L_m=1.0e-3,
        R_m=0.1,
        K_e=0.05,
        K_t=0.05,
        J_m=1.0e-4,
        b_m=1.0e-2,
        r_lever=0.001,
        A_master=1.0e-4,
        A_caliper=4.0e-4,
        tau_hyd=0.030,
        V_pwm_max=12.0,
    )


class VSource(Block):
    """Constant V_pwm for step-response testing."""

    name = "v_source"
    inputs: list[str] = []
    outputs = ["V_pwm"]

    def __init__(self, V: float):
        self.V = float(V)

    def step(self, t, u):
        return {"V_pwm": self.V}


def _step_response(V: float, t_end: float = 0.5, dt: float = 1.0e-4):
    act = MotorActuator(**_params())
    sim = Simulator([VSource(V), act], dt=dt)
    rows = sim.run(t_end)
    ts = np.array([r["t"] for r in rows])
    Fc = np.array([r["F_clamp"] for r in rows])
    return ts, Fc, act


def test_step_response_reaches_steady_state():
    V = 6.0
    ts, Fc, act = _step_response(V, t_end=0.5)
    analytic = act.steady_state_F_clamp(V)
    # After 10 hydraulic time constants the response should be within 1%.
    assert Fc[-1] == pytest.approx(analytic, rel=1e-2)


def test_step_response_starts_at_zero():
    ts, Fc, _ = _step_response(6.0, t_end=0.01)
    assert Fc[0] == 0.0


def test_hydraulic_lag_dominates_initial_response():
    # After one hydraulic τ the response should be under 75 % of steady state
    # (pure 1st-order would give 63 %; cascaded lags give even less).
    V = 6.0
    ts, Fc, act = _step_response(V, t_end=0.030)
    analytic = act.steady_state_F_clamp(V)
    ratio = Fc[-1] / analytic
    assert 0.2 < ratio < 0.75


def test_V_pwm_saturation():
    act = MotorActuator(**_params())
    # Well above max voltage should be clipped.
    over = act.steady_state_F_clamp(100.0)
    at_max = act.steady_state_F_clamp(12.0)
    assert over == pytest.approx(at_max, rel=1e-12)


def test_F_clamp_non_negative_under_reverse_voltage():
    # Negative V_pwm: a disc brake should not clamp negatively. The output
    # is clipped at zero even while the state integrates freely.
    ts, Fc, _ = _step_response(-6.0, t_end=0.1)
    assert (Fc >= 0.0).all()


def test_zero_input_holds_zero():
    ts, Fc, _ = _step_response(0.0, t_end=0.2)
    assert float(Fc.max()) == 0.0


def test_steady_state_exceeds_wheel_lock_threshold_at_half_scale():
    # Design target: steady F_clamp at half scale must be enough to lock the
    # front wheel on dry asphalt (μ_peak m g R_f ≈ 350 N·m required brake
    # torque ⇒ F_clamp > 3100 N with the default brake config).
    act = MotorActuator(**_params())
    assert act.steady_state_F_clamp(6.0) > 3100.0
