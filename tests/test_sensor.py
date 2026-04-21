"""HallSensor edge counting + WheelSpeedEstimator tracking checks."""

from __future__ import annotations

import numpy as np

from ebike_abs.block import Block
from ebike_abs.blocks.sensor import HallSensor, WheelSpeedEstimator
from ebike_abs.simulator import Simulator


class ConstantOmega(Block):
    """Emits a constant ``omega_f`` for end-to-end sensor tests."""

    name = "omega_src"
    inputs: list[str] = []
    outputs = ["omega_f"]

    def __init__(self, omega: float):
        self.omega = float(omega)

    def step(self, t, u):
        return {"omega_f": self.omega}


def _run(omega: float, t_end: float, dt: float = 1.0e-4, N_hall: int = 20):
    src = ConstantOmega(omega)
    hall = HallSensor(N_hall=N_hall)
    est = WheelSpeedEstimator(N_hall=N_hall, lpf_fc=50.0, ma_window=4, dt=dt)
    sim = Simulator([src, hall, est], dt=dt)
    rows = sim.run(t_end)
    return rows, hall, est


def test_hall_edge_count_advances_linearly():
    omega = 20.0  # rad/s
    t_end = 1.0
    rows, hall, _ = _run(omega, t_end)
    # Expected edges = ω · t / (2π / N) = ω · N · t / (2π)
    expected = omega * hall.N_hall * t_end / (2.0 * np.pi)
    actual = rows[-1]["hall_edge_count"]
    # Tolerance: one edge either side is fine (boundary effects).
    assert abs(actual - expected) <= 1.0


def test_estimator_tracks_constant_omega():
    # Sweep ω from 5 → 50 rad/s; require steady-state estimate within 0.5 %.
    for omega in (5.0, 10.0, 20.0, 35.0, 50.0):
        rows, _, _ = _run(omega, t_end=2.0)
        omega_hat = rows[-1]["omega_f_hat"]
        assert abs(omega_hat - omega) / omega < 5e-3, f"ω={omega}: ω̂={omega_hat}"


def test_estimator_is_zero_before_first_edge():
    # Very low ω so no edge fires inside a 1 ms run.
    rows, _, _ = _run(0.1, t_end=1.0e-3)
    assert rows[0]["omega_f_hat"] == 0.0
    assert rows[-1]["omega_f_hat"] == 0.0


def test_edge_count_matches_integrated_angle():
    # θ(t) = ω t. Edge boundary: 2π/N. So floor(ω t N / 2π) edges.
    omega = 12.5
    t_end = 0.8
    rows, hall, _ = _run(omega, t_end)
    theta = rows[-1]["theta_f"]
    expected_edges = int(np.floor(theta / hall.angle_per_edge))
    actual = rows[-1]["hall_edge_count"]
    assert actual == expected_edges


def test_estimator_derivative_is_near_zero_at_constant_omega():
    # At constant ω, ω̇̂_f should settle to ~0. Give it enough time to
    # accumulate ≥ 3 MA samples so the central difference activates.
    rows, _, _ = _run(20.0, t_end=2.0)
    # Skip the very first edges where the LPF/MA are still warming up.
    late = rows[len(rows) // 2 :]
    omega_hat_dots = [r["omega_f_hat_dot"] for r in late]
    # With constant ω the central difference should be exactly zero
    # after the MA has settled (all three history samples are equal).
    assert max(abs(d) for d in omega_hat_dots) < 1e-6
