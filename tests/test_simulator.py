"""Sanity check the RK4 simulator against a 1-state ODE with a closed form."""

from __future__ import annotations

import numpy as np

from ebike_abs.block import Block
from ebike_abs.simulator import Simulator


class ExpDecay(Block):
    """dx/dt = -k * x  →  x(t) = x0 * exp(-k t)."""

    name = "decay"
    inputs: list[str] = []
    outputs = ["x"]

    def __init__(self, k: float, x0: float):
        self.k = k
        self.state = np.array([x0], dtype=float)

    def output(self, t, x, u):
        return {"x": float(x[0])}

    def derivatives(self, t, x, u):
        return np.array([-self.k * x[0]])


def test_exp_decay_tracks_closed_form():
    k = 2.0
    x0 = 1.0
    dt = 1e-4
    t_end = 1.0
    sim = Simulator([ExpDecay(k, x0)], dt=dt)
    rows = sim.run(t_end)

    assert abs(rows[0]["t"] - 0.0) < 1e-12
    assert abs(rows[-1]["t"] - t_end) < 1e-9

    truth = x0 * np.exp(-k * t_end)
    got = rows[-1]["x"]
    # RK4 local error O(dt^5), global O(dt^4) → well under 1e-10 here.
    assert abs(got - truth) < 1e-10, f"RK4 drift: got {got}, truth {truth}"


def test_initial_signals_populated_before_first_step():
    sim = Simulator([ExpDecay(1.0, 3.5)], dt=1e-3)
    # After __init__ the signals dict should already carry the algebraic
    # output, so run(.., log=True) logs the t=0 row correctly.
    rows = sim.run(0.0)
    assert len(rows) == 1
    assert rows[0]["x"] == 3.5
