"""Front-wheel normal load from quasi-static moment balance about rear contact.

Moment balance (CCW about rear contact) with the vehicle decelerating at
``|a_x|``:

    N_f · L = m · g · a + m · |a_x| · h

``a_x`` is the *signed* vehicle acceleration (``dv/dt`` ≤ 0 when braking),
so the equation in signed form is:

    N_f = (m g a − m a_x h) / L

The ``a_x`` signal is read from the simulator's persistent signals and is
therefore lagged by one step (see Global §5 of ``ASSUMPTIONS.md``).
"""

from __future__ import annotations

from ..block import Block


class NormalLoad(Block):
    name = "normal_load"
    inputs = ["a_x"]
    outputs = ["N_f"]

    def __init__(self, m: float, g: float, a: float, h: float, L: float):
        self.m = float(m)
        self.g = float(g)
        self.a = float(a)
        self.h = float(h)
        self.L = float(L)

    def step(self, t, u):
        a_x = u.get("a_x", 0.0)
        N_f = (self.m * self.g * self.a - self.m * a_x * self.h) / self.L
        # Clamp at the stoppie threshold; a reached bound is a diagnostic signal.
        N_f_clamped = min(max(N_f, 0.0), self.m * self.g)
        return {"N_f": N_f_clamped}
