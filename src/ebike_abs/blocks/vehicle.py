"""VehicleTranslation — longitudinal vehicle integrator.

Sign convention (Phase A, braking only):
* ``v`` > 0 : forward speed.
* ``F_f`` ≥ 0 : magnitude of retarding longitudinal tire force at the front
  contact patch. Acts in the ``-v`` direction on the vehicle.
* ``a_x`` = dv/dt ≤ 0 during braking; emitted algebraically as ``-F_f/m``
  using the F_f from the simulator's persistent signals (lag-1).
"""

from __future__ import annotations

import numpy as np

from ..block import Block


class VehicleTranslation(Block):
    name = "vehicle"
    inputs = ["F_f"]
    outputs = ["v", "a_x"]

    def __init__(
        self,
        m: float,
        v0: float,
        drag_enabled: bool = False,
        rr_enabled: bool = False,
    ):
        self.m = float(m)
        self.drag_enabled = bool(drag_enabled)
        self.rr_enabled = bool(rr_enabled)
        self.state = np.array([float(v0)], dtype=float)

    def _retarding_force(self, F_f: float) -> float:
        # Drag and rolling resistance are off in Phase A; placeholders kept
        # here so enabling them later is a one-line change.
        return F_f

    def output(self, t, x, u):
        v = float(x[0])
        F_f = u.get("F_f", 0.0)
        a_x = -self._retarding_force(F_f) / self.m
        return {"v": v, "a_x": a_x}

    def derivatives(self, t, x, u):
        F_f = u.get("F_f", 0.0)
        return np.array([-self._retarding_force(F_f) / self.m])
