"""Block base class.

Two patterns:

1. Algebraic / stateless blocks implement :meth:`step` only. They have no
   entry in the simulator's continuous-state vector.
2. Continuous-state blocks set ``self.state`` (a 1-D numpy array of length
   ``state_dim``) and implement :meth:`derivatives` and :meth:`output`.

Discrete state (FSM mode, ring buffers, Hall edge timestamps) lives on the
block instance as ordinary attributes and is advanced inside :meth:`step`.
"""

from __future__ import annotations

import numpy as np


class Block:
    name: str = ""
    inputs: list[str] = []
    outputs: list[str] = []

    def step(self, t: float, u: dict[str, float]) -> dict[str, float]:
        """Compute outputs for algebraic / stateless blocks."""
        raise NotImplementedError(f"{type(self).__name__}.step is not implemented")

    def output(self, t: float, x: np.ndarray, u: dict[str, float]) -> dict[str, float]:
        """Compute outputs for continuous-state blocks. Defaults to :meth:`step`."""
        return self.step(t, u)

    def derivatives(self, t: float, x: np.ndarray, u: dict[str, float]) -> np.ndarray:
        """Return dx/dt for a continuous-state block."""
        return np.zeros(0)

    def commit(self, t: float, signals: dict[str, float]) -> None:
        """Advance discrete state once per simulator step.

        Called after RK4 has produced the new continuous state, with a
        fully-populated ``signals`` dict from the final evaluation. Use this
        for edge counters, FSM transitions, ring-buffer updates — anything
        that must *not* be re-run inside the RK4 substeps.
        """

    @property
    def has_state(self) -> bool:
        return hasattr(self, "state") and self.state is not None and len(self.state) > 0
