"""Fixed-step RK4 simulator.

One evaluation pass = walk blocks in declared order, feeding each block
its named inputs from a shared ``signals`` dict. Outputs update that dict
in place so downstream blocks see them. For circular dependencies (e.g.
``N_f`` needs ``a_x`` needs ``F_f`` needs ``N_f``), one arrow of the loop
is broken by reading from the *persistent* signals dict — the previous
step's values — and the lag is absorbed by the small step size.
"""

from __future__ import annotations

from collections.abc import Iterable

import numpy as np

from .block import Block


class Simulator:
    def __init__(self, blocks: Iterable[Block], dt: float, t0: float = 0.0):
        self.blocks: list[Block] = list(blocks)
        self.dt = float(dt)
        self.t = float(t0)

        self._state_blocks: list[Block] = [b for b in self.blocks if b.has_state]
        self._slices: dict[str, slice] = {}
        offset = 0
        for b in self._state_blocks:
            n = len(b.state)
            self._slices[b.name] = slice(offset, offset + n)
            offset += n
        self.n_states = offset
        self.x = (
            np.concatenate([b.state for b in self._state_blocks])
            if self._state_blocks
            else np.zeros(0)
        )
        self._persistent_signals: dict[str, float] = {}
        _, self._persistent_signals = self._evaluate(self.t, self.x)

    def _evaluate(self, t: float, x: np.ndarray) -> tuple[np.ndarray, dict[str, float]]:
        signals: dict[str, float] = dict(self._persistent_signals)
        # Walk algebraic / output computations in declared order.
        for b in self.blocks:
            u = {name: signals[name] for name in b.inputs if name in signals}
            if b.has_state:
                xb = x[self._slices[b.name]]
                outs = b.output(t, xb, u)
            else:
                outs = b.step(t, u)
            signals.update(outs)
        # Collect derivatives using the now-populated signals dict.
        dx = np.zeros_like(x)
        for b in self._state_blocks:
            sl = self._slices[b.name]
            xb = x[sl]
            u = {name: signals[name] for name in b.inputs if name in signals}
            dx[sl] = b.derivatives(t, xb, u)
        return dx, signals

    def step(self) -> dict[str, float]:
        dt = self.dt
        t = self.t
        x = self.x
        k1, _ = self._evaluate(t, x)
        k2, _ = self._evaluate(t + 0.5 * dt, x + 0.5 * dt * k1)
        k3, _ = self._evaluate(t + 0.5 * dt, x + 0.5 * dt * k2)
        k4, _ = self._evaluate(t + dt, x + dt * k3)
        self.x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        self.t = t + dt
        # Final evaluation at the new state — locks in signals for the next
        # step's lagged reads (a_x, F_f, etc.) and for logging.
        _, self._persistent_signals = self._evaluate(self.t, self.x)
        # Discrete-state blocks advance exactly once per simulator step,
        # with the fully-populated signals dict from the final evaluation.
        for b in self.blocks:
            b.commit(self.t, self._persistent_signals)
        # Re-evaluate so post-commit outputs (e.g. fresh edge counts, new
        # estimator readings) show up in the logged row and in the next
        # step's persistent signals.
        _, self._persistent_signals = self._evaluate(self.t, self.x)
        return self._persistent_signals

    def run(self, t_end: float, log: bool = True) -> list[dict[str, float]]:
        n = int(round((t_end - self.t) / self.dt))
        rows: list[dict[str, float]] = []
        if log:
            rows.append({"t": self.t, **self._persistent_signals})
        for _ in range(n):
            sig = self.step()
            if log:
                rows.append({"t": self.t, **sig})
        return rows
