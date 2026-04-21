"""Hall-effect sensor emulation and wheel-speed estimator.

``HallSensor``
    Continuous state: front-wheel angle ``theta_f`` (integral of ``omega_f``).
    Discrete state: number of edges seen and the most recent edge timestamp.
    On each simulator step (in ``commit``), any angle-crossings of
    ``2π / N_hall`` since the last step are detected and an edge is emitted
    for each one. Phase A runs with jitter and missed edges both disabled;
    the hooks are present for Phase B noise sweeps.

``WheelSpeedEstimator``
    Fully-discrete. On each edge, takes the raw edge-to-edge speed
    ``(2π/N) / Δt``, low-pass-filters it (first-order, cutoff ``lpf_fc``),
    pushes the result into a length-``ma_window`` ring buffer for a
    moving-average ``ω̂_f``, and finally takes a central-difference
    derivative ``ω̇̂_f`` from a three-sample history of MA outputs.
    Between edges the estimator emits ZOH of the last MA output.

Both blocks only mutate discrete state inside ``commit``; their
``step``/``output`` methods must be side-effect free because RK4 calls
them four times per simulator step.
"""

from __future__ import annotations

from collections import deque

import numpy as np

from ..block import Block

_TWO_PI = 2.0 * np.pi


class HallSensor(Block):
    name = "hall"
    inputs = ["omega_f"]
    outputs = [
        "theta_f",
        "hall_edge_count",
        "hall_last_edge_dt",
        "hall_last_edge_t",
    ]

    def __init__(
        self,
        N_hall: int,
        jitter_frac: float = 0.0,
        missed_prob: float = 0.0,
        rng: np.random.Generator | None = None,
    ):
        self.N_hall = int(N_hall)
        self.angle_per_edge = _TWO_PI / float(self.N_hall)
        self.jitter_frac = float(jitter_frac)
        self.missed_prob = float(missed_prob)
        self.rng = rng if rng is not None else np.random.default_rng(0)
        # Continuous-state: integrated wheel angle.
        self.state = np.zeros(1, dtype=float)
        # Discrete state. ``next_edge_angle`` is the angle at which the
        # next edge will be emitted; edge_count/last_edge_t are updated
        # only inside ``commit``.
        self.edge_count: int = 0
        self.last_edge_t: float = 0.0
        self.last_edge_dt: float = 0.0
        self.next_edge_angle: float = self.angle_per_edge

    def output(self, t, x, u):
        return {
            "theta_f": float(x[0]),
            "hall_edge_count": int(self.edge_count),
            "hall_last_edge_dt": float(self.last_edge_dt),
            "hall_last_edge_t": float(self.last_edge_t),
        }

    def derivatives(self, t, x, u):
        # Angle integrates wheel speed; we don't mirror the one-sided lock
        # guard from FrontWheelRotation here because the wheel block already
        # prevents ω_f from trending negative. If it did briefly dip
        # sub-zero between RK4 substeps, θ would just retrace — no edges
        # get counted twice because ``commit`` tracks crossings by
        # ``next_edge_angle``.
        omega_f = u.get("omega_f", 0.0)
        return np.array([max(0.0, omega_f)])

    def commit(self, t, signals):
        theta = float(signals.get("theta_f", self.state[0]))
        while theta >= self.next_edge_angle:
            # Determine the nominal edge time by linear interpolation back
            # from the current (t, theta) sample. This is approximate but
            # is O(Δt²) for a smooth ω_f and far tighter than a snap-to-t.
            omega_est = max(1e-9, signals.get("omega_f", 0.0))
            t_edge = t - (theta - self.next_edge_angle) / omega_est
            if self.missed_prob > 0.0 and self.rng.random() < self.missed_prob:
                # Missed edge: still advance the angle boundary so we don't
                # re-emit the same edge forever, but skip the timestamp.
                self.next_edge_angle += self.angle_per_edge
                continue
            if self.jitter_frac > 0.0:
                nominal_dt = t_edge - self.last_edge_t
                if nominal_dt > 0.0:
                    t_edge += self.rng.normal(0.0, self.jitter_frac * nominal_dt)
            dt_edge = t_edge - self.last_edge_t
            if dt_edge <= 0.0:
                # Defensive: monotone clock. Shouldn't happen with
                # jitter_frac ≪ 1, but keep the estimator safe.
                dt_edge = max(1e-9, t - self.last_edge_t)
            self.last_edge_dt = dt_edge
            self.last_edge_t = t_edge
            self.edge_count += 1
            self.next_edge_angle += self.angle_per_edge


class WheelSpeedEstimator(Block):
    """Edge → LPF → moving-average → central-difference ω̂_f and ω̇̂_f.

    Reads the Hall block's discrete outputs. Between edges we hold the
    last moving-average output (ZOH). Because this block has no
    continuous state, the estimator only updates inside ``commit``; its
    ``step`` method returns the cached ZOH values.
    """

    name = "speed_est"
    inputs = ["hall_edge_count", "hall_last_edge_dt", "hall_last_edge_t"]
    outputs = ["omega_f_hat", "omega_f_hat_dot", "omega_f_raw"]

    def __init__(
        self,
        N_hall: int,
        lpf_fc: float,
        ma_window: int,
        dt: float,
        omega0: float = 0.0,
    ):
        self.angle_per_edge = _TWO_PI / float(N_hall)
        self.tau_lpf = 1.0 / (2.0 * np.pi * float(lpf_fc))
        self.ma_window = int(ma_window)
        self.dt = float(dt)
        self._last_edge_count: int = 0
        self._omega_raw: float = float(omega0)
        self._omega_lpf: float = float(omega0)
        self._ma_buf: deque[float] = deque([float(omega0)] * self.ma_window, maxlen=self.ma_window)
        self._omega_hat: float = float(omega0)
        # Seed the "virtual last edge interval" so the edge-timeout bound
        # works from t=0, before any real edge has fired. Uses the
        # initial wheel speed to predict the first-edge interval.
        self._virtual_last_dt: float = self.angle_per_edge / float(omega0) if omega0 > 0.0 else 0.0
        # Central-difference history: need the last three MA outputs and
        # the times they were produced. Seed with the initial estimate
        # so the derivative is defined from step one.
        self._hat_history: deque[tuple[float, float]] = deque(maxlen=3)
        for k in range(3):
            self._hat_history.append((-float(k + 1) * dt, float(omega0)))
        self._omega_hat_dot: float = 0.0

    def step(self, t, u):
        return {
            "omega_f_hat": self._omega_hat,
            "omega_f_hat_dot": self._omega_hat_dot,
            "omega_f_raw": self._omega_raw,
        }

    def _push_hat(self, t: float, value: float) -> None:
        self._omega_hat = value
        self._hat_history.append((t, value))
        if len(self._hat_history) == 3:
            (t0, w0), _mid, (t2, w2) = (
                self._hat_history[0],
                self._hat_history[1],
                self._hat_history[2],
            )
            dt_cd = t2 - t0
            if dt_cd > 0.0:
                self._omega_hat_dot = (w2 - w0) / dt_cd

    def commit(self, t, signals):
        edge_count = int(signals.get("hall_edge_count", self._last_edge_count))
        last_edge_t = float(signals.get("hall_last_edge_t", 0.0))
        if edge_count == self._last_edge_count:
            # No new edge this step. A real ABS-grade estimator pulls its
            # estimate down by the "edge-period upper bound": the wheel
            # cannot be spinning faster than one edge per (t − t_last).
            # Without this, a locked wheel leaves the estimator stuck on
            # its last fresh reading and the FSM never fires. We bound
            # only when ``dt_since`` exceeds the most recent known edge
            # interval — inside that interval the last edge's rate is
            # still the tightest bound we have. Before the first real
            # edge, the initial-speed-derived virtual interval stands in.
            last_dt = float(signals.get("hall_last_edge_dt", 0.0))
            if last_dt <= 0.0:
                last_dt = self._virtual_last_dt
            if last_dt > 0.0:
                dt_since = t - last_edge_t
                # 1.5× slack keeps the bound from firing on the normal
                # inter-edge gap (at steady ω, dt_since reaches ≈ last_dt
                # just before the next edge); the wheel has to be slowing
                # by more than a third of the last-known rate before we
                # believe it.
                if dt_since > 1.5 * last_dt:
                    omega_upper = self.angle_per_edge / dt_since
                    if omega_upper < self._omega_hat:
                        self._push_hat(t, omega_upper)
            return
        n_new = edge_count - self._last_edge_count
        dt_edge = float(signals.get("hall_last_edge_dt", 0.0))
        if dt_edge <= 0.0:
            return
        self._omega_raw = self.angle_per_edge / dt_edge
        alpha = dt_edge / (self.tau_lpf + dt_edge)
        self._omega_lpf = self._omega_lpf + alpha * (self._omega_raw - self._omega_lpf)
        for _ in range(n_new):
            self._ma_buf.append(self._omega_lpf)
        new_hat = float(np.mean(self._ma_buf)) if self._ma_buf else 0.0
        self._push_hat(t, new_hat)
        self._last_edge_count = edge_count
