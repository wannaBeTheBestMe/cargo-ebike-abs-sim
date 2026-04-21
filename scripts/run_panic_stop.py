"""End-to-end panic stop runner.

Usage::

    python scripts/run_panic_stop.py --phase a   # prescribed F_clamp ramp
    python scripts/run_panic_stop.py --phase b   # human V_pwm → actuator chain

Phase A is the original MVP: the brake torque source is a hand-shaped
linear $F_{\text{clamp}}(t)$ ramp. Phase B uses the full motor +
hydraulic actuator chain driven by a $V_{\text{pwm}}$ ramp from the
human-baseline controller, with the Hall + estimator pipeline running
alongside the plant. Output: a multi-panel plot under ``out/runs/``.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from ebike_abs.scenarios import (
    build_phase_a_panic_stop,
    build_phase_b_panic_stop,
    load_config,
)

REPO = Path(__file__).resolve().parents[1]


def _run(phase: str, cfg: dict):
    if phase == "a":
        sim = build_phase_a_panic_stop(cfg)
    elif phase == "b":
        sim = build_phase_b_panic_stop(cfg)
    else:
        raise SystemExit(f"unknown phase: {phase!r}")
    rows = sim.run(cfg["scenario"]["t_end"])
    return rows


def _plot(phase: str, cfg: dict, rows: list[dict]) -> Path:
    R_f = cfg["vehicle"]["R_f"]
    ts = np.array([r["t"] for r in rows])
    v = np.array([r["v"] for r in rows])
    omega_f = np.array([r["omega_f"] for r in rows])
    lam = np.array([r["lambda_f_true"] for r in rows])
    F_f = np.array([r["F_f"] for r in rows])
    F_clamp = np.array([r["F_clamp"] for r in rows])

    distance = float(np.trapezoid(np.clip(v, 0.0, None), ts))
    stopped = np.where(v <= 1.0e-3)[0]
    t_stop = ts[stopped[0]] if stopped.size else float("nan")

    n_panels = 5 if phase == "a" else 6
    fig, axes = plt.subplots(n_panels, 1, figsize=(8.5, 1.8 * n_panels + 1), sharex=True)
    axes[0].plot(ts, v)
    axes[0].set_ylabel("v [m/s]")

    axes[1].plot(ts, omega_f * R_f, label=r"$\omega_f R_f$")
    axes[1].plot(ts, v, "--", alpha=0.5, label="v")
    axes[1].set_ylabel("rim speed [m/s]")
    axes[1].legend(loc="upper right")

    axes[2].plot(ts, lam, label=r"$\lambda_f^{\rm true}$")
    if phase == "b" and "lambda_f_hat" in rows[0]:
        lam_hat = np.array([r["lambda_f_hat"] for r in rows])
        axes[2].plot(ts, lam_hat, "--", alpha=0.7, label=r"$\hat\lambda_f$")
        axes[2].legend(loc="upper right")
    axes[2].set_ylabel(r"$\lambda_f$")

    axes[3].plot(ts, F_f)
    axes[3].set_ylabel(r"$F_f$ [N]")

    axes[4].plot(ts, F_clamp)
    axes[4].set_ylabel(r"$F_{\rm clamp}$ [N]")

    if phase == "b":
        V_pwm = np.array([r.get("V_pwm", 0.0) for r in rows])
        axes[5].plot(ts, V_pwm)
        axes[5].set_ylabel(r"$V_{\rm pwm}$ [V]")

    axes[-1].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)

    fig.suptitle(
        f"Phase {phase.upper()} panic stop — "
        f"stop @ {t_stop:.2f} s, distance {distance:.2f} m"
    )
    fig.tight_layout()
    out_path = REPO / "out" / "runs" / f"panic_stop_{phase}.png"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    return out_path


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("a", "b"), default="b")
    args = parser.parse_args()

    cfg = load_config(REPO / "configs" / "default.toml")
    rows = _run(args.phase, cfg)
    out_path = _plot(args.phase, cfg, rows)

    ts = np.array([r["t"] for r in rows])
    v = np.array([r["v"] for r in rows])
    lam = np.array([r["lambda_f_true"] for r in rows])
    F_f = np.array([r["F_f"] for r in rows])
    distance = float(np.trapezoid(np.clip(v, 0.0, None), ts))
    print(f"wrote {out_path}")
    print(
        f"final v = {v[-1]:.3f} m/s | peak λ = {lam.max():.3f} | "
        f"peak F_f = {F_f.max():.0f} N | distance ≈ {distance:.2f} m"
    )


if __name__ == "__main__":
    main()
