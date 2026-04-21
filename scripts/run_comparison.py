"""Phase C comparison runner — human / cadence / ABS on the shared scenario.

Produces:

* ``out/runs/comparison.png`` — four stacked panels (v, ω_f R_f, λ_true, V_pwm)
  overlaying the three runs.
* stdout — a metrics table (stopping distance, stop time, peak λ, peak
  |a_x|, fraction of above-cutoff time with |λ| > 0.5).

Usage::

    python scripts/run_comparison.py
    python scripts/run_comparison.py --scenario configs/default.toml
"""

from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from ebike_abs.scenarios import (
    build_phase_b_panic_stop,
    build_phase_c_abs_panic_stop,
    build_phase_c_cadence_panic_stop,
    load_config,
)

REPO = Path(__file__).resolve().parents[1]


def _run_all(cfg: dict) -> dict[str, list[dict]]:
    t_end = cfg["scenario"]["t_end"]
    return {
        "human": build_phase_b_panic_stop(cfg).run(t_end),
        "cadence": build_phase_c_cadence_panic_stop(cfg).run(t_end),
        "abs": build_phase_c_abs_panic_stop(cfg).run(t_end),
    }


def _metrics(cfg: dict, rows: list[dict]) -> dict[str, float]:
    R_f = cfg["vehicle"]["R_f"]
    v_cutoff = cfg["controller"]["abs"]["v_cutoff"]
    ts = np.array([r["t"] for r in rows])
    v = np.array([r["v"] for r in rows])
    omega_f = np.array([r["omega_f"] for r in rows])
    lam = np.array([r["lambda_f_true"] for r in rows])
    # Stopping distance = integral of max(v, 0) over the full run.
    distance = float(np.trapezoid(np.clip(v, 0.0, None), ts))
    # Stop time: first sample under 0.2 m/s (matches test oracle).
    stopped_ix = np.where(v < 0.2)[0]
    t_stop = float(ts[stopped_ix[0]]) if stopped_ix.size else float("nan")
    # Peak a_x via finite difference on v.
    a_x = np.diff(v) / np.diff(ts)
    peak_decel = float(np.abs(a_x).max())
    # Lock fraction above v_cutoff: ω_f ≈ 0 relative to v/R_f.
    above = v > v_cutoff
    locked = (omega_f < 0.1 * v / R_f) & above
    lock_frac = float(locked.sum() / max(above.sum(), 1))
    # Time-fraction with |λ| > 0.5, above cutoff.
    high_slip = (np.abs(lam) > 0.5) & above
    high_slip_frac = float(high_slip.sum() / max(above.sum(), 1))
    return {
        "distance_m": distance,
        "t_stop_s": t_stop,
        "peak_lambda": float(np.abs(lam).max()),
        "peak_decel_mps2": peak_decel,
        "lock_frac": lock_frac,
        "high_slip_frac": high_slip_frac,
    }


def _plot(cfg: dict, runs: dict[str, list[dict]]) -> Path:
    R_f = cfg["vehicle"]["R_f"]
    fig, axes = plt.subplots(4, 1, figsize=(9.0, 9.5), sharex=True)
    colors = {"human": "#1f77b4", "cadence": "#ff7f0e", "abs": "#2ca02c"}
    labels = {"human": "human (Phase B)", "cadence": "cadence 2 Hz", "abs": "ABS FSM"}
    for name, rows in runs.items():
        ts = np.array([r["t"] for r in rows])
        v = np.array([r["v"] for r in rows])
        omega_f = np.array([r["omega_f"] for r in rows])
        lam = np.array([r["lambda_f_true"] for r in rows])
        V_pwm = np.array([r.get("V_pwm", 0.0) for r in rows])
        c = colors[name]
        lab = labels[name]
        axes[0].plot(ts, v, color=c, label=lab)
        axes[1].plot(ts, omega_f * R_f, color=c, label=lab)
        axes[2].plot(ts, lam, color=c, label=lab)
        axes[3].plot(ts, V_pwm, color=c, label=lab)

    # Reference line: v (truth) on the rim-speed panel using the ABS run.
    abs_rows = runs["abs"]
    axes[1].plot(
        [r["t"] for r in abs_rows],
        [r["v"] for r in abs_rows],
        "k--",
        alpha=0.4,
        label="v (truth, ABS run)",
    )

    axes[0].set_ylabel("v [m/s]")
    axes[1].set_ylabel(r"$\omega_f R_f$ [m/s]")
    axes[2].set_ylabel(r"$\lambda_f^{\rm true}$")
    axes[3].set_ylabel(r"$V_{\rm pwm}$ [V]")
    axes[-1].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")
    fig.suptitle("Phase C — human vs cadence vs ABS on dry asphalt panic stop")
    fig.tight_layout()
    out_path = REPO / "out" / "runs" / "comparison.png"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    return out_path


def _print_table(metrics: dict[str, dict[str, float]]) -> None:
    header = (
        f"{'controller':<10} {'dist [m]':>10} {'t_stop [s]':>12} "
        f"{'peak |λ|':>10} {'peak |a_x|':>12} {'lock frac':>12} {'|λ|>0.5':>10}"
    )
    print(header)
    print("-" * len(header))
    for name in ("human", "cadence", "abs"):
        m = metrics[name]
        print(
            f"{name:<10} {m['distance_m']:>10.3f} {m['t_stop_s']:>12.3f} "
            f"{m['peak_lambda']:>10.3f} {m['peak_decel_mps2']:>12.2f} "
            f"{m['lock_frac']:>12.1%} {m['high_slip_frac']:>10.1%}"
        )


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--scenario",
        type=Path,
        default=REPO / "configs" / "default.toml",
        help="path to scenario TOML (default: configs/default.toml)",
    )
    args = parser.parse_args()

    cfg = load_config(args.scenario)
    runs = _run_all(cfg)
    metrics = {name: _metrics(cfg, rows) for name, rows in runs.items()}
    out_path = _plot(cfg, runs)

    _print_table(metrics)
    print(f"\nwrote {out_path}")


if __name__ == "__main__":
    main()
