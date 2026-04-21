"""Phase A end-to-end panic stop.

Loads ``configs/default.toml``, wires the plant + Dugoff tire + prescribed
clamp source, runs a fixed-step RK4 simulation, and writes a 5-panel plot
of v / rim speed / λ / F_f / F_clamp to ``out/runs/panic_stop.png``.
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from ebike_abs.scenarios import build_phase_a_panic_stop, load_config

REPO = Path(__file__).resolve().parents[1]


def main() -> None:
    cfg = load_config(REPO / "configs" / "default.toml")
    sim = build_phase_a_panic_stop(cfg)
    rows = sim.run(cfg["scenario"]["t_end"])

    ts = np.array([r["t"] for r in rows])
    v = np.array([r["v"] for r in rows])
    omega_f = np.array([r["omega_f"] for r in rows])
    lam = np.array([r["lambda_f_true"] for r in rows])
    F_f = np.array([r["F_f"] for r in rows])
    F_clamp = np.array([r["F_clamp"] for r in rows])
    R_f = cfg["vehicle"]["R_f"]

    distance = float(np.trapezoid(np.clip(v, 0.0, None), ts))
    stopped = np.where(v <= 1.0e-3)[0]
    t_stop = ts[stopped[0]] if stopped.size else float("nan")

    fig, axes = plt.subplots(5, 1, figsize=(8.5, 10), sharex=True)
    axes[0].plot(ts, v)
    axes[0].set_ylabel("v [m/s]")
    axes[1].plot(ts, omega_f * R_f, label=r"$\omega_f R_f$")
    axes[1].plot(ts, v, "--", alpha=0.5, label="v")
    axes[1].set_ylabel("rim speed [m/s]")
    axes[1].legend(loc="upper right")
    axes[2].plot(ts, lam)
    axes[2].set_ylabel(r"$\lambda_f$")
    axes[3].plot(ts, F_f)
    axes[3].set_ylabel(r"$F_f$ [N]")
    axes[4].plot(ts, F_clamp)
    axes[4].set_ylabel(r"$F_{\rm clamp}$ [N]")
    axes[4].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(True, alpha=0.3)
    fig.suptitle(
        f"Phase A panic stop — stop @ {t_stop:.2f} s, distance {distance:.2f} m"
    )
    fig.tight_layout()

    out_path = REPO / "out" / "runs" / "panic_stop.png"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=120)
    print(f"wrote {out_path}")
    print(
        f"final v = {v[-1]:.3f} m/s | peak λ = {lam.max():.3f} | "
        f"peak F_f = {F_f.max():.0f} N | distance ≈ {distance:.2f} m"
    )


if __name__ == "__main__":
    main()
