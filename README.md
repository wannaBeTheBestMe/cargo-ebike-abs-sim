# cargo-ebike-abs-sim

1D simulation study comparing an **ABS** against **cadence braking** on a
cargo e-bike. Front-only braking; rear wheel is a free rolling speed
reference for the ABS estimator.

See `PLAN.md` for the full design and `ASSUMPTIONS.md` for the per-block
modelling log.

## Quickstart

```bash
# Create a virtualenv and install
python3 -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"

# Run tests
pytest

# Phase A/B single-scenario runners (saves a multi-panel plot to out/runs/)
python scripts/run_panic_stop.py --phase a
python scripts/run_panic_stop.py --phase b

# Phase C comparison: human vs cadence vs ABS (plot + metrics table)
python scripts/run_comparison.py
```

## Status

- [x] Phase A — MVP plant, Dugoff tire, prescribed clamp, forced-lock oracle
- [x] Phase B — motor + hydraulic actuator chain, Hall + wheel-speed estimator,
  human V_pwm baseline
- [x] Phase C — ABS FSM, cadence baseline, `scripts/run_comparison.py`

## Headline finding (dry asphalt, default scenario)

`python scripts/run_comparison.py` on `configs/default.toml`:

| controller | distance [m] | t_stop [s] | peak \|λ\| | lock-fraction | time with \|λ\|>0.5 |
|---|---:|---:|---:|---:|---:|
| human (Phase B) | 4.71 | 1.02 | 1.02 | 72 % | 76 % |
| cadence (2 Hz) | 7.81 | 1.73 | 1.02 | 18 % | 25 % |
| ABS FSM        | 7.22 | 1.52 | 1.02 | 12 % | 22 % |

On dry pavement the sliding-μ is close to the peak-μ, so the locked-wheel
slide gives the shortest stop *and* the most time at dangerous slip;
ABS and cadence both trade distance for wheel control. ABS is primarily
a low-μ story — the high-μ numbers above are the expected null result
for this vehicle class and the motivating frame for the study.

Note that the MVP ABS does **not** meet PLAN's peak-|λ| < 0.30 oracle:
the 20-magnet Hall + 4-sample MA chain has enough lag that the first
lockup cycle still spikes to λ ≈ 1 before DUMP fires. See
`ASSUMPTIONS.md` → `ABSController` §5 for the detailed breakdown and
what a Phase D sensor upgrade would need to change.

For the write-up discussion — why ABS doesn't win on high-μ, modelling
sensitivities, and the pitchover check against `g·ℓ_f/h = 4.91 m/s²` (all
three strategies cross it by ~1.8×) — see [docs/DISCUSSION.md](docs/DISCUSSION.md).

## Data flow

```mermaid
flowchart LR
    HUM[Human F_hand&#40;t&#41;]
    CTRL{Controller<br/>human / ABS / cadence}
    ACT[Motor+PWM+Hydraulic<br/>1st-order lag τ≈30ms]
    BRK[Brake Torque<br/>T_b = μ_pad F_clamp r_eff n]
    WHL[(ω_f integrator)]
    VEH[(v integrator)]
    NRM[Normal Load<br/>N_f = &#40;mga + m a_x h&#41;/L]
    TIRE[Dugoff Tire<br/>F_f&#40;λ, N_f&#41;]
    SLIPT[λ_true = &#40;v − ω_f R_f&#41;/v]

    HALL[Hall N=20 + jitter]
    EST[ω̂_f: edge→LPF→MA→central-diff]
    REAR[ω_r = v/R_r → v̂]
    SLIPE[λ̂_f]

    HUM --> CTRL
    CTRL -- V_pwm --> ACT --> BRK --> WHL
    TIRE -- F_f --> WHL
    TIRE -- F_f --> VEH
    VEH -- v --> SLIPT
    WHL -- ω_f --> SLIPT
    SLIPT --> TIRE
    VEH -- a_x --> NRM --> TIRE

    WHL -- ω_f --> HALL --> EST --> SLIPE
    VEH -- v --> REAR --> SLIPE
    SLIPE --> CTRL
    EST -- ω̇̂_f --> CTRL
```

For the fully-labeled Simulink-style block diagram — every signal annotated
with symbol, units, and sampling type (continuous / algebraic / discrete /
lag-1) — see [docs/BLOCK_DIAGRAM.md](docs/BLOCK_DIAGRAM.md).
