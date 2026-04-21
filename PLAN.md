# Cargo E-Bike ABS vs Cadence Braking — Simulation Study

## Context

The user is conducting a 1D simulation study to investigate whether an **ABS** system provides meaningful benefit over **cadence braking** on a **cargo e-bike**. They have a block diagram, partial equations, and external review feedback identifying gaps. Claude is to fill in the remaining equations, implement the simulation in Python, and maintain the project on GitHub with regular commits.

**Key modeling assumption:** front-only braking. Rear wheel has slip ratio = 0 and rolls at $v / R_r$, making it a free speed reference for the ABS estimator.

## Repository

- **Name:** `cargo-ebike-abs-sim`
- **Owner:** `wannaBeTheBestMe`
- **URL:** https://github.com/wannaBeTheBestMe/cargo-ebike-abs-sim

## Libraries

| Library | Purpose | Rationale |
|---|---|---|
| `numpy` | arrays, numerics | standard |
| `scipy` | `solve_ivp`, signal utils | standard |
| `matplotlib` | plotting | standard |
| `graphviz` (Python pkg) | auto-render block diagram | satisfies nice-to-have |
| `pytest` | unit tests | standard |
| `ruff` | lint + format | lightweight |

Fixed-step RK4 at `dt = 1e-4 s` (0.1 ms) is the chosen integrator. Rationale: hybrid dynamics (discrete ABS state machine + Hall pulse emulation + moving-average filter) make variable-step solvers awkward; RK4 at 0.1 ms is stable for the ~20–50 ms hydraulic lag and the ~ms-scale wheel lockup dynamics flagged in the review.

## File Layout

```
cargo-ebike-abs-sim/
├── CLAUDE.md
├── ASSUMPTIONS.md              # per-block assumption headings
├── README.md
├── pyproject.toml              # deps + ruff config
├── .gitignore
├── src/
│   └── ebike_abs/
│       ├── __init__.py
│       ├── block.py            # Block base class + registry
│       ├── simulator.py        # fixed-step RK4 loop, logging, scheduling
│       ├── diagram.py          # graphviz rendering from Block registry
│       ├── blocks/
│       │   ├── vehicle.py      # VehicleTranslation (integrator, v)
│       │   ├── wheel.py        # FrontWheelRotation, RearWheelKinematics
│       │   ├── sensor.py       # HallSensor, WheelSpeedEstimator
│       │   ├── slip.py         # SlipRatioTrue, SlipRatioEstimated
│       │   ├── normal_load.py  # moment-balance N_f
│       │   ├── tire.py         # BrushTireModel (time-varying λ_crit)
│       │   ├── actuator.py     # Motor+PWM+Hydraulic (1st-order lag)
│       │   └── brake.py        # BrakeTorqueComputation
│       └── control/
│           ├── human.py        # human brake force profile
│           ├── abs_fsm.py      # apply/dump/hold/reapply state machine
│           └── cadence.py      # cadence-braking baseline
├── scripts/
│   ├── run_scenario.py         # single run, config-driven
│   ├── run_comparison.py       # ABS vs cadence vs no-ABS, plots + metrics
│   └── regenerate_diagram.py   # writes diagram.svg/png
├── configs/
│   └── default.yaml            # scenario + vehicle params
├── tests/
│   ├── test_tire.py
│   ├── test_sensor.py
│   ├── test_slip.py
│   └── test_integration.py     # end-to-end stopping-distance sanity check
└── out/                        # simulation outputs (gitignored)
    ├── diagram.svg
    └── runs/
```

## Architecture

### `Block` base class (`block.py`)

A lightweight block abstraction with declared `inputs` and `outputs` so the diagram renderer can wire blocks automatically:

```python
class Block:
    name: str
    inputs: list[str]      # names of signals consumed
    outputs: list[str]     # names of signals produced
    state_dim: int = 0     # continuous state size
    def derivatives(self, t, x, u) -> np.ndarray: ...   # for integrators
    def output(self, t, x, u) -> dict[str, float]: ...  # algebraic outputs
    def discrete_update(self, t, u, state) -> state: ...# for FSM / MA buffer
```

Continuous states go into the global RK4 vector; discrete states (ABS FSM, MA ring buffers, Hall edge timestamps) live on the block and update at each fixed step.

### Simulator (`simulator.py`)

- Fixed-step RK4 for continuous states (`v`, `ω_f`, hydraulic pressure, motor current).
- At each step: (1) algebraic evaluation in topological order, (2) discrete updates (FSM, Hall, MA), (3) RK4 substep for continuous states.
- Structured logging via a `SimLog` dataclass; one row per step with all relevant signals for post-hoc plotting and diffing between ABS and cadence runs.

### Diagram generation (`diagram.py`)

Every `Block` registers its inputs/outputs. `diagram.py` walks the registry and emits a graphviz `Digraph` (block → output → consumer). `regenerate_diagram.py` is idempotent — run after any block-level code change. Output committed to `out/diagram.svg`.

## Block-by-Block Design (corrections from review incorporated)

### 1. `VehicleTranslation`
- State: `v` (integrator).
- Equation: $m\dot v = -F_f - F_{\text{drag}}(v) - F_{\text{rr}}$.
- Drag and rolling resistance **off by default** (configurable); see ASSUMPTIONS.

### 2. `FrontWheelRotation`
- State: `ω_f` (integrator — **not** a Δω sum, per review).
- Equation: $I_f \dot\omega_f = F_f R_f - T_b$.
- Sign convention: $F_f > 0$ is rearward on the bike (decelerates $v$) and forward on the wheel rim (decelerates $\omega_f$ when braking).

### 3. `RearWheelKinematics`
- Algebraic only: $\omega_r = v / R_r$. No integrator (per review point 3 — $I_r \dot\omega_r$ reaction ignored, no rear brake).

### 4. `HallSensor`
- $N = 20$ magnets/rev → 18° resolution.
- Generates edge timestamps by accumulating $\omega_f$; emits an edge when $\int \omega_f \,dt$ crosses each $2\pi/N$ boundary.
- Optional Gaussian jitter on timestamps (few % of pulse interval), and configurable missed-edge probability.

### 5. `WheelSpeedEstimator`
- Edge-to-edge $\hat\omega = (2\pi/N)/\Delta t_{\text{edge}}$ on each new edge; ZOH between edges.
- First-order LPF, then moving-average (window 3–5 samples, configurable).
- Central-difference derivative $\dot{\hat\omega}$ (not forward Euler) per review point 5.

### 6. `SlipRatioTrue` (plant-side)
- $\lambda_f^{\text{true}} = (v - \omega_f R_f) / \max(v, v_\epsilon)$, using ground-truth $v$ and $\omega_f$. **Feeds the tire model.**

### 7. `SlipRatioEstimated` (controller-side)
- $\hat v = \omega_{r,\text{meas}} R_r$ (exact in our scenario since $\lambda_r = 0$).
- $\hat\lambda_f = (\hat v - \omega_{f,\text{meas}} R_f) / \max(\hat v, v_\epsilon)$. **Feeds the ABS FSM.**
- Explicitly resolves review point 1.

### 8. `NormalLoad`
- Moment balance about rear contact: $N_f = \big(m g a + m a_x h\big) / L$ where $a = $ rear-to-CG distance, $L = $ wheelbase, $h = $ CG height, $a_x = \dot v$.
- Clamped to $[0, mg]$; the bike is near stoppie threshold under hard braking and we flag (not enforce) $N_r \geq 0$.

### 9. `BrushTireModel`
- Piecewise (review point 4):
  - Linear: $F_f = C_x \lambda_f$ for $|\lambda_f| \leq \lambda_{\text{crit}}$.
  - Saturated: brush-model expression for $|\lambda_f| > \lambda_{\text{crit}}$.
- $\lambda_{\text{crit}} = \mu_{\text{peak}} N_f / C_x$ **recomputed every step** since $N_f$ is time-varying.

### 10. `MotorActuator` (PWM → DC motor → piston → hydraulic → clamping)
- DC motor: $L \dot i = V_{\text{pwm}} - R i - K_e \omega_m$; $J_m \dot\omega_m = K_t i - T_{\text{load}}$.
- Piston force = motor torque × lever ratio; Pascal's-law amplification to caliper clamping force.
- **First-order lag** on clamping force output, $\tau \approx 20$–50 ms (review point 7), to represent fluid compressibility + line compliance + pad take-up. Configurable.

### 11. `BrakeTorqueComputation`
- $T_b = \mu_{\text{pad}} F_{\text{clamp}} r_{\text{eff}} \cdot n_{\text{pads}}$ (typically 2).

### 12. `BrakingController`
Two interchangeable controllers:

**(a) Human baseline (`human.py`)** — steady hydraulic force ramp defined by a simulation-input profile $F_{\text{hand}}(t)$. Scenario sets ramp rate and hold value.

**(b) ABS FSM (`abs_fsm.py`)** — finite state machine per review point 6:
```
APPLY  ── trigger (λ̂ > 0.2 AND ω̇_f < threshold) ──►  DUMP
DUMP   ── ω̇_f > 0 AND λ̂ < 0.05 ──►  HOLD
HOLD   ── dwell timer expires     ──►  REAPPLY
REAPPLY── ramp pressure toward human command, retrigger on new lockup
APPLY  ── v < v_cutoff (~5 km/h)  ──►  BYPASS (let bike lock)
```
Thresholds configurable. Default $\dot\omega_{\text{trig}} = -100$ rad/s², $\lambda_{\text{on}} = 0.2$, $\lambda_{\text{off}} = 0.05$, $v_{\text{cutoff}} = 1.4$ m/s.

**(c) Cadence baseline (`cadence.py`)** — square-wave on/off chopping of human brake force at configurable frequency (default 2 Hz, 50% duty), simulating a human pumping the lever.

### Initial conditions
- $v(0) = v_0$ (default 30 km/h); $\omega_f(0) = \omega_r(0) = v_0 / R$.
- Hydraulic pressure zero; motor current zero; ABS FSM in `APPLY`.

## ASSUMPTIONS.md structure

One `##` heading per block, each listing numbered assumptions with rationale. Top-level sections:
- Global (single-track 1D, planar, rigid cargo mass, front-only braking)
- Per-block sections mirroring the block list above
- Controller / sensor sections
- Scenario defaults

Updated in the same commit as any block change that introduces a new assumption.

## CLAUDE.md additions

Append:
1. **Commit cadence:** commit after each meaningful unit of work (block implemented, tests passing, diagram regenerated, parameter tuning), with short descriptive messages. Push to `origin main` after each commit.
2. **ASSUMPTIONS.md discipline:** every new modeling choice gets an entry under the appropriate block heading in `ASSUMPTIONS.md` in the *same commit* that introduces the choice.
3. **Diagram regeneration:** run `python scripts/regenerate_diagram.py` and commit `out/diagram.svg` whenever block inputs/outputs change.
4. Library/tooling pointer (scipy/numpy/matplotlib/graphviz, RK4 @ 1e-4 s, pytest).

## Implementation Order (commit-per-step)

1. Scaffolding: `pyproject.toml`, `.gitignore`, empty package, CI-free setup. → initial commit.
2. `Block` base + `Simulator` skeleton (runs a trivial 1-state ODE end-to-end). → test → commit.
3. Plant blocks (vehicle, front wheel, rear kinematics, normal load). → test against closed-form coast-down → commit.
4. Brush tire model w/ time-varying $\lambda_{\text{crit}}$. → unit test saturation curve → commit.
5. Actuator chain (motor + PWM + hydraulic lag + brake torque). → step-response test → commit.
6. Hall + wheel-speed estimator (with jitter, MA, central-diff derivative). → test against known $\omega$ sweep → commit.
7. Slip calcs (true + estimated). → test with $\lambda_r=0$ invariant → commit.
8. Human baseline controller + end-to-end panic-stop run. → plots → commit.
9. ABS FSM + integration test. → commit.
10. Cadence controller. → commit.
11. Comparison script + metrics (stopping distance, peak slip, peak deceleration, lock-up time fraction). → commit.
12. `diagram.py` + `regenerate_diagram.py`. → commit `out/diagram.svg` → commit.
13. `ASSUMPTIONS.md` full pass, `CLAUDE.md` update, `README.md`. → commit.

Sub-agents will be used in parallel for independent block implementations (e.g., tire model + actuator chain can be built concurrently after the `Block` base is in place).

## Critical Files

- `src/ebike_abs/block.py` — base class; all blocks inherit.
- `src/ebike_abs/simulator.py` — fixed-step RK4 loop; single source of truth for time advancement.
- `src/ebike_abs/diagram.py` — auto-renders from the block registry; must be kept in sync with block I/O.
- `configs/default.yaml` — all tunable parameters in one place; scenarios are diffs on this.
- `ASSUMPTIONS.md` — gating document; updates accompany every modeling choice.

## Verification

1. **Unit tests** — `pytest tests/` covers tire saturation, sensor filter response, slip-ratio invariants, actuator step response.
2. **Sanity sims** (scripted, automated):
   - No-brake coast-down matches $v(t) = v_0$ (flat, no drag) to within integrator tolerance.
   - Locked-wheel ($\omega_f = 0$ forced) gives $F_f = \mu_{\text{peak}} N_f$ and stopping distance matches $v_0^2/(2 \mu g)$.
   - ABS simulation: peak slip stays below 0.3; wheel never sustains $\omega_f = 0$ for more than one dump cycle.
3. **Comparison run** — `python scripts/run_comparison.py --scenario configs/default.yaml` produces side-by-side plots of $v$, $\omega_f$, $\lambda_f$, $F_f$, brake state for human / cadence / ABS, plus stopping-distance table.
4. **Diagram** — `python scripts/regenerate_diagram.py` writes `out/diagram.svg`; visually confirm wiring matches the user's block diagram.

## Open items (defaults chosen, user can override)

- **Cadence-brake model:** square-wave chopping of human force at 2 Hz, 50% duty. Can be swapped for a sawtooth or human-measured profile later.
- **Scenario defaults:** $v_0 = 30$ km/h, dry asphalt $\mu_{\text{peak}} = 0.9$, rider + cargo mass 120 kg, wheelbase 1.2 m, CG height 1.0 m, $R_f = R_r = 0.33$ m. All in `configs/default.yaml` — easy to sweep.
- **Drag / rolling resistance:** off by default; one line to enable in config.
