# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Python simulation study comparing an **ABS (Anti-lock Braking System)** against **cadence braking** on a **cargo e-bike**. The goal is to determine whether ABS provides meaningful benefit over cadence braking in this vehicle class.

The user has a planned block diagram and a partial set of governing equations; Claude's role is to help fill in missing equations, implement the simulation in Python, and support analysis of the two braking strategies.

## Status

Phase A of `PLAN.md` in progress — MVP plant + Dugoff tire + prescribed-clamp
brake. Phase B (actuator chain, Hall + estimator) and Phase C (ABS FSM,
cadence baseline, comparison) have not started.

## Layout

- `src/ebike_abs/block.py` — `Block` base class.
- `src/ebike_abs/simulator.py` — fixed-step RK4 loop, single source of truth
  for time advancement.
- `src/ebike_abs/blocks/` — one file per plant/sensor block.
- `src/ebike_abs/control/` — one file per controller (`human`, `abs_fsm`,
  `cadence`).
- `configs/default.toml` — all tunable parameters; scenarios are diffs on this.
- `tests/` — pytest; every block has a unit test.
- `scripts/run_*.py` — top-level scenarios and comparison runs.

## Commands

```bash
pytest                                          # all unit + integration tests
python scripts/run_panic_stop.py                # Phase A end-to-end run
python scripts/run_comparison.py                # Phase C: human/cadence/ABS (later)
ruff check . && ruff format --check .           # lint + format
```

Tooling: numpy/scipy/matplotlib + pytest + ruff, RK4 at `dt = 1e-4` s, TOML
configs via stdlib `tomllib` (Python ≥ 3.11).

## Working rules

1. **Commit cadence.** One commit per meaningful unit (block implemented,
   tests passing, parameter tuning). Short descriptive messages. Push to
   `origin main` after each commit.
2. **ASSUMPTIONS.md discipline.** Every new modelling choice gets an entry
   under the appropriate block heading in `ASSUMPTIONS.md` in the *same
   commit* that introduces it.
3. **Parameter discipline.** Any change to a value in `configs/default.toml`
   or the Parameter table in `PLAN.md` moves with an `ASSUMPTIONS.md` update
   in the same commit.
4. **Phase discipline.** Follow `PLAN.md` Implementation Order. Do not add a
   block from a later phase without finishing the current phase's checkpoint.
