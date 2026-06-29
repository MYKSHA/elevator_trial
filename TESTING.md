# Running Elevator Tests

This task uses three HUD branches: `elevator_trial_baseline`, `elevator_trial_test`, and `elevator_trial_golden`.

The baseline branch contains the starting implementation in `sources/` with hidden defects. The `tests/` folder is intentionally empty on baseline and golden.

Use the `elevator_trial_test` branch for the populated test suite.

## Layout

```
sources/          RTL under test (baseline / buggy implementation)
tests/            Empty on baseline and golden; full suite on test branch
pyproject.toml    Python dependencies (cocotb, pytest)
SPEC.md           Design specification
```

## Setup

```bash
uv sync
```
