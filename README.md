# Elevator Trial — Phinity Verilog HUD Task

16-floor, six-lift elevator control system with transit hall-call buffering, e-stop recovery, and load-aware dispatch.

Local development layout:

```
sources/   baseline (buggy) RTL — graded by HUD
tests/     cocotb/pytest suite
golden/    correct reference RTL (local dev only; not on HUD branches)
```

See `SPEC.md`, `TESTING.md`, and `prompt.txt` for design and task instructions.
