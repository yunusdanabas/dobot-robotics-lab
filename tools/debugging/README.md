# Debugging Tools

Use these scripts when the robot or simulator does not behave as expected.

| File | Purpose |
|------|---------|
| `robot_parity_diagnostic.py` | Compare expected robot poses against reported poses. |
| `robot_parity_gui.py` | GUI version of the parity diagnostic. |

**Interpreting Magician lockstep output (operators and agents):** canonical notes
live in the main repo
[`simulation/diagnostics/README.md`](../../../simulation/diagnostics/README.md)
(*Interpreting parity output (Magician notes)*) and [`AGENTS.md`](../../../AGENTS.md)
(*Robot parity diagnostic*). Short version: parallel **`ok`** means moves finished,
not poses within tolerance; **`(X,Y,Z)`** is the primary metric when sim EE matches
real EEPROM TCP; motor-in-sim-only yields **~60 mm** joint-suite offsets; **`R`** /
Cartesian **joint** lines reflect sim **`j4_deg=0` FK reporting** and IK branches.

Start with `scripts/check_magician.py` or `scripts/check_mg400.py` for basic
connection problems.

## Magician hybrid compare

When Magician sim-vs-real runs have a consistent baseline pose offset from
frame/end-effector differences, use hybrid compare mode:

```bash
python ME403_LabFiles/tools/debugging/robot_parity_diagnostic.py compare \
  ME403_LabFiles/simulation/diagnostics/results/magician_sim.jsonl \
  ME403_LabFiles/simulation/diagnostics/results/magician_real.jsonl \
  --magician-hybrid --anchor-case joint_zero --pos-tol 10 --joint-tol 5
```

Hybrid mode:
- Computes a baseline XYZ offset from the selected anchor case.
- Uses normalized XYZ (baseline-subtracted) plus joint tolerance for pass/fail.
- Ignores R in pass/fail scoring for Magician.
- Still prints raw `dxyz`/`dr` so you can inspect original differences.
