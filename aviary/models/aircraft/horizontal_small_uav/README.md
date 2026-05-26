# Horizontal Small UAV Aviary Example

This example is a lightweight conceptual small UAV with:

```text
wing
horizontal tail
small turbojet engine
small conventional vertical stabilizer for the standard FLOPS geometry path
```

It does not model an X-tail or H-tail. The vertical stabilizer is retained because the
current FLOPS/Dymos setup expects vertical-tail geometry parameters during mission setup.

## Files

```text
horizontal_small_uav.csv      Aircraft input deck
phase_info.py                 Dymos climb/cruise phase definitions
run_horizontal_small_uav.py   Optimization runner
README.md                     This guide
```

## Run

From the repository root:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe `
  aviary/models/aircraft/horizontal_small_uav/run_horizontal_small_uav.py
```

Outputs are written to:

```text
run_horizontal_small_uav_out/
```

The script runs a fallout range optimization with wing span and scaled SLS thrust as
design variables. The Dymos phase setup lives in `phase_info.py` so the runner stays
focused on problem setup, execution, and reporting.
