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

## Run Without Dashboard

From the repository root:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe `
  aviary/models/aircraft/horizontal_small_uav/run_horizontal_small_uav.py
```

Outputs are written to:

```text
outputs/run_horizontal_small_uav_out/
```

The terminal summary and the generated `payload_range_data.csv` report use SI units
such as `m`, `m**2`, `kg`, `N`, and `km`.

## Run With Dashboard

Run the case first:

```powershell
& C:/Software/Anaconda/envs/aviary/python.exe `
  aviary/models/aircraft/horizontal_small_uav/run_horizontal_small_uav.py
```

Then open the dashboard from the repository root after `outputs/run_horizontal_small_uav_out` has been generated:

```powershell
$env:PYTHONPATH = (Get-Location).Path
& C:/Software/Anaconda/envs/aviary/Scripts/aviary.exe dashboard outputs/run_horizontal_small_uav_out
```

The script runs a fallout range optimization with wing span and scaled SLS thrust as
design variables. The Dymos phase setup lives in `phase_info.py` so the runner stays
focused on problem setup, execution, and SI-unit reporting.
