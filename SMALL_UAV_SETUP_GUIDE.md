# Small UAV Configuration Setup Guide

## Your Requirements
- **Length**: 2.0 m
- **Wingspan**: 1.8 m  
- **Engine**: 1 Turbojet
- **Cargo**: ~2-3 kg
- **Range**: Maximize for range (to be determined)

## Key Configuration Steps

### 1. **Create Aircraft Model Directory**

```
aviary/models/aircraft/small_uav/
├── small_uav.csv              (Configuration file)
├── run_small_uav_mission.py   (Main script)
└── README.md                  (Documentation)
```

### 2. **Configuration Parameters to Set**

#### Fuselage (Dimensions)
- `aircraft:fuselage:length` = 2.0 m
- `aircraft:fuselage:max_width` = 0.3 m (estimate)
- `aircraft:fuselage:max_height` = 0.25 m (estimate)
- `aircraft:fuselage:wetted_area` = 1.5-2.0 m² (calculated from dimensions)

#### Wing
- `aircraft:wing:span` = 1.8 m
- `aircraft:wing:area` = 0.4-0.6 m² (typical for this span, depends on chord)
- `aircraft:wing:aspect_ratio` = ~5-7 (typical for efficient UAVs)
- `aircraft:wing:sweep` = 0-5 deg (minimal for small UAVs)

#### Engines
- `aircraft:engine:num_engines` = 1
- `aircraft:engine:num_wing_engines` = 0
- `aircraft:engine:num_fuselage_engines` = 1
- `aircraft:engine:data_file` = `models/engines/turbofan_22k.csv` (or turbofan_24k.csv)
- `aircraft:engine:scale_factor` = 0.01-0.02 (SCALE DOWN significantly)

#### Payload/Cargo
- `aircraft:crew_and_payload:misc_cargo` = 2.0-3.0 kg

#### Design Parameters
- `aircraft:design:cruise_altitude` = 3000-5000 m (typical for small UAVs)
- `aircraft:design:landing_to_takeoff_mass_ratio` = 0.92
- `aircraft:design:touchdown_mass_max` = 8-10 kg (very light)

### 3. **Engine Selection**

For a small turbojet, you'll need to **scale down** an existing turbofan database:
- Start with: `turbofan_22k.csv` (22,000 lbf reference engine)
- Use `scale_factor` to scale thrust down to ~500-1000 lbf for your UAV

The scale factor is: `New Thrust / Reference Thrust`

Example: 
```
Target Thrust = 800 lbf
Reference Thrust = 22,000 lbf
Scale Factor = 800 / 22,000 = 0.0364
```

### 4. **Estimated Aircraft Mass**

For your small UAV (without knowing exact details):
- **Structure** (fuselage, wings, tail): 2-3 kg
- **Engine**: 1-1.5 kg (small turbojet)
- **Fuel System**: 0.3-0.5 kg
- **Avionics/Controls**: 0.2-0.3 kg
- **Cargo**: 2-3 kg
- **Total OEW** (Operating Empty Weight): ~6-8 kg
- **Max Takeoff Weight**: ~8-12 kg with fuel

### 5. **Fuel Capacity for Range Optimization**

For maximizing range, you need:
- `aircraft:fuel:total_capacity` = 3-5 kg (depends on endurance target)
- `aircraft:fuel:wing_fuel_fraction` = 0.8
- `aircraft:design:cruise_altitude` = optimize for altitude where small turbojet is efficient

## Next Steps

1. **Copy and modify small_cargo example** → small_uav configuration
2. **Set geometric parameters** based on your actual UAV design
3. **Calibrate engine scaling** based on your actual engine specs
4. **Run pre-mission analysis** to verify mass breakdown
5. **Set up range maximization** optimization problem

## Files to Modify/Create

1. `small_uav.csv` - Core configuration (copy from small_cargo_FLOPS.csv)
2. `run_small_uav_mission.py` - Problem setup script
3. Update engine data file or create custom scaling

## Important Notes

⚠️ **Mass Estimation**: Before running optimization, you should:
- Know your actual engine mass/thrust specs
- Define structural assumptions (carbon fiber? aluminum?)
- Clarify avionics and control system mass

⚠️ **Turbojet Efficiency**: Turbojets are less efficient than turbofans at cruise. For small UAVs maximizing range, consider:
- Operating at optimal cruise altitude (likely 5,000-10,000 ft)
- Using high-altitude cruise for better efficiency
- Small UAVs with turbojets are typically not optimal for long range

⚠️ **Scale Factor Selection**: Scaling down large engines can be inaccurate for very small engines. You may need:
- Custom engine deck for your specific turbojet
- Empirical correction factors
- Or a different modeling approach

## Resources in Codebase

- Aircraft variables: `aviary/variable_info/variables.py`
- Engine models: `aviary/models/engines/`
- Examples: 
  - `aviary/models/aircraft/small_cargo/`
  - `aviary/models/aircraft/small_single_aisle/`
- API: `aviary/api.py`
