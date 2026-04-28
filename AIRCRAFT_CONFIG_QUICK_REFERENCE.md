# Aviary Aircraft Configuration - Quick Reference

## File Locations

| Purpose | Location |
|---------|----------|
| **Variable Definitions** | `aviary/variable_info/variables.py` |
| **Small Cargo Example** | `aviary/models/aircraft/small_cargo/small_cargo_FLOPS.csv` |
| **Large Jet Example** | `aviary/models/aircraft/large_single_aisle_2/large_single_aisle_2_FLOPS_data.py` |
| **Engine Data Files** | `aviary/models/engines/turbofan_*.csv` |
| **All Aircraft Models** | `aviary/models/aircraft/<aircraft_name>/` |

---

## 1. Wing Dimensions

```csv
aircraft:wing:span,18.0,m                          # Tip-to-tip span
aircraft:wing:area,40.0,m**2                       # Planform area
aircraft:wing:aspect_ratio,8.1,unitless            # span²/area
aircraft:wing:sweep,25.0,deg                       # Sweep angle
aircraft:wing:taper_ratio,0.237,unitless           # Tip/root chord
aircraft:wing:thickness_to_chord,0.131,unitless    # t/c ratio
```

---

## 2. Fuselage Dimensions

```csv
aircraft:fuselage:length,15.0,m                    # Total length
aircraft:fuselage:max_width,2.0,m                  # Width
aircraft:fuselage:max_height,2.2,m                 # Height
aircraft:fuselage:passenger_compartment_length,8.0,m
aircraft:fuselage:num_fuselages,1,unitless
aircraft:fuselage:wetted_area,90.0,m**2
```

---

## 3. Engine Configuration

```csv
# Engine Count
aircraft:engine:num_engines,2,unitless
aircraft:engine:num_wing_engines,2,unitless        # Wings
aircraft:engine:num_fuselage_engines,0,unitless    # Fuselage (rear)

# Engine Type & Performance
aircraft:engine:data_file,models/engines/turbofan_22k.csv,unitless
aircraft:engine:reference_mass,2855.8,kg
aircraft:engine:reference_sls_thrust,98752.0,N
aircraft:engine:scale_factor,0.13514,unitless
aircraft:engine:scale_mass,True,unitless

# Placement
aircraft:engine:wing_locations,0.35,unitless       # Fraction of semispan
```

---

## 4. Cargo & Payload

### For Cargo Aircraft:
```csv
aircraft:crew_and_payload:misc_cargo,1500.0,kg     # Cargo mass
aircraft:crew_and_payload:wing_cargo,0,kg
aircraft:crew_and_payload:num_passengers,0,unitless
aircraft:crew_and_payload:num_flight_crew,2,unitless
```

### For Passenger Aircraft:
```csv
aircraft:crew_and_payload:num_passengers,162,unitless
aircraft:crew_and_payload:num_first_class,12,unitless
aircraft:crew_and_payload:num_business_class,0,unitless
aircraft:crew_and_payload:num_economy_class,150,unitless
aircraft:crew_and_payload:num_flight_crew,2,unitless
aircraft:crew_and_payload:num_flight_attendants,5,unitless
aircraft:crew_and_payload:mass_per_passenger,165.0,lbm
aircraft:crew_and_payload:baggage_mass_per_passenger,35.0,lbm
```

---

## 5. Python Code Patterns

### Load from CSV:
```python
import aviary.api as av

phase_info = {
    "cruise": {"user_options": {"num_segments": 1}}
}

prob = av.AviaryProblem()
prob.load_inputs("path/to/aircraft.csv", phase_info)
prob.check_and_preprocess_inputs()
prob.build_model()
prob.setup()
prob.run_model()
```

### Set Values Programmatically:
```python
from aviary.utils.aviary_values import AviaryValues
from aviary.variable_info.variables import Aircraft

inputs = AviaryValues()
inputs.set_val(Aircraft.Wing.SPAN, 18.0, 'm')
inputs.set_val(Aircraft.Wing.AREA, 40.0, 'm**2')
inputs.set_val(Aircraft.Fuselage.LENGTH, 15.0, 'm')
inputs.set_val(Aircraft.Engine.NUM_ENGINES, 2)
inputs.set_val(Aircraft.CrewPayload.MISC_CARGO, 1500.0, 'kg')
```

---

## 6. Example Aircraft Specs

### Small Cargo (18m span)
| Parameter | Value | Units |
|-----------|-------|-------|
| Gross Mass | 8000 | kg |
| Wingspan | 18.0 | m |
| Wing Area | 40.0 | m² |
| Length | 15.0 | m |
| Engines | 2× turbofan_22k | - |
| Cargo | 1500 | kg |
| Fuel | 2500 | kg |

### Large Jet (112ft span)
| Parameter | Value | Units |
|-----------|-------|-------|
| Gross Mass | 174200 | lbm |
| Wingspan | 112.57 | ft |
| Wing Area | 1341 | ft² |
| Length | 124.75 | ft |
| Engines | 2× turbofan_24k | - |
| Passengers | 162 | pax |
| Fuel | 46063 | lbm |

---

## 7. CSV Format Template

**Filename**: `aircraft_name_FLOPS.csv`

```csv
# Aircraft name and description
# Key specs: gross mass, wingspan, engines, payload

# Section Header (informational)
aircraft:wing:span,VALUE,UNITS
aircraft:wing:area,VALUE,UNITS
aircraft:fuselage:length,VALUE,UNITS
aircraft:fuselage:max_width,VALUE,UNITS
aircraft:engine:num_engines,VALUE,unitless
aircraft:engine:data_file,models/engines/turbofan_XXk.csv,unitless
aircraft:crew_and_payload:misc_cargo,VALUE,kg
aircraft:design:gross_mass,VALUE,kg
```

**Units**: m, ft, m**2, ft**2, kg, lbm, N, lbf, deg, unitless

---

## 8. Creating New Aircraft

### Quick Steps:
1. **Copy template**: `cp small_cargo_FLOPS.csv my_aircraft_FLOPS.csv`
2. **Edit key values**: wing span/area, fuselage length/width, engines, cargo
3. **Load in code**: `prob.load_inputs("path/my_aircraft_FLOPS.csv", phase_info)`
4. **Run**: `prob.run_model()`

### Key Checks:
- ✓ `NUM_ENGINES = NUM_WING_ENGINES + NUM_FUSELAGE_ENGINES`
- ✓ Engine data file exists: `aviary/models/engines/turbofan_*.csv`
- ✓ All dimensions physically reasonable
- ✓ Cargo/passenger counts make sense for fuselage

---

## 9. Variable Naming Pattern

All variables follow: `aircraft:subsystem:parameter`

**Examples**:
- `aircraft:wing:span` → `Aircraft.Wing.SPAN`
- `aircraft:fuselage:length` → `Aircraft.Fuselage.LENGTH`
- `aircraft:engine:num_engines` → `Aircraft.Engine.NUM_ENGINES`
- `aircraft:crew_and_payload:misc_cargo` → `Aircraft.CrewPayload.MISC_CARGO`

**Find variables**: See [aviary/variable_info/variables.py](../aviary/variable_info/variables.py)

---

## 10. Available Engine Types

**Location**: `aviary/models/engines/`

| File | Thrust | Notes |
|------|--------|-------|
| `turbofan_22k.csv` | ~22,000 lbf | Small regional jets |
| `turbofan_24k_1.csv` | ~24,000 lbf | Narrow-body commercial |
| `turbofan_28k_with_electric.csv` | ~28,000 lbf | Hybrid capability |

**Engine scaling**: Set `scale_factor` to adjust thrust/mass for specific aircraft

---

## See Also

- Full Guide: [AIRCRAFT_CONFIGURATION_GUIDE.md](AIRCRAFT_CONFIGURATION_GUIDE.md)
- Variable Reference: [aviary/variable_info/variables.py](aviary/variable_info/variables.py)
- Example Aircraft: [aviary/models/aircraft/](aviary/models/aircraft/)

