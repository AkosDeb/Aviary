# Aviary Aircraft Configuration Guide

## Quick Reference

This guide explains how to configure, define, and modify aircraft models in the Aviary framework.

---

## 1. Aircraft Configuration Methods

### A. CSV Configuration Files (Primary Method)

Aircraft configurations are stored as CSV files with three columns: variable name, value, and units.

**Location**: `aviary/models/aircraft/<aircraft_name>/<aircraft_name>.csv`

**Example**: [aviary/models/aircraft/small_cargo/small_cargo_FLOPS.csv](aviary/models/aircraft/small_cargo/small_cargo_FLOPS.csv)

**Loading in Python**:
```python
import aviary.api as av

phase_info = {
    "cruise": {
        "user_options": {
            "num_segments": 1,
            "order": 3,
        }
    }
}

prob = av.AviaryProblem()
prob.load_inputs("aviary/models/aircraft/small_cargo/small_cargo_FLOPS.csv", phase_info)
prob.check_and_preprocess_inputs()
prob.build_model()
prob.setup()
```

### B. Python Data Structure Method

Define aircraft programmatically using `AviaryValues`:

**Example**: [aviary/models/aircraft/large_single_aisle_2/large_single_aisle_2_FLOPS_data.py](aviary/models/aircraft/large_single_aisle_2/large_single_aisle_2_FLOPS_data.py)

```python
from aviary.utils.aviary_values import AviaryValues
from aviary.variable_info.variables import Aircraft

# Create data structure
aircraft_data = {}
inputs = aircraft_data['inputs'] = AviaryValues()
outputs = aircraft_data['outputs'] = AviaryValues()

# Set values
inputs.set_val(Aircraft.Wing.SPAN, 112.57, 'ft')
inputs.set_val(Aircraft.Wing.AREA, 1341.0, 'ft**2')
inputs.set_val(Aircraft.Fuselage.LENGTH, 124.75, 'ft')
inputs.set_val(Aircraft.Engine.NUM_ENGINES, 2)
```

---

## 2. Aircraft Dimensions Configuration

All dimension variables use `Aircraft.<Component>.*` naming pattern.

### Wing Dimensions
| Variable | Description | Example | Units |
|----------|-------------|---------|-------|
| `Aircraft.Wing.SPAN` | Wing tip-to-tip span | 18.0 | m |
| `Aircraft.Wing.AREA` | Wing planform area | 40.0 | m**2 |
| `Aircraft.Wing.ASPECT_RATIO` | Wing aspect ratio (span²/area) | 8.1 | unitless |
| `Aircraft.Wing.SWEEP` | Wing sweep at 25% chord | 25.03 | deg |
| `Aircraft.Wing.TAPER_RATIO` | Tip chord / root chord | 0.237 | unitless |
| `Aircraft.Wing.THICKNESS_TO_CHORD` | Average t/c ratio | 0.131 | unitless |
| `Aircraft.Wing.DIHEDRAL` | Wing dihedral angle | 5.0 | deg |

**CSV Example**:
```
aircraft:wing:span,18.0,m
aircraft:wing:area,40.0,m**2
aircraft:wing:aspect_ratio,8.1,unitless
aircraft:wing:sweep,25.0,deg
aircraft:wing:taper_ratio,0.237,unitless
aircraft:wing:thickness_to_chord,0.131,unitless
```

### Fuselage Dimensions
| Variable | Description | Example | Units |
|----------|-------------|---------|-------|
| `Aircraft.Fuselage.LENGTH` | Total fuselage length | 15.0 | m |
| `Aircraft.Fuselage.MAX_WIDTH` | Maximum fuselage width | 2.0 | m |
| `Aircraft.Fuselage.MAX_HEIGHT` | Maximum fuselage height | 2.2 | m |
| `Aircraft.Fuselage.NUM_FUSELAGES` | Number of fuselages | 1 | unitless |
| `Aircraft.Fuselage.PASSENGER_COMPARTMENT_LENGTH` | Cabin usable length | 8.0 | m |
| `Aircraft.Fuselage.WETTED_AREA` | Fuselage wetted area | 90.0 | m**2 |

**CSV Example**:
```
aircraft:fuselage:length,15.0,m
aircraft:fuselage:max_width,2.0,m
aircraft:fuselage:max_height,2.2,m
aircraft:fuselage:passenger_compartment_length,8.0,m
aircraft:fuselage:num_fuselages,1,unitless
aircraft:fuselage:wetted_area,90.0,m**2
```

### Tail Dimensions (Horizontal & Vertical)
| Variable | Description | Example | Units |
|----------|-------------|---------|-------|
| `Aircraft.HorizontalTail.AREA` | Horizontal stabilizer area | 10.0 | m**2 |
| `Aircraft.HorizontalTail.ASPECT_RATIO` | HStab aspect ratio | 4.5 | unitless |
| `Aircraft.VerticalTail.AREA` | Vertical stabilizer area | 7.0 | m**2 |
| `Aircraft.VerticalTail.ASPECT_RATIO` | VStab aspect ratio | 1.5 | unitless |
| `Aircraft.VerticalTail.NUM_TAILS` | Number of vertical tails | 1 | unitless |

---

## 3. Engine Configuration

### Engine Count and Placement
| Variable | Description | Example | Notes |
|----------|-------------|---------|-------|
| `Aircraft.Engine.NUM_ENGINES` | Total number of engines | 2 | Must match wing + fuselage |
| `Aircraft.Engine.NUM_WING_ENGINES` | Engines on wings | 2 | Typical for commercial |
| `Aircraft.Engine.NUM_FUSELAGE_ENGINES` | Engines on fuselage | 0 | Rear-mounted (some military) |
| `Aircraft.Engine.WING_LOCATIONS` | Wing mount location | 0.35 | Fraction of semispan |

**CSV Example** (2-engine configuration):
```
aircraft:engine:num_engines,2,unitless
aircraft:engine:num_wing_engines,2,unitless
aircraft:engine:num_fuselage_engines,0,unitless
aircraft:engine:wing_locations,0.35,unitless
```

### Engine Type and Performance Data

| Variable | Description | Example | Units |
|----------|-------------|---------|-------|
| `Aircraft.Engine.DATA_FILE` | Engine performance file path | models/engines/turbofan_22k.csv | path |
| `Aircraft.Engine.REFERENCE_MASS` | Baseline engine dry mass | 2855.8 | kg |
| `Aircraft.Engine.REFERENCE_SLS_THRUST` | Sea-level static thrust (baseline) | 98752.0 | N |
| `Aircraft.Engine.SCALE_FACTOR` | Scaling factor for this aircraft | 0.13514 | unitless |
| `Aircraft.Engine.SCALE_MASS` | Scale engine mass with thrust | True | Boolean |
| `Aircraft.Engine.SCALED_SLS_THRUST` | Actual thrust = reference × scale | (calculated) | N |

**CSV Example** (Turbofan configuration):
```
aircraft:engine:data_file,models/engines/turbofan_22k.csv,unitless
aircraft:engine:reference_mass,2855.8,kg
aircraft:engine:reference_sls_thrust,98752.0,N
aircraft:engine:scale_factor,0.13514,unitless
aircraft:engine:scale_mass,True,unitless
aircraft:engine:mass_scaler,1.0,unitless
```

### Available Engine Data Files
Located in [aviary/models/engines/](aviary/models/engines/):
- `turbofan_22k.csv` - ~22,000 lbf turbofan
- `turbofan_24k_1.csv` - ~24,000 lbf turbofan
- `turbofan_28k_with_electric.csv` - 28k with hybrid capability

**Note**: Actual turbine jets (turbojets) would use similar configuration with appropriate data file.

---

## 4. Cargo and Payload Configuration

### Cargo Specifications
| Variable | Description | Example | Units |
|----------|-------------|---------|-------|
| `Aircraft.CrewPayload.MISC_CARGO` | Cargo mass not tied to passengers | 1500.0 | kg |
| `Aircraft.CrewPayload.WING_CARGO` | Additional cargo in wings | 0 | kg |

**CSV Example** (Cargo aircraft):
```
aircraft:crew_and_payload:misc_cargo,1500.0,kg
aircraft:crew_and_payload:wing_cargo,0,kg
aircraft:crew_and_payload:num_passengers,0,unitless
aircraft:crew_and_payload:num_flight_crew,2,unitless
```

### Passenger Configuration (Commercial)
| Variable | Description | Example | Units |
|----------|-------------|---------|-------|
| `Aircraft.CrewPayload.NUM_PASSENGERS` | Total passenger count | 162 | unitless |
| `Aircraft.CrewPayload.NUM_FIRST_CLASS` | First class count | 12 | unitless |
| `Aircraft.CrewPayload.NUM_BUSINESS_CLASS` | Business class count | 0 | unitless |
| `Aircraft.CrewPayload.NUM_ECONOMY_CLASS` | Economy class count | 150 | unitless |
| `Aircraft.CrewPayload.NUM_FLIGHT_CREW` | Pilot/copilot count | 2 | unitless |
| `Aircraft.CrewPayload.NUM_FLIGHT_ATTENDANTS` | Cabin crew count | 5 | unitless |
| `Aircraft.CrewPayload.MASS_PER_PASSENGER` | Average mass per person | 165.0 | lbm |
| `Aircraft.CrewPayload.BAGGAGE_MASS_PER_PASSENGER` | Luggage per passenger | 35.0 | lbm |

**CSV Example** (Passenger aircraft):
```
aircraft:crew_and_payload:design:num_passengers,162,unitless
aircraft:crew_and_payload:design:num_first_class,12,unitless
aircraft:crew_and_payload:design:num_business_class,0,unitless
aircraft:crew_and_payload:design:num_economy_class,150,unitless
aircraft:crew_and_payload:num_flight_crew,2,unitless
aircraft:crew_and_payload:num_flight_attendants,5,unitless
aircraft:crew_and_payload:mass_per_passenger,165.0,lbm
aircraft:crew_and_payload:baggage_mass_per_passenger,35.0,lbm
```

---

## 5. Example Aircraft Configurations

### Small Cargo Aircraft (Complete)

**File**: [aviary/models/aircraft/small_cargo/small_cargo_FLOPS.csv](aviary/models/aircraft/small_cargo/small_cargo_FLOPS.csv)

Key specs:
- Gross mass: ~8000 kg
- Wingspan: 18 m
- Wing area: 40 m²
- Fuselage length: 15 m
- Fuselage dimensions: 2.0 m wide × 2.2 m high
- Engines: 2 × turbofan_22k
- Cargo: 1500 kg
- Fuel capacity: 2500 kg
- Range: 1000 km

**Key CSV entries**:
```
aircraft:wing:span,18.0,m
aircraft:wing:area,40.0,m**2
aircraft:wing:aspect_ratio,8.1,unitless
aircraft:fuselage:length,15.0,m
aircraft:fuselage:max_width,2.0,m
aircraft:fuselage:max_height,2.2,m
aircraft:engine:num_engines,2,unitless
aircraft:engine:data_file,models/engines/turbofan_22k.csv,unitless
aircraft:crew_and_payload:misc_cargo,1500.0,kg
aircraft:fuel:total_capacity,2500.0,kg
aircraft:design:range,1000.0,NM
```

### Large Commercial Aircraft (Single Aisle)

**File**: [aviary/models/aircraft/large_single_aisle_2/large_single_aisle_2_FLOPS_data.py](aviary/models/aircraft/large_single_aisle_2/large_single_aisle_2_FLOPS_data.py)

Key specs:
- Gross mass: 174,200 lbm
- Wingspan: 112.57 ft (34.3 m)
- Wing area: 1,341 ft² (124.6 m²)
- Fuselage length: 124.75 ft (38 m)
- Fuselage: 12.33 ft wide × 13.02 ft high
- Engines: 2 × turbofan_24k_1
- Passengers: 162
- Fuel capacity: 46,063 lbm
- Range: 2,960 NM

**Key settings**:
```python
inputs.set_val(Aircraft.Wing.SPAN, 112.57, 'ft')
inputs.set_val(Aircraft.Wing.AREA, 1341.0, 'ft**2')
inputs.set_val(Aircraft.Fuselage.LENGTH, 124.75, 'ft')
inputs.set_val(Aircraft.Fuselage.MAX_WIDTH, 12.33, 'ft')
inputs.set_val(Aircraft.Engine.NUM_ENGINES, 2)
inputs.set_val(Aircraft.Engine.DATA_FILE, 'models/engines/turbofan_24k_1.csv')
inputs.set_val(Aircraft.CrewPayload.NUM_PASSENGERS, 162)
inputs.set_val(Aircraft.Fuel.TOTAL_CAPACITY, 46063.0, 'lbm')
```

---

## 6. Creating a New Aircraft Configuration

### Step-by-Step Process

#### Option A: From CSV Template

1. **Copy existing CSV file**:
   ```bash
   cp aviary/models/aircraft/small_cargo/small_cargo_FLOPS.csv \
      aviary/models/aircraft/my_aircraft/my_aircraft_FLOPS.csv
   ```

2. **Modify key dimensions**:
   ```csv
   # Wing
   aircraft:wing:span,25.0,m
   aircraft:wing:area,65.0,m**2
   aircraft:wing:aspect_ratio,9.6,unitless
   
   # Fuselage
   aircraft:fuselage:length,20.0,m
   aircraft:fuselage:max_width,2.5,m
   
   # Engines
   aircraft:engine:num_engines,2,unitless
   
   # Cargo
   aircraft:crew_and_payload:misc_cargo,2500.0,kg
   ```

3. **Load in Python**:
   ```python
   prob = av.AviaryProblem()
   prob.load_inputs("aviary/models/aircraft/my_aircraft/my_aircraft_FLOPS.csv", phase_info)
   ```

#### Option B: From Python Code

1. **Create new Python file**: `aviary/models/aircraft/my_aircraft/my_aircraft_FLOPS_data.py`

2. **Define aircraft data**:
   ```python
   from aviary.utils.aviary_values import AviaryValues
   from aviary.variable_info.variables import Aircraft
   
   MyAircraftFLOPS = {}
   inputs = MyAircraftFLOPS['inputs'] = AviaryValues()
   
   # Wing
   inputs.set_val(Aircraft.Wing.SPAN, 25.0, 'm')
   inputs.set_val(Aircraft.Wing.AREA, 65.0, 'm**2')
   inputs.set_val(Aircraft.Wing.ASPECT_RATIO, 9.6)
   
   # Fuselage
   inputs.set_val(Aircraft.Fuselage.LENGTH, 20.0, 'm')
   inputs.set_val(Aircraft.Fuselage.MAX_WIDTH, 2.5, 'm')
   
   # Engines
   inputs.set_val(Aircraft.Engine.NUM_ENGINES, 2)
   inputs.set_val(Aircraft.Engine.DATA_FILE, 'models/engines/turbofan_22k.csv')
   ```

---

## 7. Variable Reference

Complete variable hierarchy: [aviary/variable_info/variables.py](aviary/variable_info/variables.py)

### Key Classes
- `Aircraft.Wing.*` - Wing parameters
- `Aircraft.Fuselage.*` - Fuselage parameters
- `Aircraft.Engine.*` - Engine specifications
- `Aircraft.CrewPayload.*` - Crew and cargo
- `Aircraft.HorizontalTail.*` - Horizontal stabilizer
- `Aircraft.VerticalTail.*` - Vertical stabilizer
- `Aircraft.Fuel.*` - Fuel system
- `Aircraft.Design.*` - Overall design parameters

### Common Design Variables
| Variable | Description | Example |
|----------|-------------|---------|
| `Aircraft.Design.GROSS_MASS` | Maximum takeoff weight | 174200 lbm |
| `Aircraft.Design.EMPTY_MASS` | Operating empty weight | (calculated) |
| `Aircraft.Design.LANDING_TO_TAKEOFF_MASS_RATIO` | Landing weight / MTOW | 0.84 |
| `Aircraft.Design.RANGE` | Design cruise range | 2960 NM |
| `Aircraft.Design.CRUISE_MACH` | Cruise Mach number | 0.785 |

---

## 8. Units and Conventions

### Common Units
- **Length**: m, ft, inch
- **Area**: m**2, ft**2
- **Mass**: kg, lbm
- **Force/Thrust**: N, lbf
- **Pressure**: Pa, psi
- **Angle**: deg, rad
- **Unitless**: unitless, dimensionless

### Unit Conversion
When loading values, Aviary automatically converts between unit systems:
```python
# Both are equivalent
inputs.set_val(Aircraft.Wing.SPAN, 112.57, 'ft')
inputs.set_val(Aircraft.Wing.SPAN, 34.3, 'm')
```

---

## 9. Troubleshooting

### Common Issues

**Issue**: "Variable not found" error
- **Solution**: Check spelling in [variables.py](aviary/variable_info/variables.py)
- **Example**: Use `Aircraft.Engine.NUM_ENGINES` not `Aircraft.Engine.NUMENGINES`

**Issue**: Unit mismatch
- **Solution**: Specify correct units in CSV or Python
- **CSV**: `aircraft:wing:span,18.0,m` (must match expected units)

**Issue**: Engine data file not found
- **Solution**: Path is relative to Aviary root
- **Correct**: `models/engines/turbofan_22k.csv`
- **Incorrect**: `aviary/models/engines/turbofan_22k.csv`

**Issue**: Engine count mismatch
- **Solution**: `NUM_ENGINES = NUM_WING_ENGINES + NUM_FUSELAGE_ENGINES`
- **Example**: 2 = 2 + 0 ✓, 4 = 2 + 2 ✓

---

## 10. See Also

- [aviary/api.py](aviary/api.py) - Main API
- [aviary/models/](aviary/models/aircraft/) - All example aircraft
- [aviary/models/engines/](aviary/models/engines/) - Engine data files
- User documentation in [aviary/docs/](aviary/docs/)

