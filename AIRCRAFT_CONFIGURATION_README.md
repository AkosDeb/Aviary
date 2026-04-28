# Aviary Aircraft Configuration Documentation

Comprehensive guide to configuring and modifying aircraft models in Aviary.

## 📚 Documentation Files

### [AIRCRAFT_CONFIG_QUICK_REFERENCE.md](AIRCRAFT_CONFIG_QUICK_REFERENCE.md)
**Start here!** Concise reference with:
- File locations
- CSV format templates
- Key configuration parameters
- Variable naming patterns
- Quick troubleshooting

### [AIRCRAFT_CONFIGURATION_GUIDE.md](AIRCRAFT_CONFIGURATION_GUIDE.md)
**Detailed guide** covering:
- Two configuration methods (CSV and Python)
- Complete parameter reference for all subsystems
- Wing, fuselage, engine specifications
- Cargo and payload configuration
- Creating new aircraft configurations
- Units and conventions
- Complete examples

### [AIRCRAFT_CONFIG_CODE_EXAMPLES.md](AIRCRAFT_CONFIG_CODE_EXAMPLES.md)
**Working code examples**:
- Load small cargo aircraft from CSV
- Create aircraft programmatically
- Modify existing configurations
- Multi-engine variants
- CSV templates
- Parametric studies
- Property extraction

---

## 🚀 Quick Start

### 1. Load an Existing Aircraft
```python
import aviary.api as av

prob = av.AviaryProblem()
prob.load_inputs("aviary/models/aircraft/small_cargo/small_cargo_FLOPS.csv", phase_info)
prob.check_and_preprocess_inputs()
prob.build_model()
prob.setup()
prob.run_model()
```

### 2. Create Custom Aircraft (CSV)
Create file: `my_aircraft_FLOPS.csv`
```csv
aircraft:wing:span,25.0,m
aircraft:wing:area,65.0,m**2
aircraft:fuselage:length,20.0,m
aircraft:engine:num_engines,2,unitless
aircraft:crew_and_payload:misc_cargo,2500.0,kg
```

### 3. Create Custom Aircraft (Python)
```python
from aviary.utils.aviary_values import AviaryValues
from aviary.variable_info.variables import Aircraft

inputs = AviaryValues()
inputs.set_val(Aircraft.Wing.SPAN, 25.0, 'm')
inputs.set_val(Aircraft.Wing.AREA, 65.0, 'm**2')
inputs.set_val(Aircraft.Fuselage.LENGTH, 20.0, 'm')
inputs.set_val(Aircraft.Engine.NUM_ENGINES, 2)
inputs.set_val(Aircraft.CrewPayload.MISC_CARGO, 2500.0, 'kg')
```

---

## 📋 Configuration Parameters

### Wings
| Parameter | Example | Units |
|-----------|---------|-------|
| Span | 18.0 | m |
| Area | 40.0 | m² |
| Aspect Ratio | 8.1 | unitless |
| Sweep | 25.0 | deg |

### Fuselage
| Parameter | Example | Units |
|-----------|---------|-------|
| Length | 15.0 | m |
| Width | 2.0 | m |
| Height | 2.2 | m |
| Wetted Area | 90.0 | m² |

### Engines
| Parameter | Example | Notes |
|-----------|---------|-------|
| Num Engines | 2 | Total count |
| Wing Engines | 2 | Mount location |
| Fuselage Engines | 0 | Rear-mounted |
| Data File | turbofan_22k.csv | Performance map |
| Reference Thrust | 98752 | N |
| Scale Factor | 0.135 | For this aircraft |

### Cargo/Payload
| Parameter | Example | Units |
|-----------|---------|-------|
| Misc Cargo | 1500 | kg |
| Wing Cargo | 0 | kg |
| Passengers | 0 | count |
| Crew | 2 | count |

---

## 🗂️ File Locations

### Code Repositories
- **Variable Definitions**: `aviary/variable_info/variables.py`
- **Aircraft Models**: `aviary/models/aircraft/<aircraft_name>/`
- **Engine Data**: `aviary/models/engines/turbofan_*.csv`

### Example Aircraft
| Aircraft | Location | Type |
|----------|----------|------|
| Small Cargo | `small_cargo/small_cargo_FLOPS.csv` | Cargo jet |
| Large Jet | `large_single_aisle_2/large_single_aisle_2_FLOPS_data.py` | Passenger |
| Advanced SAA | `advanced_single_aisle/advanced_single_aisle_data.py` | Passenger |
| BWB | `blended_wing_body/bwb_detailed_FLOPS_data.py` | Blended wing |

### Available Aircraft Models
Located in `aviary/models/aircraft/`:
- `advanced_single_aisle/`
- `blended_wing_body/`
- `large_single_aisle_1/`
- `large_single_aisle_2/`
- `large_turboprop_freighter/`
- `multi_engine_single_aisle/`
- `small_cargo/`
- `small_single_aisle/`
- `test_aircraft/`

---

## 🎯 Common Tasks

### Task 1: Load and View Aircraft
```python
import aviary.api as av

prob = av.AviaryProblem()
prob.load_inputs("aviary/models/aircraft/small_cargo/small_cargo_FLOPS.csv", phase_info)
prob.check_and_preprocess_inputs()
prob.build_model()
prob.setup()
prob.run_model()

# View properties
print("Wingspan:", prob.get_val(av.Aircraft.Wing.SPAN, 'm'), "m")
print("Cargo:", prob.get_val(av.Aircraft.CrewPayload.MISC_CARGO, 'kg'), "kg")
```

### Task 2: Modify Aircraft
```python
# Increase payload by 50%
cargo = prob.get_val(av.Aircraft.CrewPayload.MISC_CARGO, 'kg')
prob.model.set_input_defaults(av.Aircraft.CrewPayload.MISC_CARGO, 
                              cargo * 1.5, 'kg')
prob.run_model()
```

### Task 3: Create New Aircraft
1. Copy existing CSV: `cp small_cargo_FLOPS.csv my_aircraft_FLOPS.csv`
2. Edit key values (wing, fuselage, engines, cargo)
3. Save and load: `prob.load_inputs("my_aircraft_FLOPS.csv", phase_info)`

### Task 4: Compare Variants
```python
# Twin vs Four Engine
for engine_count in [2, 4]:
    prob = av.AviaryProblem()
    prob.load_inputs(config_file, phase_info)
    prob.model.set_input_defaults(av.Aircraft.Engine.NUM_ENGINES, engine_count)
    prob.check_and_preprocess_inputs()
    prob.build_model()
    prob.setup()
    prob.run_model()
    print(f"{engine_count} engines: MTOW = {prob.get_val(av.Aircraft.Design.GROSS_MASS, 'kg')} kg")
```

---

## ✅ Validation Checklist

When creating new aircraft configurations:

- [ ] Wing span and area are reasonable for aircraft class
- [ ] Fuselage length/width proportional to aircraft type
- [ ] Engine count is positive integer
- [ ] NUM_ENGINES = NUM_WING_ENGINES + NUM_FUSELAGE_ENGINES
- [ ] Engine data file exists in `aviary/models/engines/`
- [ ] Cargo/payload masses are positive
- [ ] All dimensions in appropriate units
- [ ] Gross mass includes structural + fuel + payload

---

## 🐛 Troubleshooting

| Issue | Solution |
|-------|----------|
| "Variable not found" | Check spelling in `variables.py` |
| Unit mismatch error | Specify units in CSV or Python |
| Engine file not found | Path is relative to Aviary root |
| Engine count error | Ensure NUM_ENGINES = NUM_WING_ENGINES + NUM_FUSELAGE_ENGINES |
| Unrealistic properties | Check dimension inputs are physically reasonable |

---

## 📖 Related Documentation

- **Aviary User Guide**: `aviary/docs/user_guide/`
- **API Reference**: `aviary/api.py`
- **Variable Hierarchy**: `aviary/variable_info/variables.py`
- **Engine Data Format**: `aviary/models/engines/`

---

## 🔍 Variable Naming Convention

All aircraft variables follow the pattern: `aircraft:subsystem:parameter`

**Examples**:
- `aircraft:wing:span` → Python: `Aircraft.Wing.SPAN`
- `aircraft:fuselage:length` → Python: `Aircraft.Fuselage.LENGTH`
- `aircraft:engine:num_engines` → Python: `Aircraft.Engine.NUM_ENGINES`

---

## 📞 Support

For detailed information on specific parameters, see:
1. [AIRCRAFT_CONFIG_QUICK_REFERENCE.md](AIRCRAFT_CONFIG_QUICK_REFERENCE.md) - Quick lookup
2. [AIRCRAFT_CONFIGURATION_GUIDE.md](AIRCRAFT_CONFIGURATION_GUIDE.md) - Complete reference
3. [AIRCRAFT_CONFIG_CODE_EXAMPLES.md](AIRCRAFT_CONFIG_CODE_EXAMPLES.md) - Working code
4. `aviary/variable_info/variables.py` - Variable definitions

---

**Last Updated**: April 2026  
**Aviary Version**: Latest  
**Documentation Scope**: Aircraft configuration and modeling

