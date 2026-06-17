

Requirements 
- Nz/Ny of 7g at 550km/h at 5km altitude
- M_crit to be higher than M at maximum Nz at 5km altitude 550km/h with some safety factor (0.05Mach). CL value for the constraint is derived from max Nz case 
- T/W greater than 1.5  for VTOL takeoff
- STT capable 
    - Close to no Cl_beta 
- Minimum-phase behaviour --> control surface actuation happening infront of CG
- Capable of flying at 5km altitude and at a speed of 550km/h
-  Flutter speed need to be 1.1 * 550km/h = 605 km/h 


Design thought about each requirements
1.1 Nz:
    - In general it not  design driving factor at such a high speed even smaller deflection can lead to the required nz.  Still it's beneficial to full span control surface on H-wing so that at any time we will have enough control authority for Pitch and Yaw
1.2 Ny:
    - This requirement is the driving factor of the vtp and rudder sizing. 
        - For the rudder (e.i. percantage of vtp span) it's ideal to choose the biggest manufacturable span for the following reason. At Intercept speed the Ny requirement would be reached at smaller deflection as well but if we have a larger rudder span we can increase our intercept envelope by maitaining the same Ny criterium at lower speed. OFC only If it's mechanicly not challaning.
        - Similar thigns can be said about the VTP sizing but as the rudder span and chord length is usually mechaniclly locked (e.i. mechanically not achievable to make it bigger) the real sizing is coming from the VTP, but here increasing the intercept envleope will peanlize the aircraft because of increasing flutter speed or increasing drag.

2. Skid-to-Turn capable 
    - it requires to thing, that can generate enough Ny  and Nz  so that we don't need to bank to turn and making our turn dynamics much slower, secondly as we are going to be at higher Sideslip angles we need to minimaze Cl_beta becuase we don't want the aircraft to start rolling when we turn, therefore the control surfaces need to counteract this induced roll from Beta but this reduces efficiency 
    - The goal of therefore is eliminate or mnimizae Cl_beta generation at any AoA and Beta conditions.
        - Design decision derived from this 
            - H-wing has to be mid fuselage mounted 
            - H-wing the wing it self should not have any Sweep as any wing with sweep at positive CL generate Cl_beta therefore it needs to be zero
            - VTP on the wing needs to be mounted in the middle e.i. it can't be skewed up or down otherwise the aerodynamic center of the vtps wouldn't align with the CG and therefore at each sideslip angle it would generate Cl_beta 
                - The VTP has no Sweep constraint if the VTP aero center is aligned with aircraft CG --> important for M_crit
            - The tail needs to be 4 control surfaces e.i. either plus tail, X-tail  or H-tail with same rules as detailed above. 
                -  Reason is that if we were to have a conventional tail then the the vtp aero cneter would be not aligned with cg in x-axis therefore when force is generated on the vtp it wuld cause the roation aroudn x-axis e.i. roll. this can be solved by fully identical bottom tail that would generate and eqaul but opposite moment around x-axis nulling the effect
            - (Not decision made yet) The wing should not have any wasin or washout as those as well can generate additional Cl_beta
            - No dihedral of any kind if allowed on the wing
        - Reamining effects 
            - any wing with aspect ratio at non zero CL value will have Cl_beta. the optimization that can be done about is we try to minimaze CL, which is only possible during intercept if we are flying faster. e.i. At 550km/h intercept our Cl_beta is going to be smaller than at, for example, 400km/h simple because to achieve the same Nz larger CL is needed at lower speeds.

3. No Supersonic flow anywhere on the aircraft
    - from requirement we have M_crit value we need to achieve. There is 3 way to achieve this, sweep, thickness and airfoil type.
    - VTP and Wing are the two critical surfaces bot with different constraint
    - Wing:
        - It can't have any sweep because of Cl_beta constraint therefore the only way to control it is via Thickness ratio and airfoil type
        - Usually thicker airfoil have better CL_max and CL_alpha compared to smaller t/c airfoils therefore from the eye of range and Nz we prefer that but M_crit is our limit out upper end. 
    - VTP:
        - As if the VTP Aerodnymic cneter is aligned with the aircraft CG it's not a problem if we introduce sweep e.i. we can have higher t/c airfoil then at the wing because Sweep is acceptable

4. Flutter speed.
    - The  biggest disadvantage of an H-wing aircraft is the decrease in Flutter speed due to the extra masses on the end of the wings. The VTP significantly descrease the flutter speed compared to same span standalone wing. 
    - The design questions here are about the material thickness and material type used and weight penalty we gain compared to a non-H-wing configuration
        - Very Rough estimate of around 1.4*Wing mass to achieve required flutter speed with h-wing configuration
    - The VTP location compared to a wing has also a major effect on flutter based on if it's infront of, behind or on top of the elastic center of the wing 
        - Decision what is the best vtp attachment location(Not Made yet)
            - No deep analysis yet but it seems like having the VTP CG on the elastic center o the wing would be the most ideal 

5. Thrust to Weight
    - This requirment is soly about VTOL capability and enabling the aircraft to not or very minimally require any ground assistance making CONOPS much easier and simpler
    - This and maximum achievable speed is the driving factor for engine sizing
    - 1.5 was choosen (based on Artur and Bryan flight tests) because at 1.3 T/W VTOL is achievable but much harder and below extremely difficults and with 1.5 we have safety margin for future developments

6. Minimum phase behevaiours 
    - It requires the aircraft when a contorl surface actuation happens and the inital position change of the aircraft is in the direction where the aircraft is suppose to go eventually. e.i. for pitch up with elevator infront of CG the inital for more lift on the elevator therefore aircraft start climb because of the extra force and then the extra force will cause the rotation and the eventuall increase in Alpha. 
    -  To achieve this control surfaces needs to be infron of CG BUT H-wing  in front of the CG will cause instability in both pitch and yaw. 
    -  The tails needs to be passive surfaces and the sizing if fully driven to achieve the required stability values with the effects of destabilizing H-wing




Aeroelasticity
for aeroeaslticity the bet is when the vtp cg is on top of he aeroelastic center of the wing
also in one of the nasa study a shows a very slight performance increase if the vtp is aft from the wing so so we could gain a small amount of boost because of the after position of the vtp not just in VTP CY_beta but also in CY_delta_r so rudder effectivity as well but stil should be double checked again


---

## Wetted Area Convention and Fuselage Cutouts

### Convention (all lifting surfaces)

```
Swet = (S_planform − S_inside_fuselage) × 2
```

Both sides of each surface counted. For wingtip-mounted VTPs `S_inside_fuselage = 0`.

### Wing wetted area

Computed by FLOPS `WingWettedArea` in pre-mission:

```
XMULT     = 0.387 × (t/c) + 2.0          # thickness correction, NACA 4-digit ≈ 2.058 at t/c=0.15
Swet_wing = XMULT × (S_wing − buried_panel)
buried_panel ≈ c_root × (d_fus / 2)      # one fuselage half-diameter strip
```

Live w.r.t. wing area DV and wing t/c DV (FLOPS recomputes in pre-mission).

### VTP wetted area

```
Swet_VTP = 4 × vtp_area          # 2 panels (L+R wingtip) × 2 sides
```

Computed by `vtp_wetted_area_comp` ExecComp at model scope. `vtp_area` is the area of
one VTP panel (full span from below-wing tip to above-wing tip), computed live by
`HTailGeometry` from the VTP span design variable.

### Fuselage exposed wetted area  (`FuselageExposedWettedAreaComp`)

The wing is mid-fuselage mounted and passes fully through the fuselage body. On each
side (left and right) the fuselage skin has a hole whose shape is the **airfoil
cross-section** at the local chord. The fuselage exposed wetted area subtracts those
two holes from the gross superellipse wetted area.

#### Airfoil cross-section area

For a NACA 4-digit thickness distribution:

```
A_airfoil = K × (t/c) × c²

K = 2 × ∫₀¹ [ 0.2969√x̄ − 0.1260x̄ − 0.3516x̄² + 0.2843x̄³ − 0.1015x̄⁴ ] dx̄ × (1/0.2)
  = 0.6843     (analytically exact, independent of t/c)
```

K is stored in `AirfoilData.cross_section_area_coeff` (default 0.6843 for all NACA
4-digit entries).

#### Chord at the fuselage wall

Taper means the chord at the fuselage wall (`y = d_fus / 2`) is slightly less than
the root chord:

```
c_fus = c_root × (1 − (1 − taper) × d_fus / b)
```

#### Full formula

```
Swet_fus_exposed = fuselage_wetted_area
                 − 2 × K_wing  × tc_wing  × c_fus²
                 − 2 × K_htail × tc_htail × c_htail_root²   (htail: default 0)
```

Hook for a future fuselage-mounted tail: pass non-zero `htail_c_root_at_fus` and
`htail_tc_at_fus` to `FuselageExposedWettedAreaComp`.

#### Inputs / Outputs

| Variable | Units | Source |
|---|---|---|
| `fuselage_wetted_area` | m² | `SuperellipseFuselageGeometry` (gross, no cutouts) |
| `wing_root_chord` | m | `WingSurface` (promoted) |
| `wing_section_tc` | – | design variable |
| `fus_max_width` | m | `load_cond` IndepVarComp |
| `wing_span` (`Aircraft.Wing.SPAN`) | m | design variable |
| `wing_taper_ratio` (`Aircraft.Wing.TAPER_RATIO`) | – | CSV |
| `htail_c_root_at_fus` | m | default 0 (future) |
| `htail_tc_at_fus` | – | default 0 (future) |
| **`fuselage_exposed_wetted_area`** | m² | output → trajectory parameter → mission drag |

#### Numerical example — UAV baseline

| Parameter | Value |
|---|---|
| Wing span b | 1.8 m |
| Wing area S | 0.45 m² |
| Taper ratio λ | 0.8 |
| t/c | 0.15 |
| Fuselage max width d_fus | 0.155 m |
| c_root = 2S/(b(1+λ)) | 0.2778 m |
| c_fus = c_root×(1−0.2×0.155/1.8) | 0.2730 m |
| A_hole = 0.6843×0.15×0.2730² | 0.00765 m² |
| Wing cutout (×2) | **0.0153 m²** |
| Swet_fus_gross (superellipse) | ~0.60 m² |
| **Swet_fus_exposed** | **~0.585 m²** |
