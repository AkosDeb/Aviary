from dataclasses import dataclass


@dataclass(frozen=True)
class Material:
    """Conceptual-design material properties for aeroelastic sizing screens.

    These values are deliberately conservative placeholders for early design
    trade studies. They are not certification allowables. Replace them with
    laminate coupon data, supplier data, or tested printed-part allowables before
    making structural decisions.
    """

    name: str
    youngs_modulus: float
    shear_modulus: float
    density: float
    tensile_allowable: float
    compressive_allowable: float
    shear_allowable: float
    notes: str


MATERIALS = {
    'aluminum_6061_t6': Material(
        name='aluminum_6061_t6',
        youngs_modulus=68.9e9,
        shear_modulus=26.0e9,
        density=2700.0,
        tensile_allowable=140.0e6,
        compressive_allowable=140.0e6,
        shear_allowable=90.0e6,
        notes='General aircraft aluminum reference grade.',
    ),
    'gfrp_quasi_iso': Material(
        name='gfrp_quasi_iso',
        youngs_modulus=25.0e9,
        shear_modulus=4.0e9,
        density=1900.0,
        tensile_allowable=250.0e6,
        compressive_allowable=180.0e6,
        shear_allowable=70.0e6,
        notes='Quasi-isotropic glass-fiber laminate preliminary value.',
    ),
    'cfrp_quasi_iso': Material(
        name='cfrp_quasi_iso',
        youngs_modulus=70.0e9,
        shear_modulus=5.0e9,
        density=1600.0,
        tensile_allowable=600.0e6,
        compressive_allowable=450.0e6,
        shear_allowable=80.0e6,
        notes='Quasi-isotropic carbon-fiber laminate preliminary value.',
    ),
    'injection_molded_gfrp': Material(
        name='injection_molded_gfrp',
        youngs_modulus=12.0e9,
        shear_modulus=4.5e9,
        density=1650.0,
        tensile_allowable=120.0e6,
        compressive_allowable=90.0e6,
        shear_allowable=45.0e6,
        notes='Short-glass injection molded polymer preliminary value.',
    ),
    'printed_pa_cf': Material(
        name='printed_pa_cf',
        youngs_modulus=8.0e9,
        shear_modulus=3.0e9,
        density=1200.0,
        tensile_allowable=70.0e6,
        compressive_allowable=50.0e6,
        shear_allowable=30.0e6,
        notes='Carbon-filled printed nylon, build-direction dependent.',
    ),
    'printed_peek_cf': Material(
        name='printed_peek_cf',
        youngs_modulus=18.0e9,
        shear_modulus=6.5e9,
        density=1400.0,
        tensile_allowable=110.0e6,
        compressive_allowable=80.0e6,
        shear_allowable=45.0e6,
        notes='Carbon-filled printed PEEK, high-performance additive option.',
    ),
    'printed_pekk_cf': Material(
        name='printed_pekk_cf',
        youngs_modulus=16.0e9,
        shear_modulus=5.8e9,
        density=1350.0,
        tensile_allowable=100.0e6,
        compressive_allowable=75.0e6,
        shear_allowable=40.0e6,
        notes='Carbon-filled printed PEKK, high-performance additive option.',
    ),
}


def get_material(name):
    try:
        return MATERIALS[name]
    except KeyError as err:
        valid = ', '.join(sorted(MATERIALS))
        raise ValueError(f'Unknown aeroelastic material {name!r}. Valid choices: {valid}') from err
