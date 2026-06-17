"""
Shared printing utilities for Aviary run scripts.

Provides consistent formatting for problem output retrieval and result printing
across multiple aircraft models and analysis scripts.
"""


def safe_get(prob, var, units=None):
    """Safely retrieve a value from an OpenMDAO Problem.

    Attempts to get a variable value from the problem, returning a user-friendly
    error message if the variable does not exist or cannot be accessed.

    Parameters
    ----------
    prob : om.Problem
        The OpenMDAO problem object.
    var : str
        The variable name (path) to retrieve.
    units : str, optional
        Target units for the retrieved value (unit conversion performed if provided).
        Default: None (no conversion).

    Returns
    -------
    float or str
        The scalar value if successful, or an error message string if retrieval failed.
    """
    try:
        value = prob.get_val(var, units=units) if units else prob.get_val(var)
        return value[0] if hasattr(value, '__len__') else value
    except Exception as err:
        return f'not available: {err}'


def print_result(label, value, unit=''):
    """Print a result with fixed-width formatting.

    Prints a label-value pair with 12-character right-aligned value field,
    suitable for most engineering quantities (mass, area, length, etc.).
    Handles both numeric and string values gracefully.

    Parameters
    ----------
    label : str
        Left-aligned descriptive label (column width ~35 chars).
    value : float or str
        The value to print. If string, printed as-is; if numeric, formatted
        to 4 decimal places.
    unit : str, optional
        Unit string appended to the value column. Default: '' (no unit).

    Examples
    --------
    >>> print_result('Wing mass', 2.3456, 'kg')
    Wing mass                       =       2.3456 kg
    >>> print_result('Engine', 'not available', '')
    Engine                          = not available
    """
    if isinstance(value, str):
        print(f'{label:<35} = {value}')
    else:
        print(f'{label:<35} = {value:>12.4f} {unit}'.rstrip())


def print_scientific_result(label, value, unit=''):
    """Print a result in scientific notation with fixed-width formatting.

    Similar to print_result but uses scientific notation (6 decimal places),
    suitable for very small or very large quantities (e.g., Reynolds number,
    pressure coefficients, aerodynamic derivatives).

    Parameters
    ----------
    label : str
        Left-aligned descriptive label (column width ~35 chars).
    value : float or str
        The value to print. If string, printed as-is; if numeric, formatted
        in scientific notation to 6 decimal places.
    unit : str, optional
        Unit string appended to the value column. Default: '' (no unit).

    Examples
    --------
    >>> print_scientific_result('Reynolds number', 1234567.0, '')
    Reynolds number                 =   1.234567e+06
    >>> print_scientific_result('CP_min', -0.00001234, '')
    CP_min                          =  -1.234000e-05
    """
    if isinstance(value, str):
        print(f'{label:<35} = {value}')
    else:
        print(f'{label:<35} = {value:>12.6e} {unit}'.rstrip())


def print_percent_result(label, value):
    """Print a result as a percentage with fixed-width formatting.

    Multiplies the value by 100 and displays as a percentage (2 decimal places).
    Suitable for efficiency factors, ratios, and other dimensionless fractions.

    Parameters
    ----------
    label : str
        Left-aligned descriptive label (column width ~35 chars).
    value : float or str
        The fractional value to print. If string, printed as-is; if numeric,
        multiplied by 100 and formatted to 2 decimal places with '%' suffix.

    Examples
    --------
    >>> print_percent_result('Span efficiency', 0.954)
    Span efficiency                 =      95.40 %
    >>> print_percent_result('Taper ratio', 'data pending')
    Taper ratio                     = data pending
    """
    if isinstance(value, str):
        print(f'{label:<35} = {value}')
    else:
        print(f'{label:<35} = {100.0 * value:>12.2f} %')
