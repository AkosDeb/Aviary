"""Pytest defaults for local Aviary test runs."""

import os
import shutil
import tempfile
from pathlib import Path

import pytest


# OpenMDAO attempts to import mpi4py when this is unset. On machines with
# mpi4py installed but no MPI runtime DLL, that fails before tests reach Aviary.
os.environ.setdefault('OPENMDAO_USE_MPI', '0')

# Keep pytest's numbered temporary directories in the repo workspace. This
# avoids locked Windows user temp folders without using pytest's fixed
# --basetemp option, which deletes the target directory on every run.
_temp_root = Path(__file__).resolve().parent / '_aviary_pytest_tmp'
_temp_root.mkdir(exist_ok=True)
os.environ.setdefault('TMP', str(_temp_root))
os.environ.setdefault('TEMP', str(_temp_root))
tempfile.tempdir = str(_temp_root)


@pytest.fixture
def tmp_path(request):
    """Local replacement for pytest's tmp_path on locked-down Windows setups."""
    test_dir = tempfile.mkdtemp(prefix=f'{request.node.name[:32]}-', dir=_temp_root)
    path = Path(test_dir)
    yield path
    shutil.rmtree(path, ignore_errors=True)
