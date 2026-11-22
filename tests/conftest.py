import os
from pathlib import Path
import shutil
import subprocess

import pytest


@pytest.fixture
def run_c_harness(tmp_path):
    """Compile and run a C11 harness in the pytest scratch directory."""
    root = Path(__file__).resolve().parents[1]
    compiler = os.environ.get("CC", "gcc")
    assert shutil.which(compiler), f"C compiler unavailable: {compiler}"

    def run(name, *, includes=(), sources=()):
        executable = tmp_path / f"{name}.exe"
        command = [compiler, "-std=c11", "-Wall", "-Wextra", "-Werror", "-pedantic"]
        for include in includes:
            command.extend(["-I", str(root / include)])
        command.append(str(root / "tests" / "native" / f"{name}.c"))
        command.extend(str(root / source) for source in sources)
        command.extend(["-lm", "-o", str(executable)])
        subprocess.run(command, cwd=tmp_path, check=True)
        subprocess.run([str(executable)], cwd=tmp_path, check=True)

    return run
