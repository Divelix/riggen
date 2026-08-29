"""``python -m riggen`` — hand over to the bundled ``riggen`` executable.

maturin places the binary in the wheel's ``scripts`` directory, which is the
environment's ``bin/`` (``Scripts\\`` on Windows), so the ``riggen`` command
is normally the binary itself. This module is for the cases where that
directory is not on ``PATH`` — ``python -m riggen`` always works when
``import riggen`` does. It is the shape of rerun's ``rerun_cli/__main__``
(ADR-0002).
"""

from __future__ import annotations

import os
import subprocess
import sys
import sysconfig


def binary_path() -> str:
    """The bundled executable, wherever this interpreter keeps its scripts."""
    name = "riggen.exe" if sys.platform == "win32" else "riggen"
    scripts = sysconfig.get_path("scripts")
    path = os.path.join(scripts, name)
    if os.path.isfile(path):
        return path
    # A user-site install keeps its scripts elsewhere; ask for that layout.
    scripts_user = sysconfig.get_path("scripts", f"{os.name}_user")
    path_user = os.path.join(scripts_user, name)
    if os.path.isfile(path_user):
        return path_user
    sys.exit(
        f"riggen: the bundled executable is missing (looked in {scripts} "
        f"and {scripts_user}). Reinstall with `pip install --force-reinstall riggen`."
    )


def main() -> None:
    binary = binary_path()
    args = [binary, *sys.argv[1:]]
    if sys.platform == "win32":
        # Windows has no exec: run it and forward the exit code.
        sys.exit(subprocess.call(args))
    os.execv(binary, args)


if __name__ == "__main__":
    main()
