"""``python -m riggen`` — hand over to the bundled ``riggen`` executable.

maturin places the binary in the wheel's ``scripts`` directory, which is the
environment's ``bin/`` (``Scripts\\`` on Windows), so the ``riggen`` command
is normally the binary itself. This module is for the cases where that
directory is not on ``PATH`` — ``python -m riggen`` always works when
``import riggen`` does. It is the shape of rerun's ``rerun_cli/__main__``
(ADR-0002). ``RIGGEN_BINARY`` overrides the lookup (``riggen.show`` uses the
same one); an install with no binary — a build from the sdist — is told
how to get one.
"""

from __future__ import annotations

import os
import subprocess
import sys

from .show import _command, binary_path


def main() -> None:
    try:
        binary = binary_path()
    except FileNotFoundError as e:
        sys.exit(f"riggen: {e}")
    args = _command(binary, *sys.argv[1:])
    if sys.platform == "win32" or args[0] != str(binary):
        # Windows has no exec (and a `.py` stand-in runs under this
        # interpreter): run it and forward the exit code.
        sys.exit(subprocess.call(args))
    os.execv(args[0], args)


if __name__ == "__main__":
    main()
