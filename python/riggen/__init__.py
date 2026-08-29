"""riggen — the blazingly fast, lightweight robot assembler for RL researchers.

This is the 0.0.1 name reservation. The application (a native GPU window for
assembling meshes into a kinematic tree and exporting MJCF / URDF) ships as
0.1.0; see https://github.com/Divelix/riggen.
"""

from __future__ import annotations

import sys

__version__ = "0.0.1"

REPOSITORY = "https://github.com/Divelix/riggen"


def main() -> int:
    """The `riggen` console script: says the application is coming."""
    print(
        f"riggen {__version__} is a name reservation.\n"
        "The robot assembler ships as riggen 0.1.0; follow it at "
        f"{REPOSITORY}",
        file=sys.stderr,
    )
    return 0
