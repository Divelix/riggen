"""riggen — the blazingly fast, lightweight robot assembler for RL researchers.

The application is the native ``riggen`` executable this wheel carries
(ADR-0002): a GPU window for assembling meshes into a kinematic tree, placing
joints, computing inertials and collision geometry, and exporting MJCF and
URDF. Run it as ``riggen`` or ``python -m riggen``. This package holds
nothing else until the v0.2 SDK; see https://github.com/Divelix/riggen.
"""

from __future__ import annotations

from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("riggen")
except PackageNotFoundError:  # a checkout on sys.path, not an installed wheel
    __version__ = "0.0.0+unknown"

REPOSITORY = "https://github.com/Divelix/riggen"
