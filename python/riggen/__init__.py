"""riggen — the blazingly fast, lightweight robot assembler for RL researchers.

Two things in one wheel. The application is the native ``riggen``
executable (ADR-0002): a GPU window for assembling meshes into a kinematic
tree, placing joints, computing inertials and collision geometry, and
exporting MJCF and URDF — run it as ``riggen`` or ``python -m riggen``. The
SDK is this package (ADR-0009): the same document, the same rules, from a
script or a notebook.

>>> import riggen
>>> robot = riggen.Robot("pendulum")
>>> robot.root.add_mesh("base.stl", scale=0.001)
>>> arm = robot.root.add_link(
...     "arm", riggen.Revolute("y", origin=(0, 0, 0.5), limits=(-90, 90), degrees=True),
...     mesh="arm.stl", scale=0.001, material="PLA",
... )
>>> robot.export("out", format="mjcf")

``riggen.show(robot)`` opens the window on the document; place a joint by
hand there, save, and ``viewer.wait()`` hands the saved document back.

Every edit is one document command, applied on a copy and kept only on
success; a refused edit raises a :class:`riggen.EditError` subclass and
changes nothing. Meters, radians, right-handed, Z-up, with ``degrees=True``
wherever an angle is typed. See https://github.com/Divelix/riggen#python.
"""

from __future__ import annotations

from importlib.metadata import PackageNotFoundError, version

from .errors import (
    CannotRemoveRoot,
    CannotReparentRoot,
    EditError,
    ExportError,
    FileError,
    InertialError,
    InvalidDocument,
    MaterialInUse,
    MovableJointOnRootPath,
    RiggenError,
    RiggenWarning,
    UnknownId,
    UnknownMaterial,
    UrdfImportError,
    ValidationError,
    WouldCreateCycle,
)
from .show import Viewer, show
from .robot import (
    ComputedInertial,
    Continuous,
    Dynamics,
    Fixed,
    Geom,
    HybridInertial,
    Inertial,
    Joint,
    JointSpec,
    Limits,
    Link,
    Material,
    OverrideInertial,
    Pose,
    Prismatic,
    Revolute,
    Robot,
    load,
    load_urdf,
)

try:
    __version__ = version("riggen")
except PackageNotFoundError:  # a checkout on sys.path, not an installed wheel
    __version__ = "0.0.0+unknown"

REPOSITORY = "https://github.com/Divelix/riggen"

__all__ = [
    # the document
    "Robot",
    "Link",
    "Joint",
    "Geom",
    "load",
    "load_urdf",
    # the window
    "show",
    "Viewer",
    # values
    "Pose",
    "Limits",
    "Dynamics",
    "Material",
    "Inertial",
    # joint specs
    "JointSpec",
    "Fixed",
    "Revolute",
    "Continuous",
    "Prismatic",
    # inertial specs
    "ComputedInertial",
    "OverrideInertial",
    "HybridInertial",
    # errors
    "RiggenError",
    "RiggenWarning",
    "EditError",
    "InvalidDocument",
    "UnknownId",
    "UnknownMaterial",
    "WouldCreateCycle",
    "CannotRemoveRoot",
    "CannotReparentRoot",
    "MaterialInUse",
    "MovableJointOnRootPath",
    "ValidationError",
    "FileError",
    "ExportError",
    "UrdfImportError",
    "InertialError",
]
