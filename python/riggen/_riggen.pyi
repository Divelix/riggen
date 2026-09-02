"""Type stubs for ``riggen._riggen``, the extension module (crates/riggen-py).

Mirrors everything the Rust module exposes; ``pyright`` reads this, not the
``.so``. Values cross the boundary in the document's own serde shape — the
v1 schema of docs/02-data-model.md §Schema — with ids as ``int``s; the
``*Doc`` TypedDicts below are that shape. The public API over this is
``riggen.Robot`` (``python/riggen/robot.py``, plans/python-sdk step 6).
"""

from __future__ import annotations

import os
from pathlib import Path
from typing import Any, Literal, TypedDict

from typing_extensions import NotRequired

__version__: str

def rpy_to_quat(rpy: tuple[float, float, float]) -> list[float]:
    """``(roll, pitch, yaw)`` radians → ``[x, y, z, w]`` (URDF's Rz·Ry·Rx)."""

def quat_to_rpy(quat: tuple[float, float, float, float]) -> list[float]:
    """``[x, y, z, w]`` → ``(roll, pitch, yaw)``; pitch in ``[-π/2, π/2]``."""

PathLike = str | os.PathLike[str]

class PoseDoc(TypedDict):
    """A rigid transform: translation ``t`` then rotation ``r`` as a
    quaternion ``[x, y, z, w]`` (glam's order)."""

    t: list[float]
    r: list[float]

class LimitsDoc(TypedDict):
    lower: float
    upper: float
    effort: float
    velocity: float

class DynamicsDoc(TypedDict):
    damping: float
    friction: float
    armature: float

class MimicDoc(TypedDict):
    """One joint following another: ``q = multiplier * q(joint) + offset``.
    ``joint`` is the leader's id."""

    joint: int
    multiplier: float
    offset: float

# One MJCF ``<actuator>`` element for the joint (ADR-0014):
# ``{"Position": {"kp", "kv"}}`` / ``{"Velocity": {"kv"}}`` /
# ``{"Motor": {"gear"}}``.
ActuatorDoc = dict[str, Any]

JointKind = Literal["Fixed", "Revolute", "Continuous", "Prismatic"]

class JointInput(TypedDict):
    """A joint as ``add_link`` / ``set_joint`` take it: ``origin`` is the child
    link frame in the parent link frame; ``axis`` is in the child frame.
    Endpoints are not needed — the command sets them — and ignored if given."""

    name: str
    kind: JointKind
    origin: PoseDoc
    axis: list[float]
    limits: LimitsDoc | None
    dynamics: DynamicsDoc
    mimic: MimicDoc | None
    actuator: ActuatorDoc | None

class JointDoc(JointInput):
    """A joint as ``joints()`` returns it: with its endpoints."""

    parent: int
    child: int

class GeomDoc(TypedDict):
    id: int
    mesh: int
    pose: PoseDoc
    color: list[float] | None

# ``"None" | "SameAsVisual" | "ConvexHull"`` or ``{"Primitives": [...]}`` /
# ``{"Meshes": [GeomDoc, ...]}`` /
# ``{"ConvexDecomposition": {"max_hulls", "resolution", "concavity"}}``.
CollisionDoc = str | dict[str, Any]
# ``{"Computed": {"density_override": float | None}}`` /
# ``{"Override": {"mass", "com", "inertia"}}`` / ``{"Hybrid": {"mass"}}``.
InertialDoc = dict[str, Any]

class LinkDoc(TypedDict):
    name: str
    visuals: list[GeomDoc]
    collision: CollisionDoc
    inertial: InertialDoc
    material: str | None

class MaterialDoc(TypedDict):
    density: float
    color: list[float]

class AssetDoc(TypedDict):
    """``path`` is absolute in memory; ``fix_up`` a quaternion ``[x, y, z, w]``
    or ``None``. ``content_hash`` is computed, and ignored on ``set_asset``."""

    path: str
    content_hash: NotRequired[int]
    scale: float
    fix_up: list[float] | None

class FrameDoc(TypedDict):
    name: str
    parent: int
    pose: PoseDoc

class Robot:
    """The document (``riggen_core::Robot``). Every edit method applies one
    command on a copy and keeps it only on success; a refused edit raises a
    ``riggen.EditError`` subclass and changes nothing."""

    def __init__(self, name: str) -> None: ...
    @staticmethod
    def load(path: PathLike) -> tuple[Robot, list[str]]: ...
    def save(self, path: PathLike) -> None: ...
    def to_json(self) -> str: ...
    @staticmethod
    def from_json(text: str) -> Robot: ...
    def copy(self) -> Robot: ...

    name: str
    @property
    def root(self) -> int: ...
    @property
    def next_id(self) -> int: ...
    def links(self) -> dict[int, LinkDoc]: ...
    def joints(self) -> dict[int, JointDoc]: ...
    def frames(self) -> dict[int, FrameDoc]: ...
    def assets(self) -> dict[int, AssetDoc]: ...
    def materials(self) -> dict[str, MaterialDoc]: ...
    def link(self, name: str) -> int | None: ...
    def joint(self, name: str) -> int | None: ...
    def frame(self, name: str) -> int | None: ...
    def parent_joint(self, link: int) -> int | None: ...
    def child_joints(self, link: int) -> list[int]: ...
    def subtree(self, link: int) -> list[int]: ...

    def add_asset(
        self, path: PathLike, *, scale: float = 1.0, fix_up: list[float] | None = None
    ) -> int: ...
    def set_asset(self, mesh: int, asset: AssetDoc) -> None: ...
    def add_link(
        self,
        name: str,
        parent: int,
        joint: JointInput,
        *,
        mesh: PathLike | None = None,
        scale: float = 1.0,
        fix_up: list[float] | None = None,
        material: str | None = None,
    ) -> int: ...
    def remove_link(self, link: int) -> None: ...
    def rename_link(self, link: int, name: str) -> None: ...
    def rename_joint(self, joint: int, name: str) -> None: ...
    def add_frame(self, name: str, link: int, *, pose: PoseDoc | None = None) -> int: ...
    def remove_frame(self, frame: int) -> None: ...
    def rename_frame(self, frame: int, name: str) -> None: ...
    def set_frame(self, frame: int, value: FrameDoc) -> None: ...
    def add_geom(
        self,
        link: int,
        mesh: int,
        *,
        pose: PoseDoc | None = None,
        color: list[float] | None = None,
    ) -> int: ...
    def remove_geom(self, link: int, geom: int) -> None: ...
    def set_geom_pose(self, link: int, geom: int, pose: PoseDoc) -> None: ...
    def set_joint(self, joint: int, value: JointInput) -> None: ...
    def move_joint_frame(self, joint: int, origin: PoseDoc, axis: list[float]) -> None: ...
    def reparent(self, link: int, new_parent: int, *, keep_world_pose: bool = False) -> None: ...
    def set_root(self, link: int) -> None: ...
    def set_link_material(self, link: int, material: str | None) -> None: ...
    def upsert_material(self, name: str, material: MaterialDoc) -> None: ...
    def remove_material(self, name: str) -> None: ...
    def rename_material(self, from_: str, to: str) -> None: ...
    def set_inertial(self, link: int, spec: InertialDoc) -> None: ...
    def set_collision(self, link: int, policy: CollisionDoc) -> None: ...

    def validate(self) -> list[str]: ...
    def check(self) -> None: ...
    def fk(self, q: dict[int, float]) -> dict[int, PoseDoc]: ...
    def fk_frames(self, q: dict[int, float]) -> dict[int, PoseDoc]: ...
    def origin_for_world(self, link: int, world: PoseDoc) -> PoseDoc | None: ...
    def inertial(self, link: int) -> tuple[float, list[float], list[list[float]]]: ...
    def export(
        self,
        dir: PathLike,
        *,
        format: Literal["mjcf", "urdf", "sdf", "both", "all"] = "all",
        mesh_paths: str = "relative",
        floating_base: bool = False,
        fk_samples: bool = False,
    ) -> list[Path]: ...
    def fk_samples_json(self) -> str: ...
    @staticmethod
    def load_urdf(
        path: PathLike, packages: dict[str, PathLike] | None = None
    ) -> tuple[Robot, list[str]]: ...
    @staticmethod
    def load_mjcf(path: PathLike) -> tuple[Robot, list[str]]: ...
