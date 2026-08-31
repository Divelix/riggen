"""The public API: :class:`Robot` with :class:`Link` / :class:`Joint` /
:class:`Geom` handles, :class:`Pose`, the joint and inertial specs, and the
:func:`load` / :func:`load_urdf` readers.

Pure Python over ``riggen._riggen`` (docs/01-architecture.md §Python SDK).
Every mutating property or method here is exactly one document command,
applied on a copy and kept only on success — a refused edit raises a
:class:`riggen.EditError` subclass and changes nothing. Meters, radians,
right-handed, Z-up (docs/02-data-model.md §Conventions); ``degrees=True``
is offered wherever an angle is typed.
"""

from __future__ import annotations

import math
import os
import warnings
from dataclasses import dataclass
from pathlib import Path
from typing import Any, ClassVar, Literal, Union

from . import _riggen
from .errors import RiggenWarning

__all__ = [
    "Pose",
    "Limits",
    "Dynamics",
    "JointSpec",
    "Fixed",
    "Revolute",
    "Continuous",
    "Prismatic",
    "Inertial",
    "ComputedInertial",
    "OverrideInertial",
    "HybridInertial",
    "Material",
    "Geom",
    "Link",
    "Joint",
    "Robot",
    "ConvexDecomposition",
    "load",
    "load_urdf",
]

Vec3 = tuple[float, float, float]
Quat = tuple[float, float, float, float]
"""A unit quaternion as ``(w, x, y, z)`` — MuJoCo's order."""
Matrix3 = tuple[Vec3, Vec3, Vec3]
"""Three rows."""
PathLike = Union[str, "os.PathLike[str]"]
Axis = Union[Literal["x", "y", "z", "-x", "-y", "-z"], Vec3]
"""A joint axis: a principal axis by name, or any vector (normalised)."""
PoseLike = Union["Pose", Vec3]
"""A :class:`Pose`, or a bare ``(x, y, z)`` for a translation."""
LimitsLike = Union["Limits", tuple[float, float]]
"""A :class:`Limits`, or ``(lower, upper)`` with zero effort and velocity."""
CollisionPolicy = Union[
    Literal["none", "same_as_visual", "convex_hull"], "ConvexDecomposition", dict[str, Any]
]
"""One of the three simple policies by name, a
:class:`ConvexDecomposition`, or a raw document value
(``{"Primitives": [...]}``, ``{"Meshes": [...]}``) as the file spells it."""


# ---- small value types ------------------------------------------------------


def _floats(value: Any, n: int, what: str) -> tuple[float, ...]:
    try:
        out = tuple(float(v) for v in value)
    except TypeError:
        raise TypeError(f"{what} must be a sequence of {n} numbers, not {value!r}") from None
    if len(out) != n:
        raise ValueError(f"{what} must have {n} components, not {len(out)}")
    return out


def _vec3(value: Any, what: str = "vector") -> Vec3:
    x, y, z = _floats(value, 3, what)
    return (x, y, z)


@dataclass(frozen=True, init=False)
class Pose:
    """A rigid transform: ``xyz`` meters then a rotation, "this frame in the
    parent frame". The rotation is given as ``rpy`` — URDF's roll, pitch,
    yaw, applied ``Rz(yaw)·Ry(pitch)·Rx(roll)`` — or as a ``quat`` in
    ``(w, x, y, z)`` order; radians unless ``degrees=True``.

    >>> Pose((0, 0, 0.5), rpy=(0, 90, 0), degrees=True)
    """

    xyz: Vec3
    quat: Quat

    def __init__(
        self,
        xyz: Vec3 = (0.0, 0.0, 0.0),
        *,
        rpy: Vec3 | None = None,
        quat: Quat | None = None,
        degrees: bool = False,
    ) -> None:
        if rpy is not None and quat is not None:
            raise TypeError("give rpy or quat, not both")
        if rpy is not None:
            r = _vec3(rpy, "rpy")
            if degrees:
                r = (math.radians(r[0]), math.radians(r[1]), math.radians(r[2]))
            x, y, z, w = _riggen.rpy_to_quat(r)
            q: Quat = (w, x, y, z)
        elif quat is not None:
            if degrees:
                raise TypeError("degrees applies to rpy, not quat")
            w, x, y, z = _floats(quat, 4, "quat")
            norm = math.sqrt(w * w + x * x + y * y + z * z)
            if not norm > 0:
                raise ValueError("quat must not be zero")
            q = (w / norm, x / norm, y / norm, z / norm)
        else:
            q = (1.0, 0.0, 0.0, 0.0)
        object.__setattr__(self, "xyz", _vec3(xyz, "xyz"))
        object.__setattr__(self, "quat", q)

    IDENTITY: ClassVar[Pose]

    @property
    def rpy(self) -> Vec3:
        """Roll, pitch, yaw in radians; pitch in ``[-π/2, π/2]``, roll folded
        into yaw at gimbal lock — the same rotation, not always the same
        three numbers that built it."""
        w, x, y, z = self.quat
        r, p, yaw = _riggen.quat_to_rpy((x, y, z, w))
        return (r, p, yaw)

    @property
    def rpy_degrees(self) -> Vec3:
        """:attr:`rpy` in degrees."""
        r, p, y = self.rpy
        return (math.degrees(r), math.degrees(p), math.degrees(y))

    def to_doc(self) -> _riggen.PoseDoc:
        """The document's shape: ``{"t": [x, y, z], "r": [x, y, z, w]}``."""
        w, x, y, z = self.quat
        return {"t": list(self.xyz), "r": [x, y, z, w]}

    @classmethod
    def from_doc(cls, doc: _riggen.PoseDoc) -> Pose:
        x, y, z, w = doc["r"]
        return cls(tuple(doc["t"]), quat=(w, x, y, z))  # type: ignore[arg-type]

    def __repr__(self) -> str:
        xyz = tuple(round(v, 9) for v in self.xyz)
        if self.quat == (1.0, 0.0, 0.0, 0.0):
            return f"Pose({xyz})"
        rpy = tuple(round(v, 6) for v in self.rpy)
        return f"Pose({xyz}, rpy={rpy})"


Pose.IDENTITY = Pose()


def _pose(value: PoseLike | None, what: str = "pose") -> Pose:
    if value is None:
        return Pose.IDENTITY
    if isinstance(value, Pose):
        return value
    return Pose(_vec3(value, what))


def _axis(value: Axis) -> Vec3:
    if isinstance(value, str):
        sign = -1.0 if value.startswith("-") else 1.0
        name = value.lstrip("+-")
        if name not in ("x", "y", "z"):
            raise ValueError(f"axis must be x, y, z (optionally negated) or a vector, not {value!r}")
        return tuple(sign if c == name else 0.0 for c in "xyz")  # type: ignore[return-value]
    x, y, z = _vec3(value, "axis")
    norm = math.sqrt(x * x + y * y + z * z)
    if not norm > 0:
        raise ValueError("axis must not be zero")
    return (x / norm, y / norm, z / norm)


@dataclass(frozen=True, init=False)
class Limits:
    """A joint's range (radians for revolute, meters for prismatic), the
    maximum ``effort`` (N·m or N) and ``velocity`` (rad/s or m/s) it may be
    driven with — URDF's ``<limit>``; zero effort or velocity means "not
    stated". ``degrees=True`` converts ``lower`` / ``upper`` only."""

    lower: float
    upper: float
    effort: float
    velocity: float

    def __init__(
        self,
        lower: float,
        upper: float,
        *,
        effort: float = 0.0,
        velocity: float = 0.0,
        degrees: bool = False,
    ) -> None:
        if degrees:
            lower, upper = math.radians(lower), math.radians(upper)
        object.__setattr__(self, "lower", float(lower))
        object.__setattr__(self, "upper", float(upper))
        object.__setattr__(self, "effort", float(effort))
        object.__setattr__(self, "velocity", float(velocity))

    def to_doc(self) -> _riggen.LimitsDoc:
        return {"lower": self.lower, "upper": self.upper, "effort": self.effort, "velocity": self.velocity}

    @classmethod
    def from_doc(cls, doc: _riggen.LimitsDoc) -> Limits:
        return cls(doc["lower"], doc["upper"], effort=doc["effort"], velocity=doc["velocity"])


def _limits(value: LimitsLike | None, degrees: bool) -> Limits | None:
    if value is None or isinstance(value, Limits):
        if degrees and value is not None:
            raise TypeError("degrees applies to a (lower, upper) pair; a Limits is already in radians")
        return value
    lower, upper = _floats(value, 2, "limits")
    return Limits(lower, upper, degrees=degrees)


@dataclass(frozen=True)
class Dynamics:
    """MJCF-side joint parameters: viscous ``damping``, dry ``friction``
    (frictionloss), rotor ``armature``. All zero by default."""

    damping: float = 0.0
    friction: float = 0.0
    armature: float = 0.0

    def to_doc(self) -> _riggen.DynamicsDoc:
        return {"damping": self.damping, "friction": self.friction, "armature": self.armature}

    @classmethod
    def from_doc(cls, doc: _riggen.DynamicsDoc) -> Dynamics:
        return cls(doc["damping"], doc["friction"], doc["armature"])


# ---- joint specs ------------------------------------------------------------


@dataclass(frozen=True, init=False)
class JointSpec:
    """What a joint is, apart from its name and endpoints: its kind, the
    ``origin`` (the child link frame in the parent link frame), the ``axis``
    (in the child frame; the joint frame *is* the child link frame), limits
    and dynamics. Build one with :class:`Fixed`, :class:`Revolute`,
    :class:`Continuous` or :class:`Prismatic`; read one from
    :attr:`Joint.spec`."""

    kind: ClassVar[str]
    origin: Pose
    axis: Vec3
    limits: Limits | None
    dynamics: Dynamics

    def _init(self, origin: PoseLike | None, axis: Axis, limits: Limits | None, dynamics: Dynamics) -> None:
        object.__setattr__(self, "origin", _pose(origin, "origin"))
        object.__setattr__(self, "axis", _axis(axis))
        object.__setattr__(self, "limits", limits)
        object.__setattr__(self, "dynamics", dynamics)

    def to_doc(self, name: str, mimic: _riggen.MimicDoc | None = None) -> _riggen.JointInput:
        """The document's joint, without endpoints. A coupling is not part
        of the spec — it belongs to the joint, not to its kind — so it is
        passed through rather than described here."""
        return {
            "name": name,
            "kind": self.kind,  # type: ignore[typeddict-item]  # a ClassVar[str] narrowed by the subclass
            "origin": self.origin.to_doc(),
            "axis": list(self.axis),
            "limits": None if self.limits is None else self.limits.to_doc(),
            "dynamics": self.dynamics.to_doc(),
            "mimic": mimic,
        }

    @staticmethod
    def from_doc(doc: _riggen.JointDoc) -> JointSpec:
        cls = _KINDS[doc["kind"]]
        spec = object.__new__(cls)
        limits = None if doc["limits"] is None else Limits.from_doc(doc["limits"])
        JointSpec._init(spec, Pose.from_doc(doc["origin"]), tuple(doc["axis"]), limits, Dynamics.from_doc(doc["dynamics"]))  # type: ignore[arg-type]
        return spec


class Fixed(JointSpec):
    """A rigid connection at ``origin``."""

    kind = "Fixed"

    def __init__(self, origin: PoseLike | None = None) -> None:
        self._init(origin, "z", None, Dynamics())


class Revolute(JointSpec):
    """A hinge about ``axis`` with ``limits`` (radians; ``degrees=True`` for
    a ``(lower, upper)`` pair); ``(-π, π)`` when not given."""

    kind = "Revolute"

    def __init__(
        self,
        axis: Axis = "z",
        *,
        origin: PoseLike | None = None,
        limits: LimitsLike | None = None,
        dynamics: Dynamics = Dynamics(),
        degrees: bool = False,
    ) -> None:
        lim = _limits(limits, degrees) or Limits(-math.pi, math.pi)
        self._init(origin, axis, lim, dynamics)


class Continuous(JointSpec):
    """A hinge about ``axis`` with no limits (a wheel)."""

    kind = "Continuous"

    def __init__(self, axis: Axis = "z", *, origin: PoseLike | None = None, dynamics: Dynamics = Dynamics()) -> None:
        self._init(origin, axis, None, dynamics)


class Prismatic(JointSpec):
    """A slider along ``axis`` with ``limits`` in meters; ``(-1, 1)`` when
    not given."""

    kind = "Prismatic"

    def __init__(
        self,
        axis: Axis = "z",
        *,
        origin: PoseLike | None = None,
        limits: LimitsLike | None = None,
        dynamics: Dynamics = Dynamics(),
    ) -> None:
        self._init(origin, axis, _limits(limits, False) or Limits(-1.0, 1.0), dynamics)


_KINDS: dict[str, type[JointSpec]] = {c.kind: c for c in (Fixed, Revolute, Continuous, Prismatic)}


# ---- inertials, materials, collision ----------------------------------------


@dataclass(frozen=True)
class Inertial:
    """Mass (kg), centre of mass (m, link frame) and the inertia tensor
    about the CoM in link axes (kg·m², three rows) — what a link exports."""

    mass: float
    com: Vec3
    inertia: Matrix3


@dataclass(frozen=True)
class ComputedInertial:
    """From the link's meshes at its material's density, or at ``density``
    (kg/m³) when given. The default for every link."""

    density: float | None = None


@dataclass(frozen=True)
class OverrideInertial:
    """Measured values that replace the computed ones; ``inertia`` is about
    ``com``, in link axes, three rows."""

    mass: float
    com: Vec3
    inertia: Matrix3


@dataclass(frozen=True)
class HybridInertial:
    """The computed tensor and CoM, scaled to a weighed ``mass`` (kg)."""

    mass: float


InertialSpec = Union[ComputedInertial, OverrideInertial, HybridInertial]


def _inertial_spec_to_doc(spec: InertialSpec) -> _riggen.InertialDoc:
    if isinstance(spec, ComputedInertial):
        return {"Computed": {"density_override": spec.density}}
    if isinstance(spec, HybridInertial):
        return {"Hybrid": {"mass": float(spec.mass)}}
    if isinstance(spec, OverrideInertial):
        rows = [_vec3(r, "inertia row") for r in spec.inertia]
        if len(rows) != 3:
            raise ValueError("inertia must be three rows")
        # The document stores the matrix column-major (glam).
        flat = [rows[r][c] for c in range(3) for r in range(3)]
        return {"Override": {"mass": float(spec.mass), "com": list(_vec3(spec.com, "com")), "inertia": flat}}
    raise TypeError(f"not an inertial spec: {spec!r}")


def _inertial_spec_from_doc(doc: _riggen.InertialDoc) -> InertialSpec:
    ((kind, body),) = doc.items()
    if kind == "Computed":
        return ComputedInertial(body["density_override"])
    if kind == "Hybrid":
        return HybridInertial(body["mass"])
    m = body["inertia"]
    rows = ((m[0], m[3], m[6]), (m[1], m[4], m[7]), (m[2], m[5], m[8]))
    return OverrideInertial(body["mass"], tuple(body["com"]), rows)  # type: ignore[arg-type]


_COLLISION_NAMES = {"none": "None", "same_as_visual": "SameAsVisual", "convex_hull": "ConvexHull"}
_COLLISION_DOCS = {v: k for k, v in _COLLISION_NAMES.items()}


@dataclass(frozen=True)
class ConvexDecomposition:
    """Collision geometry as several convex pieces that keep the part's
    concavity, where one convex hull would fill it — a gripper finger, a
    C-bracket, a U-channel. V-HACD; the export writes one mesh file and one
    collision geom per piece (ADR-0011).

    >>> link.collision = riggen.ConvexDecomposition(max_hulls=4)

    The document stores these three numbers and never the pieces, so the
    decomposition is recomputed at export from the mesh as it is then. It
    costs tens of milliseconds to a second per part, on the export, not on
    the assignment.

    ``max_hulls`` is a real ceiling on the piece count. ``resolution`` is
    the side of the voxel grid the part is rasterised into — cost is
    O(n³), and detail thinner than one voxel is invisible to the
    algorithm. ``concavity`` is how much of the part's volume a piece may
    fail to fill, as a fraction of the whole, before it is split again:
    smaller means more pieces and a tighter fit. The defaults are the
    window's.
    """

    max_hulls: int = 8
    resolution: int = 64
    concavity: float = 0.01

    def _to_doc(self) -> dict[str, Any]:
        for name, value in (("max_hulls", self.max_hulls), ("resolution", self.resolution)):
            if int(value) != value or value < 1:
                raise ValueError(f"{name} must be a positive whole number, not {value!r}")
        if not 0.0 <= self.concavity <= 1.0:
            raise ValueError(f"concavity must be between 0 and 1, not {self.concavity!r}")
        return {
            "ConvexDecomposition": {
                "max_hulls": int(self.max_hulls),
                "resolution": int(self.resolution),
                "concavity": float(self.concavity),
            }
        }

    @staticmethod
    def _from_doc(body: dict[str, Any]) -> "ConvexDecomposition":
        return ConvexDecomposition(
            max_hulls=body["max_hulls"],
            resolution=body["resolution"],
            concavity=body["concavity"],
        )


@dataclass(frozen=True)
class Material:
    """A density (kg/m³) for computed inertials and a linear RGBA colour for
    the viewport (stored as 32-bit floats, so ``0.8`` reads back as
    ``0.800000011920929``)."""

    density: float
    color: tuple[float, float, float, float] = (0.7, 0.7, 0.7, 1.0)


# ---- handles ----------------------------------------------------------------


class _Handle:
    """A reference into a :class:`Robot` by id. Stays valid across edits;
    after the thing it names is removed, its methods raise
    :class:`riggen.UnknownId`."""

    __slots__ = ("robot", "id")

    def __init__(self, robot: Robot, id: int) -> None:
        self.robot = robot
        self.id = id

    def __eq__(self, other: object) -> bool:
        return type(other) is type(self) and other.robot is self.robot and other.id == self.id  # type: ignore[attr-defined]

    def __hash__(self) -> int:
        return hash((id(self.robot), self.id))


class Geom(_Handle):
    """One visual mesh on a link, at a pose in the link frame."""

    __slots__ = ("link",)

    def __init__(self, link: Link, id: int) -> None:
        super().__init__(link.robot, id)
        self.link = link

    @property
    def _doc(self) -> _riggen.GeomDoc:
        for geom in self.robot._inner.links()[self.link.id]["visuals"]:
            if geom["id"] == self.id:
                return geom
        raise _unknown("geom", self.id)

    @property
    def pose(self) -> Pose:
        """The mesh in the link frame."""
        return Pose.from_doc(self._doc["pose"])

    @pose.setter
    def pose(self, value: PoseLike) -> None:
        self.robot._inner.set_geom_pose(self.link.id, self.id, _pose(value).to_doc())

    @property
    def mesh(self) -> Path:
        """The mesh file (absolute)."""
        return Path(self.robot._inner.assets()[self._doc["mesh"]]["path"])

    def remove(self) -> None:
        """Removes this geom from its link."""
        self.robot._inner.remove_geom(self.link.id, self.id)

    def __repr__(self) -> str:
        return f"Geom({self.mesh.name!r} on {self.link.name!r})"


class Frame(_Handle):
    """A named pose on a link — a TCP, a sensor mount, a grasp pose. It
    carries no mass and no geometry: an MJCF ``<site>`` and a URDF massless
    dummy link on a fixed joint (ADR-0012). Its name shares the links'
    namespace, so it may not repeat a link's."""

    __slots__ = ()

    @property
    def _doc(self) -> _riggen.FrameDoc:
        try:
            return self.robot._inner.frames()[self.id]
        except KeyError:
            raise _unknown("frame", self.id) from None

    def _set(self, **changes: Any) -> None:
        doc = dict(self._doc)
        doc.update(changes)
        self.robot._inner.set_frame(self.id, doc)  # type: ignore[arg-type]

    @property
    def name(self) -> str:
        """Unique among frames *and* links: URDF writes both as ``<link>``."""
        return self._doc["name"]

    @name.setter
    def name(self, value: str) -> None:
        self.robot._inner.rename_frame(self.id, value)

    @property
    def parent(self) -> Link:
        """The link this frame hangs on."""
        return Link(self.robot, self._doc["parent"])

    @parent.setter
    def parent(self, value: Link) -> None:
        """Moves the frame to another link, **keeping its stored pose** —
        so it moves in the world. To keep the world pose, read
        :meth:`world` first and write it back through :attr:`pose`."""
        self._set(parent=value.id)

    @property
    def pose(self) -> Pose:
        """The frame in its link's frame."""
        return Pose.from_doc(self._doc["pose"])

    @pose.setter
    def pose(self, value: PoseLike) -> None:
        self._set(pose=_pose(value).to_doc())

    def world(self, q: dict[str | Joint, float] | None = None) -> Pose:
        """This frame's world pose at the joint values ``q`` (by name or
        handle; missing joints at zero)."""
        return self.robot.frame_poses(q)[self.name]

    def remove(self) -> None:
        """Removes this frame. Nothing else changes: nothing hangs off it."""
        self.robot._inner.remove_frame(self.id)

    def __repr__(self) -> str:
        return f"Frame({self.name!r} on {self.parent.name!r})"


class Link(_Handle):
    """A body of the tree. Its :attr:`joint` connects it to :attr:`parent`;
    the root has neither. Properties that set are one edit each."""

    __slots__ = ()

    @property
    def _doc(self) -> _riggen.LinkDoc:
        try:
            return self.robot._inner.links()[self.id]
        except KeyError:
            raise _unknown("link", self.id) from None

    @property
    def name(self) -> str:
        """The link's name — the body name in the export; unique, an identifier."""
        return self._doc["name"]

    @name.setter
    def name(self, value: str) -> None:
        self.robot._inner.rename_link(self.id, value)

    @property
    def material(self) -> str | None:
        """The material's name (a key of :attr:`Robot.materials`), or ``None``."""
        return self._doc["material"]

    @material.setter
    def material(self, value: str | None) -> None:
        self.robot._inner.set_link_material(self.id, value)

    @property
    def joint(self) -> Joint | None:
        """The joint this link hangs from; ``None`` for the root."""
        j = self.robot._inner.parent_joint(self.id)
        return None if j is None else Joint(self.robot, j)

    @property
    def parent(self) -> Link | None:
        """The link this one hangs from; ``None`` for the root."""
        j = self.joint
        return None if j is None else j.parent

    @property
    def joints(self) -> list[Joint]:
        """The joints to this link's children, in creation order."""
        return [Joint(self.robot, j) for j in self.robot._inner.child_joints(self.id)]

    @property
    def children(self) -> list[Link]:
        """The links hanging from this one."""
        return [j.child for j in self.joints]

    @property
    def subtree(self) -> list[Link]:
        """This link and every descendant, parents before children."""
        return [Link(self.robot, l) for l in self.robot._inner.subtree(self.id)]

    @property
    def geoms(self) -> list[Geom]:
        """The visual meshes of this link, in the order added."""
        return [Geom(self, g["id"]) for g in self._doc["visuals"]]

    @property
    def frames(self) -> list[Frame]:
        """The named frames on this link, in creation order."""
        return [f for f in self.robot.frames if f.parent == self]

    def add_frame(self, name: str, pose: PoseLike | None = None) -> Frame:
        """A named frame on this link at ``pose`` (identity by default) —
        a TCP, a sensor mount. ``name`` may not repeat a link's or another
        frame's (ADR-0012)."""
        return Frame(self.robot, self.robot._inner.add_frame(name, self.id, pose=_pose(pose).to_doc()))

    def add_mesh(
        self,
        path: PathLike,
        *,
        pose: PoseLike | None = None,
        scale: float = 1.0,
        fix_up: Quat | None = None,
        color: tuple[float, float, float, float] | None = None,
    ) -> Geom:
        """Registers the mesh file and adds it as a visual of this link.
        ``scale`` converts the file's unit to meters (``0.001`` for
        millimetres); ``fix_up`` (``(w, x, y, z)``) rotates a Y-up file to
        Z-up and the like, after scaling."""
        inner = self.robot._inner
        mesh = inner.add_asset(path, scale=scale, fix_up=_fix_up(fix_up))
        geom = inner.add_geom(self.id, mesh, pose=_pose(pose).to_doc(), color=None if color is None else list(color))
        return Geom(self, geom)

    def add_link(
        self,
        name: str,
        joint: JointSpec,
        *,
        mesh: PathLike | None = None,
        scale: float = 1.0,
        fix_up: Quat | None = None,
        material: str | None = None,
        joint_name: str | None = None,
    ) -> Link:
        """A new child link under this one, hanging from ``joint``. With
        ``mesh``, the file is its visual (at identity in the link frame —
        move it with :attr:`Geom.pose`). The joint is named
        ``f"{name}_joint"`` unless ``joint_name`` says otherwise."""
        return self.robot.add_link(name, self, joint, mesh=mesh, scale=scale, fix_up=fix_up, material=material, joint_name=joint_name)

    def remove(self) -> None:
        """Removes this link, its joint and its whole subtree. The root
        cannot be removed."""
        self.robot._inner.remove_link(self.id)

    def reparent(self, new_parent: Link, *, keep_world_pose: bool = True) -> None:
        """Hangs this link (with its subtree) under ``new_parent``. With
        ``keep_world_pose`` the joint origin is rewritten so nothing moves
        in the zero configuration; without it the origin is kept and the
        part jumps."""
        self.robot._inner.reparent(self.id, new_parent.id, keep_world_pose=keep_world_pose)

    def place(self, world: PoseLike) -> None:
        """Writes this link's joint origin so the link sits at ``world`` in
        the zero configuration — "put it there", whatever the parent is.
        Not for the root."""
        origin = self.robot._inner.origin_for_world(self.id, _pose(world, "world").to_doc())
        if origin is None:
            raise ValueError("the root link has no joint to place")
        joint = self.joint
        assert joint is not None
        joint.origin = Pose.from_doc(origin)

    def make_root(self) -> None:
        """Makes this link the root, reversing the fixed joints on the way
        up; refused across a movable joint."""
        self.robot._inner.set_root(self.id)

    @property
    def collision(self) -> CollisionPolicy:
        """``"none"``, ``"same_as_visual"`` (the default) or
        ``"convex_hull"``, or a :class:`ConvexDecomposition`; other policies
        come back as the document's value."""
        doc = self._doc["collision"]
        if isinstance(doc, str):
            return _COLLISION_DOCS.get(doc, doc)  # type: ignore[return-value]
        if set(doc) == {"ConvexDecomposition"}:
            return ConvexDecomposition._from_doc(doc["ConvexDecomposition"])
        return doc

    @collision.setter
    def collision(self, value: CollisionPolicy) -> None:
        if isinstance(value, str):
            if value not in _COLLISION_NAMES:
                raise ValueError(
                    f"collision must be one of {sorted(_COLLISION_NAMES)}, "
                    f"a ConvexDecomposition, or a document value, not {value!r}"
                )
            self.robot._inner.set_collision(self.id, _COLLISION_NAMES[value])
        elif isinstance(value, ConvexDecomposition):
            self.robot._inner.set_collision(self.id, value._to_doc())
        else:
            self.robot._inner.set_collision(self.id, value)

    @property
    def inertial_spec(self) -> InertialSpec:
        """How the exported inertial is obtained: :class:`ComputedInertial`
        (the default), :class:`OverrideInertial` or :class:`HybridInertial`."""
        return _inertial_spec_from_doc(self._doc["inertial"])

    @inertial_spec.setter
    def inertial_spec(self, value: InertialSpec) -> None:
        self.robot._inner.set_inertial(self.id, _inertial_spec_to_doc(value))

    @property
    def inertial(self) -> Inertial:
        """The inertial this link exports, with its meshes read from disk.
        Raises :class:`riggen.InertialError` when it cannot be computed."""
        mass, com, rows = self.robot._inner.inertial(self.id)
        return Inertial(mass, tuple(com), tuple(tuple(r) for r in rows))  # type: ignore[arg-type]

    def __repr__(self) -> str:
        return f"Link({self.name!r})"


@dataclass(frozen=True)
class Mimic:
    """A coupled degree of freedom: ``q(this) = multiplier * q(joint) +
    offset``. Exported as URDF's ``<mimic>`` and as an MJCF
    ``<equality><joint polycoef>`` — a *soft* solver constraint there, not
    a reduction (ADR-0013).

    ``joint`` is the leader: a movable joint that is not the follower and
    does not itself follow one — chains are refused, as is a leader whose
    range, mapped through ``(multiplier, offset)``, leaves the follower's
    own limits."""

    joint: Joint
    multiplier: float = 1.0
    offset: float = 0.0

    def to_doc(self) -> _riggen.MimicDoc:
        return {"joint": self.joint.id, "multiplier": float(self.multiplier), "offset": float(self.offset)}

    @classmethod
    def from_doc(cls, robot: Robot, doc: _riggen.MimicDoc) -> Mimic:
        return cls(Joint(robot, doc["joint"]), doc["multiplier"], doc["offset"])


class Joint(_Handle):
    """The edge from :attr:`parent` to :attr:`child`. Setting
    :attr:`origin`, :attr:`axis`, :attr:`limits`, :attr:`dynamics`,
    :attr:`mimic` or :attr:`spec` is one edit each; the endpoints change
    only through :meth:`Link.reparent`."""

    __slots__ = ()

    @property
    def _doc(self) -> _riggen.JointDoc:
        try:
            return self.robot._inner.joints()[self.id]
        except KeyError:
            raise _unknown("joint", self.id) from None

    def _set(self, **changes: Any) -> None:
        doc = dict(self._doc)
        doc.update(changes)
        self.robot._inner.set_joint(self.id, doc)  # type: ignore[arg-type]

    @property
    def name(self) -> str:
        """The joint's name — as exported; unique, an identifier."""
        return self._doc["name"]

    @name.setter
    def name(self, value: str) -> None:
        self.robot._inner.rename_joint(self.id, value)

    @property
    def kind(self) -> Literal["fixed", "revolute", "continuous", "prismatic"]:
        """Set by assigning a :attr:`spec`."""
        return self._doc["kind"].lower()  # type: ignore[return-value]

    @property
    def parent(self) -> Link:
        """The link this joint is attached to."""
        return Link(self.robot, self._doc["parent"])

    @property
    def child(self) -> Link:
        """The link this joint moves; the joint frame is its link frame."""
        return Link(self.robot, self._doc["child"])

    @property
    def spec(self) -> JointSpec:
        """Kind, origin, axis, limits and dynamics as one value; assign a
        :class:`Revolute` (etc.) to retype the joint in one edit."""
        return JointSpec.from_doc(self._doc)

    @spec.setter
    def spec(self, value: JointSpec) -> None:
        # Retyping a joint does not decouple it — the mimic is the joint's,
        # not the spec's — but a fixed joint has no value to drive, so the
        # coupling goes with the kind (ADR-0013).
        mimic = None if value.kind == "Fixed" else self._doc["mimic"]
        self.robot._inner.set_joint(self.id, value.to_doc(self.name, mimic))

    @property
    def origin(self) -> Pose:
        """The child link frame in the parent link frame."""
        return Pose.from_doc(self._doc["origin"])

    @origin.setter
    def origin(self, value: PoseLike) -> None:
        self._set(origin=_pose(value, "origin").to_doc())

    @property
    def axis(self) -> Vec3:
        """The unit axis in the child frame."""
        return tuple(self._doc["axis"])  # type: ignore[return-value]

    @axis.setter
    def axis(self, value: Axis) -> None:
        self._set(axis=list(_axis(value)))

    @property
    def limits(self) -> Limits | None:
        """``None`` for a fixed or continuous joint; a ``(lower, upper)``
        pair (radians / meters) is accepted when setting."""
        return None if self._doc["limits"] is None else Limits.from_doc(self._doc["limits"])

    @limits.setter
    def limits(self, value: LimitsLike | None) -> None:
        lim = _limits(value, False)
        self._set(limits=None if lim is None else lim.to_doc())

    @property
    def dynamics(self) -> Dynamics:
        """Damping, friction, armature."""
        return Dynamics.from_doc(self._doc["dynamics"])

    @dynamics.setter
    def dynamics(self, value: Dynamics) -> None:
        self._set(dynamics=value.to_doc())

    @property
    def mimic(self) -> Mimic | None:
        """The joint this one follows, or ``None`` — see :class:`Mimic`."""
        doc = self._doc["mimic"]
        return None if doc is None else Mimic.from_doc(self.robot, doc)

    @mimic.setter
    def mimic(self, value: Mimic | None) -> None:
        self._set(mimic=None if value is None else value.to_doc())

    def move_frame(self, origin: PoseLike, axis: Axis | None = None) -> None:
        """Moves the pivot without moving anything in the world: the child's
        geoms, joints and frames are re-expressed so every pose in the zero
        configuration stays. ``axis`` is in the *new* child frame; the old
        one is kept when not given."""
        new_axis = self.axis if axis is None else _axis(axis)
        self.robot._inner.move_joint_frame(self.id, _pose(origin, "origin").to_doc(), list(new_axis))

    def __repr__(self) -> str:
        return f"Joint({self.name!r}: {self.parent.name!r} -> {self.child.name!r}, {self.kind})"


def _unknown(kind: str, id: int) -> Exception:
    from .errors import UnknownId

    prefix = {"link": "l", "joint": "j", "geom": "g", "frame": "f"}[kind]
    return UnknownId(f"no {kind} {prefix}{id} in the document")


def _fix_up(value: Quat | None) -> list[float] | None:
    if value is None:
        return None
    w, x, y, z = _floats(value, 4, "fix_up")
    return [x, y, z, w]


# ---- the robot --------------------------------------------------------------


class Robot:
    """The document: a tree of :class:`Link` s joined by :class:`Joint` s,
    plus materials and the mesh files it references. Starts as one root link
    ``base_link`` and the default materials (aluminium, steel, PLA, ABS,
    nylon, rubber).

    >>> robot = Robot("pendulum")
    >>> robot.root.add_mesh("base.stl")
    >>> arm = robot.root.add_link("arm", Revolute("y", origin=(0, 0, 0.5)), mesh="arm.stl", material="PLA")
    >>> robot.export("out", format="mjcf")
    """

    __slots__ = ("_inner",)

    def __init__(self, name: str = "robot") -> None:
        self._inner = _riggen.Robot(name)

    @classmethod
    def _wrap(cls, inner: _riggen.Robot) -> Robot:
        robot = object.__new__(cls)
        robot._inner = inner
        return robot

    # -- reading -------------------------------------------------------------

    @property
    def name(self) -> str:
        """The model name — the exported file stem."""
        return self._inner.name

    @name.setter
    def name(self, value: str) -> None:
        self._inner.name = value

    @property
    def root(self) -> Link:
        """The fixed base every other link hangs from — ``base_link`` at first."""
        return Link(self, self._inner.root)

    @property
    def links(self) -> list[Link]:
        """Every link, in creation order."""
        return [Link(self, l) for l in self._inner.links()]

    @property
    def joints(self) -> list[Joint]:
        """Every joint, in creation order."""
        return [Joint(self, j) for j in self._inner.joints()]

    @property
    def frames(self) -> list[Frame]:
        """Every named frame, in creation order."""
        return [Frame(self, f) for f in self._inner.frames()]

    @property
    def materials(self) -> dict[str, Material]:
        """By name; edit with :meth:`add_material` / :meth:`remove_material`."""
        return {name: Material(m["density"], tuple(m["color"])) for name, m in self._inner.materials().items()}  # type: ignore[arg-type]

    def link(self, name: str) -> Link:
        """The link called ``name``; ``KeyError`` when there is none."""
        id = self._inner.link(name)
        if id is None:
            raise KeyError(f"no link named {name!r}")
        return Link(self, id)

    def joint(self, name: str) -> Joint:
        """The joint called ``name``; ``KeyError`` when there is none."""
        id = self._inner.joint(name)
        if id is None:
            raise KeyError(f"no joint named {name!r}")
        return Joint(self, id)

    def frame(self, name: str) -> Frame:
        """The frame called ``name``; ``KeyError`` when there is none."""
        id = self._inner.frame(name)
        if id is None:
            raise KeyError(f"no frame named {name!r}")
        return Frame(self, id)

    # -- building ------------------------------------------------------------

    def add_link(
        self,
        name: str,
        parent: Link | str,
        joint: JointSpec,
        *,
        mesh: PathLike | None = None,
        scale: float = 1.0,
        fix_up: Quat | None = None,
        material: str | None = None,
        joint_name: str | None = None,
    ) -> Link:
        """A new link under ``parent`` (a handle or a name), hanging from
        ``joint``; see :meth:`Link.add_link`."""
        parent_id = self.link(parent).id if isinstance(parent, str) else parent.id
        id = self._inner.add_link(
            name,
            parent_id,
            joint.to_doc(joint_name or f"{name}_joint"),
            mesh=mesh,
            scale=scale,
            fix_up=_fix_up(fix_up),
            material=material,
        )
        return Link(self, id)

    def add_material(self, name: str, density: float, color: tuple[float, float, float, float] | None = None) -> None:
        """Adds or replaces a material; ``density`` in kg/m³, ``color`` linear RGBA."""
        material = Material(density) if color is None else Material(density, color)
        self._inner.upsert_material(name, {"density": float(material.density), "color": list(material.color)})

    def remove_material(self, name: str) -> None:
        """Removes a material no link uses."""
        self._inner.remove_material(name)

    # -- kinematics and export -----------------------------------------------

    def fk(self, q: dict[str | Joint, float] | None = None) -> dict[str, Pose]:
        """The world pose of every link, by name, for the joint values ``q``
        (by name or handle; radians or meters; missing joints at zero)."""
        state = {(self.joint(k).id if isinstance(k, str) else k.id): float(v) for k, v in (q or {}).items()}
        names = {id: link["name"] for id, link in self._inner.links().items()}
        return {names[id]: Pose.from_doc(pose) for id, pose in self._inner.fk(state).items()}

    def frame_poses(self, q: dict[str | Joint, float] | None = None) -> dict[str, Pose]:
        """The world pose of every named frame, by name, for the joint
        values ``q`` — ``world(link) ∘ frame.pose``. :meth:`fk` stays links
        only, because it is what the export is checked against."""
        state = {(self.joint(k).id if isinstance(k, str) else k.id): float(v) for k, v in (q or {}).items()}
        names = {id: frame["name"] for id, frame in self._inner.frames().items()}
        return {names[id]: Pose.from_doc(pose) for id, pose in self._inner.fk_frames(state).items()}

    def validate(self) -> list[str]:
        """Every invariant the document breaks, as messages; empty for a
        document built through this API, which refuses such edits."""
        return self._inner.validate()

    def save(self, path: PathLike) -> None:
        """Writes the ``.riggen`` file (mesh paths relative to it)."""
        self._inner.save(path)

    def export(
        self,
        dir: PathLike,
        *,
        format: Literal["mjcf", "urdf", "both"] = "both",
        mesh_paths: str = "relative",
        floating_base: bool = False,
        fk_samples: bool = False,
    ) -> list[Path]:
        """Writes ``<name>.xml`` (MJCF) and/or ``<name>.urdf`` into ``dir``
        beside ``meshes/`` (binary STL in meters); with ``fk_samples``,
        ``<name>.fk.json`` too. ``mesh_paths`` (URDF only) is
        ``"relative"``, ``"absolute"`` or ``"package://<name>"``. Returns
        every path written; raises :class:`riggen.ExportError` listing every
        reason the document cannot be exported."""
        return self._inner.export(dir, format=format, mesh_paths=mesh_paths, floating_base=floating_base, fk_samples=fk_samples)

    def to_json(self) -> str:
        """The document as ``.riggen`` JSON text (mesh paths absolute)."""
        return self._inner.to_json()

    @classmethod
    def from_json(cls, text: str) -> Robot:
        """The inverse of :meth:`to_json`."""
        return cls._wrap(_riggen.Robot.from_json(text))

    def copy(self) -> Robot:
        """An independent copy."""
        return Robot._wrap(self._inner.copy())

    def __repr__(self) -> str:
        return f"Robot({self.name!r}: {len(self._inner.links())} links, {len(self._inner.joints())} joints)"


def _warn(messages: list[str]) -> None:
    for message in messages:
        warnings.warn(message, RiggenWarning, stacklevel=3)


def load(path: PathLike) -> Robot:
    """Reads a ``.riggen`` file. A mesh file that changed or went missing
    since the save is a :class:`riggen.RiggenWarning`, not an error."""
    inner, warned = _riggen.Robot.load(path)
    _warn(warned)
    return Robot._wrap(inner)


def load_urdf(path: PathLike, packages: dict[str, PathLike] | None = None) -> Robot:
    """Imports a URDF; ``packages`` maps ``package://name/`` prefixes to
    directories. What the URDF held that the document does not (primitive
    visuals, a ``<safety_controller>``, …) is a
    :class:`riggen.RiggenWarning`."""
    inner, warned = _riggen.Robot.load_urdf(path, packages)
    _warn(warned)
    return Robot._wrap(inner)
