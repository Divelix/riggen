"""The exceptions riggen raises. One hierarchy, rooted at :class:`RiggenError`.

The extension module (``crates/riggen-py``) raises these by name with the
Rust error's message, so ``except riggen.EditError`` catches every refused
edit and ``except riggen.RiggenError`` everything riggen itself reports.
The :class:`EditError` subclasses are ``riggen_core::EditError``'s variants,
one class each (docs/02-data-model.md §Commands and history).
"""

from __future__ import annotations

__all__ = [
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


class RiggenError(Exception):
    """Base of every error riggen raises."""


class RiggenWarning(UserWarning):
    """Something worth knowing about a file that did open: a mesh that
    changed or went missing since the save (:func:`riggen.load`), or what a
    URDF held that the document does not (:func:`riggen.load_urdf`).
    Emitted through :mod:`warnings`."""


class EditError(RiggenError):
    """An edit was refused. The document is exactly as it was before the call,
    the id counter included."""


class InvalidDocument(EditError):
    """The edit's result would break a document invariant — a duplicate name,
    a revolute joint without limits, a zero axis, a dangling reference
    (``riggen_core::ValidationError``)."""


class UnknownId(EditError):
    """An id the call names is not in the document; the message says which
    kind (link, joint, geom, mesh) and which id."""


class UnknownMaterial(EditError):
    """No material of that name in the document."""


class WouldCreateCycle(EditError):
    """``reparent``: the new parent is the link itself or one of its
    descendants."""


class CannotRemoveRoot(EditError):
    """``remove_link`` on the root."""


class CannotReparentRoot(EditError):
    """``reparent`` of the root."""


class MaterialInUse(EditError):
    """``remove_material`` while a link uses the material; the message names
    the lowest such link."""


class MovableJointOnRootPath(EditError):
    """``set_root`` across a revolute, continuous or prismatic joint: only
    fixed joints can be reversed."""


class ValidationError(RiggenError):
    """The document breaks an invariant (``check()``, ``Robot.from_json``)."""


class FileError(RiggenError):
    """A ``.riggen`` file could not be read or written: I/O, malformed JSON,
    an unsupported schema version, or an invalid document on disk."""


class ExportError(RiggenError):
    """``export`` refused: the message lists every reason."""


class UrdfImportError(RiggenError):
    """``load_urdf`` could not turn the file into a document."""


class InertialError(RiggenError):
    """``inertial`` could not be computed — no mesh, an open mesh, a zero
    density."""
