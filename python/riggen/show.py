""":func:`show`: open a document in the ``riggen`` window from a script, keep
scripting, and get the document back as the window saved it.

The GUI is never entered from inside a Python call (ADR-0002, ADR-0009):
:func:`show` writes the robot to a temporary ``.riggen`` file, spawns the
bundled ``riggen`` executable on it, and returns a :class:`Viewer`.
:meth:`Viewer.wait` blocks until the window closes and returns the document
re-read from that file if the window saved it, else the robot that was
passed in — the "place the joint by hand, keep scripting" loop.

The executable is the one in this interpreter's ``bin/`` (the wheel's), or
whatever ``RIGGEN_BINARY`` points at; :func:`binary_path` is shared with
``python -m riggen``.
"""

from __future__ import annotations

import hashlib
import os
import subprocess
import sys
import sysconfig
import tempfile
from pathlib import Path

from .robot import Robot, load

__all__ = ["show", "Viewer", "binary_path"]

REPOSITORY = "https://github.com/Divelix/riggen"


def binary_path() -> Path:
    """The ``riggen`` executable: ``RIGGEN_BINARY`` if set, else the one the
    wheel put beside this interpreter (``sysconfig``'s scripts directory, the
    user-site layout second). Raises ``FileNotFoundError`` with the ways to
    get one when there is none — a source build from the sdist installs the
    SDK alone."""
    override = os.environ.get("RIGGEN_BINARY")
    if override:
        path = Path(override)
        if path.is_file():
            return path
        raise FileNotFoundError(f"RIGGEN_BINARY={override!r} is not a file")
    name = "riggen.exe" if sys.platform == "win32" else "riggen"
    looked = []
    for scheme in (None, f"{os.name}_user"):
        scripts = sysconfig.get_path("scripts") if scheme is None else sysconfig.get_path("scripts", scheme)
        candidate = Path(scripts) / name
        if candidate.is_file():
            return candidate
        looked.append(scripts)
    raise FileNotFoundError(
        f"the bundled riggen executable is missing (looked in {', '.join(looked)}). "
        "This install has no binary — a build from the source distribution gets the SDK "
        "alone. Install a wheel for your platform (`pip install --force-reinstall riggen`), "
        f"build the app with `cargo install --git {REPOSITORY} riggen-app`, "
        "or point RIGGEN_BINARY at one."
    )


def _command(binary: Path, *args: str | os.PathLike[str]) -> list[str]:
    """`RIGGEN_BINARY` may name a Python script (the SDK's own tests do), which
    runs under this interpreter."""
    argv = [str(a) for a in args]
    if binary.suffix == ".py":
        return [sys.executable, str(binary), *argv]
    return [str(binary), *argv]


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class Viewer:
    """A running ``riggen`` window on a temporary copy of a robot; returned by
    :func:`show`. :attr:`path` is the ``.riggen`` file it opened."""

    def __init__(self, robot: Robot, path: Path, process: subprocess.Popen[bytes]) -> None:
        self._robot = robot
        self.path = path
        self.process = process
        self._saved = _digest(path)
        self._result: Robot | None = None

    def poll(self) -> int | None:
        """The window's exit code, or ``None`` while it is open."""
        return self.process.poll()

    def wait(self, timeout: float | None = None) -> Robot:
        """Blocks until the window closes (``subprocess.TimeoutExpired`` after
        ``timeout`` seconds) and returns the document: re-read from
        :attr:`path` if the window saved it there, else the very robot
        :func:`show` was given. Idempotent."""
        self.process.wait(timeout)
        if self._result is None:
            self._result = load(self.path) if _digest(self.path) != self._saved else self._robot
        return self._result

    @property
    def robot(self) -> Robot:
        """What :meth:`wait` returned; before the window closes, the robot
        :func:`show` was given."""
        return self._result if self._result is not None else self._robot

    def kill(self) -> None:
        """Closes the window without saving."""
        if self.process.poll() is None:
            self.process.kill()
            self.process.wait()

    def __repr__(self) -> str:
        state = "open" if self.poll() is None else f"exited {self.poll()}"
        return f"Viewer({self.path.name!r}, {state})"


def show(robot: Robot, *, block: bool = False) -> Viewer:
    """Opens ``robot`` in the ``riggen`` window and returns at once (or, with
    ``block``, once the window has closed). Edit and save there; then
    :meth:`Viewer.wait` hands the saved document back.

    >>> viewer = riggen.show(robot)      # place the joint by hand, Ctrl+S
    >>> robot = viewer.wait()            # the document as the window saved it
    """
    binary = binary_path()
    directory = Path(tempfile.mkdtemp(prefix="riggen-show-"))
    path = directory / f"{robot.name}.riggen"
    robot.save(path)
    process = subprocess.Popen(_command(binary, path))
    viewer = Viewer(robot, path, process)
    if block:
        viewer.wait()
    return viewer
