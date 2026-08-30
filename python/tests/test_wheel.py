"""The wheel smoke test (plans/m4-distribution, plans/python-sdk): does the
installed `riggen` run on a machine that never saw Rust, and does its
extension module import?

    python python/build_wheel.py
    uv venv target/wheel-venv && uv pip install --python target/wheel-venv dist/riggen-*.whl
    python python/tests/test_wheel.py target/wheel-venv

Takes one argument, a virtual environment the wheel is installed into, and
runs from the repository root (the sample arm is read from
`assets/fixtures/arm/`). Headless on purpose: no CI runner has a display,
so the window itself is the human's half of the acceptance. Plain script,
no pytest — it is what the `wheel` CI job and `release.yml`'s smoke jobs
run, and nothing else.
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ARM = ROOT / "assets" / "fixtures" / "arm" / "arm.riggen"
VERSION_LINE = re.compile(r"^riggen \d+\.\d+\.\d+ \(\S+ \S+\)$")


def scripts_dir(venv: Path) -> Path:
    return venv / ("Scripts" if sys.platform == "win32" else "bin")


def run(args: list[str | os.PathLike[str]], **kwargs) -> subprocess.CompletedProcess[str]:
    print("$", " ".join(str(a) for a in args))
    return subprocess.run(args, check=True, capture_output=True, text=True, **kwargs)


def check_export(riggen: Path) -> None:
    with tempfile.TemporaryDirectory(prefix="riggen-wheel-") as tmp:
        out = Path(tmp) / "arm"
        result = run([riggen, "--export", "mjcf", "--out", out, ARM])
        written = [Path(line) for line in result.stdout.splitlines()]
        assert (out / "arm.xml").is_file(), result.stdout
        assert (out / "meshes" / "base.stl").is_file(), result.stdout
        missing = [p for p in written if not p.is_file()]
        assert not missing, f"listed but not written: {missing}"
        print(f"  wrote {len(written)} files")


def check_version(riggen: Path, python: Path) -> None:
    direct = run([riggen, "--version"]).stdout.strip()
    assert VERSION_LINE.match(direct), f"unexpected --version output: {direct!r}"
    print(f"  {direct}")
    via_module = run([python, "-m", "riggen", "--version"]).stdout.strip()
    assert via_module == direct, f"python -m riggen says {via_module!r}, the binary {direct!r}"
    help_text = run([riggen, "--help"]).stdout
    assert "usage:" in help_text and "--export" in help_text, help_text


def check_extension(python: Path) -> None:
    """`import riggen._riggen` works, agrees on the version, and the wheel is
    the abi3 one (ADR-0009) — one wheel per platform for every CPython ≥ 3.10."""
    code = (
        "import riggen, riggen._riggen as m; from importlib.metadata import distribution;"
        " assert m.__version__ == riggen.__version__, (m.__version__, riggen.__version__);"
        " print(m.__version__); print(distribution('riggen').read_text('WHEEL'))"
    )
    out = run([python, "-c", code]).stdout
    version, wheel = out.split("\n", 1)
    tags = [line.split(":", 1)[1].strip() for line in wheel.splitlines() if line.startswith("Tag:")]
    assert tags and all(t.startswith("cp310-abi3-") for t in tags), f"wheel tags: {tags}"
    print(f"  riggen._riggen {version}, tags {tags}")


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    venv = Path(argv[1]).resolve()
    scripts = scripts_dir(venv)
    riggen = scripts / ("riggen.exe" if sys.platform == "win32" else "riggen")
    python = scripts / ("python.exe" if sys.platform == "win32" else "python")
    assert riggen.is_file(), f"no riggen binary in {scripts}"
    assert python.is_file(), f"no python in {scripts}"

    check_version(riggen, python)
    check_export(riggen)
    check_extension(python)
    print("ok")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
