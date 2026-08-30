"""Build the `riggen` wheel: both halves, one recipe (ADR-0009).

    python python/build_wheel.py [--target <triple>] [--binary-only] [-- <maturin args>]

1. `cargo build --release -p riggen-app [--target T]` — the native app.
2. Copy `target/[T/]release/riggen[.exe]` into `riggen._riggen.data/scripts/`,
   maturin's wheel data directory (`pyproject.toml`); maturin puts it in
   `riggen-<ver>.data/scripts/`, which installs to the environment's `bin/`.
3. `maturin build --release --out dist [--target T]` — the extension module
   `riggen._riggen` from `crates/riggen-py`, the Python package, and the
   data directory, into one `cp310-abi3-<platform>` wheel.

`--binary-only` stops after 2: CI's manylinux container runs maturin itself
(maturin-action), this script only fills the data directory first
(`before-script-linux`). This file is the one place the recipe lives, for
the human, `ci.yml` and `release.yml`. Runs from anywhere; paths are
relative to the repository root. Uses `maturin` from PATH, else `uvx maturin`.
"""

from __future__ import annotations

import argparse
import os
import shutil
import stat
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DATA_SCRIPTS = ROOT / "riggen._riggen.data" / "scripts"


def run(args: list[str], **kwargs) -> None:
    print("$", " ".join(args), flush=True)
    subprocess.run(args, check=True, cwd=ROOT, **kwargs)


def binary_name(target: str | None) -> str:
    windows = "windows" in target if target else sys.platform == "win32"
    return "riggen.exe" if windows else "riggen"


def build_binary(target: str | None) -> Path:
    cmd = ["cargo", "build", "--release", "-p", "riggen-app"]
    if target:
        cmd += ["--target", target]
    run(cmd)
    built = ROOT / "target" / (target or "") / "release" / binary_name(target)
    if not built.is_file():
        sys.exit(f"build_wheel: cargo built nothing at {built}")
    if DATA_SCRIPTS.exists():
        shutil.rmtree(DATA_SCRIPTS)
    DATA_SCRIPTS.mkdir(parents=True)
    dest = DATA_SCRIPTS / built.name
    shutil.copy2(built, dest)
    dest.chmod(dest.stat().st_mode | stat.S_IXUSR | stat.S_IXGRP | stat.S_IXOTH)
    print(f"  {dest.relative_to(ROOT)}: {dest.stat().st_size / 1e6:.1f} MB")
    return dest


def build_wheel(target: str | None, extra: list[str]) -> None:
    maturin = [shutil.which("maturin") or "uvx", *([] if shutil.which("maturin") else ["maturin"])]
    cmd = [*maturin, "build", "--release", "--out", "dist"]
    if target:
        cmd += ["--target", target]
    run(cmd + extra)
    wheels = sorted((ROOT / "dist").glob("riggen-*.whl"), key=lambda p: p.stat().st_mtime)
    if wheels:
        wheel = wheels[-1]
        print(f"  {wheel.relative_to(ROOT)}: {wheel.stat().st_size / 1e6:.1f} MB")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--target", help="Rust target triple, passed to both cargo and maturin")
    parser.add_argument("--binary-only", action="store_true", help="fill the data directory and stop")
    parser.add_argument("extra", nargs="*", help="arguments after `--` go to `maturin build`")
    args = parser.parse_args(argv[1:])
    build_binary(args.target)
    if not args.binary_only:
        build_wheel(args.target, args.extra)
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
