"""The M2 arm from its STLs, joints typed by hand, exported for MuJoCo.

    python examples/arm.py [--out DIR] [--format mjcf|urdf|both]

The four parts are `assets/fixtures/arm/*.stl` in millimetres. A fixed
`base` on the root, then shoulder (yaw), upper and fore (pitch), each
revolute with limits in degrees and a little damping; every mesh is placed
in its link frame so the joint pivots sit where the parts meet. The
result is `assets/fixtures/arm/arm.riggen` — the SDK suite checks that
this file's export is byte-identical to that document's — and the
`mujoco` CI check loads what it writes.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Literal

import riggen
from riggen import Dynamics, Fixed, Limits, Revolute

PARTS = Path(__file__).resolve().parents[1] / "assets" / "fixtures" / "arm"
MM = 0.001


def hinge(axis: Literal["x", "y", "z"], z: float, degrees: float) -> Revolute:
    """A revolute joint `z` meters up the parent, symmetric limits in
    degrees, the motor's effort/velocity, a little damping."""
    return Revolute(
        axis,
        origin=(0, 0, z),
        limits=Limits(-degrees, degrees, effort=5.0, velocity=3.0, degrees=True),
        dynamics=Dynamics(damping=0.05),
    )


def build() -> riggen.Robot:
    robot = riggen.Robot("arm")
    base = robot.root.add_link("base", Fixed(), mesh=PARTS / "base.stl", scale=MM, material="aluminium")
    shoulder = base.add_link("shoulder", hinge("z", 0.04, 170), mesh=PARTS / "shoulder.stl", scale=MM, material="aluminium")
    shoulder.geoms[0].pose = (0, 0, -0.04)
    upper = shoulder.add_link("upper", hinge("y", 0.055, 100), mesh=PARTS / "upper.stl", scale=MM, material="PLA")
    upper.geoms[0].pose = (0, 0, -0.095)
    fore = upper.add_link("fore", hinge("y", 0.1, 120), mesh=PARTS / "fore.stl", scale=MM, material="PLA")
    fore.geoms[0].pose = (-0.09, -0.07, -0.135)
    return robot


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=Path("target/sdk-arm"))
    parser.add_argument("--format", choices=["mjcf", "urdf", "both"], default="mjcf")
    args = parser.parse_args()
    robot = build()
    for path in robot.export(args.out, format=args.format, fk_samples=True):
        print(path)
    tip = robot.fk({"upper_joint": 0.5, "fore_joint": -0.5})["fore"]
    print(f"{robot}; fore at {tip}")


if __name__ == "__main__":
    main()
