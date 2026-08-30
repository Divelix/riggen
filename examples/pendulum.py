"""A two-link pendulum in ten lines — the README's example.

    python examples/pendulum.py [--out DIR]

Builds the same document as `assets/fixtures/pendulum.riggen` from the two
unit-cube fixtures, then saves it and exports MJCF into `--out`
(`target/pendulum` by default). Every call is one document edit; the
result loads in `riggen` (the app) and in `mujoco`.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import riggen

FIXTURES = Path(__file__).resolve().parents[1] / "assets" / "fixtures"


def build() -> riggen.Robot:
    robot = riggen.Robot("pendulum")
    base = robot.root
    base.add_mesh(FIXTURES / "cube_binary.stl")
    base.material = "aluminium"
    arm = base.add_link(
        "arm",
        riggen.Revolute(
            "y",
            origin=(0, 0, 0.5),
            limits=riggen.Limits(-90, 90, effort=10, velocity=3, degrees=True),
            dynamics=riggen.Dynamics(damping=0.1),
        ),
        mesh=FIXTURES / "cube_ascii.stl",
        material="PLA",
        joint_name="hinge",
    )
    arm.geoms[0].pose = (0, 0, 0.5)  # the cube sits half a unit above the hinge
    return robot


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, default=Path("target/pendulum"))
    args = parser.parse_args()
    robot = build()
    args.out.mkdir(parents=True, exist_ok=True)
    robot.save(args.out / "pendulum.riggen")
    for path in robot.export(args.out, format="mjcf", fk_samples=True):
        print(path)
    print(robot, "at rest:", {name: pose.xyz for name, pose in robot.fk().items()})


if __name__ == "__main__":
    main()
