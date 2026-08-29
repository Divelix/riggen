"""The M3 acceptance (ADR-0004 §2, plans/m3-sim-ready step 5).

For every directory given on the command line: load its `*.xml` with
`mujoco.MjModel.from_xml_path`, failing on any compiler warning, and — when
a `<name>.fk.json` sits beside it — set each sampled joint configuration,
`mj_forward`, and compare every body's world pose with what `riggen_core::fk`
wrote, to 1e-6.

    uv run --with mujoco --with numpy python python/tests/test_mjcf_load.py target/sample

Plain script, no pytest: the CI job is the four lines in the plan's
Acceptance block and nothing else.
"""

import json
import sys
from pathlib import Path

import mujoco
import numpy as np

TOLERANCE = 1e-6


class WarningError(RuntimeError):
    pass


def fail_on_warning(message: str) -> None:
    raise WarningError(message)


def load(xml: Path) -> mujoco.MjModel:
    mujoco.set_mju_user_warning(fail_on_warning)
    try:
        return mujoco.MjModel.from_xml_path(str(xml))
    finally:
        mujoco.set_mju_user_warning(None)


def check_fk(model: mujoco.MjModel, samples: dict) -> int:
    data = mujoco.MjData(model)
    checked = 0
    for sample in samples["samples"]:
        mujoco.mj_resetData(model, data)
        for name, q in zip(samples["joints"], sample["q"]):
            joint = model.joint(name)
            data.qpos[joint.qposadr[0]] = q
        mujoco.mj_forward(model, data)
        for name, pose in sample["links"].items():
            body = data.body(name)
            pos = np.asarray(pose["pos"])
            quat = np.asarray(pose["quat"])
            dpos = np.abs(body.xpos - pos).max()
            # q and -q are the same rotation.
            dquat = min(np.abs(body.xquat - quat).max(), np.abs(body.xquat + quat).max())
            if dpos > TOLERANCE or dquat > TOLERANCE:
                raise AssertionError(
                    f"body {name!r} at q={sample['q']}: mujoco pos {body.xpos} quat {body.xquat}, "
                    f"riggen pos {pos} quat {quat} (dpos {dpos:.2e}, dquat {dquat:.2e})"
                )
            checked += 1
    return checked


def main(argv: list[str]) -> int:
    dirs = [Path(a) for a in argv] or [Path("target/sample")]
    failures = 0
    for directory in dirs:
        xmls = sorted(directory.glob("*.xml"))
        if not xmls:
            print(f"FAIL {directory}: no .xml in it")
            failures += 1
            continue
        for xml in xmls:
            try:
                model = load(xml)
            except Exception as e:  # noqa: BLE001 — any load problem is the verdict
                print(f"FAIL {xml}: {type(e).__name__}: {e}")
                failures += 1
                continue
            summary = f"{model.nbody} bodies, {model.njnt} joints, {model.nmesh} meshes"
            fk = xml.with_suffix(".fk.json")
            if fk.exists():
                try:
                    n = check_fk(model, json.loads(fk.read_text()))
                except AssertionError as e:
                    print(f"FAIL {xml}: {e}")
                    failures += 1
                    continue
                summary += f", {n} body poses match riggen's FK to {TOLERANCE:g}"
            print(f"ok   {xml}: {summary}")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
