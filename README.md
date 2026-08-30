# riggen

[![CI](https://github.com/Divelix/riggen/actions/workflows/ci.yml/badge.svg)](https://github.com/Divelix/riggen/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/riggen)](https://pypi.org/project/riggen/)

**The blazingly fast, lightweight robot assembler for RL researchers.**
Drop meshes in, get a simulation-ready MJCF or URDF out.

![The sample arm in riggen: the link tree, the viewport with joint glyphs, the Joints window](https://raw.githubusercontent.com/Divelix/riggen/main/docs/assets/arm.png)

## Install

```sh
uv tool install riggen      # or: uvx riggen  /  pip install riggen
riggen --example arm        # the bundled sample robot
```

One wheel per platform (Linux x86_64 / aarch64, macOS arm64 / x86_64,
Windows x86_64), Python 3.10 or later, nothing else to install — the
`riggen` command is a native executable, and there is no Rust toolchain
on this path. On a platform without a wheel, `pip install` builds it from
source with `cargo` on `PATH`.

## The first minute

1. `riggen --example arm` opens a four-part arm: link tree on the left,
   the viewport in the middle, Properties on the right. Orbit with the
   middle mouse button, zoom with the wheel, `Home` to frame everything.
2. Drag the sliders in **Window › Joints** — the arm moves; that is the
   kinematic tree you will build for your own robot.
3. **File › Export…**, pick MJCF, choose a directory. The dialog lists
   anything that would stop the export (a link with no mass, a joint with
   no axis) and writes `arm.xml` beside `meshes/*.stl` when there is
   nothing.
4. Load it:

   ```sh
   python -c "import mujoco; m = mujoco.MjModel.from_xml_path('out/arm.xml'); print(m.nbody, 'bodies')"
   ```

   or `python -m mujoco.viewer --mjcf out/arm.xml`.

Then your robot: `riggen base.stl upper.stl fore.stl` drops each mesh as a
link under the root (meters or millimetres — the import units are in the
status bar). Reparent by dragging rows in the tree; **Place joint** puts a
revolute joint on a bore by clicking its edge; **Align** snaps a part's
bore concentric with its parent's; Properties computes the inertial from
the mesh and a material, and fits a hull or primitives for collision.

## What it does

- **Assembles**: STL and OBJ meshes into a kinematic tree — fixed, revolute
  and prismatic joints, placed by clicking geometry, with limits.
- **Computes**: mass, centre of mass and the inertia tensor from the mesh
  and a material density, or from a spec you type; convex hulls and fitted
  boxes / cylinders / spheres for collision.
- **Exports**: MJCF and URDF from the same document, meshes baked to
  meters as STL, so MuJoCo loads it with zero warnings and its forward
  kinematics agree with riggen's (that is a CI job, not a hope).
- **Imports**: an existing URDF, `package://` paths resolved beside the
  file, to fix and convert it.
- **Stays out of the way**: a native window through wgpu, a document that
  is plain JSON (`.riggen`), undo for everything, and a headless CLI.

## Command line

```
usage:
  riggen [FILE...]        open a .riggen document, or drop meshes (.stl, .obj) as links
  riggen --example arm    open the bundled sample arm
  riggen --export mjcf|urdf|both --out DIR [--fk-samples] INPUT
                          write INPUT's export to DIR without opening a window

options:
  --example NAME          open a bundled example: arm (the five-link sample robot)
  --export FORMAT         headless export of INPUT (.riggen or .urdf): mjcf, urdf or both
  --out DIR               where --export writes; created if missing
  --fk-samples            with --export: also write <name>.fk.json, five sampled joint configurations
  --timing                print the time from launch to the first frame on stderr
  -h, --help              print this help
  -V, --version           print the version and the git commit it was built from
```

`riggen --export` needs no display, so it runs in CI and in scripts: give
it a `.riggen` or a `.urdf` and it writes the model, the meshes and, with
`--fk-samples`, five joint configurations with every body's world pose —
the file `python/tests/test_mjcf_load.py` checks MuJoCo against.

`python -m riggen` is the same executable, for an environment whose
`bin/` is not on `PATH`.

## Developing

Rust stable, then:

```sh
git clone https://github.com/Divelix/riggen && cd riggen
git config core.hooksPath .githooks   # fmt, clippy -D warnings, test before every commit
cargo run                             # the app
cargo test --workspace                # incl. the visual snapshot suite (needs a Vulkan driver;
                                      # lavapipe / mesa-vulkan-drivers is the reference)
python python/build_wheel.py                             # the wheel: the app binary + the extension module
```

The Rust route to the binary is `cargo install --git
https://github.com/Divelix/riggen riggen-app`; publishing the workspace to
crates.io so that `cargo install riggen` works is a later release.

Read, in order: [`SEED.md`](SEED.md) (what and why),
[`docs/01-architecture.md`](docs/01-architecture.md),
[`docs/02-data-model.md`](docs/02-data-model.md),
[`docs/03-roadmap.md`](docs/03-roadmap.md), [`docs/adr/`](docs/adr/) — then
[`AGENTS.md`](AGENTS.md) for the rules, agent or human. A Python SDK
(`riggen-py`, the same core through PyO3) is v0.2.

## Licence

MIT or Apache-2.0, at your option.
