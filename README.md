# riggen

[![CI](https://github.com/Divelix/riggen/actions/workflows/ci.yml/badge.svg)](https://github.com/Divelix/riggen/actions/workflows/ci.yml)
[![PyPI](https://img.shields.io/pypi/v/riggen)](https://pypi.org/project/riggen/)

**The blazingly fast, lightweight robot assembler for RL researchers.**
Drop meshes in, get a simulation-ready MJCF or URDF out — in a window, or
from ten lines of Python.

![The sample arm in riggen: the link tree, the viewport with joint glyphs, the Joints window](https://raw.githubusercontent.com/Divelix/riggen/main/docs/assets/arm.png)

## Install

```sh
uv tool install riggen      # or: uvx riggen  /  pip install riggen
riggen --example arm        # the bundled sample robot
```

One wheel per platform (Linux x86_64 / aarch64, macOS arm64 / x86_64,
Windows x86_64), Python 3.10 or later, nothing else to install — the
`riggen` command is a native executable, `import riggen` the SDK over the
same core, and there is no Rust toolchain on this path. On a platform
without a wheel, `pip install` builds the SDK from source with `cargo` on
`PATH` and tells you how to get the app (see [Python](#python)).

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
the mesh and a material, and fits a hull, convex pieces or primitives for
collision.

## What it does

- **Assembles**: STL and OBJ meshes into a kinematic tree — fixed, revolute
  and prismatic joints, placed by clicking geometry, with limits.
- **Computes**: mass, centre of mass and the inertia tensor from the mesh
  and a material density, or from a spec you type; convex hulls, convex
  decomposition (V-HACD, so a C-bracket keeps its notch) and fitted
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

## Python

The same document, the same rules, from a script or a notebook:

```sh
uv add riggen        # or: pip install riggen — the wheel that has the app has the SDK
```

```python
import riggen

robot = riggen.Robot("pendulum")
robot.root.add_mesh("base.stl", scale=0.001)          # your STL, in millimetres
robot.root.material = "aluminium"
arm = robot.root.add_link(
    "arm",
    riggen.Revolute("y", origin=(0, 0, 0.5), limits=(-90, 90), degrees=True),
    mesh="arm.stl", scale=0.001, material="PLA",      # and the part it moves
)
arm.geoms[0].pose = (0, 0, 0.5)                       # the mesh half a unit above the hinge
robot.export("out", format="mjcf")                    # out/pendulum.xml + out/meshes/*.stl
```

Every call is one document edit, checked the way the window checks it: a
duplicate name, a hinge without limits or a link hung under its own child
raises a `riggen.EditError` subclass and changes nothing. Meters and
radians, Z-up; `degrees=True` wherever an angle is typed.

```python
robot.fk({"arm_joint": 0.3})["arm"]                   # Pose((0.0, 0.0, 0.5), rpy=(0.0, 0.3, 0.0))
arm.inertial                                          # Inertial(mass=…, com=…, inertia=…) from the mesh
robot.link("arm").joint.limits = (-1.0, 1.0)          # radians; one edit
robot.save("pendulum.riggen")                         # the window opens this file
```

Place the joint by hand, keep scripting:

```python
viewer = riggen.show(robot)      # the riggen window on a copy; click the bore, Ctrl+S
robot = viewer.wait()            # the document as the window saved it
```

Then MuJoCo:

```python
import mujoco
model = mujoco.MjModel.from_xml_path("out/pendulum.xml")
```

`riggen.load(path)` reads a `.riggen`, `riggen.load_urdf(path)` an existing
URDF (with `packages={"name": "dir"}` for `package://` paths); export writes
`format="urdf"` or `"both"` too, and `fk_samples=True` adds the five joint
configurations CI compares against MuJoCo. [`examples/pendulum.py`](examples/pendulum.py)
is the snippet above as a file; [`examples/arm.py`](examples/arm.py) builds
the bundled arm from its four STLs with typed joints — its export is
byte-identical to the app's. Everything is typed (`py.typed`) and
documented in docstrings: `help(riggen.Link)`.

The wheel is `cp310-abi3`: one build for every CPython from 3.10 on,
which is why it needs no per-version matrix — and why it does not install
on free-threaded CPython (3.13t / 3.14t) yet. An install from the source
distribution (any other platform) compiles the SDK alone; `riggen.show()`
and `python -m riggen` then say how to get the app: a wheel, `cargo
install --git`, or `RIGGEN_BINARY` pointing at a binary you built.

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

A notebook on the dev build, next to the fixtures — the SDK's own venv,
the extension installed editable, the app from a local build:

```sh
uv venv target/sdk-venv --python 3.12
VIRTUAL_ENV=$PWD/target/sdk-venv uvx maturin develop --uv   # rerun after a Rust change (~10 s)
uv pip install --python target/sdk-venv ipykernel mujoco pytest
target/sdk-venv/bin/python -m ipykernel install --user --name riggen-dev --display-name "riggen (dev)"
cargo build --release -p riggen-app && export RIGGEN_BINARY=$PWD/target/release/riggen   # for riggen.show()
```

Put notebooks in `scratch/` (gitignored) on the "riggen (dev)" kernel;
anything worth keeping becomes an `examples/*.py`, which the test suite
runs. The SDK suite itself: `target/sdk-venv/bin/python -m pytest
python/tests/sdk`.

To try a TestPyPI build in a `uv` project instead, give riggen its own
index — an *explicit* one, or uv's dependency-confusion guard will find
some other package's old version on TestPyPI and refuse:

```toml
[[tool.uv.index]]
name = "testpypi"
url = "https://test.pypi.org/simple/"
explicit = true

[tool.uv.sources]
riggen = { index = "testpypi" }
```

then `uv add "riggen==<version>"`.

The Rust route to the binary is `cargo install --git
https://github.com/Divelix/riggen riggen-app`; publishing the workspace to
crates.io so that `cargo install riggen` works is a later release.

Read, in order: [`SEED.md`](SEED.md) (what and why),
[`docs/01-architecture.md`](docs/01-architecture.md),
[`docs/02-data-model.md`](docs/02-data-model.md),
[`docs/03-roadmap.md`](docs/03-roadmap.md), [`docs/adr/`](docs/adr/) — then
[`AGENTS.md`](AGENTS.md) for the rules, agent or human. The SDK's own
tests are `python/tests/sdk/` (pytest, against the built wheel).

## Licence

MIT or Apache-2.0, at your option.
