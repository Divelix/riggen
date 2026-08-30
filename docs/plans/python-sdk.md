# Plan: python-sdk

- Started: 2026-08-30
- Milestone: v0.2 (the "Python SDK" line of `docs/03-roadmap.md` §v0.2)
- Idea: docs/ideas/python-sdk.md (absorbed; the human said yes to all four
  decisions on 2026-08-30)
- Idea (verbatim from the human): "yes to all four, /plan python-sdk"

## Goal

`pip install riggen` (or `uv add riggen`) gives the app *and* `import
riggen`. In a script or a notebook, `riggen.Robot("pendulum")` builds a
document with the same rules the GUI enforces — `add_link` with a mesh and
its joint, `set_joint`, `reparent`, materials, inertial and collision
policies — each call applying exactly one `riggen_core::Command` and
raising a typed `riggen.EditError` when the command would. `robot.validate()`,
`robot.fk(q)`, `robot.inertial(link)`, `robot.export(dir, format)`,
`riggen.load(path)` / `robot.save(path)` and `riggen.load_urdf(path)` are
the headless half of M1–M3 over `riggen-core`, `riggen-export` and
`riggen-mesh`, linked into one abi3 extension module `riggen._riggen`.
`riggen.show(robot)` writes a temp `.riggen`, spawns the bundled `riggen`
binary on it and returns a `Viewer` whose `wait()` hands the document back
as the GUI saved it — the "place the joint by hand, keep scripting" loop of
`SEED.md` §5. One wheel per platform, one version, one release workflow;
everything M4 decided (the binary in `scripts/`, no console script,
`python -m riggen`, the README as the PyPI page) still holds. The SDK's own
suite runs on the built wheel in CI, and the M3 acceptance (MuJoCo loads it
clean, FK matches) runs over an arm the SDK built from its STLs.

## Non-goals

- Anything the GUI can do that the document cannot: no `History`/undo in
  the SDK (a script has no undo), no selection, no snapping, no circle fit.
- A live link between a running script and the GUI (streaming `q` to the
  window). ADR-0002's open question closes as "no" (idea decision 3); the
  backlog line stays.
- numpy as a dependency (idea decision 4): plain tuples and nested lists.
- New document features: sites/frames export, mimic joints, actuators,
  convex decomposition, MJCF import — their own roadmap lines. The SDK
  exposes `Frame` as the document holds it today and nothing more.
- Free-threaded CPython (3.13t/3.14t): abi3 wheels do not install there.
  One README line and a backlog line.
- Building the `riggen` binary from an sdist install (OPEN 3): a source
  build gets the extension module only.
- crates.io publishing, macOS signing, the screencast, GUI changes. The
  only app-side change is `main`'s body moving into `riggen_app::run` if
  step 7 needs it — it does not, so none.
- A docs site. The API is documented in docstrings, the stubs and one
  README section.

## Design deltas

**Packaging** (01 §Cargo workspace, §Python distribution; ADR-0009, step 3):

- maturin cannot put a `[[bin]]` and a PyO3 cdylib from one crate into one
  wheel (its docs advise against it, and Rerun does not do it either: the
  `rerun` CLI is built separately and copied into the package as a data
  file). So: `pyproject.toml` switches to `bindings = "pyo3"`,
  `manifest-path = "crates/riggen-py/Cargo.toml"`, `module-name =
  "riggen._riggen"`, `features = ["extension-module"]`, and **maturin's default wheel data
  directory `riggen._riggen.data/`** beside `pyproject.toml` (not an
  explicit `data = …`, see the step 1 finding), whose `scripts/`
  subdirectory lands in `riggen-<ver>.data/scripts/`, which is exactly
  where M4's `bindings = "bin"` put the binary. `riggen._riggen.data/` is
  gitignored and filled by the build: `cargo build --release -p riggen-app
  [--target T]` then a copy of `target/[T/]release/riggen[.exe]` into
  `riggen._riggen.data/scripts/`. `python/build_wheel.py` does both halves and
  then `maturin build` (`--binary-only` for CI's container, which runs
  maturin itself); it is the one place the recipe lives, for the human,
  `ci.yml` and `release.yml`. `__main__.py` is unchanged: the binary is
  still in `sysconfig.get_path("scripts")`, the `riggen` command is still
  the binary, no console script.
- `crates/riggen-py`: `[lib] name = "_riggen", crate-type = ["cdylib"]`,
  `test = false`, `doctest = false` (a cdylib built with
  `extension-module` cannot link a test binary; the bindings are tested
  from Python). Dependencies: `riggen-mesh`, `riggen-core`,
  `riggen-export`, `pyo3 = "0.28"` with `abi3-py310` (matching
  `requires-python`); `extension-module` is a crate feature that only
  maturin turns on, so `cargo clippy --workspace` and `cargo test
  --workspace` keep building it against the host's `python3`. Never egui
  or wgpu; a CI check (`cargo tree -p riggen-py -e normal | grep -c wgpu`
  = 0) pins the layer rule for the crate that will be tempted first.
- The layer map gains `riggen-py` beside `riggen-app`, over
  `riggen-export`, as drawn; nothing above it, nothing of it named below.
- Wheel tags become `cp310-abi3-<platform>`; still one wheel per platform,
  five targets, the same `release.yml` matrix. `RIGGEN_GIT_HASH` and
  `RIGGEN_BUILD_DATE` reach both cargo invocations. The sdist carries
  `crates/riggen-py` and builds the extension only (OPEN 3); `show()` and
  `python -m riggen` on such an install say the binary is missing and how
  to get it (`cargo install --git`).
- `[project]` gains `Programming Language :: Python :: 3 :: Only` and the
  `Typing :: Typed` classifier; `python/riggen/py.typed` and
  `python/riggen/_riggen.pyi` ship in the wheel.

**The extension module `riggen._riggen`** (01, new §Python SDK; the
mapping table lives there): a thin, typed layer over the document, one
method per `Command`, no sugar.

- `Robot(name)`: wraps `riggen_core::Robot`. `Robot.load(path)` →
  `(robot, warnings)`, `robot.save(path)`, `robot.to_json()` /
  `Robot.from_json()` (the v1 schema, for notebooks that want to diff).
- Ids are `int`s at this layer (`LinkId`, `JointId`, `GeomId`, `MeshId`,
  `FrameId` are `u32` newtypes); names are looked up with `robot.link(name)`
  / `robot.joint(name)`. Read access: `robot.root`, `robot.links()`,
  `robot.joints()`, `robot.materials()`, `robot.frames()`, `robot.assets()`
  returning plain dicts / dataclass-like objects, `robot.parent_joint(id)`,
  `robot.child_joints(id)`, `robot.subtree(id)`.
- Edits, each applying one `Command` via `Command::apply` on the document
  (no `History`): `add_link(name, parent, joint, *, mesh=None, scale=1.0,
  fix_up=None, material=None) -> LinkId` (the mesh path is absolutised
  and hashed into a `MeshAsset` first, `AddLink` allocates the ids),
  `remove_link`, `rename_link`, `rename_joint`, `add_geom`, `remove_geom`,
  `set_geom_pose`, `set_joint`, `move_joint_frame`, `reparent(link,
  new_parent, keep_world_pose)`, `set_link_material`, `upsert_material`,
  `remove_material`, `set_asset`, `set_inertial`, `set_collision`,
  `set_root`. `EditError` variants map to `riggen.EditError` subclasses
  (`InvalidDocument`, `UnknownId`, `WouldCreateCycle`, …) with the Rust
  message.
- Kinematics and inertials: `validate() -> list[str]` (every
  `validation_errors` entry) and `check()` raising `ValidationError`;
  `fk(q: dict[JointId, float]) -> dict[LinkId, Pose]`;
  `origin_for_world`; `inertial(link) -> (mass, com, inertia)` through
  `riggen_export::MeshStore::load` + `compose_inertial`, with
  `InertialError` typed.
- Export and import: `export(dir, format="mjcf"|"urdf"|"both",
  mesh_paths="relative"|…, floating_base=False) -> list[Path]` =
  `resolve` + `export`, `ExportError`s joined the way `cli::run` joins
  them; `fk_samples_json()`; `load_urdf(path, packages: dict[str, Path])
  -> (robot, warnings)`.
- `Pose` crosses the boundary as `((x, y, z), (w, x, y, z))` and is also
  accepted as `xyz_rpy=((x, y, z), (r, p, y))`; vectors as 3-tuples,
  matrices as 3×3 nested lists. Radians and meters, as the document.
- `__version__` from `CARGO_PKG_VERSION` — the same number
  `importlib.metadata` reports, from the same `Cargo.toml`.

**The Python package** (`python/riggen/`): the public API, pure Python,
typed, documented, over `_riggen`.

- `riggen.Robot`, with `Link` and `Joint` handles that carry their id and
  a reference to the robot (`link.name`, `link.joints`, `link.material =
  "PLA"`, `joint.limits = (-1.57, 1.57)` each forwarding to one edit);
  `riggen.Pose(xyz, rpy=…, quat=…)`; `riggen.Revolute(axis, origin,
  limits, dynamics)`, `Continuous`, `Prismatic`, `Fixed`; `axis` accepts
  `"x" | "y" | "z"` or a 3-tuple; a `degrees=` keyword on limits and RPY.
- `riggen.load(path)`, `riggen.load_urdf(path, packages=None)`,
  `robot.save`, `robot.export`, `robot.validate`, `robot.fk(q: dict[str |
  Joint, float]) -> dict[str, Pose]` (by *name* at this layer).
- Exceptions: `riggen.RiggenError` ← `EditError`, `ValidationError`,
  `ExportError`, `UrdfImportError` (not `ImportError`), `InertialError`.
- `riggen.show(robot, *, block=False) -> Viewer`: `tempfile.mkdtemp
  (prefix="riggen-show-")`, `robot.save(dir / f"{name}.riggen")` (mesh
  paths are absolute in memory, `save` rebases them), `subprocess.Popen
  ([binary_path(), path])` with `binary_path` shared with `__main__.py`
  and overridable by `RIGGEN_BINARY` (OPEN 4); `Viewer.wait() -> Robot`
  returns the document re-read from the file if its content hash changed,
  else the one passed in; `Viewer.poll()`, `.kill()`, `.path`. Never
  imports egui-anything; never blocks unless asked.
- `examples/pendulum.py` and `examples/arm.py` (the M2 arm from
  `assets/fixtures/arm/*.stl`, joints typed, exported to a directory given
  on the command line) are the README's snippets and the acceptance's
  input.

**Tests** (01 §Testing):

- `python/tests/sdk/` is a pytest suite (OPEN 1), run against the built
  wheel in the `wheel` CI job (`uv run --python <venv> --with pytest
  pytest python/tests/sdk`) and locally against `maturin develop`:
  pendulum built through the SDK equals `assets/fixtures/pendulum.riggen`
  after `save` (paths rebased); every edit method's error path; `fk` of
  `arm.riggen` equals `riggen --export --fk-samples`; SDK export of
  `arm.riggen` is byte-identical to the CLI's; `load_urdf(arm.urdf)`
  re-export matches the CLI's; `show()` with `RIGGEN_BINARY` pointing at a
  stub that rewrites the file → `wait()` returns the rewritten robot.
- `test_wheel.py` grows `python -c "import riggen; riggen.Robot('x')"`
  and the tag check (`cp310-abi3`); the `mujoco` job gains
  `examples/arm.py` → `test_mjcf_load.py`.
- `pyright` over `python/riggen` and the stubs in the `wheel` job.

**Docs**: 01 §Layer map, §Cargo workspace, §Python distribution (rewritten),
new §Python SDK (the mapping table); 02 §Commands (one line: the SDK's
edit methods are these commands, one each); 03 v0.2 status; ADR-0009;
ADR-0002 gets an "Amended by ADR-0009" status line; README §Python;
AGENTS.md; BACKLOG.

## Steps

Ordered so the packaging unknown — an abi3 PyO3 wheel that also carries
the binary in `scripts/`, from this workspace, on five targets — retires
before any binding is written; the bindings are existing, tested Rust
behind a thin layer, and the Python API on top is the part the human
will want to read before it is final.

- [x] Step 1 — The wheel builds locally with both halves: `crates/
  riggen-py` (cdylib `_riggen`, PyO3 0.28 abi3-py310, `extension-module`
  feature, `test = false`, exposing only `__version__`), `pyproject.toml`
  switched (`bindings = "pyo3"`, `module-name`, `features`, `data =
  "python/data"`), `python/build_wheel.py`, the data directory gitignored,
  `python/riggen/_riggen.pyi` + `py.typed`. `python python/build_wheel.py`
  produces `riggen-0.1.0-cp310-abi3-manylinux_2_28_x86_64.whl` (or
  `linux_x86_64` locally) with `riggen-0.1.0.data/scripts/riggen` *and*
  `riggen/_riggen.abi3.so` inside; `test_wheel.py` gains `python -c
  "import riggen._riggen"` and passes; `cargo fmt/clippy/test --workspace`
  green with the new crate. `ci.yml`: the `wheel` job runs the binary
  half in maturin-action's `before-script-linux`, the `cargo tree` layer
  check, and `test_wheel.py`; the `wasm` job is untouched (`-p
  riggen-app`). Report wheel and `.so` sizes. **This is the plan's risk;
  stop and report if maturin's data directory does not install into
  `bin/`, or PyO3 abi3 does not build in the manylinux container.**
- [x] Step 2 — The five targets: `release.yml`'s `build` matrix runs the
  binary half per target (`before-script-linux` for the two containers, a
  preceding step for macOS/Windows, `--target` passed to both cargo runs),
  `smoke` unchanged plus the import line. Ends by asking the human to
  push and dispatch to TestPyPI; acceptance of the step is `uvx
  --index-url https://test.pypi.org/simple/ --index-strategy
  unsafe-best-match --from riggen python -c "import riggen._riggen"` on
  the dev machine and a green smoke matrix. A failing target is dropped
  and reported, not worked around blind. The workspace version goes to
  `0.2.0-dev` in this step (OPEN 5): TestPyPI already holds a 0.1.0 and
  `skip-existing` would make a same-version dispatch a silent no-op.
  **Accepted 2026-08-30**: the dispatch on `13f753d` (run 33303264056)
  built all five wheels and the sdist, the three smokes passed, TestPyPI
  holds `0.2.0.dev0`; on the dev machine `uvx --refresh … --from
  "riggen==0.2.0.dev0" python -c "import riggen._riggen"` imports and
  `riggen --version` from the same install runs. Wheel sizes: linux
  x86_64 9.7 MB, linux aarch64 9.2, macOS arm64 6.2, macOS x86_64 6.6,
  Windows 7.4; sdist 143 KB.
- [x] Step 3 — ADR-0009 "one wheel: PyO3 abi3 extension plus the binary
  as wheel data": the layout step 1–2 proved, why not cdylib+bin in one
  crate, why not two wheels, why not pure Python, the sdist consequence
  (OPEN 3), the closing of ADR-0002's open question; ADR-0002 gets the
  "Amended by ADR-0009" line; 01 §Python distribution rewritten in the
  present tense. Docs only.
- [x] Step 4 — The document in `_riggen`: `Robot` (new/load/save/json),
  ids, read access, `Pose` conversion, every edit method with `EditError`
  → exception subclasses; `riggen.RiggenError` hierarchy in
  `python/riggen/errors.py`; stubs updated. pytest: the pendulum built
  through `_riggen` saves equal to `assets/fixtures/pendulum.riggen`
  (paths rebased, `next_id` included), one test per error variant
  (cycle, root removal, unknown id, material in use, movable joint on the
  root path), `SetJoint` ignores parent/child as 02 says.
- [x] Step 5 — Kinematics, inertials, export, import in `_riggen`:
  `validate`/`check`, `fk`, `origin_for_world`, `inertial` (via
  `MeshStore`), `export` (all `ExportOptions`), `fk_samples_json`,
  `load_urdf`. pytest: `fk(arm)` equals `riggen --export --fk-samples`'s
  JSON to 1e-9; SDK `export(arm)` byte-identical to the CLI's directory;
  `load_urdf(arm.urdf)` then export equals the CLI's; `inertial` of the
  arm's base equals the readout in the fixture; an invalid robot's
  `export` raises `ExportError` listing every error.
- [ ] Step 6 — The public Python API: `riggen.Robot` with `Link`/`Joint`
  handles, `Pose`, the joint constructors, `axis="z"`, `degrees=`,
  `load`, `load_urdf`, `fk` by name, docstrings on everything, the stubs
  complete, `pyright` clean, `examples/pendulum.py` and `examples/arm.py`.
  pytest: the arm built by `examples/arm.py` exports byte-identical to
  `arm.riggen`'s export (the joints typed to the same values); every
  public name has a docstring (a test that walks `riggen.__all__`).
  `ci.yml`'s `mujoco` job runs `examples/arm.py` and `test_mjcf_load.py`
  on its output. **The human reads `python/riggen/__init__.py` and the
  examples at the end of this step; naming changes land here, not later
  (OPEN 2).**
- [ ] Step 7 — `riggen.show()`: `Viewer`, the temp document, `binary_path`
  shared with `__main__.py`, `RIGGEN_BINARY` override (OPEN 4), `wait()`
  read-back on content-hash change, `block=True`. pytest with a stub
  binary (a Python script that loads the file through `riggen`, adds a
  link, saves, exits): `wait()` returns the robot with the extra link;
  with a stub that exits without saving, `wait()` returns the original.
  By hand on the dev machine: `riggen.show(pendulum)` opens the window
  on the document, Save in the GUI, `wait()` returns the edit. The
  missing-binary message (sdist installs) tested with `RIGGEN_BINARY`
  pointing nowhere.
- [ ] Step 8 — README §Python (install, the ten-line pendulum, `show()`,
  export to MuJoCo, the abi3/free-threaded line), `python/riggen/
  __init__.py` docstring rewritten (it says "nothing else until the v0.2
  SDK" today), `examples/` linked, `uvx twine check` on the sdist. The
  wheel size delta from step 1 goes into the roadmap line at retirement.
- [ ] Step 9 — Acceptance and drift: the Acceptance block below green
  (the notebook half by the human), 01/02 read against the code with the
  discrepancy list emptied, the by-hand findings under a v0.2-SDK heading
  in `docs/BACKLOG.md`, the roadmap's v0.2 SDK line marked done with the
  wheel sizes and the API surface count, the version to `0.2.0` (OPEN 5).
  Then `/retire-plan`; the `v0.2.0` tag and its push are the human's.

## Acceptance

```sh
python python/build_wheel.py                                     # cargo build riggen-app → riggen._riggen.data/scripts → maturin build
uv venv target/wheel-venv && uv pip install --python target/wheel-venv dist/riggen-*.whl
python python/tests/test_wheel.py target/wheel-venv               # M4's checks + import riggen._riggen + the tag
uv pip install --python target/wheel-venv pytest && target/wheel-venv/bin/python -m pytest python/tests/sdk
target/wheel-venv/bin/python examples/arm.py --out target/sdk-arm  # the arm from its STLs, through the SDK
uv run --with mujoco --with numpy python python/tests/test_mjcf_load.py target/sdk-arm   # M3's bar, over SDK output
cargo test --workspace && cargo clippy --workspace --all-targets -- -D warnings
```

Plus the human's half: in a fresh venv with the TestPyPI (then PyPI)
wheel, a notebook builds a two-link pendulum in ten lines, `riggen.show
(robot)` opens it, a joint is placed by hand and saved, `wait()` returns
it, `export("mjcf")` loads in `mujoco.viewer`. `release.yml` green for
the five targets. The `v0.2.0` tag is the release.

## Docs to update on completion

- `docs/01-architecture.md` §Layer map — `riggen-py` present, not
  "(v0.2)"; §Cargo workspace — the tree: `crates/riggen-py`,
  `python/riggen/{__init__, _riggen.pyi, errors, robot, show}.py`,
  `riggen._riggen.data/` (ignored, the build fills it), `python/build_wheel.py`,
  `python/tests/sdk/`, `examples/`; `pyproject.toml`'s comment line;
  §Python distribution — the two halves of the wheel, the data
  directory, `cp310-abi3` tags, the sdist consequence, the build recipe;
  new §Python SDK — the mapping table (Python name → Rust type / command
  / function), the exception hierarchy, `show()`'s protocol; §Testing —
  the pytest suite, pyright, the `mujoco` job's SDK input, the layer
  check.
- `docs/02-data-model.md` §Commands and history — one line: the SDK's
  edit methods are these commands, one call each, no history.
- `docs/03-roadmap.md` §v0.2 — the SDK line marked done with date, wheel
  sizes per platform, decisions (data-dir layout, abi3, no numpy, no live
  link), what the by-hand notebook run said.
- `docs/adr/0002-…` — status "Amended by ADR-0009" (step 3).
- `docs/BACKLOG.md` — free-threaded CPython wheels (needs non-abi3 per-
  version wheels or PyO3's `Py_GIL_DISABLED` support); the by-hand
  findings under a v0.2-SDK heading; anything from OPEN 2's review that
  was deferred.
- `README.md` — §Python (step 8); the "Developing" section's build line.
- `AGENTS.md` current state — v0.2 SDK done, the `import riggen` line,
  next the remaining v0.2 lines; the reading order gains nothing.
- Delete `docs/ideas/python-sdk.md` — done with this plan's creation.

## Open questions

Findings from step 1 (2026-08-30), each a deviation from the design deltas
above:

- **The data directory is `riggen._riggen.data/` at the repository root,
  not `python/data`.** An explicit `data = …` in `pyproject.toml` must
  exist or maturin fails (`No such data directory`), the directory cannot
  be tracked (maturin rejects any entry at its root that is not one of
  `data scripts headers purelib platlib`, so no README/.gitkeep), and the
  sdist has no binary to put there — so a source build would have failed.
  maturin's *default* location, `<module-name>.data/` beside
  `pyproject.toml`, is skipped when absent: the tree build carries the
  binary, the sdist build gets the extension only, exactly OPEN 3. The
  name is maturin's to choose (`<module-name>.data`). The deltas and the
  docs list above say the new path.
- PyO3 is 0.29 (current on crates.io), not 0.28.

Findings from step 2 (2026-08-30):

- The matrix names full triples (`x86_64-unknown-linux-gnu`, …,
  `x86_64-pc-windows-msvc`) so one `${{ matrix.target }}` reaches both
  maturin-action and `build_wheel.py --target`; maturin-action accepts
  them. Verified locally: `build_wheel.py --target x86_64-unknown-linux-gnu`
  finds `target/<triple>/release/riggen` and the wheel carries it.
- `0.2.0-dev` is `0.2.0.dev0` to maturin and pip. `riggen --version`
  prints Cargo's spelling (`test_wheel.py`'s regex accepts a pre-release
  suffix); `_riggen.__version__` maps Cargo's `-dev`/`-alpha.N`/
  `-beta.N`/`-rc.N` to PEP 440 so it equals `importlib.metadata`'s string,
  which `test_wheel.py` asserts.
- **The acceptance command must pin the version**: `uvx --index-url
  https://test.pypi.org/simple/ --index-strategy unsafe-best-match --from
  "riggen==0.2.0.dev0" python -c "import riggen._riggen"` — TestPyPI holds
  the 0.1.0 final, and uv prefers a final over a pre-release unless the
  pre-release is asked for by name. And **`--refresh`**: uv caches the
  index, so a lookup made before the upload keeps answering "no version"
  until refreshed. Verified 2026-08-30 (the step's box).

Findings from step 4 (2026-08-30):

- **`_riggen` speaks the v1 schema, not a Python-side `Pose` tuple.** The
  deltas said `Pose` crosses as `((x, y, z), (w, x, y, z))`; instead every
  value crosses as the dict the `.riggen` file spells (02 §Schema),
  through `serde_json::Value`, with ids as ints by key (01 §Python SDK).
  One 40-line converter replaces a hand-written mapping per struct that
  would drift from the schema; the friendly spellings (`Pose(xyz, rpy)`,
  `Revolute(axis="z", …)`, `degrees=`) are step 6's pure-Python job,
  where they belong. Step 5's `fk` returns poses in the same shape.
- Beyond the listed methods: `add_asset` (a second geom on a link needs a
  registered mesh; the app's drop does the same), `copy`, `next_id`, and
  `name` as a settable property (no `RenameRobot` command exists).
- `errors.py` already holds `ExportError`, `UrdfImportError`,
  `InertialError` for step 5, so the hierarchy is written once.
- The suite runs on the installed wheel in `ci.yml` (pytest installed
  into the wheel venv, `<venv>/bin/python -m pytest`, rather than `uv run
  --with` — one interpreter, no overlay to reason about); the acceptance
  block below says the same.

Findings from step 5 (2026-08-30):

- `validate()` / `check()` are always empty on a `_riggen.Robot`: every
  way to get one (the edit methods, `load`, `from_json`, `load_urdf`)
  validates. Kept as the deltas said, for the day a document is assembled
  another way; the tests assert the empty case.
- The suite compares the SDK against the **bundled `riggen` binary**
  (`riggen.__main__.binary_path()`, `RIGGEN_BINARY`, or a `target/`
  build; skipped if none): in the `wheel` job that is the same wheel's
  binary, so "SDK export is byte-identical to the CLI's" is checked
  against the CLI a user would run. `test_wheel.py` and the SDK suite
  share the venv for that reason.
- `mesh_paths` is one string — `"relative"`, `"absolute"`,
  `"package://<name>"` — rather than a style plus a package name;
  `export` returns `pathlib.Path`s (PyO3's `PathBuf`).
- A floating base on the arm is refused (its root `base_link` has no
  mass, ADR-0008 OPEN 3) — the test asserts the refusal on the arm and the
  `<freejoint>` on the pendulum.
- The `cargo tree` layer check lives in the `clippy` job, which already
  has the toolchain, rather than the container-based `wheel` job.
- The sdist holds `riggen-py` and its three lower crates only — maturin
  packages the workspace's path dependencies, and `riggen-app` is not one
  — so the M4 `include` of `assets/fixtures/arm` and the snapshot
  `exclude` went with `bindings = "bin"`. An sdist can never build the
  binary (OPEN 3, as accepted).
- Sizes, linux x86_64: wheel 9.7 MB (M4: 9.6), the binary 21.9 MB, the
  extension 357 KB with only `__version__`; the 566 KB `riggen-app` SBOM
  of M4 is gone (the binary is data now), a 48 KB `riggen-py` one remains.
  Local wheels tag `manylinux_2_34` (auditwheel on the `.so`); CI's
  container gives `manylinux_2_28`.

All five decided by the human on 2026-08-30 ("I agree with recommended"):

- `OPEN 1` — **decided: pytest** for `python/tests/sdk/`; `test_wheel.py`
  and `test_mjcf_load.py` stay plain one-check scripts.
- `OPEN 2` — **decided: the human reviews the public API's names and
  shapes at the end of step 6** and they are frozen there — a rename
  after 0.2.0 is a break. The design deltas above are the draft.
- `OPEN 3` — **decided: accept** that an sdist install (any platform
  outside the five) builds the extension only; `show()` and `python -m
  riggen` explain how to get a binary. No nested cargo build in a
  `build.rs`.
- `OPEN 4` — **decided: `RIGGEN_BINARY`** environment override for
  `show()` and `__main__.py` (Rerun's `RERUN_CLI_PATH`); the default
  stays `sysconfig`. The tests use it.
- `OPEN 5` — **decided: `0.2.0-dev`** in the workspace `Cargo.toml` from
  step 2, `0.2.0` at step 9.
