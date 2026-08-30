# ADR-0009: One wheel — a PyO3 abi3 extension module plus the binary as wheel data

- Status: Accepted
- Date: 2026-08-30
- Amends: ADR-0002 (the v0.2 half of its decision, and its open question)

## Context

ADR-0002 shipped the app as a maturin `bindings = "bin"` wheel (M4) and
deferred the SDK to v0.2 as "two build artefacts per platform (bin +
cdylib); CI handles it the way Rerun's does". plans/python-sdk made that
concrete: one `pip install riggen` must give both the `riggen` window and
`import riggen`, from one project, one version, one release workflow.

Three facts shaped the layout:

1. maturin builds one kind of thing per `pyproject.toml`: `bindings =
   "bin"` packages a crate's `[[bin]]`s, `bindings = "pyo3"` packages a
   cdylib. It does not put a `[[bin]]` and an extension module from one
   crate into one wheel, and its documentation advises against trying.
   Rerun, the reference for this experience, builds the `rerun` CLI
   separately and copies it into the package as a data file.
2. An extension module is normally built per CPython version, so the M4
   matrix — five targets, one `py3-none-<platform>` wheel each — would
   become five targets × every supported interpreter. PyO3's `abi3`
   builds against the stable ABI instead: one `cp310-abi3-<platform>`
   wheel for every CPython ≥ 3.10.
3. maturin's wheel data directory (`<name>-<ver>.data/scripts/`) is
   exactly where `bindings = "bin"` put the binary in M4: the
   environment's `bin/`. Whatever is put in `<module-name>.data/scripts/`
   beside `pyproject.toml` lands there.

## Decision

1. **Two crates, one wheel.** `crates/riggen-py` is a cdylib `_riggen`
   (PyO3, `abi3-py310` matching `requires-python`) over `riggen-core`,
   `riggen-export` and `riggen-mesh`, built by maturin with `bindings =
   "pyo3"` and `module-name = "riggen._riggen"`. `crates/riggen-app`'s
   `riggen` binary is built by cargo for the same target and copied into
   maturin's wheel data directory `riggen._riggen.data/scripts/`, which
   maturin packages as `riggen-<ver>.data/scripts/riggen[.exe]`. The
   user-visible layout of ADR-0002 is unchanged: the command is the
   binary, there is no console script, `python -m riggen` execs it.
2. **abi3.** Wheel tags are `cp310-abi3-<platform>`; the release matrix
   stays five wheels. The costs: no free-threaded CPython (abi3 wheels do
   not install on 3.13t / 3.14t), and the stable ABI's slightly slower
   boundary calls — irrelevant for a document-editing API.
3. **One recipe.** `python/build_wheel.py` does `cargo build --release -p
   riggen-app [--target T]`, the copy, then `maturin build`; `--binary-only`
   stops before maturin for the containers where maturin-action runs
   maturin itself. The human, `ci.yml` and `release.yml` all call it.
4. **The data directory is maturin's default, not an explicit setting.**
   `data = "…"` in `pyproject.toml` must exist or maturin fails, and the
   directory cannot be tracked — maturin rejects any entry at its root
   that is not one of `data scripts headers purelib platlib`, so no
   README or `.gitkeep`. The default `<module-name>.data/` is skipped
   when absent. So the tree build carries the binary and the sdist —
   which has no binary — builds the extension alone. The directory's
   name (`riggen._riggen.data/`) is maturin's; it is gitignored.
5. **`riggen-py` never links egui or wgpu.** It sits beside `riggen-app`
   in the layer map, over `riggen-export`; `ci.yml` pins it with `cargo
   tree -p riggen-py` (no `egui`, `eframe`, `wgpu` in its normal
   dependencies). `extension-module` is a crate feature only maturin
   enables, so the workspace checks still see the crate; `test = false`
   because a cdylib built as an extension cannot link a test binary — the
   bindings are tested from Python.
6. **ADR-0002's open question closes as "no".** The SDK and the GUI share
   a file, not a process: `riggen.show(robot)` writes a temporary
   `.riggen`, spawns the bundled binary on it, and `wait()` reads the
   document back if the GUI saved. A live link (streaming `q` to the
   window) is a backlog line, not the headline feature.

## Consequences

- An sdist install — any platform outside the five, or `pip install .` —
  gets `import riggen` from source and **no binary** (plans/python-sdk
  OPEN 3, accepted): `python -m riggen` and `show()` say so and point to
  `cargo install --git`. There is no nested cargo build in a `build.rs`.
- The sdist carries `riggen-py` and the three crates below it only;
  `riggen-app` is not a dependency of the extension, so M4's sdist
  `include`/`exclude` entries are gone with `bindings = "bin"`.
- Free-threaded CPython is unsupported until someone asks; a per-version
  wheel matrix (or PyO3's free-threaded support) is the backlog line.
- Wheel tags move from `py3-none-<platform>` to `cp310-abi3-<platform>`.
  Nothing changes for a user: `requires-python` was already ≥ 3.10.
- PyO3's build script needs a `python3` on the machine that runs `cargo
  clippy --workspace`, and `cargo build --workspace` links the cdylib
  against that interpreter's libpython. CI's runners and the manylinux
  containers have one; a developer box needs the dev files.
- Versions: the workspace `Cargo.toml` is the one source, but a
  pre-release is spelled two ways — Cargo `0.2.0-dev`, PEP 440
  `0.2.0.dev0`. `_riggen.__version__` maps Cargo's `-dev` / `-alpha.N` /
  `-beta.N` / `-rc.N` to PEP 440 so it equals
  `importlib.metadata.version("riggen")`; the binary prints Cargo's.
- Sizes at step 1, linux x86_64: wheel 9.7 MB (M4: 9.6), the binary
  21.9 MB, the extension 357 KB with nothing but `__version__` in it. The
  566 KB CycloneDX SBOM maturin wrote for `riggen-app` is gone (the
  binary is data now); a 48 KB one for `riggen-py` remains.

## Alternatives considered

- **One crate with a `[[bin]]` and a cdylib** — maturin packages one or
  the other. It would also put the extension in the same crate as
  eframe, and the layer rule exists to keep them apart.
- **Two wheels** (`riggen` the binary, `riggen-sdk` the extension, one
  depending on the other) — two projects, two version numbers kept in
  lock-step, two PyPI pages, and `pip install riggen` has to know about
  the other one. Rerun ships one wheel; nothing here justifies two.
- **A pure-Python SDK over the binary's CLI** (a subprocess per
  operation, JSON in and out) — no FK, inertials or validation without
  the CLI growing an RPC surface; the document's rules would be
  re-implemented in Python or round-tripped through files on every call.
- **Per-version wheels instead of abi3** (`cp310` … `cp314`, five each) —
  free-threaded support and a faster boundary, for five times the
  release matrix and the smoke jobs. Not until it is asked for.
- **The extension only; the binary via `cargo install`** — breaks the
  M4 promise `pip install riggen` → a window.
- **Building the binary from the sdist** (a `build.rs` running cargo for
  `riggen-app`) — a build script spawning cargo inside cargo, a doubled
  build time on every source install, and every GUI system dependency
  pulled into `pip install` on the platforms least able to satisfy them.
- **Living with `data = "python/data"` and a tracked placeholder** —
  impossible: see Decision 4.
