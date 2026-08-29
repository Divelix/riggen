# Plan: m4-distribution

- Started: 2026-08-29
- Milestone: M4
- Idea (verbatim from the human): "plan m4"

## Goal

`uv tool install riggen && riggen` (or `uvx riggen`, or `pip install riggen`)
on a machine that has never seen Rust installs a wheel that carries the
native `riggen` executable and opens the window; `python -m riggen` does the
same. Wheels for linux x86_64 / aarch64, macOS arm64 / x86_64 and Windows
x86_64 are built by a GitHub Actions matrix, smoke-tested on a clean venv
per OS (`riggen --version`, `riggen --export` on the sample arm), published
to TestPyPI on demand and to PyPI plus a GitHub Release when a `v*` tag is
pushed. `riggen --help` documents the CLI and `riggen --version` prints the
version and the git hash it was built from. The window is visible in under
500 ms on the dev machine, and the headless part of that path is asserted
in a test. The repository has a README with the install line, a hero
image of the sample arm, a screencast slot and the 60-second first run,
which is also the wheel's PyPI page.

## Non-goals

- The Python SDK (`riggen-py`, PyO3) and `riggen.show()` — v0.2 (ADR-0002).
- Publishing the workspace to crates.io so `cargo install riggen` works.
  The `riggen` 0.0.1 reservation crate stays as it is; the README says
  `cargo install --git`. See OPEN 1.
- macOS code signing / notarization and a Windows installer. pip-installed
  files carry no quarantine attribute, so an unsigned binary in a wheel
  runs from a terminal; a signed `.app` is a later concern.
- conda-forge, Homebrew, AUR, the web demo build.
- Any change to what the app does. The only new behaviour is CLI flags
  and a timing readout.

## Design deltas

**Packaging** (01 §Cargo workspace, §Python distribution):

- `pyproject.toml` moves from `python/` to the **repository root** (OPEN
  3), build backend `maturin>=1.8,<2`, `[tool.maturin] bindings = "bin"`,
  `manifest-path = "crates/riggen-app/Cargo.toml"`, `python-source =
  "python"`, `dynamic = ["version"]` so the version is read from the
  workspace's `Cargo.toml` and lives once. `readme = "README.md"` is the
  root README; `license-files` are the root texts, so `python/LICENSE-*`
  and `python/README.md` go. `[project.scripts]` is **removed**: maturin
  puts the `riggen` binary into the wheel's `scripts/` directory, which is
  the venv's `bin/`, so the `riggen` command *is* the binary — no Python
  interpreter in front of it, which matters for the startup budget. A
  console script of the same name would shadow it.
- `python/riggen/__main__.py` locates the binary in
  `sysconfig.get_path("scripts")` (`riggen.exe` on Windows) and
  `os.execv`s it (`subprocess.call` + `sys.exit` on Windows, which has no
  exec). `python/riggen/__init__.py` keeps `__version__`, read from the
  installed distribution's metadata (`importlib.metadata`), and the
  0.0.1 reservation text goes. This is Rerun's `rerun_cli/__main__.py`
  shape (ADR-0002).
- Wheels are `py3-none-<platform>` (no ABI: nothing links CPython), one
  per platform for every Python ≥ 3.10. The sdist carries the workspace,
  so `pip install` on an unsupported platform builds from source with a
  Rust toolchain.
- `[profile.release]` in the workspace: `strip = true`, `lto = "thin"`,
  `codegen-units = 1` — the wheel is the product now and its size and
  startup are what the user sees. The measured wheel size per platform
  goes into the roadmap status line.

**`riggen-app`** (01 §Crates, the CLI paragraph):

- `cli::parse` grows `--help` / `-h`, `--version` / `-V`, `--timing`, and
  (OPEN 4) `--example arm`. `--help` prints one usage block covering the
  file form, the export form and every flag; `--version` prints `riggen
  0.1.0 (2b60ae4 2026-08-29)`. The parser stays hand-rolled: five flags do
  not earn `clap` and its compile time.
- `build.rs` sets `RIGGEN_GIT_HASH` and `RIGGEN_BUILD_DATE`: from the
  `RIGGEN_GIT_HASH` environment variable when present (the release
  workflow sets it; an sdist has no `.git`), else `git rev-parse --short
  HEAD` (`-dirty` appended when the tree is), else `unknown`;
  `rerun-if-changed=../../.git/HEAD` and the ref it points at.
- Startup timing: `main` captures `Instant::now()` first thing and hands
  it to `RiggenApp::new`; the first `update` that ends with a rendered
  frame records `first_frame_ms`, which `debug_state()` reports under a
  new `timing` section and `--timing` prints to stderr as `startup: first
  frame after N ms` (once). The OS window and the wgpu adapter are created
  by eframe before `new` runs and are inside that number on the real app;
  in the test harness the number starts at `new`.
- `--example arm` (if OPEN 4 says yes): the five files of
  `assets/fixtures/arm/` (64 KB) are `include_bytes!`d, written to
  `<temp>/riggen-example-arm/` and opened as the document, so the first
  run after `uv tool install riggen` is one command with nothing to
  download.

**CI** (01 §Testing):

- `ci.yml` gains a `wheel` job: linux x86_64 only, `PyO3/maturin-action`
  with `manylinux: 2_28`, then `python/tests/test_wheel.py` against a
  fresh `uv venv` with the wheel installed. It is the job that guards
  `pyproject.toml` and `__main__.py` on every push.
- `.github/workflows/release.yml`: `build` matrix — `ubuntu-latest`
  x86_64 and aarch64 (maturin-action's cross container, `manylinux:
  2_28`), `macos-latest` building both `aarch64-apple-darwin` and
  `x86_64-apple-darwin` (a cross-target on the arm64 runner; the Intel
  runners are retired), `windows-latest` x86_64, and the sdist; `smoke`
  on ubuntu / macos / windows runs `test_wheel.py` on the downloaded
  artefact; `publish` uses `pypa/gh-action-pypi-publish` with **trusted
  publishing** (no tokens in secrets) to TestPyPI on `workflow_dispatch`
  and to PyPI on a `v*` tag push, and a `softprops/action-gh-release`
  step attaches the wheels to the GitHub Release. `RIGGEN_GIT_HASH` is
  passed from `github.sha`.
- `python/tests/test_wheel.py`: given a venv's scripts directory, runs
  `riggen --version` (matches `riggen \d+\.\d+\.\d+ \(`), `riggen --export
  mjcf --out … assets/fixtures/arm/arm.riggen` (files exist), and `python
  -m riggen --version` (same output). Headless on purpose: no runner has
  a display.

**Tests** (01 §Testing): `startup_first_frame_under_budget` in
`riggen-app/tests/visual` through `harness::with_app` — `RiggenApp::new`
to the first rendered frame under 500 ms on the CPU adapter, 2000 ms when
the `CI` environment variable is set (lavapipe on a shared runner is not
the dev machine; the roadmap's number is the dev machine's). Startup
regressions are what this guards, not the absolute number.

**Docs**: root `README.md` (new; the PyPI page too), `docs/assets/arm.png`
(hero image from the visual-debug scratch capture of `arm.riggen`), 01
§Cargo workspace, §Crates (CLI), §Python distribution, §Testing; 03 M4
status line; AGENTS.md; BACKLOG.

## Steps

Ordered so the milestone's risk — a maturin `bin` wheel from this
workspace that installs and runs on a clean venv, then the same across five
targets in CI — is retired first; README and polish come after.

- [ ] Step 1 — The wheel builds and runs locally: `pyproject.toml` at the
  root (maturin, `bindings = "bin"`, dynamic version, no console script),
  `python/riggen/__main__.py` execs the bundled binary, `python/riggen/
  __init__.py` reads its version from metadata, `python/LICENSE-*` and
  `python/README.md` removed, `python/tests/test_wheel.py`, `dist/` in
  `.gitignore`. `uv build` produces `riggen-0.1.0-py3-none-linux_x86_64.
  whl` and the sdist; `uv venv target/wheel-venv && uv pip install --python
  target/wheel-venv dist/*.whl && python python/tests/test_wheel.py
  target/wheel-venv` passes (`--export` is the smoke until step 2 adds
  `--version`, so the test grows in step 2); `uvx --from dist/riggen-*.whl
  riggen --export …` works. Report the wheel and binary sizes. **This is
  the milestone's risk; stop and report if maturin cannot package a
  workspace bin this way.**
- [ ] Step 2 — `--help`, `--version`, `build.rs` (`RIGGEN_GIT_HASH`
  override → `git` → `unknown`), `[profile.release]`, and `--example arm`
  if OPEN 4 says yes. Tests: `parse` on each flag and on `-h`/`-V`,
  `--help` text lists every flag (a test that greps the usage for each
  flag name, so a new flag cannot be forgotten), `--version` format via
  `CARGO_BIN_EXE_riggen` in an integration test, `--example arm` extracts
  five files and opens them (harness test). `test_wheel.py` gains
  `--version` and `python -m riggen --version`.
- [ ] Step 3 — Startup timing: the `Instant` from `main`, `first_frame_ms`
  in `debug_state().timing`, `--timing`, and the
  `startup_first_frame_under_budget` test (500 ms / 2000 ms under `CI`).
  Measure the real window on the dev machine with `cargo run --release --
  --timing` and with the installed wheel; if either is over 500 ms, profile
  `RiggenApp::new` (font atlas, pipeline creation, `persistence` load) and
  fix the largest item in this step; the number goes in the roadmap status
  line at retirement.
- [ ] Step 4 — `ci.yml` `wheel` job (linux x86_64, maturin-action,
  `manylinux: 2_28`, `test_wheel.py` in a fresh venv, `rust-cache`).
  Green on `main`. Note the job's wall time beside the `mujoco` job's.
- [ ] Step 5 — `release.yml`: the five-target matrix + sdist, the three
  smoke jobs, TestPyPI on `workflow_dispatch`, PyPI + GitHub Release on
  `v*`, trusted publishing, `RIGGEN_GIT_HASH` from `github.sha`. Ends by
  asking the human to (a) add trusted publishers for `Divelix/riggen` /
  `release.yml` on TestPyPI (new project) and PyPI (existing project
  `riggen`, environment `pypi`), (b) push, (c) dispatch the workflow to
  TestPyPI. Acceptance of the step: `uvx --index-url https://test.pypi.
  org/simple/ --index-strategy unsafe-best-match riggen --version` prints
  0.1.0 with a hash on the dev machine. The linux aarch64 cross build and
  the macOS x86_64 cross-target are the unknowns here; a failing target is
  dropped from the matrix and reported, not worked around blind.
- [ ] Step 6 — README and the hero image: `docs/assets/arm.png` captured
  through the visual-debug scratch target (`RIGGEN_SCRATCH_OPEN=assets/
  fixtures/arm/arm.riggen`, collision view off, selection cleared); root
  `README.md` — one-line pitch, the hero, the screencast slot (OPEN 2),
  install (`uv tool install riggen` / `uvx riggen` / `pip install
  riggen`), the first run (`riggen --example arm` or the download line),
  what it does in five bullets, the CLI from `--help`, export and MuJoCo
  in four lines, building from source, links to `docs/`, licence; CI and
  PyPI badges. `uvx twine check dist/*` passes on the sdist so the PyPI
  page renders. Snapshot: none (no UI change); the capture is checked by
  reading the PNG.
- [ ] Step 7 — Screencast per OPEN 2: either the human's recording
  dropped into the README's slot, or the generated one (a `#[ignore]`d
  `screencast` scenario in `tests/visual` replaying the `five_minute_arm`
  clicks with a drawn cursor, one PNG per rendered frame under
  `target/screencast/`, `ffmpeg` → `docs/assets/screencast.gif` at 720
  px wide, 10 fps, palette-optimised, under 8 MB). Skipped with a note if
  OPEN 2 says the human records later and the README ships with the
  still.
- [ ] Step 8 — Acceptance and drift: the Acceptance block below green
  (the container half by the agent, the window half by the human on a
  clean VM), 01 read against the code with the discrepancy list emptied,
  the by-hand findings ("what was annoying installing and first-running
  it") under an M4 heading in `docs/BACKLOG.md`, the roadmap status line
  with sizes and the startup number. Then `/retire-plan`, tag `m4`; the
  `v0.1.0` tag and its push are the human's (git rules), and that push is
  the release.

## Acceptance

```sh
uv build                                                       # wheel + sdist at dist/
uv venv target/wheel-venv && uv pip install --python target/wheel-venv dist/riggen-*.whl
python python/tests/test_wheel.py target/wheel-venv             # --version, --export, python -m riggen
cargo test --workspace                                         # incl. startup_first_frame_under_budget, cli flags
# after step 5, on a machine / container that has no Rust and no checkout:
docker run --rm python:3.12-slim sh -c \
  'pip install -q --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple riggen && riggen --version'
```

Plus the human's half: a clean VM (or a fresh user account) runs `uv tool
install riggen && riggen --example arm` (or `riggen path/to/arm.riggen`)
and the arm appears in under a second; `release.yml` is green for a
`workflow_dispatch` to TestPyPI, so a `v0.1.0` tag push is the PyPI
release. Tag `m4` on the retirement commit.

## Docs to update on completion

- `docs/01-architecture.md` §Cargo workspace — the tree: `pyproject.toml`
  at the root, `python/` as package + tests only, `build.rs`,
  `.github/workflows/release.yml`, `docs/assets/`, `README.md`; §Crates —
  the CLI paragraph lists `--help`, `--version`, `--timing`, `--example`;
  §Python distribution — rewritten in the present tense: the binary in
  `scripts/`, `__main__` exec, version from Cargo, wheel tags, the sdist
  fallback; §Testing — `test_wheel.py`, the `wheel` CI job, the startup
  budget test, the release workflow's smoke jobs.
- `docs/03-roadmap.md` §M4 — status line: decisions (root pyproject,
  binary in `scripts/`, no console script, OPEN 1/2/4 outcomes), wheel
  sizes per platform, the measured startup time, what the by-hand
  install said.
- `docs/BACKLOG.md` — new lines: publish the workspace to crates.io so
  `cargo install riggen` works (OPEN 1); the M4 by-hand findings under an
  M4 heading; macOS signing / notarization if the human's VM run hits
  Gatekeeper.
- `AGENTS.md` current state — M4 done, tag `m4`, the install line, next
  v0.2 (Python SDK); the docs reading order gains `README.md` if it says
  anything the docs do not.
- No new ADR: ADR-0002 already holds the decision; the root `pyproject`
  and the `scripts/` placement are its consequences, recorded in 01.

## Open questions

- `⚠ OPEN 1:` **crates.io.** The roadmap's M4 line "reserve the crates.io
  name" is already done (`riggen` 0.0.1 published 2026-08-29). Making
  `cargo install riggen` real means publishing `riggen-mesh`, `-core`,
  `-export`, `-viewport` and `-app` too, with every release, and keeping
  their versions in step. Recommendation: **out of M4** — the wheel is
  the promise (SEED §5, ADR-0002); one backlog line, README says `cargo
  install --git https://github.com/Divelix/riggen riggen-app`. Human
  decides by step 6 (the README line).
- `⚠ OPEN 2:` **Who records the screencast.** The agent can generate one
  from the `five_minute_arm` scenario (deterministic, re-generatable on
  every UI change, but a synthetic cursor and stepwise motion on a
  1440×900 harness frame); the human can record the real thing in 30 s
  with real mouse motion. Recommendation: **the human records**, the
  agent writes the storyboard (the arm build's click sequence as a
  numbered list in the step) and ships the README with the hero still
  and an empty slot until the recording exists; step 7 is then a
  one-file commit. Human decides by step 6.
- `⚠ OPEN 3:` **`pyproject.toml` at the root or in `python/`.** Root:
  one README for GitHub and PyPI, `uv build` / `uvx --from .` from the
  checkout, licence files without copies. `python/`: the tree stays as 01
  draws it, but the README and licences are duplicated or reached with
  `..` paths maturin does not accept for `readme`. Recommendation:
  **root**. Human decides by step 1.
- `⚠ OPEN 4:` **Bundle the sample arm: `riggen --example arm`.** 64 KB
  embedded in the binary makes the first run after install one command
  and the README's first-run section three lines; without it the user
  downloads five files from GitHub or clones. Recommendation: **yes**,
  as part of step 2; the fixture files stay the single source
  (`include_bytes!` from `assets/fixtures/arm/`). Human decides by step 2.
