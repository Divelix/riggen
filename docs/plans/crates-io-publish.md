# Plan: crates-io-publish

- Started: 2026-09-16
- Milestone: v0.7
- Idea (verbatim from the human): "crates.io publish" — then "crates-io-publish option A"
- Idea: docs/ideas/crates-io-publish.md (absorbed)

## Goal

`cargo install riggen --locked` builds the app from crates.io, with no
checkout. Five crates publish: `riggen-mesh`, `-core`, `-export`,
`-viewport`, and the app, whose package is now named `riggen` and replaces
the 0.0.1 name reservation. All five carry the one workspace version, the
same one the wheel reports (ADR-0009's single version source, extended to
crates.io). `release.yml` publishes them on the same `v*` tag push that
publishes to PyPI, and never from a `-dev` commit. CI checks that the
workspace still packages on every push. The README gives
`cargo install riggen` as an install line, with its real cost: a Rust
toolchain, the Linux windowing headers, and a long first compile.

## Non-goals

- `riggen-py` on crates.io: it is a cdylib extension module, and
  `publish = false` keeps it off crates.io.
- Prebuilt binaries through `cargo binstall` or a release-asset archive.
  The wheel is the path without a Rust toolchain (ADR-0002).
- The screencast and macOS notarization, the other two v0.7 lines.
  Nothing here depends on them.
- Moving `crates/riggen-app/` to a new directory, and renaming the lib
  target (`riggen_app`). Only the *package* name changes (see Design
  deltas).
- Library API polish (docs.rs pages, `#![warn(missing_docs)]`). The
  libraries publish because the app's dependencies have to be on
  crates.io; they are not yet presented as reusable.

## Design deltas

Option A as the idea recommends it. The idea's decisions 2 and 3 take its
recommendations: this plan goes ahead alone, and it takes **no new ADR**,
because it is ADR-0009's version rule applied to one more index. The
record goes in a new Architecture §Crates.io distribution.

- **`Cargo.toml` `[workspace.dependencies]`**: each internal crate gets
  `version = "=0.7.0-dev"` next to its `path`. The version requirement is
  exact, so the crates publish as a set and a mix of versions cannot
  resolve. This is a second place the version is written down, but Cargo
  checks it: a path dependency whose version does not match the
  requirement fails the build. `/close-cycle`'s two bump steps change both
  places.
- **Package metadata**: `repository`, `homepage`, `keywords` and
  `categories` move to `[workspace.package]`, taken from the stub. Each of
  the five crates gets its own `description`. The app gets
  `readme = "../../README.md"` (to be confirmed in step 1; if Cargo will
  not package a readme from outside the crate, use a symlink, as the stub
  already does for its licence files).
- **The app package**: `name = "riggen"`, with `[lib] name = "riggen_app"`
  kept, so that `riggen_app::` in the tests, `riggen_app.wasm` and
  `web/main.js` stay the same. Every `-p riggen-app` becomes `-p riggen`:
  `ci.yml`, `python/build_wheel.py`, `web/build.sh`, the README, and the
  `visual-debug` skill. `python/riggen/show.py`'s hint becomes
  `cargo install riggen --locked`. The stub `crates/riggen/` is deleted
  and removed from `members`. Its 0.0.1 release on crates.io is left as
  it is: 0.7.0 supersedes it and nothing needs to be yanked.
- **The app's package contents**: an `include` list keeps the 23 MB
  `tests/` directory (the snapshot PNGs) out of the package, which must
  stay under crates.io's 10 MB limit. The `--example arm` bytes are the
  problem. Today `example.rs` uses `include_bytes!` on
  `../../../assets/fixtures/arm/*`, a path outside the package that fails
  to compile from the registry. The five files (about 50 KB) get a home
  inside the crate: `crates/riggen-app/assets/arm/`, as symlinks to the
  fixtures, which Cargo dereferences when it packages. The fixtures stay
  the single source for the tests.
- **`build.rs`**: a crate built from the registry has no `.git`. A new
  source goes between the environment variables and `git`: the
  `git.sha1` in `.cargo_vcs_info.json`, which `cargo package` writes. It
  gives the first 7 characters and needs no dirty check (the release
  refuses a dirty tree). With it, `riggen --version` from
  `cargo install` names its commit instead of `unknown`.
- **`release.yml`**: a new `publish-crates-io` job with the same `v*` tag
  condition as `publish-pypi`, running after `smoke`, in its own
  environment `crates-io`. It first checks that the workspace version has
  no `-` suffix (defence in depth: the tag rule in
  `.agents/rules/git.md` already forbids tagging a `-dev` commit), then
  runs `cargo publish --workspace --locked`. Cargo orders the packages by
  dependency and waits for each one to reach the index. A
  `workflow_dispatch` never publishes here, because crates.io has no
  staging index.
- **`ci.yml`**: a new `package` job runs
  `cargo publish --workspace --dry-run --locked` on every push. Cargo
  resolves the not-yet-published internal crates from a local overlay, so
  a later `include_bytes!` or a missing `description` fails on the commit
  that causes it rather than at release time.
- **Architecture**: the crate tree loses `riggen/` and notes the package
  name. A new §Crates.io distribution sits beside §Python distribution.

## Steps

Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [x] **[3]** Step 1 — **the workspace packages.** Make
  `cargo publish --workspace --dry-run --allow-dirty` pass locally. This
  covers the exact-version requirements, the shared and per-crate
  metadata, `publish = false` on `riggen-py`, the app's `include` list,
  the `assets/arm/` symlinks with `example.rs` pointed at them, and the
  readme question. This step settles the unknowns: whether the dry-run
  overlay verifies five unpublished crates together, whether an
  out-of-package readme is accepted, and the `.crate` sizes, which go in
  the commit message. The app keeps its current package name for now.
  Test: the dry-run's output, plus `cargo package -p riggen-app --list`
  showing no `tests/` entry and the five `assets/arm/` files, plus the
  existing `--example arm` test still passing from the workspace.
- [x] **[2]** Step 2 — **the app is `riggen`.** Rename the package, keep
  `[lib] name = "riggen_app"`, delete `crates/riggen/` and its `members`
  entry, and change every `-p riggen-app` (`ci.yml`, `build_wheel.py`,
  `web/build.sh`, the README developing block, the `visual-debug` skill).
  Update `show.py`'s hint and its test, if one pins the text. Update
  Cargo.lock. Test: `cargo test` passes; `web/build.sh` still produces
  `riggen_app_bg.wasm`; `python python/build_wheel.py --binary-only`
  still finds the binary; the dry-run from step 1 still passes with a
  `riggen` package in it.
- [ ] **[2]** Step 3 — **`--version` from the registry.** Read
  `.cargo_vcs_info.json` in `build.rs`, between the environment variables
  and `git`. Test: a unit test of the parsing function against a sample
  JSON, and a manual check: `cargo install --path` on a `cargo package`
  output, from an unpacked `.crate` outside the repository, prints the
  hash of HEAD.
- [ ] **[2]** Step 4 — **CI guards it.** Add a `package` job to `ci.yml`
  (`cargo publish --workspace --dry-run --locked --target-dir
  target/package-verify`, see Open questions), with the Linux
  headers the `test` job already installs. Test: the job goes green on
  push (the human pushes; the agent reads the run with `gh run watch`).
- [ ] **[2]** Step 5 — **the release publishes.** Add the
  `publish-crates-io` job to `release.yml`, with the `-dev` guard, the
  `crates-io` environment, and the one-time setup written in the file's
  header comment, as `publish-pypi` already has it. Test:
  `actionlint` passes, if installed. Otherwise, a `workflow_dispatch`
  run shows the job as *skipped*, which is its correct behaviour there.
- [ ] **[1]** Step 6 — **the README says it.** In §Install, add
  `cargo install riggen --locked` as the second route, with its cost: a
  Rust toolchain, the Linux packages (the list from `ci.yml`), and a
  first compile of several minutes, measured in step 1 and stated here.
  In §Python, remove the "later release" paragraph and change the
  `cargo install --git` fallback to the crates.io line. Test: read by the
  agent against `ci.yml`'s package list; `test_wheel.py` and the SDK
  suite still pass, in case they check README text.

## Acceptance

- `cargo publish --workspace --dry-run --locked` passes in CI on `main`.
- From a clean directory outside the checkout, run
  `cargo install --locked --path <unpacked riggen-0.7.0-dev.crate>`, with
  its internal dependencies taken from the dry-run's local registry. It
  builds a binary whose `riggen --version` names HEAD's hash and whose
  `riggen --export mjcf --out … <unpacked>/assets/arm/arm.riggen` writes
  the arm from the copy the package carries (`--example` is for the
  window and refuses `--export`, `cli.rs`). This is the local stand-in
  for `cargo install riggen`.
- The real check is the roadmap's. At the `v0.7.0` tag, which the human
  pushes at `/close-cycle` and not in this plan, `publish-crates-io`
  goes green, and on a clean machine with only `cargo`,
  `cargo install riggen --locked` installs `riggen 0.7.0` and opens the
  arm. The human runs this. `/close-cycle` records the result.

## Docs to update on completion

- `docs/ARCHITECTURE.md`: in the crate tree, remove `riggen/` and note
  `riggen-app/` → package `riggen`. Add a new §Crates.io distribution
  covering the five crates, the exact-version requirements, the
  `include` list and the `assets/arm/` symlinks, the `.cargo_vcs_info.json`
  source of the hash, the two jobs, the one-time setup, and the measured
  `.crate` sizes and cold-install time. Update §Python distribution's
  `-p riggen-app` recipe and its "`cargo install --git`" mention.
- `docs/ROADMAP.md` v0.7: mark the crates.io line done in the status
  line when the cycle closes.
- `.agents/skills/close-cycle/SKILL.md` steps 8 and 9: bump the
  `[workspace.dependencies]` requirements together with
  `[workspace.package].version`.
- `.agents/rules/git.md` §Tags: a `v*` tag now publishes to crates.io
  too, which cannot be undone except by a yank. This is one more reason
  never to tag a `-dev` commit.
- `SEED.md` §5 and the crate table: the app's package name.
- `docs/BACKLOG.md`: remove the crates.io line if it is still listed.
- `AGENTS.md` current state: v0.7's first line landed.

## Open questions

- Settled in step 1: the dry-run overlay (`target/package/tmp-registry`)
  verifies the five unpublished crates together; an out-of-package
  `readme = "../../README.md"` is accepted and copied to the tarball root
  as `README.md`, so no symlink is needed; Cargo drops the three
  `tests/` targets from the packaged manifest with a warning rather than
  failing. The stub `riggen@0.0.1` still packages in the dry run
  (`already exists on crates.io index`, a warning) until step 2 deletes
  it. `cargo install --path` from inside the checkout reuses the
  workspace `target/`, so the cold timing had to be taken with a fresh
  `--target-dir`.

- ⚠ OPEN: **The first publish of the four library crates** (human,
  before the `v0.7.0` tag, not blocking any step). crates.io trusted
  publishing can only be set up on a crate that already exists, and
  `riggen-mesh`, `-core`, `-export` and `-viewport` do not exist yet.
  Two ways:
  - (a) A `CARGO_REGISTRY_TOKEN` secret in the `crates-io` environment
    for the first release, replaced by trusted publishing afterwards.
  - (b) The human runs the first `cargo publish --workspace` by hand at
    `v0.7.0` and sets up trusted publishing on all five crates before
    `v0.8.0`.

  Step 5 writes the job for trusted publishing
  (`rust-lang/crates-io-auth-action`) with a token fallback, so either
  way works. **Agent's read: (a).** The tag stays the only way a
  release happens.
- Settled in step 2 (human): **the directory keeps its name.**
  `crates/riggen-app/` holds the package `riggen`, and the crate tree
  says so. The step also found that `[lib] name = "riggen_app"` has to be
  written out: Cargo derives the lib name from the package name.
- Found in step 2: **a dry run in the workspace `target/` breaks the next
  `cargo test`.** The verify build compiles the packaged app into
  `target/debug`. Because the lib is also a `cdylib`, its rlib has no
  hash in its name (`libriggen_app.rlib`), so that build overwrites the
  workspace's copy with one linked against other `eframe`/`serde`
  builds. Cargo still thinks its own unit is fresh, and the bin and
  `tests/visual` fail with "multiple different versions of crate". The
  fix is `cargo clean -p riggen`. The prevention is to run every dry run
  with `--target-dir target/package-verify`. Step 4's job does the same,
  and §Crates.io distribution will say so.
- ⚠ OPEN: **`--locked` in the README** (agent, step 6). Without it,
  `cargo install` ignores the packaged `Cargo.lock` and may resolve a
  newer egui or wgpu than the ones pinned together under ADR-0001.
  Recommend `--locked` unless step 1's install shows a reason not to.
