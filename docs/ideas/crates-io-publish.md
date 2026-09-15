# Idea: crates-io-publish

- Status: Open
- Raised: 2026-09-15
- Prompt (verbatim from the human): "crates.io publish"

## Problem

`cargo install riggen` doesn't work today. README §Install says outright:
"crates.io so that `cargo install riggen` works is a later release" and
points a Rust user at `cargo install --git` instead — a full checkout
clone, no pinned release, no `cargo add` for the library crates either.
This has sat on the backlog since M4 (plans/m4-distribution OPEN 1) and
is now v0.7's first line, chosen over GUI polish and the web demo's gaps
by the human's call (`/close-cycle` v0.6).

## Constraints it runs into

- `riggen` is already reserved on both PyPI and crates.io — name
  availability was checked 2026-08-29 (SEED.md line 13) and registered
  early as `crates/riggen`, an empty 0.0.1 stub (commit `29ade3f`). No
  squatting risk; the reservation is already ours.
- ADR-0009 already settled a "one version source" principle for the
  PyPI wheel: `pyproject.toml`'s `dynamic = ["version"]` reads
  `version.workspace = true` on `riggen-app`, so the wheel has been
  reporting the workspace version — the one that moves with the roadmap
  cadence, already at `0.7.0-dev` — since M4. That version has been
  public on PyPI/TestPyPI this whole time.
- `crates/riggen/Cargo.toml`'s own comment reads: the stub carries "its
  own version rather than the workspace's, so 0.1.0 stays free for the
  real thing." That was written when the workspace version was still
  near 0.1; it now assumes a starting point the workspace already passed
  five roadmap cycles ago, publicly.
- Only a `v*` tag push runs `publish-pypi` in `release.yml`; a
  `workflow_dispatch` goes to TestPyPI instead. crates.io has no
  equivalent staging index — whatever is uploaded there is live
  immediately and can only be yanked, never deleted.
- Confirmed with `cargo publish -p <crate> --dry-run --allow-dirty` just
  now: `riggen-mesh` (a leaf, no internal path deps) packages and
  verifies clean. `riggen-core` and `riggen-app` both fail immediately:
  *"all dependencies must have a version requirement specified when
  publishing … dependency `riggen-mesh` does not specify a version."*
  Every one of `riggen-mesh`, `-core`, `-export`, `-viewport` is declared
  in `[workspace.dependencies]` as `{ path = "crates/…" }` with no
  `version` — that has to change before anything above the leaf crate
  can publish.
- None of the five crates sets `description` (`cargo publish` only warns
  about its absence locally; crates.io's own upload is expected to
  refuse a crate without one — the metadata `crates/riggen`'s stub
  already carries, `repository`/`keywords`/`categories` included, isn't
  copied onto the crates that would actually ship).
- `cargo install riggen` always compiles from source — crates.io has no
  prebuilt-binary channel the way the PyPI wheel matrix does. A user
  needs a Rust toolchain and, on Linux, the same windowing/graphics dev
  headers CI already has for winit/wgpu (X11/Wayland/Vulkan), plus
  several minutes compiling `wgpu`/`egui`/`parry3d-f64` from scratch —
  worth saying in the README rather than leaving it to be discovered.
- Publishing `riggen-app` under the name `riggen` collides with the
  existing `crates/riggen` stub: Cargo refuses two packages of the same
  name in one workspace, so the stub has to be retired (or its directory
  taken over) in the same change, not left standing beside it.

## Options

### A — Publish under the shared workspace version, on the same tag as PyPI (recommended)

Give `riggen-mesh`, `-core`, `-export`, `-viewport` a `version` beside
their `path` in `[workspace.dependencies]`; add `description` (and the
`repository`/`keywords`/`categories` the stub already has) to all five
crates; retire `crates/riggen`'s stub in favour of `riggen-app`
publishing itself as `riggen`; add a `publish-crates-io` job to
`release.yml` beside `publish-pypi` — same `v*`-tag trigger, same rule
against ever publishing a `-dev` commit, `cargo publish --workspace` in
dependency order. `cargo install riggen` and `pip install riggen` then
report the same version for the same commit, extending ADR-0009's "the
workspace `Cargo.toml` is the one source" — already chosen for the
wheel — to the binary crate too.

Cost: one short plan — the four `version` additions, five `description`s,
the stub's retirement, a `release.yml` job, and a README correction
replacing "a later release" with what actually happens, plus a dry-run
smoke step before the real publish job.

Forecloses: a crates.io version number that means something different
from the PyPI/internal one — a second cadence some future agent would
have to remember to bump by hand.

### B — Give the published `riggen` binary its own version, starting at 0.1.0

Honour the stub's comment literally: internal crates keep
`version.workspace = true`, but the crate published to crates.io as
`riggen` carries a version bumped by hand, starting at 0.1.0, unrelated
to the roadmap-driven workspace number.

Cost: a third version scheme to maintain (`riggen-py` already carries
one Cargo → PEP 440 translation for `__version__`) with nothing in the
codebase checking the two stay related once they diverge — and a real,
observable oddity: `cargo install riggen` reporting `0.1.0` and
`pip install riggen` reporting `0.7.0` for output built from the
identical commit and binary.

### C — Publish only the four library crates now; leave `riggen` (the binary) for later

`riggen-mesh`, `-core`, `-export`, `-viewport` go to crates.io as
reusable libraries; `crates/riggen`'s stub and the `cargo install --git`
fallback stay exactly as they are.

Cost: doesn't do what the backlog line actually asks (`cargo install
riggen` installs the app — SEED.md §5), so v0.7's stated goal isn't met;
the version question in A vs. B is deferred rather than answered, and
returns whole the next time this is picked up.

### Do nothing

`cargo install --git` stays the only Rust-native route: a full checkout,
no pinned release, no `cargo add` for the libraries either. No stale
comment fixed, no version decision made, no registry commitment taken.
Keeps exactly the friction SEED.md §5 named this as the fix for.

## Recommendation

A. The PyPI wheel already answered this question — one version, read
from the workspace, moving with the roadmap — and it has been public
since M4 with nothing in the backlog or an ADR suggesting it was a
mistake. Deciding differently for crates.io buys a version number that
*reads* as freshly young at the cost of a second number to maintain
forever and a mismatch a user can actually see between the two install
paths for the same release. I'd change my mind if the human specifically
wants the crates.io crate to read as pre-1.0-and-new independent of how
far the internal roadmap has run — that's a defensible position, just a
different one than ADR-0009 already chose for the wheel.

## Decision for the human

1. Shared workspace version (A) vs. an independent 0.1.0 for the
   crates.io crate (B) — **A, unless you want `riggen` on crates.io to
   read as freshly-young regardless of the roadmap number.**
2. Does this become a plan now, or wait until the screencast and
   notarization are also ready? Nothing in A depends on either of the
   other two v0.7 items. **My read: it can go alone.**
3. Worth a new ADR, or a line in `docs/ARCHITECTURE.md` (§Python
   distribution, or a new §Crates.io distribution) plus fixing the
   stub's stale comment, folded into the plan's own docs-to-update list?
   No layer rule or non-goal is being bent — this is a mechanical
   extension of ADR-0009's existing choice. **No new ADR, unless you
   disagree.**
