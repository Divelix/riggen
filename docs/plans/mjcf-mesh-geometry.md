# Plan: mjcf-mesh-geometry

- Started: 2026-09-06
- Milestone: v0.4 (the file half)
- Idea (verbatim from the human): "So five points remain? Choose one that
  most est [sic, "easiest"] to do first and /plan it" — picked from
  `docs/03-roadmap.md`'s v0.4 §The file: nothing silently lost, as the
  least coupled to the other four (no schema bump, no interaction with the
  actuators table, the mimic/tendon couplings, or the include/attach/frame
  composition resolver).

## Goal

MJCF import stops dropping the two mesh shapes ADR-0015 §1 named and
warned about instead of reading: a `.msh` file (MuJoCo's own binary mesh
format) and an inline `<mesh vertex="…" face="…">` with no `file`
attribute. Both become an ordinary `Geom` on an ordinary `CollisionPolicy`
or visual, exactly like a `.stl`/`.obj` mesh does today — no new
`ImportWarning`/`ImportError` variant, because both stop being warned
about at all. `menagerie_style.xml` grows one of each and the corpus test
shows zero `GeomDropped` for them.

## Non-goals

- The other four v0.4 file-half items: `Robot::actuators` as a top-level
  table, the `<general>`/`<adhesion>`/`<muscle>` escape hatch, mimic-chain
  / `<joint ref>` / `<tendon><fixed>` couplings, and `<include>` /
  `<attach>` / `<replicate>` / `<frame>` composition. Untouched by this
  plan; still backlog lines under v0.4.
- No `.msh` **writer** — MJCF export keeps writing `.stl` for every mesh
  (ADR-0008); an imported `.msh` or inline mesh re-exports as an ordinary
  `.stl`, same as a `.obj` import does today. Nothing in the writer changes.
- No inline `<mesh>` attributes beyond `vertex`/`face`/optional `normal` —
  MJCF's inline `texcoord` is dropped the way a UV channel already is for
  every other mesh source (riggen has no material/texture model to hang it
  on).
- SDF import stays out of scope (ADR-0016 §6), unaffected either way.

## Design deltas

### `.msh` (step 1) — no open question

A `.msh` file is an ordinary mesh with a backing file on disk; the only
gap is that nothing in `riggen-mesh` can parse MuJoCo's binary format. It
slots into the same dispatch `riggen_mesh::load_mesh` /
`load_mesh_bytes` already does for `.stl` / `.obj`
(`crates/riggen-mesh/src/lib.rs:36`), and `mjcf_in.rs::mesh_id`
(`crates/riggen-export/src/mjcf_in.rs:854`) stops treating `.msh` as a
drop reason — the existing path/hash/registration logic
(`mjcf_in.rs:858`–`906`) already handles any file extension.

MuJoCo's binary mesh layout (from `mjCMesh::LoadSDF`/`LoadMSH` — no
spec doc, only the loader's own read order, confirmed against a mesh
written by `mujoco.export_mesh`/`mj_saveMesh` or a Menagerie `.msh` file
found during this step): a header of four `int32` counts — `nvertex`,
`nnormal`, `ntexcoord`, `nface` — then `nvertex` × 3 `float32` (positions),
`nnormal` × 3 `float32`, `ntexcoord` × 2 `float32`, `nface` × 3 `int32`
(vertex indices), little-endian, no material/color. `riggen_mesh::TriMesh`
only needs positions and face indices; normals/texcoords are read (to
advance the cursor correctly) and discarded, the same as OBJ's own unused
channels.

### Inline `<mesh vertex face>` (steps 2–3) — the open question

The parse itself is small: MJCF's `vertex`/`face`/`normal` are
space-separated number lists, the same shape `xml.rs`'s existing
attribute readers already parse for poses and scales — turning them into
`TriMesh` is direct.

The real gap is that `MeshAsset` (`crates/riggen-core/src/robot.rs:37`)
has no field for a mesh with no backing file — `path` is not optional,
`content_hash` is computed by hashing that path through a `FileSource`
(`mjcf_in.rs:888`), and `docs/02-data-model.md`'s schema section is
explicit that **a `.riggen` file stores paths, not mesh bytes**
(`file::to_json`, `crates/riggen-core/src/file.rs:260`, only rebases
`asset.path`, never writes geometry into the JSON) — a saved-and-reopened
document depends on the referenced file still existing on disk. An inline
mesh has no such file, so importing it as anything other than an ordinary,
disk-backed `MeshAsset` would make the imported document one `Save` away
from a `MeshNotFound` the user never sees coming.

**Recommendation** (⚠ open, see below): at import time, write the parsed
mesh out as a real `.stl` (`riggen_mesh`'s own writer, already used for
convex-hull output, `crates/riggen-mesh/src/hull.rs`) next to the source
MJCF, named after the `<mesh name="…">` (falling back to `inline_N` for an
unnamed one), suffixed to avoid colliding with a file already at that path
— and then register it exactly like any other imported mesh. On the web
build, where there is nowhere to write, the same bytes go into the app's
`DroppedSet` (`crates/riggen-app/src/app/file_io.rs:39`) under a
synthesized name instead of a real file, which is the existing precedent
for "a mesh with no filesystem backing" (ADR-0017). Either way the mesh
becomes ordinary, disk- or drop-backed `MeshAsset` from the moment it is
imported — no new `MeshAsset` variant, no schema bump, no change to
`file::save`/`to_json`.

This makes `mjcf_in::load` / `from_mjcf`
(`crates/riggen-export/src/mjcf_in.rs:230`, `:252`) return the synthesized
`(name, bytes)` pairs alongside `(Robot, Vec<ImportWarning>)`, so each of
its three callers can place them where its own platform expects a mesh to
live:
- `crates/riggen-app/src/app/file_io.rs:250` (`finish_import`, the GUI) —
  write to disk beside the source file for `Files::Disk`, insert into the
  `DroppedSet` for `Files::Dropped`.
- `crates/riggen-app/src/cli.rs:264` (the CLI import) — write to disk
  beside the source file; the CLI has no `Files` abstraction at all today.
- `crates/riggen-py/src/robot.rs:816` (`load_mjcf`) — write to disk beside
  the source file, the same as the GUI's `Disk` case.

**Decided (step 2): "write a file beside the source MJCF."** The human
confirmed the recommendation as-is: no new `FileSource`/`MeshAsset`
concept, every caller's code stays identical to importing an ordinary
mesh; the web build's `DroppedSet` equivalent for "nowhere to write"
carries over unchanged. Recorded as a paragraph in `docs/02-data-model.md`
§Geometry (plumbing, not a user-facing tradeoff — no ADR).

## Steps

- [x] **Step 1 — `.msh` reads.** `riggen_mesh::msh` (or a `load_msh` in an
  existing module): parse the binary layout above into a `TriMesh`,
  `MeshError::Truncated`/`Invalid` on a short or inconsistent file, in the
  vocabulary `stl.rs`/`obj.rs` already use. Wire `.msh` into
  `load_mesh`/`load_mesh_bytes`'s extension dispatch
  (`crates/riggen-mesh/src/lib.rs:36`). Remove `mjcf_in.rs`'s `.msh` drop
  branch (`:854`) so it falls through to the ordinary path/hash/register
  logic. A `.msh` fixture: write one by hand (a cube, matching
  `cube_binary.stl`'s vertices) with a `#[ignore]`d regenerator following
  `stl.rs:172`'s pattern, plus round-trip and truncation tests in
  `riggen-mesh`, and one MJCF import test asserting a `.msh`-referencing
  `<geom>` produces a `Geom` with no warning.
- [x] **Step 2 — the open question, answered.** The human confirms or
  amends the "write beside the source" recommendation above (or picks an
  alternative); this step records the decision — a paragraph in
  `docs/02-data-model.md` §Geometry, or a new ADR if the human's answer
  turns out to be a real tradeoff rather than plumbing — before step 3
  is implemented against it. No code.
- [ ] **Step 3 — inline `<mesh vertex face>` reads.** Parse `vertex` /
  `face` / optional `normal` into a `TriMesh` in `mjcf_in.rs`; change
  `mjcf_in::load` / `from_mjcf`'s return type to also carry the
  synthesized `(name, bytes)` pairs; update its three callers per the
  Design deltas list to place those bytes per step 2's decision; remove
  the inline-mesh drop branch (`mjcf_in.rs:851`). Tests: a unit test that
  an inline mesh imports as a `Geom` with no warning and the synthesized
  file/entry exists where step 2 said it would; an app-level test (beside
  `file_io.rs`'s existing import tests) that a document imported with an
  inline mesh survives a save/reopen round trip.
- [ ] **Step 4 — the corpus.** Grow `assets/fixtures/menagerie_style.xml`
  with one `.msh`-referencing `<geom>` and one inline `<mesh vertex face>`
  `<geom>`; update `import.rs`'s pinned warning-by-warning test
  (`crate::mjcf_in::load(&root.join("menagerie_style.xml"), &memory)`,
  `mjcf_in.rs:406`, and its `Disk` sibling at `:408`) so `GeomDropped` no
  longer fires for either. Confirm the `mujoco` CI job's round-trip
  (export → reimport → export, held to the *original* document's
  `fk.json`) still passes with the two new geoms in the model — both
  export as ordinary `.stl` either way, so no MuJoCo-side change is
  expected, but the job has not run against them until this step.

Each step is its own commit; step 2 is `docs:`, the rest `feat(mesh,export,app,py):` as appropriate for what each touches.

## Acceptance

Importing `menagerie_style.xml` produces zero `GeomDropped` warnings for
its `.msh` and inline-mesh geoms (down from the two the fixture pins
today); both render as ordinary link geometry and re-export as ordinary
`.stl` files; a document imported with either mesh kind survives a native
save/reopen round trip with no `MeshNotFound`; `cargo test` (including the
new `riggen-mesh`, `riggen-export`, `riggen-app` and `riggen-py` cases) and
the `mujoco` CI job pass; `cargo fmt --check` and `cargo clippy
--all-targets -- -D warnings` pass.

## Docs to update on completion

- `docs/02-data-model.md` §Geometry (around line 706) and §Nothing is
  dropped silently (around line 733) — `.msh` and inline mesh move from
  "refused" language to "read"; the step-2 decision on where a
  file-less mesh's bytes land gets its paragraph (or ADR citation) here.
- `docs/adr/0015-*.md` is append-only and is not edited; if step 2
  produces a new ADR, its README row is added and 0015's row gains a note
  that §1's "warned and skipped" line for `.msh`/inline mesh is narrowed.
- `docs/03-roadmap.md` §v0.4 — the "Geometry the import refuses" bullet's
  two items are struck or annotated done; the section's status line is
  untouched (three of five file-half items remain).
- `AGENTS.md` current state — unchanged unless this closes enough of the
  file half to be worth a mention; likely not yet, with three items still
  open.

## Open questions

None open. The step-2 question (where a file-less imported mesh's bytes
are materialized) is decided: beside the source MJCF, `docs/02-data-model.md`
§Geometry.
