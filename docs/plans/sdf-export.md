# Plan: sdf-export

- Started: 2026-08-31
- Milestone: v0.2 (`docs/03-roadmap.md` § "Still open" — second half of
  "MJCF import; SDF export")
- Idea (verbatim from the human): "MJCF import; SDF export"

## Goal

`riggen --export sdf --out DIR robot.riggen`, File › Export… with SDF
ticked, and `robot.export(dir, format="sdf")` write `<name>.sdf` beside the
same `meshes/` the other two writers use — a third dumb serialiser of
`ResolvedRobot`, exactly the "one more writer, not a new resolve" ADR-0004
§ Consequences promised. It carries links, joint kinds with `<axis>`,
`<limit>` and `<dynamics>`, inertials, visual and collision geometry
(meshes and primitives, with SDF's **native `<capsule>`** — the one place
SDF beats URDF), our frames as SDF's own `<frame>`, and the mimic and
actuator per ADR-0016. `Format` stops being a three-valued enum, because
"both" is a lie once there are three writers. CI loads the written file in
**libsdformat itself** — the spec's own parser, through its Python
bindings — and holds its link, frame and mimic readings to `fk`, the way
the `mujoco` job already does for MJCF.

## Non-goals

- **SDF import** — no `sdf_in.rs`. The reading direction stays URDF (and
  MJCF, from `docs/plans/mjcf-import.md`).
- **A Gazebo model package** — no `model.config`, no `<world>`, no
  `<include>`, no `<plugin>`, no `<gazebo>` extension blocks (ADR-0016
  fixes this; the export directory stays what ADR-0008 made it).
- No new document field. Everything written comes out of `ResolvedRobot`
  as it stands today.
- Nothing in the viewport or the panels changes; the only UI is the export
  dialog's format control.
- USD, still and explicitly (`docs/03-roadmap.md` § "What not to spend
  agent time on").

## Design deltas

- **`Format` becomes a set, not an enum.** `Format { mjcf: bool, urdf:
  bool, sdf: bool }` with `writes_mjcf/urdf/sdf`, `Default` = all three,
  and a `FromStr` accepting `mjcf`, `urdf`, `sdf`, `both` (kept, = the two
  old ones) and `all`. Touches `resolve.rs`, `export.rs`, `cli.rs`
  (`--export`'s value and `--help`), `export_dialog.rs` (three checkboxes
  where three radio buttons were, and its snapshot), `riggen-py`'s string
  parse and the SDK's `Literal`.
- **`riggen-export/src/sdf.rs`**, beside `mjcf.rs` and `urdf.rs`, reading
  the same `ResolvedRobot`. `MeshPathStyle` maps without a new variant:
  `Relative` → `meshes/<stem>.stl`, `Package(n)` → `model://n/meshes/…`,
  `Absolute` → `file:///…`.
- **`xml.rs` grows text elements.** SDF puts its numbers in element bodies
  (`<mass>2.7</mass>`, `<pose>x y z r p y</pose>`), which the writer has
  never needed: `Xml::text(tag, attrs, &str)`, plus `xml::pose6(&Pose)`
  over the existing `Pose::to_xyz_rpy` that `urdf::origin_attrs` already
  uses.
- `docs/02-data-model.md` §Format mapping gains a **third column**, and
  §`ResolvedRobot`'s "Adding SDF later is a new writer, not a new resolve"
  becomes the present tense.
- `docs/01-architecture.md` §Crates tree comment, the export-dialog
  paragraph (line 330) and the CLI line (line 624).
- **ADR-0016** — the SDF conventions: version, pose convention, frames,
  mimic, actuator, mesh URI, and what validates it in CI. Listed as step 1.

## Steps

- [x] 1 — **ADR-0016: SDF conventions, and the checker that proves them.**
  Decide the `<sdf version>` (OPEN 1), `<pose relative_to>` versus poses
  flattened into the model frame (OPEN 2), our `Frame` as SDF's `<frame
  attached_to>`, the mimic as SDF 1.11's `<axis><mimic>` or a comment
  (OPEN 1 decides it too), the actuator as a comment (SDF has no actuator
  either, so ADR-0014's URDF reasoning applies verbatim), and mesh
  `<uri>`s. Written **after** running the candidate checkers of OPEN 3
  against a hand-written two-link SDF, so the decision is a measurement,
  not a guess. *Retires the "can CI tell us we are wrong?" unknown first —
  a writer nobody validates is a text generator.*
- [x] 2 — **`Format` becomes a set.** The struct, `FromStr`/`Display`, and
  every caller: CLI value and `--help`, the export dialog's three
  checkboxes, `riggen-py`'s parse and error message, the SDK `Literal` and
  `_riggen.pyi`, the CLI and dialog tests, the refreshed dialog snapshot
  (shown to the human, per ADR-0003). No SDF is written yet and no
  existing output changes — the shared-type churn lands alone and
  revertible.
- [ ] 3 — **`xml.rs`: text elements and `pose6`.** `Xml::text`,
  `xml::pose6`, unit tests including escaping inside a body and the
  `-0` folding `num` already does.
- [ ] 4 — **`sdf::write`: model, links, inertials.** `<sdf><model
  name>` → `<link name><pose …><inertial><pose><mass><inertia ixx…/>`, in
  the resolved order. A golden test against `test_util::every_joint_kind`,
  the shape `mjcf.rs` and `urdf.rs` both use.
- [ ] 5 — **Geometry.** `<visual>` / `<collision>` with `<geometry><mesh>
  <uri>`, and `<box><size>` (full extents) / `<cylinder>` / `<sphere>` /
  native `<capsule>` — so the "a capsule becomes a cylinder plus a
  warning" line of the URDF writer has no counterpart here. `MeshPathStyle`
  applied to the URIs, tested at all three styles. The golden grows.
- [ ] 6 — **Joints, frames, mimic, actuator.** `<joint type name>` with
  `<parent>` / `<child>` and **no `<pose>`**, `<axis><xyz>` with **no
  `expressed_in`** — SDF's own defaults are ADR-0004's joint frame
  (ADR-0016 §2) — `<limit lower upper>` with `effort` / `velocity` only
  when non-zero, `<dynamics damping friction>` only when non-zero, all
  four `JointKind`s by their own SDF names, the `world_joint` for a
  non-floating base, `<frame attached_to>` for every `ResolvedSite`, the
  native `<mimic>` and the actuator comment. The golden is complete;
  `export.rs`'s directory test writes all three files.
- [ ] 7 — **CI and the acceptance.** An `sdf` job that adds the OSRF apt
  repository, installs `gz-jetty-sdformat-python`, exports the arm with
  `--fk-samples`, and runs `python/tests/test_sdf_load.py` — the mirror of
  `test_mjcf_load.py`: `Root.load()` raises on any error, every link's and
  every frame's world pose at the five sampled configurations matches
  `arm.fk.json`, and every `<mimic>` agrees with the sampled `q` while a
  coupling the samples show and the file lacks fails. Over the `.riggen`
  and the URDF-imported arm both.

## Acceptance

`cargo test --workspace` green, and:

```sh
riggen --export sdf --fk-samples --out target/sample-sdf assets/fixtures/arm/arm.riggen
python3 python/tests/test_sdf_load.py target/sample-sdf
```

with `gz-jetty-sdformat-python` installed from
`packages.osrfoundation.org` (ADR-0016 §6 — an apt package, not a pip
one, so no `uv run --with`): `libsdformat` parses the file with no error,
and every link's and every frame's world pose at the five sampled
configurations matches `arm.fk.json` to **1e-9**. The bar is tighter than
the `mujoco` job's 1e-6 because nothing here is a simulator with its own
integrator — only the reference parser's f64 arithmetic against ours.

## Docs to update on completion

- `docs/02-data-model.md` — §Format mapping's third column (every row, and
  new rows only where SDF differs: capsule, frame, mesh URI); §`ResolvedRobot`
  loses "Adding SDF later is a new writer" for the fact; `ExportOptions`'s
  `Format` in the code block.
- `docs/01-architecture.md` — §Crates tree comment (`sdf` beside `mjcf`,
  `urdf`), the export-dialog paragraph, the CLI usage line, the SDK table
  row for `export`'s `format`.
- `docs/adr/README.md` — the ADR-0016 row.
- `docs/03-roadmap.md` — the "Still open" line loses "SDF export"; a done
  paragraph is added. If `docs/plans/mjcf-import.md` is retired too, that
  line is gone and `/close-cycle` is next.
- `AGENTS.md` — current state, one line.
- `README.md` — `--export`'s formats, and the export section.

## Open questions

All four are **decided**; step 1's measurements are in ADR-0016 §Context.

- ✅ OPEN 1: **Which SDF version.** `version="1.11"`, mimic native
  (ADR-0016 §1). Measured: `libsdformat14` (Gazebo Harmonic, the LTS) and
  `libsdformat15` both read 1.11 and round-trip `<axis><mimic>`, so the
  newer spec costs no audience.
- ✅ OPEN 2: **`<pose relative_to>` or flattened poses.** `relative_to`,
  with no `<pose>` on the joint and no `expressed_in` on the axis — SDF's
  own defaults are ADR-0004's joint frame (ADR-0016 §2). The writer does no
  arithmetic. The cost, stated: `pybullet` ignores `relative_to` and reads
  our SDF wrong; its users want the `.urdf` the same export writes.
- ✅ OPEN 3: **What checks the file in CI.** `gz-jetty-sdformat-python`
  from `packages.osrfoundation.org` — the reference parser's own bindings
  (ADR-0016 §6). `Root.load()` raises on any error, and
  `semantic_pose().resolve()` gives **f64** poses, so the 1e-9 bar holds.
  `pybullet` is out (f32, and silently misreads `relative_to`); `gz sdf -k`
  is out (exits 0 on hard errors, computes no poses); PyPI's `pinocchio`
  wheel has no SDF parser compiled in.
- ✅ OPEN 4: **Order against `docs/plans/mjcf-import.md`.** MJCF import
  went first and is retired (commit c9f7780).
