# Plan: unweighed-links

- Started: 2026-09-13
- Milestone: v0.6 — the import gap's last mile
- Idea: docs/ideas/unweighed-links.md (absorbed); every decision at its preferred answer
- Idea (verbatim from the human): "next thing in roadmap"

## Goal

A link with geometry and no mass stops blocking an export unless it moves.
An unweighed **static** link is written without `<inertial>`, and a static
link's tensor-shape failures under `check` no longer block. The export says
which static links carry no mass: a `warning:` line in the CLI, a
`RiggenWarning` in the SDK, a note in the dialog. A **moving** link with
geometry and no mass is still refused, and the refusal names the link and
the fix. One gesture gives every unweighed link a material, in the GUI and
the SDK. Measured over Menagerie (HEAD `ca95c4d`), 45 of the 53 files that
import and refuse to export now export. The 8 that remain each name a
moving link without mass.

## Non-goals

- **No density at import.** ADR-0015 §7 stands, and MJCF's
  `inertiafromgeom` 1000 kg/m³ is not borrowed (the idea's option B).
- **Not the other v0.6 lines:** `validate` checking geom poses and
  `Override` numbers, a `PackageMap` UI, `MoveJointFrame` on collision
  geometry. A `NonFinite` inertial keeps blocking here, static or not.
- **Not the 89 import refusals.** Composite joints, multiple roots,
  identifiers and `<attach>` stay where ADR-0022 and ADR-0026 left them.
- **No mass model.** No reading of geom `mass`/`density` attributes
  (`MassFromGeomIgnored` stays a warning), no boolean of overlapping
  shells.
- **No schema change.** The document is untouched, and the new command is
  not a stored field.

## Design deltas

- **`riggen-export::resolve`** (DATA-MODEL §ResolvedRobot).
  - Today's rule: "a moving link must have mass; an empty static body gets
    no `<inertial>`". It becomes "a moving link must have mass; a static
    link with no mass to compose, empty or unweighed, gets no
    `<inertial>`".
  - The pass covers `InertialError::NoDensity` under **`Computed`** only. A
    static `Hybrid` link without a density still blocks, because the user
    typed a mass for it.
  - On a static link, `check`'s `NotPositiveDefinite` and
    `TriangleInequality` are written through as the document holds them.
    `NonFinite`, `NotSymmetric` and a negative mass still block.
- **`ResolvedRobot` gains `massless: Vec<String>`**: the static links with
  geometry written without `<inertial>`, in link order. The writers never
  read it. The CLI, the SDK and the dialog report it.
- **`ExportError`.** `NoDensity` on a moving link with geometry becomes
  `ZeroMassMovableLink`. Its message is told apart by whether the link has
  geometry: "moves and has no mass — give it a material or a density"
  versus today's "add a mesh, a material, or an override".
  `Inertial { error: NoDensity }` stays for a static `Hybrid`.
- **`riggen-core::Command` gains `AssignMaterialToUnweighed(String)`**
  (DATA-MODEL §Commands).
  - It follows `SetActuators`: many links, one command, one history entry.
  - Every link with visuals, no material, and an inertial that needs a
    density it has not got (`Computed { density_override: None }` or
    `Hybrid`) gets the material.
  - Refused for an unknown material. No change when nothing qualifies.
- **SDK.**
  - `Robot.export` warns `RiggenWarning` once per massless static link.
    `_riggen.Robot.export` returns `(paths, warnings)`, the `load_*`
    shape.
  - `robot.assign_material_to_unweighed(name) -> list[Link]`.
- **App.**
  - The export dialog lists the massless static links under the ready line
    as weak notes (not blockers).
  - The one-click assign sits where ⚠ OPEN 1 decides.
- **ADR-0032**, "the export gate guards what a simulator reads". It amends
  DATA-MODEL's rule, cites ADR-0015 §7 as standing, and records the scan.

## Steps

Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [ ] **[3]** Step 1 — The static-link pass in `resolve` and
  `ResolvedRobot::massless`. Unit tests cover:
  - an unweighed static link (with geometry and without);
  - a static `Hybrid` without a density, which still blocks;
  - a static zero tensor, which is written;
  - a static `NonFinite`, which blocks;
  - an unweighed moving link, which still blocks.

  Then establish MuJoCo's verdict in scratch, never CI:
  - Re-export every Menagerie file that now exports and load it with
    `mujoco.MjModel.from_xml_path` under the `test_mjcf_load.py` warning
    hook. A body left without `<inertial>` falls back to `inertiafromgeom`
    over our re-exported geoms, and MuJoCo may warn about a mesh's volume
    where the source never did.
  - Load a synthetic static body whose tensor violates the triangle
    inequality, to settle ⚠ OPEN 2.

  Record both results here. DATA-MODEL §ResolvedRobot and §Inertials
  change in the same commit.
- [ ] **[2]** Step 2 — The fixture: `menagerie_style_arm.xml`'s `tool`
  body, static and holding only a site today, gains a mesh geom and no
  `<inertial>`.
  - The corpus route of the `mujoco` job passes: zero warnings, `fk` to
    1e-6.
  - The `mjcf_in` corpus tests and goldens are updated for the new geom.
  - Run locally through `uv run --with mujoco` before committing.
- [ ] **[1]** Step 3 — ADR-0032, with step 1's measurement.
- [ ] **[2]** Step 4 — The moving-link refusal: `NoDensity` on a moving
  link becomes `ZeroMassMovableLink`, and the message depends on whether
  the link has geometry.
  - Resolve tests pin both messages.
  - `export_blocked` is refreshed if its text changes, and the image is
    shown to the human.
- [ ] **[2]** Step 5 — The notice in the CLI and the SDK.
  - The CLI prints `warning: link "X" is static and carries no mass;
    written without <inertial>` for each `massless` entry. Covered by
    `tests/cli.rs`.
  - `_riggen.Robot.export` returns the warnings, `Robot.export` warns
    `RiggenWarning`, and `_riggen.pyi` follows. Covered by an SDK test.
- [ ] **[2]** Step 6 — The notice in the export dialog: weak lines under
  the ready line. New snapshot `export_massless_static`, image shown to
  the human.
- [ ] **[1]** Step 7 — `Command::AssignMaterialToUnweighed` in core. Tests:
  one undo reverts every link; unknown material refused; `Override` links,
  links with a material and empty links untouched. Also
  `robot.assign_material_to_unweighed` in the SDK, with its test.
- [ ] **[2]** Step 8 — The one-click assign in the app, where ⚠ OPEN 1
  decides. After the command the dialog re-resolves, so the blockers it
  cleared disappear. New snapshot, image shown to the human.
- [ ] **[1]** Step 9 — The acceptance scan: all 261 Menagerie `.xml`
  through `--export mjcf` with the release build. Bucket as ADR-0026's
  table does, and record the numbers here for `/retire-plan`.

## Acceptance

- The scan (step 9) shows **164** of 172 importing files exporting, or the
  number step 1 justified if MuJoCo's verdict moved it. Every file still
  refused names a moving link with no mass. No file that exported before
  stops exporting.
- Every file step 1 newly exports loads in MuJoCo with zero warnings.
- The `mujoco` CI job passes with `menagerie_style.xml` carrying a static
  link with geometry and neither material nor density.
- `cargo test --workspace` passes, including the resolve tests, the
  command tests, `tests/cli.rs`, and the snapshots `export_massless_static`
  and the one-click scenario.
- `pytest python/tests/sdk` passes.

## Docs to update on completion

- `docs/DATA-MODEL.md`:
  - §ResolvedRobot: the static-link rule, `massless`, the
    `ZeroMassMovableLink` wording (step 1/4, confirm at retirement).
  - §Inertials: export-time checks on a static link.
  - §Commands and history: `AssignMaterialToUnweighed`.
  - §MJCF import and §URDF import: `NoInertial` no longer implies an
    export refusal for a static body.
- `docs/ARCHITECTURE.md`:
  - the export dialog's notes;
  - the SDK table rows for `export` (warnings) and the new method (the
    rows near line 1395 and 1455);
  - wherever the one-click action lives (Materials window or export
    dialog).
- `docs/ROADMAP.md` v0.6: the second and third lines marked *Landed*, with
  the measured numbers (the stale "31" becomes 53 → 8).
- `docs/adr/0026-…`: not edited (append-only). ADR-0032 carries the new
  measurement.
- `README.md`: only if it describes the export refusing an unweighed
  link. Check at retirement.
- `python/riggen/_riggen.pyi` and the `Robot.export` docstring: already in
  step 5, verify.
- `AGENTS.md` current state: the v0.6 lines that landed.

## Open questions

- ⚠ OPEN 1 (human, by step 8): **where the one-click assign lives.**
  - Preferred: in the export dialog, right under the blockers — "Assign
    [material ▾] to the N unweighed links" — because that is where the
    user meets the refusal.
  - Alternative: a row action in the Materials window ("assign to
    unweighed links"), which is where materials already live but
    invisible from the refusal.
- ⚠ OPEN 2 (agent, by step 1): **does `TriangleInequality` on a static
  link pass through, or only `NotPositiveDefinite`?** Decided by MuJoCo's
  verdict on a synthetic static body. If MuJoCo refuses it, it keeps
  blocking, since the gate guards what the simulator reads.
