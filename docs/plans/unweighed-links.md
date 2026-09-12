# Plan: unweighed-links

- Started: 2026-09-13
- Milestone: v0.6 — the import gap's last mile
- Idea: docs/ideas/unweighed-links.md (absorbed); every decision at its preferred answer
- Idea (verbatim from the human): "next thing in roadmap"

## Goal

A link with geometry and no mass stops blocking an export unless it moves.
An unweighed **static** link is written without `<inertial>`. A static
link's tensor-shape failures under `check` keep blocking: MuJoCo refuses
them on a static body too (step 1's verdict, ⚠ OPEN 2). The export says
which static links carry no mass: a `warning:` line in the CLI, a
`RiggenWarning` in the SDK, a note in the dialog. A **moving** link with
geometry and no mass is still refused, and the refusal names the link and
the fix. One gesture gives every unweighed link a material, in the GUI and
the SDK. Measured over Menagerie (HEAD `ca95c4d`), **41** of the 53 files
that import and refuse to export now export (step 1). Of the 12 that
remain, 8 name a moving link without mass, and 4 (`ufactory_lite6`) a
static root whose file says `diaginertia="0 0 0"` — OPEN 3 took
those four (step 3b, ADR-0032 §4), so the plan's number is 164.

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
  - On a static link, every `inertial::check` failure still blocks —
    `NotPositiveDefinite` and `TriangleInequality` included, since MuJoCo
    refuses both on a body with no joint (step 1, ⚠ OPEN 2). A typed mass
    of exactly zero is "nothing to write", not a failure; a negative mass
    is a failure.
- **`ResolvedRobot` gains `massless: Vec<String>`**: the static links with
  geometry written without `<inertial>`, in link order — unweighed
  `Computed` and a typed zero mass alike. The writers never read it. The
  CLI, the SDK and the dialog report it.
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
  - The one-click assign sits in the export dialog, under the blockers
    (OPEN 1, decided).
- **ADR-0032**, "the export gate guards what a simulator reads". It amends
  DATA-MODEL's rule, cites ADR-0015 §7 as standing, and records the scan.

## Steps

Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [x] **[3]** Step 1 — The static-link pass in `resolve` and
  `ResolvedRobot::massless`. Unit tests cover:
  - an unweighed static link (with geometry and without), and a typed
    zero mass;
  - a static `Hybrid` without a density, which still blocks;
  - a static zero tensor, a triangle-inequality tensor, a `NonFinite` and
    a negative mass, which all still block (the verdict below);
  - an unweighed moving link, which still blocks.

  MuJoCo's verdict, established in scratch (MuJoCo 3.13.0):
  - **Synthetic bodies.** A body with no joint carrying
    `fullinertia="1 1 3 0 0 0"` is refused (`inertia must satisfy
    A + B >= C`), and so is one carrying `fullinertia="0 0 0 0 0 0"`
    (`inertia must have positive eigenvalues`) — with any mass, under a
    static or a moving parent. The second refusal is the `fullinertia`
    spelling's: `diaginertia="0 0 0"` on a static body **loads**, with
    `mass="1"` or `mass="0"`, while `diaginertia="1 1 3"` is refused.
    So ⚠ OPEN 2 is answered "neither passes", and the zero tensor opens
    ⚠ OPEN 3. A static body with no `<inertial>` and a visual geom
    (`contype="0" group="2"`, riggen's spelling) loads and gets
    `inertiafromgeom` mass at 1000 kg/m³ — a static body with no geom
    gets zero.
  - **The Menagerie scan** (all 261 `.xml`, release build, before = HEAD
    `d18a679`, after = this step): exported 119 → **160**; refused by the
    export gate 53 → **12**; every import bucket unchanged (no root 43,
    multiple roots 21, composite joint 15, identifier 7, ball 2,
    `<attach>` 1). No file changed outcome for the worse. Of the 12: 8
    name a moving link with no density (`google_robot` ×2, `sharpa_wave`
    ×4, `wonik_allegro` ×2) and 4 are `ufactory_lite6`'s static
    `link_base` with `mass="1.65394" diaginertia="0 0 0"` (⚠ OPEN 3).
  - **The MuJoCo load** of all 160 exports under the `test_mjcf_load.py`
    warning hook: **158 load with zero warnings, every one of the 41
    newly exporting among them.** The 2 failures are
    `pndbotics_adam_lite` (both files), `mesh volume is too small:
    hipRollRight_1 … Try setting inertia to shell` — the original
    declares `<mesh inertia="shell">`, which riggen does not carry, and
    the file exported and failed identically before this step (its
    re-export is byte-for-byte unchanged). Not this plan's; a backlog
    line at retirement.
- [x] **[2]** Step 2 — The fixture: `menagerie_style_arm.xml`'s `tool`
  body, static and holding only a site today, gains a mesh geom and no
  `<inertial>`.
  - The corpus route of the `mujoco` job passes: zero warnings, `fk` to
    1e-6.
  - The `mjcf_in` corpus tests and goldens are updated for the new geom.
  - Run locally through `uv run --with mujoco` before committing.

  Landed with `pad` (the inline 1 cm tetrahedron) as `tool`'s visual. Run
  locally: 7 bodies, 40 poses to 1e-6, the four actuators, two
  equalities and the tendon the original's, zero warnings. The import
  pins one more `NoInertial { link: "tool" }`; the in-memory import test
  now resolves the corpus and finds `massless == ["tool"]`. Four
  menagerie-style snapshots moved: ids shift by one, and the `tcp`
  frame's marker is sized by its link's 1 cm geometry now.
- [x] **[1]** Step 3 — ADR-0032, with step 1's measurement. It also
  carries ⚠ OPEN 3's decision (§4) and the MuJoCo probe table it rests on.
- [x] **[2]** Step 3b — A static link's exactly-zero tensor (ADR-0032 §4,
  ⚠ OPEN 3 decided yes).
  Landed. The four `ufactory_lite6` files (`lite6`, both grippers,
  `scene`) export with `link_base` as `mass="1.65394"
  diaginertia="0 0 0"`, and all four load in MuJoCo 3.13 with zero
  warnings under the `test_mjcf_load.py` hook. The "near-zero" case is
  `diag(1e-30, 1, 1)`, and the singular non-zero one is `diag(1, 1, 0)`,
  which replaced step 1's `zero` case.
  - `resolve`: on a static link with a finite positive mass, a finite CoM
    and a bitwise-zero tensor, `NotPositiveDefinite` does not block. A
    moving link's zero tensor, a near-zero one and a singular non-zero one
    still do. Resolve tests pin all four; step 1's `zero` case moves to
    "passes".
  - The MJCF writer spells a zero tensor `diaginertia="0 0 0"`, every other
    one `fullinertia`. A writer test pins both; URDF and SDF unchanged.
  - Checked in scratch: one `ufactory_lite6` file exports and loads in
    MuJoCo with zero warnings.
- [x] **[2]** Step 4 — The moving-link refusal: `NoDensity` on a moving
  link becomes `ZeroMassMovableLink`, and the message depends on whether
  the link has geometry.
  - Resolve tests pin both messages.
  - `export_blocked` is refreshed if its text changes, and the image is
    shown to the human.

  Landed as `ZeroMassMovableLink { link, name, unweighed }`, where
  `unweighed` means a `Computed` link with geometry and no density. A
  moving `Hybrid` with geometry keeps `Inertial { NoDensity }`: its mass
  is typed, and the missing density shapes its tensor.
  `export_blocked`'s ghost has no geometry, so its text and image are
  unchanged. The SDK suite's pendulum-arm line moved to the new wording
  (78 passed).
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

- The scan (step 9) shows **164** of 172 importing files exporting (OPEN 3
  took the four `ufactory_lite6` files, step 3b). Every file still refused
  names a moving link with no mass. No file that exported before stops
  exporting.
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

- ~~⚠ OPEN 1~~ **Decided by the human, 2026-09-13: in the export dialog**
  (the preferred answer). The question as it stood: **where the one-click
  assign lives.**
  - Preferred: in the export dialog, right under the blockers — "Assign
    [material ▾] to the N unweighed links" — because that is where the
    user meets the refusal.
  - Alternative: a row action in the Materials window ("assign to
    unweighed links"), which is where materials already live but
    invisible from the refusal.
- ~~⚠ OPEN 2 (agent, by step 1)~~ **Decided, step 1: neither passes.**
  MuJoCo 3.13 refuses a triangle-inequality tensor on any body, and a
  `fullinertia` with a non-positive eigenvalue on any body — a static
  link's `check` failures keep blocking, and the static pass is
  `NoDensity` under `Computed` alone. The design delta is amended above.
- ~~⚠ OPEN 3~~ **Decided, step 3: yes, narrowly** — by the agent, on the
  human's delegation ("decide yourself on open questions"); ADR-0032 §4,
  step 3b. Re-probed on 3.13: `diaginertia="0 0 0"` loads on a static
  body as the root, as a hinge's child and as a hinge's parent, at mass
  1.65394 and 0; `fullinertia` zeros are refused in all six. ⚠ OPEN 1 was
  answered by the human: the export dialog. The question as it stood:
  **is a
  static link's exactly-zero tensor written, as `diaginertia="0 0 0"`?**
  MuJoCo loads it in that spelling and refuses it as `fullinertia`;
  `ufactory_lite6` ships its static root that way (4 of the 12 files the
  gate still refuses), so the `Override { mass, inertia: ZERO }` it
  imports to is a faithful copy of a file MuJoCo accepts.
  - Preferred: yes, narrowly — a static link whose tensor is exactly zero
    passes `check` at the gate and the MJCF writer spells it
    `diaginertia="0 0 0"`; URDF and SDF write the zeros as they are (no
    parser refuses them). Anything else non-positive-definite keeps
    blocking. One writer arm, one gate arm, the four files export, and
    the tensor MuJoCo reads is the one the file said.
  - Alternative: no — a zero tensor with a positive mass is a
    contradiction the user should resolve, and the four files stay a
    "fix your source" refusal. Cheaper, and the number stays 160.
  - Either way it is a new step between 3 and 4, or a line in the
    backlog.
