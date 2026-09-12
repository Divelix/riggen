# Idea: unweighed-links

- Status: Accepted (2026-09-13, every question at its preferred answer) → plans/unweighed-links
- Raised: 2026-09-13
- Prompt (verbatim from the human): "next thing in roadmap"

Covers v0.6's second and third lines together — "every file that imports,
exports" and "a default material for an imported link" — because they are
one question: what riggen does with a link that has geometry and no mass.

## Problem

An imported body with geoms and no usable `<inertial>` becomes
`InertialSpec::Computed { density_override: None }` with no material
(ADR-0015 §7, and `urdf_in` does the same). `resolve` then refuses it:
`compose_inertial` returns `NoDensity`, and the only arm that forgives it is
the one for a link with **no visuals** (`resolve.rs`, the
`Err(InertialError::NoDensity) if link.visuals.is_empty()` arm). The file
opens, looks fine, and cannot be written.

Measured today (HEAD `ca95c4d`, release build, all 261 Menagerie `.xml`
through `--export mjcf`): **119** export, **53** import and are refused by
the export gate, 89 do not import. The roadmap's "31" is stale; ADR-0026's
after-table already said 53. Of the 53:

| Bucket | Files | Links refused |
|---|---|---|
| `NoDensity`, every refused link **static** (no joint of its own) | 39 | 68 of 120 |
| `NoDensity` on `skydio_x2`'s root — static under the CLI's default, moving under `floating_base` | 2 | |
| `NoDensity` on at least one **moving** link (`google_robot` ×2, `wonik_allegro` ×2, `sharpa_wave` ×4) | 8 | 52 |
| `NotPositiveDefinite` on `ufactory_lite6`'s static `link_base`: `mass="1.65" diaginertia="0 0 0"`, welded to the world | 4 | |

The static links are the welded base of an arm (`fr3_link0`, `base_link`,
`Base`), a camera mount, a finger pad, a neck bracket. MuJoCo puts no
degree of freedom on any of them, and it merges a welded body's mass into
its parent. **Most of the gap is riggen demanding a number that no
simulator reads.**

## Constraints it runs into

- **ADR-0015 §7.** It deliberately does not compute an inertial at import:
  "a fabricated density in an `Override` is worse than a warning and a
  `Computed`". Option B conflicts with it head-on.
- **SEED.md §4, differentiator 3.** "Inertia sanity checks that block
  export with an explanation." Loosening the gate has to stay on the side
  of what a simulator actually uses.
- **DATA-MODEL §ResolvedRobot.** The rule is already "a link whose parent
  joint is movable must have mass; an empty static body is fine and gets
  no `<inertial>`". Option A widens *empty* to *unweighed*.
- **MJCF semantics.** A body without `<inertial>` under the default
  `inertiafromgeom="auto"` gets its mass from its geoms at 1000 kg/m³, and
  that count includes visual geoms unless they say `density="0"`. Our
  writer's `class="visual"` does not say it (`mjcf.rs`). A body that stays
  without `<inertial>` on export therefore means to MuJoCo exactly what it
  meant in the source file.
- **URDF / SDF.** A `<link>` with no `<inertial>` is legal. A massless link
  on a fixed joint is merged or ignored, as in MuJoCo.
- **The one-gesture rule (AGENTS.md).** A one-click "assign to every link"
  is one command, so one history entry.

## Options

### A — A static link needs no mass: an unweighed one exports without `<inertial>`

`resolve` treats `NoDensity` on a non-moving link the way it already treats
a link with no geoms: `inertial: None`, no error. It could go one step
further (A+): `check` failures on a static link, such as lite6's zero
tensor, become non-blocking.

- Buys **41** files, **45** with A+. The MJCF re-export means to MuJoCo
  what the original meant. Nothing is invented, and ADR-0015 §7 stands.
- Also covers riggen-authored robots: a bracket welded to a moving link,
  left without a material, stops blocking the export.
- The cost: that bracket's mass silently goes missing from the **URDF and
  SDF**, where nothing re-derives it from geometry (MuJoCo still does). An
  export-dialog notice ("3 static links carry no mass") keeps that honest.
  Whether one exists or needs building is for the plan.
- Leaves 8 files refused, each by a moving link. Those refusals should say
  so: "link "ff_link_1" moves and has no mass — give it a material or a
  density".
- About 2 steps, plus a snapshot if a notice is added. Worth a short ADR,
  because it redefines what the export gate guards.

### B — Default density at import: MJCF's own 1000 kg/m³

An unweighed MJCF body imports as `Computed { density_override:
Some(1000.0) }`, with a warning saying so.

- Buys up to all 49 `NoDensity` files, including the 8 A leaves. The case
  for it is that MJCF *defines* this density for an absent `<inertial>`, so
  the number is arguably the file's and not riggen's.
- Contradicts ADR-0015 §7, so it needs its own ADR. It is only
  *approximately* MuJoCo's answer: riggen weighs visual meshes only, while
  MuJoCo weighs every geom, collision primitives and geom
  `mass`/`density` attributes included (`MassFromGeomIgnored` already warns
  about the latter).
- Unmeasured risk: an open Menagerie mesh swaps `NoDensity` for `OpenMesh`,
  so the true yield is below 49 until scanned.
- The density means nothing for URDF import, so the two imports would
  diverge.
- 1–2 steps.

### C — The user's material, in one gesture

A Materials-window or tree action, "assign ⟨material⟩ to every link without
one", plus the same in the SDK (`robot.set_material_where_unset("PLA")` or
similar). The refusal message points at it.

- Keeps every number the user's choice, which is this line of the roadmap
  as written.
- Moves **no** file in a headless scan, so on its own it does not meet
  v0.6's accept ("every file that imports also exporting").
- 2 steps, with snapshots.

### Do nothing

- 53 of 172 importing files (31%) stay one-way.
- The message, `no material and no density override`, names neither the
  fix nor why a welded base should need a density.

## Recommendation

**A+ now, with a sharper refusal for the moving links; C as the roadmap's
"default material" line; B not taken.**

- A closes 45 of the 53 by fixing the gate rather than the data. A welded
  link's mass is a number the simulator folds away, and refusing to write
  a file over it is the gate being wrong, not the file.
- The 8 left refused are the honest remainder: a moving finger with no
  mass would get a density riggen made up. The accept explicitly allows
  "names any refusal left with a reason about the user's own file", and C
  is the one-click answer for that user.
- B loses because it buys 8 files at the cost of reversing an ADR, for a
  number that is only near MuJoCo's own.

What would change my mind:
- A B-scan showing the 8 import with closed meshes, *and* the human wanting
  the scan at 100% rather than "every refusal explained".
- The URDF mass-loss notice proving expensive enough that A is no longer
  small.

## Decision for the human

1. **Does an unweighed static link export without `<inertial>` (A)?** Yes,
   preferred. Or: no, keep the gate and do only C.
2. **Does A+ also relax `check` on static links, admitting lite6's zero
   tensor?** Yes, preferred: MuJoCo accepts it and the body is welded to
   the world. Or: no, 4 files stay refused.
3. **Do the 8 moving-link files stay refused with a better message, or get
   B's 1000 kg/m³ at MJCF import?** Refused and explained, preferred. B
   needs an ADR amending ADR-0015 §7.
4. **Is C, the one-click material for unweighed links, the roadmap's
   "default material" line?** Yes, preferred, in the same plan as A or the
   next one.
5. **ADR:** one short ADR for A — "the export gate guards what a simulator
   reads; a static link's mass is not" — amending the DATA-MODEL
   §ResolvedRobot rule. The roadmap's "31" becomes 53 when the plan lands.

**Decided (2026-09-13):** the preferred answer to all five.
- A+: an unweighed static link exports without `<inertial>`, and `check`
  does not block on a static link.
- The export dialog says which static links carry no mass.
- The 8 moving-link files stay refused, with a message that names the
  moving link and the fix.
- B is not taken, and ADR-0015 §7 stands.
- C, the one-click material for unweighed links in the GUI and the SDK, is
  the roadmap's "default material" line.
- One ADR, for A.
