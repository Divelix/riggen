# ADR-0032: The export gate guards what a simulator reads — a static link needs no mass, a moving one does, and a static zero tensor is written as `diaginertia="0 0 0"`

- Status: Accepted
- Date: 2026-09-13
- Amends docs/DATA-MODEL.md §`ResolvedRobot`'s rule ("an empty static body
  is fine") and the round-trip sentence of ADR-0015 §7 ("a body without one
  was an empty static body"). ADR-0015 §7's decision — no density is
  invented at import — **stands**.

## Context

An imported body with geoms and no usable `<inertial>` becomes
`InertialSpec::Computed { density_override: None }` with no material
(ADR-0015 §7; `urdf_in` does the same). Until this ADR `resolve` refused
it: `compose_inertial` answered `NoDensity`, and the only arm that forgave
it was the one for a link with no geometry. The file opened, looked fine,
and could not be written.

What that cost was measured, not guessed (`docs/ideas/unweighed-links.md`,
absorbed by `plans/unweighed-links`): the release binary over all 261
MuJoCo Menagerie `.xml` through `--export mjcf`, at `ca95c4d`. **119
export; 53 import and are refused by the export gate** (the roadmap's "31"
was stale — ADR-0026's after-table already said 53); 89 do not import. Of
the 53:

| Refused because | Files |
|---|---|
| `NoDensity`, every refused link **static** (no joint of its own) | 39 |
| `NoDensity` on `skydio_x2`'s root — static unless `floating_base` | 2 |
| `NoDensity` on at least one **moving** link (`google_robot` ×2, `wonik_allegro` ×2, `sharpa_wave` ×4) | 8 |
| `NotPositiveDefinite` on `ufactory_lite6`'s static `link_base`: `mass="1.65394" diaginertia="0 0 0"` | 4 |

The static links are welded arm bases, a camera mount, a finger pad, a
neck bracket. No simulator puts a degree of freedom on them; MuJoCo folds a
welded body's mass into its parent. Most of the gap was riggen demanding a
number nothing reads. SEED.md's differentiator 3 — "inertia sanity checks
that block export with an explanation" — is kept only if the gate stays
on the side of what a simulator actually uses.

### What MuJoCo actually does

Probed on MuJoCo 3.13.0 with scratch models under a warning hook that
raises, before anything was decided:

| Body | `<inertial>` | MuJoCo 3.13 |
|---|---|---|
| no joint, a visual geom (`contype="0" group="2"`) | absent | loads; mass from the geom at 1000 kg/m³ (`inertiafromgeom="auto"`) |
| no joint, no geom | absent | loads; mass 0 |
| no joint (root, child of a hinge, or parent of a hinge) | `mass="1.65394"` or `"0"`, `diaginertia="0 0 0"` | **loads**, body inertia stored as zero |
| the same three places | any mass, `fullinertia="0 0 0 0 0 0"` | refused: `inertia must have positive eigenvalues` |
| no joint, or a joint | any mass, `fullinertia="1 1 3 0 0 0"` or `diaginertia="1 1 3"` | refused: `inertia must satisfy A + B >= C` |
| a joint | mass 0 | refused: `mass and inertia of moving bodies must be larger than mjMINVAL` (ADR-0022) |

Two findings contradict the obvious guess. A static body is **not**
exempt from MuJoCo's tensor checks — a triangle-inequality violation is
refused on a body with no joint. And the zero tensor is refused or
accepted by *spelling*: `diaginertia` skips the eigenvalue check that
`fullinertia` runs, and riggen's writer spells every tensor `fullinertia`
(ADR-0008).

## Decision

### 1. A static link with nothing to weigh it by gets no `<inertial>`

The rule becomes: **a moving link must have mass; a static link with no
mass to compose — empty, or with geometry and nothing to weigh it by —
gets no `<inertial>`.** "Moving" is what it was: the parent joint is
movable, or the link is the root and `floating_base` is set.

The pass covers `InertialError::NoDensity` under `Computed` only. A
static `Hybrid` without a density still blocks: the user typed a mass for
it, and the tensor that mass scales cannot be made. A typed mass of
exactly zero is "nothing to write" on a static link; a negative mass fails
`check` as before.

For MJCF this changes nothing a simulator sees: a body the source wrote
without `<inertial>` is re-exported without one, and MuJoCo weighs it from
its geoms exactly as it weighed the original. URDF and SDF readers do not
re-derive mass from geometry, so there the link is massless, which on a
fixed joint is merged or ignored — and which is why §2 exists.

### 2. The export says which links it wrote without mass

`ResolvedRobot::massless` lists the static links with geometry written
without `<inertial>`, in link order — unweighed `Computed` and a typed zero
mass alike. The writers never read it. The CLI prints a `warning:` line per
link, the SDK warns `RiggenWarning`, and the export dialog lists them as
notes under its ready line. A link left unweighed by mistake is therefore
not left silently, and the fix is one gesture away (§5).

### 3. Everything a static link *does* carry is checked as a moving link's is

`inertial::check` blocks on a static link as on a moving one —
`NonFinite`, `NonPositiveMass`, `NotSymmetric`, `NotPositiveDefinite`,
`TriangleInequality` — because MuJoCo refuses those on any body (the
table above). The one exception is §4.

### 4. A static link's exactly-zero tensor is written, as `diaginertia="0 0 0"`

`ufactory_lite6`'s static root carries a positive mass and a zero tensor,
which MuJoCo loads in the spelling the file used. The import reads it into
`Override { mass, com, inertia: ZERO }` — a faithful copy — and refusing to
write it back would be refusing a file MuJoCo accepts.

- On a **static** link whose tensor is bitwise zero (every entry `0.0`),
  with a finite positive mass and a finite CoM, `resolve` does not raise
  `NotPositiveDefinite`. Any other non-positive-definite tensor — a
  near-zero one, a singular non-zero one, a negative moment — still
  blocks.
- The MJCF writer spells exactly that tensor `diaginertia="0 0 0"`; every
  other tensor stays `fullinertia` (ADR-0008 unchanged for them). The
  writer does not ask whether the link moves: the gate has already refused
  a zero tensor on a moving one, since MuJoCo refuses a moving body's
  inertia below `mjMINVAL`.
- URDF and SDF write the zeros as they are; neither format's parser
  refuses a zero `<inertia>`.

Decided by the agent on the human's delegation (plans/unweighed-links
OPEN 3), at the idea's preferred answer.

### 5. A moving link without mass is refused by name, with the fix

A moving link with no density becomes `ExportError::ZeroMassMovableLink`,
whose message depends on what the link has: with geometry, "moves and has
no mass — give it a material or a density"; without, "add a mesh, a
material, or an override", as before. The one-click fix is
`Command::AssignMaterialToUnweighed(material)`, one history entry over
every link with visuals, no material and an inertial that needs a density
it has not got, offered in the export dialog under the blockers and as
`robot.assign_material_to_unweighed` in the SDK. It is the roadmap's
"default material for an imported link": the number is the user's choice,
not riggen's.

## Consequences

- Measured at step 1 (HEAD `d18a679` → the commit that landed §1 and §3):
  exported **119 → 160**, refused by the export gate **53 → 12**, every
  import bucket unchanged, no file's outcome worse. All 41 newly exporting
  files load in MuJoCo with zero warnings. §4 is expected to take the
  gate's refusals to **8** and the exports to **164**; the plan's
  acceptance scan records the number.
- The 8 left refused each name a moving link with no mass — the honest
  remainder, and a message about the user's own file.
- `ResolvedRobot` gains `massless`; `ExportError::ZeroMassMovableLink`
  covers a moving link with geometry too; `Command` gains
  `AssignMaterialToUnweighed`. No schema change.
- A riggen-authored robot gets the same pass: a bracket welded to a moving
  link and left without a material exports, and the export says so.
- The URDF and SDF of such a robot are missing that bracket's mass, where
  MuJoCo's copy is not. The notice of §2 is the whole mitigation.
- The `mujoco` CI corpus carries a static body with a mesh and no
  `<inertial>` (`menagerie_style_arm.xml`'s `tool`), so the claim of §1 is
  checked on every push.

## Alternatives considered

- **Keep the gate; offer only the one-click material (the idea's C).**
  Moves no file in a headless scan, and keeps refusing a welded base for a
  number no simulator reads.
- **MJCF's 1000 kg/m³ at import (the idea's B).** Buys the 8 moving-link
  files at the cost of reversing ADR-0015 §7, for a density riggen would
  apply to visual meshes only where MuJoCo weighs every geom — near
  MuJoCo's answer, not it. The URDF import would diverge.
- **Relax every `check` on a static link (the idea's A+ as first worded).**
  MuJoCo refuses a triangle-inequality tensor and a zero `fullinertia` on
  a body with no joint; writing them would trade a riggen refusal for a
  MuJoCo one. §4 keeps the one relaxation MuJoCo agrees with.
- **Refuse lite6's zero tensor ("fix your source").** A positive mass with
  no rotational inertia is odd, but it is the file's, MuJoCo loads it, and
  the import already copied it faithfully; refusing it would be the gate
  guarding a number instead of a simulator.
- **Write every tensor `diaginertia` plus `quat`.** Would need the
  principal axes, which `principal_moments` deliberately does not return,
  and would churn every golden for the one case `fullinertia` cannot say.
