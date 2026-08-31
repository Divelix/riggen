# ADR-0015: MJCF import reads the subset the document can hold, resolves `<default>` rather than storing it, and speaks the URDF import's vocabulary

- Status: Accepted
- Date: 2026-08-31

## Context

Two ADRs left a clause open for this one. ADR-0012 wrote a `Frame` as an
MJCF `<site>` and promised the reverse "once MJCF import exists". ADR-0014
kept `<general>`, `<adhesion>`, `<muscle>` and inherited gains out of the
document because "hand-editing is the better answer until MJCF import can
read one back". The last v0.2 roadmap line is "MJCF import; SDF export".

Reading MJCF is not the mirror of reading URDF. URDF is a robot
description and `urdf-rs` hands over a typed tree; MJCF is a MuJoCo
*scene* — tendons, sensors, cameras, lights, contacts, keyframes, solver
options, composites, hfields — with no Rust parser, and with three things
URDF does not have at all:

- a **`<default>` class tree** whose `childclass` inheritance decides what
  every unqualified attribute on every `<geom>` and `<joint>` means, so a
  file cannot be read element by element;
- **five spellings of one rotation** (`quat`, `euler`, `axisangle`,
  `xyaxes`, `zaxis`) under a `<compiler angle eulerseq>` that defaults to
  **degrees**, not radians;
- **several `<joint>`s in one `<body>`**, MuJoCo's way of spelling a ball
  or planar DoF, against a document whose joints *are* the edges of the
  link tree (ADR-0005).

Four things needed deciding rather than discovering: how much of MJCF we
read, what parses it, whether the import gets its own warning vocabulary,
and where the line between a warning and a refusal falls.

## Decision

### 1. The subset

Everything the document has a field for is read; everything else is named
in a warning and skipped; a handful of *shapes* are refused outright.

| MJCF | Becomes |
|---|---|
| `<body>` nested under `<worldbody>` | `Link`, the nesting is the tree; `pos`/`quat`/`euler`/… is the parent joint's `origin` |
| one `<joint type="hinge">` with `range` / without | `JointKind::Revolute` / `Continuous` |
| one `<joint type="slide">` | `JointKind::Prismatic` |
| a body with no `<joint>` | `JointKind::Fixed` |
| `range`, `damping`, `frictionloss`, `armature` | `Limits`, `Dynamics` |
| `<inertial pos mass fullinertia\|diaginertia quat>` | `InertialSpec::Override`, the tensor rotated into link axes |
| `<asset><mesh name file scale>` | `MeshAsset` under `meshdir` / `assetdir` |
| `<geom type="mesh">` | a visual `Geom`, or a `CollisionPolicy::Meshes` entry |
| `<geom type="box\|cylinder\|sphere\|capsule">` | `CollisionPolicy::Primitives`, `size` undone from half-extents |
| `<site>` | `Frame` on its body (ADR-0012, the promised symmetry) |
| `<equality><joint polycoef="o m 0 0 0">` | `Joint::mimic` (ADR-0013) |
| `<position kp kv>` / `<velocity kv>` / `<motor gear>` on a joint | `ActuatorSpec` (ADR-0014) |
| `<compiler angle eulerseq meshdir assetdir autolimits>` | applied, not stored |
| `<default>` / `childclass` | resolved, not stored (§3) |

Warned and skipped: `<tendon>`, `<sensor>`, `<camera>`, `<light>`,
`<contact>`, `<keyframe>`, `<option>`, `<flag>`, `<hfield>`, `<skin>`,
`<composite>`, `<numeric>`, `<custom>`, `<visual>`, `<statistic>`,
`<texture>`, `<material>`; `<general>`/`<muscle>`/`<adhesion>` actuators
and any actuator whose target is not a joint; `plane`, `ellipsoid`,
`sdf` and `hfield` geoms; `.msh` and inline `<mesh vertex face>` assets; a
`<freejoint>` on the root body; a `<geom mass|density>` that would be a
body's only mass. **Elements are warned by name and counted; attributes
are not** — `solref`, `solimp`, `friction`, `margin`, `group`, `rgba`,
`ref`, `springref`, `gravcomp` and their kin appear on nearly every
element, and one warning each would bury the ones that matter. The
exception is an attribute that changes the robot rather than decorating
it, and those are in the warned list above.

Not read and not warned about, because they are the *file* rather than the
model: XML comments, processing instructions, `<mujoco model>`'s own
attributes beyond `model`.

### 2. `quick-xml` at 0.36, and a read-only DOM in `xml.rs`

`xml.rs` grows a reading half beside its writing half: `parse(&str) ->
Result<Node, ParseError>` over `quick-xml`, where a `Node` is a tag name,
a `BTreeMap` of attributes and its children — sixty lines, no serde, no
schema. `quick-xml` 0.36 is **already compiled** in `riggen-export`'s
dependency tree under `urdf-rs`, so the workspace gains a
`[dependencies]` line and no crate.

Not a hand-rolled parser: entity decoding, CDATA, encodings and
self-closing tags are exactly the boring correctness the writer's
`escape` deliberately does not have to invert. Not a serde derive: MJCF's
shape is decided by `<default>` at read time, and a typed tree would have
to be re-walked anyway. Not a `mjcf-rs`: there is none.

The five orientation spellings collapse to one `DQuat` in the same file,
the mirror of `xml::quat_wxyz` — one place, tested. `<compiler angle>`
defaults to **degrees**, `eulerseq` to `xyz`; both are read before any
body is.

### 3. `<default>` is resolved at import, never stored

A `Defaults` tree keyed by class, with `childclass` inheritance, answers
"what are this element's effective attributes" during the read, and is
dropped when the read ends. The document holds resolved numbers, exactly
as `resolve` hands the writers resolved numbers (ADR-0004 §1). Storing the
class tree would put a second, MJCF-shaped description of every joint and
geom beside the document's own, and every editing command would have to
keep the two agreeing.

The cost is stated plainly: **re-exporting an imported file loses its
class structure.** Our own `<default class="visual">` / `class="collision">`
is regenerated by the writer, so our own files round-trip; a foreign
file's twenty classes come back as explicit attributes on every element.

### 4. One import vocabulary with the URDF import

`ImportWarning` and `ImportError` grow MJCF variants; there is no second
pair of enums. The status bar, `RiggenWarning` in the SDK and the CLI's
stderr already speak these two types, and a `MeshNotFound` is the same
event whichever file it came out of. `MimicDropped`, `NonUniformScale`,
`PrimitiveVisualDropped`, `NoInertial`, `MeshNotFound`, `Io`, `Parse`,
`UnsupportedJoint`, `NoRoot`, `MultipleRoots` and `Invalid` are reused
unchanged. The new ones are `ElementDropped { element, count }`,
`GeomDropped { link, kind }`, `FreeJointDropped { body }`,
`ActuatorDropped { actuator, reason }`, `FrameDropped { site, reason }`,
`MassFromGeomIgnored { link }`, and the errors in §5.

### 5. Warning or refusal

**A warning is something the file holds that the document has no field
for; a refusal is a file whose *shape* the document cannot represent,
where importing anyway would silently change the robot.** By that rule:

- **Several `<joint>`s in one `<body>` → `ImportError::CompositeJoint`.**
  Synthesising a massless intermediate link per extra DoF would open more
  of Menagerie, at the price of links the user never drew — and ADR-0006's
  "a drop is a link" makes the tree the user's. The synthesis is a backlog
  line, not this plan.
- **A `<joint>` on the root body → `ImportError::JointOnRoot`.** The
  document's root link has no parent joint; dropping the joint would weld
  the robot to the world.
- **`type="ball"`, and `type="free"` anywhere but the root body →
  `ImportError::UnsupportedJoint`**, beside URDF's `floating`/`planar`/
  `spherical`. A `<freejoint>` (or `type="free"`) on the **root** body is a
  *warning*: it is exactly what `ExportOptions::floating_base` writes, and
  what it costs is a boolean that was never a document field.
- **`<include>`, `<replicate>`, `<attach>`, and `<compiler
  coordinate="global">` → `ImportError::UnsupportedElement`.** A file that
  composes other files, or that means something different in every
  coordinate, is a resolver we are not writing (the plan's non-goal).
- **Several bodies directly under `<worldbody>` → `MultipleRoots`**, the
  URDF import's own verdict for the same situation. A body cycle needs no
  check at all: XML nesting cannot make one.
- Everything else — a refused coupling, an actuator we cannot express, a
  site whose name collides — is **dropped with a warning and the file
  still opens**, the rule the URDF import already follows.

### 6. The visual / collision split

For a file that does not use our own `class="visual"` / `class="collision"`:

1. our two class names, when present;
2. else `contype == 0 && conaffinity == 0` is a visual and everything else
   is a collision — MuJoCo's own idiom for a decorative geom;
3. else every geom is a visual and the link is
   `CollisionPolicy::SameAsVisual`.

Never a silent loss of geometry, and step 3 is the honest answer for a
file that simply never made the distinction: `SameAsVisual` is what the
app's own new links carry.

### 7. A body whose geoms are its only mass

`<inertial>` absent and MuJoCo's `inertiafromgeom` implied: the import
**warns `NoInertial` and leaves `InertialSpec::Computed`**, exactly as the
URDF import does for a `<link>` with no `<inertial>`. Computing one here
would need a density the imported document has not got — MuJoCo's own
fallback is 1000 kg/m³, a number the user never chose, frozen into an
`Override` they would then have to notice and undo. The round trip is
unaffected: our writer emits `<inertial>` for every link that has mass,
and a body without one was an empty static body.

### 8. What this closes

ADR-0012's "once MJCF import exists" and ADR-0014's "until MJCF import can
read one back" are answered here: a `<site>` becomes a `Frame`, and
`<position>` / `<velocity>` / `<motor>` become an `ActuatorSpec`. The two
ADRs are not edited — they are append-only — and `<general>` remains
unread, so ADR-0014's escape-hatch backlog line survives this ADR rather
than being closed by it.

## Consequences

- An MJCF opens by every route a URDF does, and our own export round-trips
  exactly: the `mujoco` CI job gains a fourth route that exports, imports,
  re-exports, and holds MuJoCo to the same poses, sites, `mjEQ_JOINT` rows
  and `model.nu`.
- A Menagerie model with a ball wrist or a composite free-flyer is
  *refused by name*, not half-imported. The error says which body and
  which joints, so the user can decide.
- Re-exporting an imported foreign file produces a flat, class-free MJCF
  that is semantically the same model. Diffing it against the original is
  not a useful operation, and the docs say so.
- `riggen-export` gains one dependency line and no compiled crate.
- The import vocabulary is now shared by two formats, so a third (SDF)
  extends it again rather than inventing a third pair of enums.

## Alternatives considered

- **Synthesising massless links for a composite joint.** More files open,
  and the link tree stops being what the user drew. Backlog.
- **A typed MJCF tree behind serde derives.** `<default>` makes the
  meaning of an attribute depend on a class the derive cannot see, so the
  typed tree would be re-walked into a second, resolved one — two
  representations for one read.
- **Storing the `<default>` classes in the document** so a re-export
  reproduces them. That is MJCF's model in `Robot`, which ADR-0004
  rejected for the writers and which every command would have to maintain.
- **A separate `MjcfWarning` / `MjcfError` pair.** Cleaner enums, and
  three places (status bar, SDK, CLI) that would each have to match on two
  types to say the same sentence.
- **Reading `<general>` into a new `ActuatorSpec` variant.** It is
  MJCF's full actuator model — `dyntype`, `gaintype`, `biastype` and their
  `prm` vectors — arriving through the import before any UI can show or
  edit it. ADR-0014 put it in the backlog and it stays there.
- **`inertiafromgeom` computed at import.** See §7: a fabricated density
  in an `Override` is worse than a warning and a `Computed`.
