# ADR-0024: `<general>` is a fourth `ActuatorSpec`, and the ranges move onto the actuator

- Status: Accepted
- Date: 2026-09-07
- Amends: [ADR-0014](0014-actuators-on-the-joint-mjcf-only-three-presets.md)

## Context

ADR-0014 put `<general>`, its `prm` vectors, `<default class>` gains and
explicit `ctrllimited`/`forcelimited` on the "out" list, and said why:
"Hand-editing is the better answer **until MJCF import can read it back**."
ADR-0023 has since landed `Robot::actuators`, a model-level table an
actuator not on a joint could in principle live in — but the table changed
*where* an actuator is named, not *what kinds* MJCF import can read. Every
`<actuator>` that is not one of the three presets is still named in an
`ActuatorDropped` warning and thrown away.

Two losses follow directly from that, both real in the menagerie corpus
that motivates this cycle:

**A `<general>` driving a joint round-trips to nothing.** MuJoCo's own
actuator model — `dyntype`/`gaintype`/`biastype` and their `prm` vectors —
is what a `<position>`/`<velocity>`/`<motor>` desugars to internally; a
file that writes the general form directly (a custom activation dynamics,
a affine bias no preset can express) says something none of the three
presets can hold, and riggen currently answers with a warning and silence
in the re-export.

**`ctrllimited`/`forcelimited` and their ranges are re-derived, not read.**
`Actuator::ranges` do not exist in the document today: the MJCF writer
computes `ctrlrange` from the joint's limits and `forcerange` from
`Limits::effort` on every write (ADR-0014), regardless of what the
imported file said. A `<position ctrlrange="-1 1">` narrower than its
joint's range, or a `<position>` with **no** `ctrlrange` under
`autolimits="true"` (meaning: unlimited), both come back out as the
joint's own numbers — a silent change of meaning distinct from, and
smaller than, the `ActuatorDropped` loss above, but a loss all the same.

The condition ADR-0014 named for revisiting the three-presets-only clause
now holds. Four things needed deciding.

## Decision

**1. `General` is a fourth `ActuatorSpec` variant, not a verbatim attribute
blob.** A blob (an opaque bag of the element's own attribute strings) is
cheaper to write and carries every attribute MJCF ever adds, but nothing in
`validate` could check it, the SDK would hand a user strings to parse
instead of typed fields, and the document stops being the typed thing
every other import vocabulary lands in (ADR-0015 §4). Typed also lets the
writer trim what MuJoCo zero-fills — a `gainprm="200"` round-trips as
`gainprm="200"`, not `gainprm="200 0 0 0 0 0 0 0 0 0"` — which keeps a
document riggen alone has touched diff-friendly.

```rust
pub enum ActuatorSpec {
    Position { kp: f64, kv: f64 },
    Velocity { kv: f64 },
    Motor    { gear: f64 },
    General(General),
}

pub struct General {
    pub dyntype: DynType,   // None | Integrator | Filter | FilterExact | Muscle | User
    pub gaintype: GainType, // Fixed | Affine | Muscle | User
    pub biastype: BiasType, // None | Affine | Muscle | User
    pub dynprm: Vec<f64>,   // ≤ 10 each (mjNDYN / mjNGAIN / mjNBIAS)
    pub gainprm: Vec<f64>,
    pub biasprm: Vec<f64>,
    pub gear: f64,          // the first of six; Motor already reads one
}
```

**2. The presets stay sugar and are not desugared into `General`.** MuJoCo
defines `<position>`/`<velocity>`/`<motor>` as `<general>` shortcuts, so
folding all four `ActuatorSpec` variants into one is tempting; it is also
wrong. The combo, the glyph's preset marks, the SDK's `Position` /
`Velocity` / `Motor` classes and every golden fixture would lose the name
the user typed, to buy nothing the round trip needs. The read rule: an
element is a preset **iff every attribute it carries is one that preset
can express**; otherwise it is a `General`. That rule is mechanically
checkable on import, and it keeps a document riggen alone has touched
byte-identical to what it writes today.

**3. The ranges move onto the actuator, and `*limited` is a tri-state.**

```rust
/// `None` is "the writer derives it" — a riggen-authored actuator — and
/// `None` on a flag is MJCF's own `auto` under `autolimits="true"`.
pub struct ActuatorRanges {
    pub ctrl: Option<[f64; 2]>,
    pub force: Option<[f64; 2]>,
    pub ctrl_limited: Option<bool>,
    pub force_limited: Option<bool>,
}
```

`Limits::effort`/`Limits::velocity` keep coming back to the *joint*,
because URDF has nowhere else to read them; the actuator additionally
keeps what the element said, and the writer prefers it over the
joint-derived numbers whenever it is present. `Option<bool>` is exactly
MuJoCo's `auto | true | false`, and it is what lets an imported
`<position>` with no `ctrlrange` come back out unlimited instead of
clamped to the joint's range.

**4. What a `<general>` carries, and what it counts.** Carried — becomes
document state, round-trips: `dyntype`, `gaintype`, `biastype`, `dynprm`,
`gainprm`, `biasprm`, `gear`, and the four range fields of point 3.
Counted and warned once per attribute name, the way an unread element
already is (`docs/DATA-MODEL.md` §Nothing is dropped silently):
`actdim`, `actearly`, `actrange`, `lengthrange`, `cranklength`. `group`
stays decorative and silent, as it already is for the three presets. The
promise is bounded and stated, rather than "everything" and untrue.

**The boundary stays where ADR-0023 drew it: the target, not the tag.**
`ActuatorTarget` keeps its one variant, `Joint`. An escape-hatch actuator
that drives a joint is read; one that drives a tendon, site or body keeps
its `ActuatorDropped` warning regardless of whether its `<...>` tag is a
preset name or `<general>` — that boundary is unchanged by this ADR and is
ADR-0023's to move.

`<muscle>` and `<adhesion>` are not decided here. Both are attribute maps
that *could* fit `General` mechanically, but a `<muscle>` on a joint needs
`actuator_lengthrange`, which riggen does not compute and cannot write
back without risking a load failure, and `<adhesion>` drives a body, which
is already out by the target rule above. Whether either comes in travels
with the plan that executes this ADR, as an open question for the human.

## Consequences

- **`ActuatorSpec` stops being `Copy`.** Three `Vec`s in a variant end the
  by-value-copy convenience it has had since ADR-0014; it keeps `Clone` +
  `PartialEq`. Every call site that passed one by value (`properties.rs`,
  `glyphs.rs`, `fk_samples.rs`, `mjcf.rs`, `riggen-py`) needs the mechanical
  edit to a reference or a clone.
- **Schema 5.** `Actuator` gains `ranges: ActuatorRanges`, `#[serde(default)]`
  so a v4 actuator (which has neither field) deserialises with all four as
  `None` — exactly what a v4 document meant. `upgrade_v4_to_v5` is empty,
  for the same reason `upgrade_v1_to_v2` and `upgrade_v2_to_v3` are.
- `check_actuators` gains two refusals for a `General`: a non-finite `prm`
  entry or `gear`, and a `prm` vector longer than ten. The five existing
  refusals are unchanged; `InvalidActuatorGain`'s "kp/kv ≥ 0, gear ≠ 0" is a
  statement about the *presets* and does not extend to a `General`, whose
  gains are MuJoCo's to interpret, not riggen's to bound.
- The MJCF writer emits `<general>` with the three type names and the
  trimmed `prm` vectors for a `General`, and for **every** preset now
  prefers `ActuatorRanges` over the joint-derived numbers when present,
  emitting an explicit `ctrllimited`/`forcelimited` whenever the
  corresponding flag is `Some`.
- `fk_samples.rs`'s independent restatement of actuator ranges (its own
  doc comment: "two statements of one intent") gains the same rule in its
  own words: the actuator's own range, else the joint's.
- MJCF import's target-attribute list gains `jointinparent`, so a
  `<general jointinparent=...>` is dropped naming *that* attribute rather
  than a bare "unread element".
- URDF and SDF are untouched: neither writes nor imports an actuator
  (ADR-0014, ADR-0016 §5); a `General` changes neither writer's comment
  beyond the preset name it already prints.

## Alternatives considered

- **A verbatim attribute-string blob instead of a typed variant.** See
  point 1 — cheaper, but unvalidatable, untyped at the SDK boundary, and
  loses the zero-fill trimming that keeps riggen-authored documents
  diff-friendly.
- **Desugaring all four variants into `General`.** See point 2 — MuJoCo's
  own model, but it erases the name the user typed and everything built on
  it: the combo, the glyph marks, the SDK's three preset classes, every
  golden fixture.
- **A fixed `[f64; 10]` per `prm` vector**, MuJoCo's own array shape, which
  would keep `ActuatorSpec: Copy`. Rejected: 240 bytes in one variant next
  to `Position`'s 16 trips `clippy::large_enum_variant` under `-D
  warnings`, and it would write ten numbers to the file where the source
  wrote one, defeating the trimming point 1 exists for.
- **Leaving `ctrlrange`/`forcerange` derived, only adding the `*limited`
  flags.** Half the loss named in Context — a narrower or wider imported
  `ctrlrange` would still be silently replaced by the joint's own number —
  for less schema change. Rejected: it fixes the flag but not the range it
  gates, which is the more common of the two file shapes in the corpus.
- **Deciding `<muscle>`/`<adhesion>` here.** Both parse as attribute maps
  `General` could hold, so folding them in now was tempting. Rejected as
  premature: `<muscle>`'s `lengthrange` and `<adhesion>`'s body target are
  each a correctness question this ADR's four points do not touch, and
  answering them without a tendon document type or a body-actuator display
  surface would be deciding ahead of the parts they depend on.

## Addendum (2026-09-08, plans/actuator-escape-hatch retired)

The two questions this ADR left with the plan were decided by the human
on the plan's recommendation: **`<muscle>` and `<adhesion>` stay out**
(a muscle needs an `actuator_lengthrange` riggen does not compute; an
adhesion drives a body) and travel with the couplings bullet, where a
tendon exists and a `<muscle>` can be written back as one; and **the
properties panel shows a `General` as a read-only row** — its name, the
word `general`, its three type names — with the combo still able to
remove it or make it a preset. Editing ten `prm` numbers in a grid is
MJCF's actuator model in a properties panel, which ADR-0014 declined and
this ADR does not reopen.

One refinement to point 3 as executed: the import records a
`ctrllimited` / `forcelimited` flag only where the writer's own
`autolimits="true"` would not reproduce what the file meant — a written
flag as written, `auto` beside a range left `None`, and `Some(false)`
where the file named no range at all — so a document riggen alone has
touched and a foreign `<position ctrlrange>` both still write today's
one-attribute line. MuJoCo's own `auto` rule, probed rather than
assumed, is *lower < upper*.
