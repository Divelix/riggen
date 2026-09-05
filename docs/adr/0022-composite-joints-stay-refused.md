# ADR-0022: A `<body>` with several `<joint>`s stays refused; if that is ever reopened, the writer collapses by inspection

- Status: Accepted
- Date: 2026-09-06

## Context

ADR-0015 §5 refused several `<joint>`s in one `<body>` —
`ImportError::CompositeJoint` — and gave one reason: synthesising a
massless intermediate link per extra DoF would put a link in the tree that
the user never drew, against ADR-0006's "a drop is a link". It called the
synthesis a backlog line.

v0.4 is the cycle about a foreign MJCF surviving import → edit → export
with nothing silently lost, and a refusal is the harshest loss there is:
for a user whose model has one such body, riggen does nothing at all. A
promise like that has to ask the old question again, and the old answer
rested on taste. So it was asked again with the corpus and MuJoCo in
front of it rather than the principle alone.

Two facts decided it, and neither was available when ADR-0015 was written.

**How often it happens** (`~/Documents/code/sim/mujoco_menagerie`,
measured 2026-09-05):

| | count |
|---|---|
| `<body>` elements | 2201 |
| bodies with more than one `<joint>` | 120 |
| model directories containing one | **7 of 68** |
| of those, composite on the **root** body | 2 |
| `type="ball"` joints in all of Menagerie | **2** |

The seven are `flybody` (25 composite bodies), `ms_human_700` (43),
`iit_softfoot` (45), `hello_robot_stretch` and `hello_robot_stretch_3` (2
each — the gripper's two rubber tips), `stanford_tidybot` and
`umi_gripper`. The last two put the composite on the body directly under
`<worldbody>`, so `ImportError::JointOnRoot` refuses them whatever this
ADR decides. Synthesis would therefore open **five** directories, three of
which are a fly, a musculoskeletal human and a soft foot; the one
mainstream robot it unblocks is Hello Robot's Stretch, over two rubber
tips. All 120 composite bodies are hinges and slides — a `ball` never
appears in one, and appears twice in the whole corpus (`agility_cassie`).

**The naive synthesis does not work at all.** An intermediate link has no
geoms and no mass: the original body's `<inertial>` and geoms belong to
the innermost link, below the last joint. MuJoCo rejects the re-export —
measured, not assumed:

```
ValueError: Error: mass and inertia of moving bodies must be larger than mjMINVAL
```

Our own `resolve` already says the same thing:
`ExportError::ZeroMassMovableLink`, "link \"x\" moves but has no mass"
(`crates/riggen-export/src/resolve.rs`). A plain synthesis produces a
document riggen imports and then cannot export — a file that can get in
and not out, which is a worse failure than one that never gets in.

## Decision

### 1. The refusal stands

Several `<joint>`s in one `<body>` remains `ImportError::CompositeJoint`.
ADR-0015 §5's line — *a warning is something the file holds that the
document has no field for; a refusal is a file whose shape the document
cannot represent, where importing anyway would silently change the robot*
— is unchanged, and this is still the wrong side of it. What changes is
the reason it is on that side: not that a synthesised link is untidy, but
that the version which is cheap does not round-trip, the versions that
round-trip are expensive, and the corpus values the whole thing at five
directories.

The loss is loud. A refusal names the body and its joints; nothing is
half-imported and nothing is silently changed. v0.4's promise is that
nothing is *silently* lost, and a named refusal keeps it.

### 2. The refusal says what to do about it

The message gains what the diagnosis alone never had: that MuJoCo spells a
multi-DoF joint as several `<joint>`s in one `<body>`, and that splitting
the body into nested bodies with one joint each is the same model in
MuJoCo's own terms and imports. A user who hits this can act on it in a
text editor instead of concluding riggen cannot read their file.

### 3. If it is reopened, start at A2: no mark, the writer collapses by
inspection

Recorded so the next attempt does not begin where the obvious one does:

- **Import** synthesises *ordinary* links — no mark, no schema field, no
  `.riggen` bump.
- **The MJCF writer** folds any link that is massless, geom-less,
  frame-less and has exactly one child into its child's `<body>`, giving
  back the original `<body>` with N `<joint>`s. The collapse is MJCF's
  business, so it lives in the writer and `ResolvedRobot` stays
  convention-neutral (ADR-0004 §1).
- **`resolve`** stops raising `ZeroMassMovableLink` for exactly that
  shape when the format is MJCF.

A synthesised link is then an ordinary link: give it a mesh or a mass and
it simply stops being collapsible and becomes a real body, which is the
right answer and needs no invariant.

**Not the marked variant** (a "this was one MuJoCo body" flag on the link
or the joint), even though it round-trips structurally identically. A mark
is a second, MJCF-shaped description of the tree beside the document's
own, and every command that can add a geom, a frame, a mass or a child to
a marked link, reparent it, or delete one of the chain would have to
decide what the mark means afterwards — the cost ADR-0015 §3 refused for
`<default>` classes, bought again in a smaller costume.

Two warts belong to A2 and are written down here so they are priced in
rather than rediscovered:

- **MJCF → URDF stays blocked for these files.** URDF has no collapse; a
  chain of massless links is URDF's own spelling, and ADR-0012 already
  writes one for a frame — but on a `Fixed` joint, where nothing needs
  mass. On a movable joint, KDL-based consumers drop or complain about a
  massless link. SEED §4's fourth differentiator is exactly that
  direction.
- **A user's own massless pass-through link** would silently start
  exporting instead of erroring, because the writer cannot tell it from a
  synthesised one — which is the point of having no mark, and the price of
  it.

### 4. What would change this answer

Any of: a user (or the human) actually blocked on a model that matters —
Stretch is the plausible one; a second corpus (Robot Descriptions, Isaac
assets) where composite bodies are common rather than 5%; or MuJoCo
relaxing the massless-moving-body rule, which would make the plain
synthesis nearly free. Absent one of those, this is decided and not to be
re-argued from principle a third time.

## Consequences

- Five Menagerie directories stay unopenable, one of them a real robot,
  and the user is told why and what to do instead.
- v0.4's ⚠ OPEN is closed with a *no*; the file half is the four plans it
  already had, not five.
- `Robot`, `Joint`, `q`, `fk`, the scrubbers and the glyph band are
  untouched; `.riggen` stays schema 3.
- ADR-0015 §5's first bullet is re-affirmed, not superseded — that ADR is
  append-only and its rule is intact; only the reasoning under this one
  bullet is now the measurement above.
- The reopen conditions are a backlog line, not a rejected idea: the
  question was answered, and the answer was no.

## Alternatives considered

- **Synthesise plain links, no collapse** — what the roadmap line
  literally proposed. MuJoCo refuses the re-export under `mjMINVAL` and so
  does `ZeroMassMovableLink`; it survives only by inventing a tiny mass
  per synthesised link, which is ADR-0015 §7's fabricated
  `inertiafromgeom` density in a new costume, and it changes the model's
  dynamics rather than just its file.
- **Synthesise and mark the chain, collapse the marked links on export.**
  Structurally identical round trip, at the price of a permanent invariant
  every editing command has to maintain. §3 above.
- **`JointKind::Ball` and `Planar` in the document** — one edge, three
  DoF, the honest data-model answer. It touches `q` (no longer a scalar
  per joint), `fk`, `History`, View's scrubbers and the glyph band
  (ADR-0021), mimics, actuators, `validate`, both writers (URDF has no
  ball), SDF and the schema: ten-plus steps, most of them in the window
  half v0.4 just closed — and the corpus prices it at two joints in one
  model.
