# Idea: composition

- Status: Open
- Raised: 2026-09-09
- Prompt (verbatim from the human): "what is next step? Close cycle?" —
  answered with the one unlanded v0.4 line, `docs/03-roadmap.md`'s
  **Composition** bullet, and "Idea first".

## Problem

`<include>`, `<attach>`, `<replicate>` and MuJoCo 3's `<frame>` are the last
`ImportError::UnsupportedElement` in the file half of v0.4 — one `REFUSED`
list at `crates/riggen-export/src/mjcf_in.rs:306`, checked by `refuse()`
over the whole tree before anything is read. A file that carries any of the
four does not open at all.

How much that costs was measured, not guessed: every one of MuJoCo
Menagerie's 261 `.xml` files run through the built `riggen --export mjcf`,
and the outcome bucketed. Twice, at two corpus commits — `8161bba`
(2026-09-04) without meshes, and the local clone at
`~/Documents/code/sim/mujoco_menagerie` (`da76818`, 2026-08-09) with all
2241 of them. Every bucket below is identical in both; the meshes only
decide whether the 59 stop at import or go all the way out again.

| Outcome | Files |
|---|---|
| **`<include>` is not supported** | **113** |
| imports **and re-exports**, meshes and all | 59 |
| no root link: every link is a child | 40 |
| exports refused: no material and no density override | 31 |
| composite joint (ADR-0022) | 8 |
| link name is not a valid identifier | 3 |
| inertia tensor not positive-definite | 3 |
| more than one root link | 2 |
| ball joint | 1 |
| **`<frame>` is not supported** | **1** |
| **`<attach>` is not supported** | **1** |
| `<replicate>` | **0** |

`<include>` is the largest single refusal in the corpus — 43% of it, more
than every other riggen refusal combined. The other three are three files.
96 of the 113 are a `scene*.xml`, which is the file Menagerie's own README
tells you to load; the other 17 are real robot models that split themselves
(`aloha`, `pal_tiago`, `pal_talos`, `pal_tiago_dual`, `ms_human_700`).

Resolving each `<include>` against the scan tells us what flattening would
buy: **56** of the 113 import outright, **13** more are nested includes that
a recursive pass reaches too, and **7** are fragments that only exist to be
included. **22** more import and then fail to *export* on an unrelated
density gate (see §Side finding). The remaining ~15 stay refused for reasons
this idea does not touch — composite joints, ball joints, multiple roots,
invalid identifiers.

A trap worth writing down: a substring grep for `<frame` returns 13 files,
all `<framequat>` / `<framezaxis>` **sensors**. `refuse()`'s exact
`c.tag == "frame"` is right; any frequency claim built on substring grep is
not.

## Constraints it runs into

- **ADR-0015 §5** drew the line: a warning is a field the document has not
  got, a refusal is a *shape* it cannot represent. Composition is neither —
  it is the same model spelled across files and wrappers, so flattening it
  changes nothing about the robot. §5 refused the four because "a resolver
  we are not writing (the plan's non-goal)". That non-goal is exactly what
  this idea reopens, so it needs an ADR.
- **ADR-0015 §3** is the precedent to copy: `<default>` is resolved at
  import and never stored, with the cost stated plainly — a re-export is
  flat and class-free. Composition should resolve the same way, and pay the
  same stated cost.
- **ADR-0017** (web, bytes in): an `<include>` needs a *second file* through
  `FileSource`, which on the web is `DroppedSet`. A `scene.xml` dropped
  without its `robot.xml` is the common case and has to fail loudly.
- **ADR-0022** keeps composite joints refused; 8 of the include files are
  refused for that anyway, and stay refused.
- `SEED.md`'s non-goals touch none of the four.

Four facts in the code make the pre-pass cheap, all verified:

- `Compiler::read`, `Defaults::read`, `read_assets`, `read_equalities`,
  `read_tendons` and `read_actuators` already iterate `root.kids(…)`, so
  spliced-in sibling blocks work with no change. Only `run()`'s
  `root.child("worldbody")` is singular and needs to merge.
- `count_dropped` walks the tree it is given, so it counts the *flattened*
  tree and stays honest for free.
- The pass must run **before** `Compiler::read` and `Defaults::read`: an
  included `<compiler angle="radian"/>` changes how the main file's `euler`
  is read (measured against MuJoCo 3.12).
- MuJoCo resolves an include against the **main** model's directory, with
  the including file's directory as a fallback; the included root may be
  `<mujoco>` or `<mujocoinclude>`; an `<include>` is legal anywhere,
  including inside a `<body>`; children are spliced at the include site; and
  including one file twice is a hard error, not a dedupe.

## Options

### A — Flatten all four

One `mjcf_compose` pass rewriting the parsed `Node` tree into a
composition-free `<mujoco>` tree; `REFUSED` shrinks to
`<compiler coordinate="global">`. Nothing blocks it — `mj_saveLastXML`
proves MuJoCo does exactly this rewrite itself.

`<replicate>` is a subtree copy with `Tᵏ` frame composition and an index
suffix, plus renaming every `<actuator>`, `<tendon>`, `<equality>` and
`<sensor>` entry that names something defined inside the block. `<attach>`
is a full recursive namespace prefixing over bodies, joints, geoms, sites,
meshes, materials, actuators, tendons, sensors **and default class names**,
plus re-rooting the submodel's anonymous `<default>` as a named
`<prefix>main` that the attached subtree inherits as `childclass` — get that
last part wrong and every unqualified attribute in the submodel silently
changes.

Cost: ~10 steps. Buys 2 files over option B. Forecloses nothing.

### B — Flatten `<include>` and `<frame>`; refuse `<replicate>` and `<attach>` by name

The same pass, two of the four. `<frame>` is pose composition onto each
child, the frame's `childclass` pushed onto the spliced children, a rotating
frame also rotating a wrapped `<joint>`'s `axis`, and nested frames
inheriting — all local arithmetic, no second file. `<include>` is splice,
path resolution, recursion and a visited set. `<replicate>` and `<attach>`
keep `UnsupportedElement` with a message that says what they mean.

Cost: ~5 steps.

1. `mjcf_compose.rs`: the rewrite scaffold and `<frame>` folding, unit-tested.
2. `<include>`: splice, main-dir-first resolution, both root tags, recursion,
   the duplicate-include error, a cycle guard.
3. `run()` merges sibling `<worldbody>` blocks; `REFUSED` shrinks; messages
   for the two that stay.
4. Corpus: `menagerie_style.xml` grows an included fragment and a `<frame>`;
   the `mujoco` CI job copies the second file beside it.
5. App / SDK / web: a sibling `.xml` through `DroppedSet`, the
   missing-include message, docs.

### C — `<include>` only

Steps 2–5 above, ~4 steps. Buys the same ~76 files: `<frame>` is worth 1
model and its scene.

### Do nothing

43% of Menagerie stays refused, including every `scene.xml`. The v0.4
section's premise — "the import warns and drops, so a round trip through
riggen still costs the user their hand-edited XML" — is contradicted by the
largest single thing the import refuses.

## Recommendation

**B.**

**A** loses on shape, not effort: `<attach>` is the biggest piece of work in
the set, buys one file, and is the only one of the four that is genuinely
*two robots composed* — which raises a document question (one `Robot` or
two? whose names win?) that deserves its own ADR when a user actually asks
for a gripper on an arm. `<replicate>` buys zero files today.

**C** loses by one step. `<frame>` shares the whole pre-pass scaffold, is
pure local arithmetic, is where MuJoCo 3 is heading, and is the form
`<attach>` most often takes — doing it now is what makes `<attach>` a small
follow-up rather than a rewrite.

**Do nothing** loses on the 43%.

What would change my mind: if `<frame>` folding needs more than pose
composition, `childclass` push and joint-axis rotation (measured: it does
not), or if a user asks for attachment, which is `<attach>`'s real use and
would move it out of "buys one file".

## Side finding (not this idea)

31 of the 261 import cleanly and then fail to **export**: `link "base": no
material and no density override`. That is ADR-0015 §7's stated consequence
— a body with no `<inertial>` becomes `InertialSpec::Computed` with no
density — and it is hit by every model with an inertial-less static base.
It gates re-export on more Menagerie models than everything except
`<include>`. Worth a `docs/BACKLOG.md` line of its own.

## Decision for the human

1. **Which option?** Preferred **B** — `<include>` and `<frame>` flattened,
   `<replicate>` and `<attach>` refused by name with a message that says
   what they mean. Alternatives: A (all four), C (`<include>` only).
2. **A missing included file** — a new `ImportError::IncludeNotFound` naming
   the file (preferred: on the web a `scene.xml` dropped without its
   `robot.xml` is the common case, and the message has to say what else to
   drop), or reuse `Io`?
3. **The same file included twice** — reproduce MuJoCo's hard error
   (preferred: a file MuJoCo refuses should not open in riggen), or dedupe
   silently?
4. **ADR?** Yes — it reopens ADR-0015 §5's named non-goal and shrinks
   `REFUSED`. Preferred: one ADR-0026, "composition is resolved at import,
   never stored", carrying the `<replicate>` / `<attach>` refusal and
   ADR-0015 §3's cost restated.
5. **The roadmap line** names all four. Preferred: amend it to the two that
   land, naming the two that stay refused, when the plan retires.
