# ADR-0026: Composition is resolved at import, never stored — `<include>` and `<frame>` are flattened by a pre-pass, `<replicate>` and `<attach>` stay refused by name

- Status: Accepted
- Date: 2026-09-09
- Amends ADR-0015 §5: the fourth bullet's "a resolver we are not writing"
  is written, for two of its four elements

## Context

ADR-0015 §5 refused `<include>`, `<replicate>`, `<attach>` and `<frame>`
with one sentence — "a file that composes other files … is a resolver we
are not writing (the plan's non-goal)" — and put the four behind a single
`REFUSED` list checked over the whole tree before anything is read. The
v0.4 roadmap's last file-half line is that resolver.

What the refusal costs was measured, not guessed (`docs/ideas/composition.md`,
absorbed by `plans/composition`): the built binary over all 261 `.xml` in
MuJoCo Menagerie, twice, at two corpus commits, with the same buckets both
times. **113 files — 43% of the corpus, more than every other riggen
refusal combined — stop at `<include> is not supported`**, 96 of them the
`scene.xml` Menagerie's own README says to load. Resolving each include
against the scan: 56 of the 113 then import outright, 13 more through a
nested include, 22 import and fail on an unrelated export gate (§4 below),
7 are fragments that only exist to be included, and ~15 stay refused for
composite joints, ball joints, multiple roots or invalid identifiers.
`<frame>` refuses 1 model (`apptronik_apollo`), `<attach>` 1
(`iit_softfoot/scene.xml`), `<replicate>` none.

Composition is on neither side of ADR-0015 §5's line. A warning is a field
the document has not got; a refusal is a *shape* it cannot represent, where
importing anyway would silently change the robot. `<include>` and `<frame>`
are the same model spelled across files and wrappers: flattening them
changes nothing about the robot, and MuJoCo's own `mj_saveLastXML` performs
exactly that rewrite — the saved file has no `<include>` and no `<frame>`
in it. The precedent for how to treat a thing like that is ADR-0015 §3:
`<default>` is resolved at import, never stored, and the cost — a re-export
is flat — is stated rather than hidden.

### What MuJoCo actually does

Probed on MuJoCo 3.13.0 with scratch models and `mj_saveLastXML` before
anything was decided, because two of the rules below contradict the
obvious guess. The findings are pinned as the module comment of
`crates/riggen-export/src/mjcf_compose.rs` so the pass is written against
them, not against memory.

**`<include>`**

| Probe | MuJoCo 3.13 |
|---|---|
| `file` relative to which directory | the **main** model's directory first; the *including* file's directory if that path does not exist; a miss names the second path in the error |
| root tag of the included file | any — `<mujoco>`, `<mujocoinclude>`, even `<fragment>`; the root's own attributes are ignored (`<mujoco model="sub">` does not rename the model) |
| where an `<include>` may sit | anywhere — top level, `<worldbody>`, `<body>`, `<default>`, `<asset>`, `<frame>`; its children are spliced **at the site**, in place, in document order |
| a `<worldbody>` fragment included *inside* `<worldbody>` | schema error (`unrecognized element`) — splicing does not unwrap |
| the same file twice | `XML Error: File 'x.xml' already included`, a hard error; a cycle and a self-include hit the same error |
| what "the same file" means | the path **as written**, normalised (`a.xml` and `./a.xml` collide); not the file that was opened (`y.xml` found by fallback and `sub/y.xml` are two keys for one file, and both load) |
| several `<compiler>` blocks after splicing | one merge, per attribute, last in document order wins — and every block is read **before any body**, so an included `<compiler angle="radian"/>` placed *after* `<worldbody>` still governs it |
| several `<worldbody>` blocks | their bodies are all root bodies |
| several `<default>` blocks | one tree; a class name defined in two blocks is `repeated default class name` (error); two anonymous top-level `<default>`s merge per attribute |
| a `<mesh file>` declared in an included file | main directory + `meshdir` + `file` first; else the **including file's directory + `file`, with no `meshdir`** |

**`<frame>`**

| Probe | MuJoCo 3.13 |
|---|---|
| pose | `child = frame ∘ child` for `<body>`, `<geom>` (`fromto` included), `<site>`, `<camera>`, `<light>` (`pos` and `dir`), `<joint>` (`pos` and `axis`); nested frames compose outer-first |
| orientation spellings | the frame's `quat` / `euler` / `axisangle` / `xyaxes` / `zaxis` are read under the `<compiler>`'s `angle` and `eulerseq` exactly as a body's; a child's own orientation in any spelling composes rather than being replaced |
| a body under a frame | the body's pose composes; everything *inside* the body is untouched (its joint's `axis` is body-local and stays so) |
| `childclass` | the innermost enclosing `<frame childclass>` beats the enclosing body's `childclass` for every direct child, and a wrapped `<body>` with no `childclass` of its own takes the frame's as its own — its whole subtree inherits it; a body's own `childclass` wins |
| `class` on a frame | acts as `childclass`; the frame schema accepts unknown attributes (`bogus="1"` loads), unlike `<body>`, which refuses them |
| `<inertial>` under a frame | accepted and **not** transformed; `<freejoint>` under a frame: accepted, no pose to compose |
| `mj_saveLastXML` | writes the flattened model — no `<include>`, no `<frame>`, one `<compiler>`, one `<worldbody>` |

The idea's earlier finding stands: an included `<compiler angle="radian"/>`
changes how the *main* file's `euler` is read, so composition has to happen
before `Compiler::read`, not after.

## Decision

### 1. Composition is resolved at import, never stored

A pre-pass, `riggen-export::mjcf_compose::compose(root, path, source)`,
rewrites the parsed `Node` tree into a composition-free `<mujoco>` tree.
It runs first — before `refuse`, before `Compiler::read`, before
`Defaults::read` — and it is a pure tree → tree function: no document
types, no `Import` state, and the same `FileSource` the meshes come through
(ADR-0017's one seam, so a dropped set on the web works unchanged).

Nothing about composition reaches the document. There is no
`Robot::includes`, no record of which file a body came from, no "save back
into the files it came from". The document holds resolved numbers, as it
does for `<default>` since ADR-0015 §3, and for the same reason: a second,
MJCF-shaped description of the tree beside the document's own is a thing
every editing command would have to keep agreeing with it. **The cost is
stated: a re-export of a split model is one flat file**, and a model that
used `<frame>` comes back with the frame's pose baked into its children.
ADR-0015 §3 already made diffing a re-export against its original not a
useful operation; this ADR adds nothing new to that sentence.

### 2. `<include>` is spliced, with MuJoCo's rules and MuJoCo's errors

- The `file` is resolved against the **main** model's directory first and
  the including file's directory second, recursively, through `FileSource`.
- Any root tag is accepted; the included root's attributes are ignored.
- Children are spliced at the include site, in order, wherever the site is.
- The visited set is keyed on the normalised path as written. A second
  include of the same key — which is also how a cycle and a self-include
  present — is `ImportError::DuplicateInclude { file }`, a refusal. A file
  MuJoCo refuses should not open in riggen (the idea's decision 3).
- A file that neither directory holds is
  `ImportError::IncludeNotFound { file, from }` — not `Io`, because the
  message has to say what else to drop: on the web a `scene.xml` dropped
  without its `robot.xml` is the common case (ADR-0017), and `from` is the
  file that asked for it (the idea's decision 2).

After splicing, the reader sees what MuJoCo's parser sees: possibly
several `<compiler>`, `<worldbody>` and `<default>` blocks. `Compiler::read`
already merges every `<compiler>` per attribute in document order, and
reads them before any body — the same rule the probe found. `Defaults::read`
already reads every `<default>` as one tree. `run()` reads one `<worldbody>`
today and will read all of them, with `NoRoot` / `MultipleRoots` judged on
the total count of root `<body>`s.

### 3. `<frame>` is folded into what it wraps

Every child of a `<frame>` — `<body>`, `<geom>`, `<site>`, `<joint>`,
`<camera>`, `<light>`, a nested `<frame>` — gets the frame's pose composed
onto its own, in whichever of the five spellings either was written, under
the file's `<compiler angle eulerseq>`; a `<joint>`'s `axis` and a
`<geom>`'s `fromto` are rotated; the frame's `childclass` (and `class`,
which MuJoCo treats the same) is pushed onto every child that names no
class of its own, and onto a wrapped `<body>` that has no `childclass` as
its `childclass`; nested frames compose outer-first. An `<inertial>` under
a frame is left exactly as written, which is what MuJoCo does with it. The
frame element then disappears, and the reader never learns it was there.

### 4. `<replicate>` and `<attach>` stay refused, and say what they are

`REFUSED` shrinks from four elements to two. The two that stay are not
"the same model spelled differently":

- **`<attach>`** is a submodel attached under a name prefix — a full
  recursive rename over bodies, joints, geoms, sites, meshes, materials,
  actuators, tendons, sensors *and default class names*, plus re-rooting
  the submodel's anonymous `<default>` as a named class the attached
  subtree inherits. It buys one corpus file, and it is the only one of the
  four that is genuinely **two robots composed** — one `Robot` or two, and
  whose names win, is a document question that gets its own ADR when a
  user asks for a gripper on an arm. It is a small follow-up rather than a
  rewrite precisely because the `<frame>` scaffold exists: `<attach>` is
  most often a frame with a submodel in it.
- **`<replicate>`** is a subtree copy with `Tᵏ` frame composition, index
  suffixes, and the renaming of every model-level entry that names
  something inside the block. It buys zero corpus files today.

Both keep `ImportError::UnsupportedElement`, whose message now says what
the element means and why it is not read, on the ADR-0022 §2 principle
that a refusal should leave the user able to act.

`<compiler coordinate="global">` stays refused; it is unrelated to
composition.

### 5. Two dropped `.xml` in one gesture

After a dropped set is installed, an `.xml` that another dropped `.xml`
`<include>`s is a **fragment** and is not opened as a document; every other
`.xml` opens as it does today. Without this, dropping `scene.xml` and
`robot.xml` together opens both, and the second silently replaces the
first (the plan's decision 2).

### 6. Two places riggen knowingly differs from MuJoCo

Both measured above, both recorded so they are priced in rather than
rediscovered:

- **A mesh found only beside the included file.** MuJoCo falls back from
  main-directory-plus-`meshdir` to the including file's directory without
  `meshdir`. The corpus has one model that relies on it — `ms_human_700`,
  whose `assets/asset/*.xml` reference `../geometry/*.stl` — and all six
  of its entry files are refused for composite joints (ADR-0022) before a
  mesh is ever looked for. Whether the pass carries the fallback (it can:
  after splicing it knows the merged `meshdir`, the including directory
  and `FileSource::exists`, so it could rewrite a spliced `<mesh file>` to
  the path that exists) is an open question on the plan; until it does, a
  file that needs it gets `MeshNotFound` naming the main-directory path.
- **A class name defined in two `<default>` blocks.** MuJoCo refuses the
  file; `Defaults::absorb` merges the second into the first ("a class
  opened twice adds to itself"). That leniency predates this ADR, is not
  composition's to change, and opens a file MuJoCo would not — the
  opposite of the idea's decision 3 in spirit. Noted here; not changed
  here.

## Consequences

- 113 Menagerie files stop being refused for `<include>`; at least 69 of
  them import (56 + 13) and the `<frame>` model with them, with no bucket
  growing — the plan's acceptance measurement, re-run at retirement.
- A user's split model opens, every route a single file does: the app,
  the CLI, the SDK, the web drop. It edits as one document and exports as
  one file, and the docs say so.
- `REFUSED` is `["replicate", "attach"]`; `ImportError` gains
  `IncludeNotFound` and `DuplicateInclude`; `UnsupportedElement`'s doc
  comment loses two names. No core, schema, `ResolvedRobot` or writer
  change.
- The `mujoco` CI corpus becomes two files with a `<frame>` in one of
  them, so the round trip is exercised on a composed model, not a flat one.
- ADR-0015 §5's fourth bullet is amended, not superseded: its rule stands,
  two of its four names move to the other side of it with the measurement
  above as the reason, and the two that stay carry a reason of their own.
- The side finding the scan produced — 31 files import and then fail to
  *export* on `link "base": no material and no density override`, ADR-0015
  §7's stated consequence — is a `docs/BACKLOG.md` line, not this ADR.

## Alternatives considered

- **All four (option A).** `<attach>` is the largest piece of work in the
  set, buys one file, and raises the one-`Robot`-or-two question that
  should be answered when someone needs it answered; `<replicate>` buys
  none. +2 steps for 2 files.
- **`<include>` only (option C).** One step cheaper, loses the `<frame>`
  model and — more to the point — the scaffold that makes `<attach>` a
  follow-up. `<frame>` is pure local arithmetic (measured: pose
  composition, `childclass` push, axis and `fromto` rotation, nothing
  else) and is where MuJoCo 3 is heading.
- **Storing composition** — `Robot::includes`, a per-link source file, a
  writer that splits a re-export back into the files it came from. That
  is ADR-0015 §3's rejected alternative in a larger costume: a second
  description of the tree, maintained by every command, for a diffable
  re-export that ADR-0015 already declared not a useful operation.
- **Resolving includes lazily inside the reader** rather than as a
  pre-pass. It would have to happen before `Compiler::read` anyway (an
  included `<compiler>` governs the main file), and it would spread the
  visited set and the path rule through a reader that has five
  entry points already iterating `root.kids(…)`.
- **Deduplicating a double include silently.** MuJoCo makes it a hard
  error; a file MuJoCo refuses should not open in riggen.
- **`Io` for a missing include.** Loses `from` and the "what else to
  drop" sentence the web case needs.
