# Plan: composition

- Started: 2026-09-09
- Milestone: v0.4, the file half — "Composition" (`docs/ROADMAP.md` §v0.4),
  the last unlanded line of the cycle
- Idea: `docs/ideas/composition.md` (absorbed) — option **B**, its
  measured corpus numbers, and its three subsidiary preferences

## Goal

A model spelled across several files, or wrapped in MuJoCo 3's `<frame>`,
opens. One pre-pass — `mjcf_compose` — rewrites the parsed XML tree into a
composition-free `<mujoco>` tree before anything reads it: `<include>` is
resolved through the same `FileSource` the meshes come through, spliced at
its site, recursively, against the **main** model's directory with the
including file's as a fallback, with `<mujocoinclude>` accepted as a root
tag, a file included twice a hard error the way MuJoCo makes it, and a
missing file its own `ImportError::IncludeNotFound` naming what else to
drop; `<frame>` is folded into the elements it wraps — pose composition
onto each child, its `childclass` pushed down, a rotating frame rotating a
wrapped `<joint axis>`, nested frames composing. Nothing about composition
reaches the document: it holds resolved numbers, exactly as `<default>`
does since ADR-0015 §3, and a re-export is one flat file. `REFUSED` shrinks
from four elements to two — `<replicate>` and `<attach>` keep
`UnsupportedElement`, now with a message that says what each means and why
it is not read. Measured payoff: **113** of Menagerie's 261 files stop
being refused for `<include>`, of which 56 import outright and 13 more
through recursion, plus the 1 `<frame>` model.

## Non-goals

- **`<attach>`** — a full recursive namespace prefixing over bodies,
  joints, geoms, sites, meshes, materials, actuators, tendons, sensors
  *and* default class names, plus re-rooting the submodel's anonymous
  `<default>` as a named `<prefix>main` inherited as `childclass`. It buys
  one file, and it is the only one of the four that is genuinely two
  robots composed — one `Robot` or two, whose names win — which is its own
  ADR when a user asks for a gripper on an arm.
- **`<replicate>`** — a subtree copy with `Tᵏ` composition, index suffixes
  and the renaming of every `<actuator>` / `<tendon>` / `<equality>` /
  `<sensor>` entry that names something inside the block. It buys **zero**
  files in the corpus today.
- `<compiler coordinate="global">`: stays refused, unrelated to
  composition.
- Storing composition. There is no `Robot::includes`, no "save back into
  the files it came from", and no attempt to make a re-export diffable
  against the original — ADR-0015 §3's stated cost, restated.
- The **side finding**: 31 corpus files import cleanly and then fail to
  *export* on `link "base": no material and no density override`
  (ADR-0015 §7). A `docs/BACKLOG.md` line, not this plan.
- Composite joints (ADR-0022), ball joints, multiple roots, invalid
  identifiers: the ~15 files that stay refused for reasons flattening does
  not touch.
- SDF import; any new writer; anything in the window half.

## Design deltas

- **ADR-0026** (step 1): *composition is resolved at import, never stored*
  — the ADR-0015 §3 precedent applied to files and frames, reopening
  §5's named non-goal ("a resolver we are not writing") for two of its
  four elements and carrying the reasons the other two stay refused.
  ADR-0015 marked "§5 amended by 0026" in `docs/adr/README.md`.
- `riggen-export::mjcf_compose` (new file, ~250 lines): `compose(root:
  Node, path: &Path, source: &dyn FileSource) -> Result<Node,
  ImportError>`, a pure tree → tree rewrite. Owns include splicing, the
  visited set, and `<frame>` folding. No document types, no `Import`
  state: it runs **before** `Compiler::read` and `Defaults::read`, because
  an included `<compiler angle="radian"/>` changes how the main file's
  `euler` is read (measured against MuJoCo 3.12).
- `riggen-export::mjcf_in`: `from_mjcf` calls `compose` before `refuse`;
  `REFUSED` becomes `["replicate", "attach"]` with per-element messages;
  `run()`'s singular `root.child("worldbody")` becomes a merge over every
  `<worldbody>`; `Compiler::read` already merges every `<compiler>` in
  document order (last wins per attribute, MuJoCo's rule — step 1 measured
  it, and that it is read before any body). `Defaults::read`, `read_assets`, `read_equalities`, `read_tendons`, `read_actuators` and
  `count_dropped` already iterate `root.kids(…)` / walk the tree they are
  given, so spliced siblings work unchanged and the drop counts stay
  honest for free.
- `riggen-export::import`: `ImportError::IncludeNotFound { file, from }`
  and `ImportError::DuplicateInclude { file }`;
  `UnsupportedElement`'s doc comment loses `<include>` and `<frame>`.
- `riggen-app::file_io`: `load_dropped` stops opening an `.xml` that
  another dropped `.xml` includes (OPEN 2) — otherwise a `scene.xml` +
  `robot.xml` drop opens both, one replacing the other. Mesh paths from an
  included file resolve against the **main** file's `base_dir` and
  `meshdir` first, which is what MuJoCo does first (probed in step 1);
  MuJoCo's second try, the including file's directory, is OPEN 5.
- `riggen-py`: no new API. `MjcfImportError`'s docstring and
  `_riggen.pyi` lose "it composes other files" and gain the missing-file
  case.
- No core, no schema, no `ResolvedRobot`, no writer change.

## Steps

Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [x] **[3]** Step 1 — ADR-0026, with MJCF's real composition semantics
  probed on MuJoCo 3.13.0 first (a scratch model plus `mj_saveLastXML`,
  which does this rewrite itself; findings pinned as comments in
  `mjcf_compose`): include resolved against the main model's dir with the
  including file's as fallback; `<mujocoinclude>` as a root tag; an
  `<include>` inside a `<body>`; the same file included twice; several
  `<compiler>` / `<worldbody>` / `<default>` blocks after splicing; a
  `meshdir` in an included file; `<frame>` pose composition, `childclass`
  push, a wrapped `<joint axis>` under a rotating frame, and nesting.
  Row in `docs/adr/README.md`; ADR-0015 marked amended.
  *Found (2026-09-09, all pinned in the ADR's tables and the module
  comment):* `Compiler::read` already merges every `<compiler>` per
  attribute in document order, which is MuJoCo's rule — step 3 loses that
  bullet; include identity is the path **as written**, normalised, not the
  file opened; a frame's `childclass` reaches a wrapped body's whole
  subtree, `class` on a frame means `childclass`, and an `<inertial>`
  under a frame is not transformed — step 4 amended; a mesh declared in an
  included file has a fallback riggen does not have — OPEN 5; a class
  name in two `<default>` blocks is MuJoCo's error and our merge
  (ADR-0026 §6, noted, unchanged).
- [ ] **[3]** Step 2 — `mjcf_compose.rs`: the pass and `<include>`.
  Splice at the site, both root tags, recursion with the main-dir-first
  resolution, a visited set that makes a duplicate include
  `DuplicateInclude` and a cycle terminate, `IncludeNotFound` naming the
  file and the file that asked for it. `from_mjcf` calls it; `REFUSED`
  loses `"include"`. Unit tests over `MemorySource`: a two-file model, a
  three-deep chain, an include inside a `<body>`, an include of a
  `<mujocoinclude>` fragment, the duplicate, the cycle, the missing file.
- [ ] **[2]** Step 3 — the merge, so a split model actually opens.
  `run()` over every `<worldbody>`, still `NoRoot` / `MultipleRoots` by
  the total count of root `<body>`s (`Compiler::read` and
  `Defaults::read` already merge — step 1's finding — so a test pins that
  rather than a change). Test: a model whose `<worldbody>`, `<asset>`,
  `<default>` and `<actuator>` live in four files imports identical to the
  same model written flat, warnings included; an included `<compiler
  angle="radian"/>` placed after the `<worldbody>` still governs it. Plus
  OPEN 5's answer, if it is yes.
- [ ] **[2]** Step 4 — `<frame>` folding, and the two that stay refused.
  Pose composition onto every child (`<body>`, `<geom>`, `<site>`,
  `<joint>`, `<camera>`, `<light>`, nested `<frame>`) in whichever of the
  five spellings either side used, the frame's `childclass` — or `class`,
  which MuJoCo treats the same — pushed onto children that name no class
  and onto a wrapped `<body>` with no `childclass` as its `childclass`, a
  rotating frame rotating a wrapped `<joint axis>` and a `<geom fromto>`,
  nested frames composing outer-first, an `<inertial>` left untouched; the
  frame element then disappears. `REFUSED` loses
  `"frame"`; `<replicate>` and `<attach>` get messages that name what they
  mean (a subtree copy; a submodel attached under a prefix) rather than
  just the tag. Unit tests per case, each against the hand-flattened
  equivalent.
- [ ] **[2]** Step 5 — the corpus and the `mujoco` job.
  `menagerie_style.xml` grows a `<frame>` around a body with a joint in
  it and moves a block (its `<asset>` and one body) into a sibling
  `menagerie_style_arm.xml` it `<include>`s; the CI job copies both files
  into `target/corpus/`. The re-export still loads with zero warnings,
  still agrees with the *original* document's `fk.json` to 1e-6, and its
  `<actuator>` / `<equality>` / `<tendon>` blocks still equal the
  original's field for field with `ROUND_TRIP_DROPPED` empty.
- [ ] **[2]** Step 6 — the app, the SDK and the docs. `load_dropped`
  treats a dropped `.xml` that another dropped `.xml` includes as a
  fragment and does not open it (OPEN 2); the missing-include message
  reaches the status bar saying which file to drop as well; a visual
  snapshot for that status line; `MjcfImportError`'s docstring and
  `_riggen.pyi`; a `python/tests/sdk` case for a two-file load and for
  the missing include; the docs in §Docs to update.

## Acceptance

The v0.4 accept, restricted to this bullet:

```sh
cargo test
mkdir -p target/corpus/arm && cp assets/fixtures/menagerie_style.xml \
  assets/fixtures/menagerie_style_arm.xml target/corpus/ \
  && cp assets/fixtures/arm/{base.stl,shoulder.stl,thing.msh} target/corpus/arm/
cargo run -p riggen-app -- --export mjcf --fk-samples --out target/sample-corpus target/corpus/menagerie_style.xml
uv run --no-project --with mujoco --with numpy python python/tests/test_mjcf_load.py \
  target/sample-corpus@target/corpus/menagerie_style.xml
```

passes with `ROUND_TRIP_DROPPED` empty and no `ElementDropped` naming
`<include>` or `<frame>` — the corpus model now being two files with a
frame in it. The `mujoco` CI job is that command.

Plus one measurement, not a CI dependency (AGENTS.md's Menagerie rule):
the built binary run over all 261 `.xml` in
`~/Documents/code/sim/mujoco_menagerie`, bucketed as the idea bucketed
them. `<include> is not supported` must be **0** (from 113), `<frame>` and
the imports must move to at least **69** more files importing (56 + 13),
and no bucket may grow. The new table goes in the retirement commit's
message.

## Docs to update on completion

- `docs/DATA-MODEL.md` §MJCF import — a **Composition** paragraph
  before "Read before any body is": the pre-pass, what it resolves, what
  it refuses, the include resolution rule, and the stated cost (a
  re-export is one flat file); the sentence naming `<include>` /
  `<frame>` among the refusals.
- `docs/ARCHITECTURE.md` — the crate layout line for
  `mjcf_compose.rs`; §Testing, the corpus's second file.
- `docs/ROADMAP.md` §v0.4 — the Composition bullet *Landed
  (plans/composition, ADR-0026)*, amended to name the two that landed and
  the two that stay refused (idea decision 5).
- `docs/BACKLOG.md` — `<attach>` (with the one-`Robot`-or-two question)
  and `<replicate>` as lines of their own; the density-gate side finding
  as a third.
- `docs/adr/README.md` — the 0026 row; 0015's status gains "§5 amended by
  0026".
- `AGENTS.md` current state — composition landed; **Next:** close v0.4.
- `README.md` — only if its feature list names what MJCF import reads
  (check at retire).

## Open questions

- ~~`⚠ OPEN 1:`~~ the option — **decided (human, 2026-09-09, before step
  1): B.** `<include>` and `<frame>` are flattened; `<replicate>` and
  `<attach>` are refused by name with a message that says what each
  means. **A** (all four) was +2 steps for 2 files and the one genuinely
  two-robots-composed question; **C** (`<include>` only) would have lost
  the `<frame>` model and the scaffold that makes `<attach>` a follow-up
  rather than a rewrite.
- ~~`⚠ OPEN 2:`~~ two `.xml` in one drop — **decided (human, 2026-09-09,
  before step 6): the recommendation.** After the set is installed, an
  `.xml` that another dropped `.xml` `<include>`s is a *fragment* and is
  not opened as a document; everything else opens as it does today.
  Without it, dropping `scene.xml` + `robot.xml` opens both —
  `load_dropped` opens every `replaces_document` file in the set, so the
  second silently replaces the first.
- ~~`⚠ OPEN 3:`~~ a missing included file — **decided (idea 2):**
  `ImportError::IncludeNotFound { file, from }`, not `Io`. On the web a
  `scene.xml` dropped without its `robot.xml` is the common case, and the
  message has to say what else to drop (ADR-0017).
- `⚠ OPEN 5:` (human, names **step 3**) a `<mesh file>` declared in an
  included file — MuJoCo looks in the main directory + `meshdir` first and
  then in the *including* file's directory with no `meshdir` (ADR-0026
  §6, measured). riggen has only the first. **Carry the fallback in the
  pass?** It can: after splicing, `compose` knows the merged `meshdir`,
  each spliced `<mesh>`'s including directory and `FileSource::exists`,
  so it could rewrite a `file` whose main-directory path is missing to the
  including-directory path that exists — ~20 lines, no reader change, and
  composition still never reaches the reader. It buys zero corpus files
  today: the one model that relies on it (`ms_human_700`, six entry files,
  `../geometry/*.stl` from `assets/asset/`) is refused for composite
  joints first (ADR-0022). *Recommendation:* yes, in step 3 — it is what
  MuJoCo does, it is cheap, and a `MeshNotFound` naming a path MuJoCo
  never even tried is a confusing message. Alternative: no, and a backlog
  line.
- ~~`⚠ OPEN 4:`~~ the same file included twice — **decided (idea 3):**
  MuJoCo's hard error, reproduced. A file MuJoCo refuses should not open
  in riggen.
