# Plan: composite-joints

- Started: 2026-09-06
- Milestone: v0.4 (the file half)
- Idea: `docs/ideas/composite-joints.md` (absorbed)
- Idea (verbatim from the human): "yes" — to running `/idea` on v0.4's
  ⚠ OPEN, and then, on the idea's three decisions: refuse as today, with
  the sharper message as its own step in this plan.

## Goal

v0.4's ⚠ OPEN is closed with a *no*, in writing. **ADR-0022** records that
a `<body>` with several `<joint>`s stays `ImportError::CompositeJoint` —
not from ADR-0006's taste argument, which is what got the question
reopened, but from the corpus and from MuJoCo: synthesising massless
intermediate links would open five Menagerie directories (three of which
are a fly, a musculoskeletal human and a soft foot), and the naive version
does not work at all, because MuJoCo refuses a moving body under
`mjMINVAL` mass and so does our own `ExportError::ZeroMassMovableLink` —
the import would open files riggen cannot export. The ADR also names the
shape to start from if it is ever reopened (A2: no mark, no schema change,
the writer collapses by inspection), so the next person does not begin at
A. And the refusal stops being a dead end: its message says what the shape
*means* and what to do about it, so a user who hits it can split the body
into nested bodies — a MuJoCo-equivalent edit — and import.

## Non-goals

- No synthesis, no collapse-by-inspection in the MJCF writer, no
  `resolve` exemption: this plan writes down the *no*.
- No `JointKind::Ball` / `Planar`: no schema change, no `q` change, no
  touch of `fk`, the scrubbers or the glyph band.
- `ImportError::JointOnRoot` is not softened — the two Menagerie
  directories with a composite on the root body (`stanford_tidybot`,
  `umi_gripper`) stay refused whatever this plan decides, and their
  message is already specific.
- No new snapshot: the message reaches the UI as the `String` that
  `finish_import` hands back (`crates/riggen-app/src/app/file_io.rs:264`),
  and no snapshot scenario imports a composite file. Keep it **one line**
  so the status bar it lands in stays one line.

## Design deltas

- **`docs/adr/0022-composite-joints-stay-refused.md`** (new, Accepted).
  Re-decides ADR-0015 §5's first bullet with evidence rather than
  principle. ADR-0015 is append-only and is not edited; the ADR index row
  for 0015 gains "§5's composite-joint bullet re-affirmed by 0022".
  The three things worth keeping out of the idea, and the ADR's spine:
  - **The corpus** (`~/Documents/code/sim/mujoco_menagerie`, 2026-09-05):
    2201 `<body>` elements, 120 with more than one `<joint>`, in **7 of 68**
    model directories — `flybody` (25), `ms_human_700` (43),
    `iit_softfoot` (45), `hello_robot_stretch` and `hello_robot_stretch_3`
    (2 each, the gripper's rubber tips), `stanford_tidybot` and
    `umi_gripper` (composite on the **root** body, so refused by
    `JointOnRoot` regardless). Synthesis would open **five**. All 120 are
    hinges and slides; `type="ball"` appears **twice in the whole corpus**
    (`agility_cassie`), which is what prices option B out.
  - **MuJoCo's own rule**, measured, not assumed: a moving body with mass
    under `mjMINVAL` is rejected — `ValueError: mass and inertia of moving
    bodies must be larger than mjMINVAL`. `ExportError::ZeroMassMovableLink`
    (`crates/riggen-export/src/resolve.rs:168`) already says the same
    thing. So a plain synthesis (option C, what the roadmap line literally
    proposed) yields a document riggen imports and cannot export; only a
    fabricated mass saves it, which is ADR-0015 §7's rejected
    `inertiafromgeom` density in a new costume.
  - **If reopened, start at A2**: synthesise ordinary links, no mark and no
    schema field; the MJCF writer folds any link that is massless,
    geom-less, frame-less and has exactly one child into its child's
    `<body>`; `resolve` stops erroring for exactly that shape when the
    format is MJCF. Not A (a `Link` mark is the second, format-shaped
    description ADR-0015 §3 refused, and every command that adds a geom,
    a mass, a frame or a child to a marked link would have to decide what
    the mark then means). Its two known warts, stated in the ADR so they
    are not rediscovered: MJCF → URDF — differentiator §4.4 — stays
    blocked for these files, and a *user's own* massless pass-through link
    would silently start exporting instead of erroring.
  - **What would change the answer**: someone actually blocked on a model
    that matters (Stretch is the plausible one); a second corpus (Robot
    Descriptions, Isaac assets) where composite bodies are common rather
    than 5%; or MuJoCo relaxing the massless-body rule, which would make C
    nearly free.
- **`ImportError::CompositeJoint`'s `Display`**
  (`crates/riggen-export/src/import.rs:259`). Today: `body "w" has 2
  joints (w0, w1); the link tree holds one per body`. It names the body and
  the joints — the diagnosis — and stops there. It gains the meaning and
  the way out: that MuJoCo spells a multi-DoF joint this way, and that
  splitting the body into nested bodies (one joint each) is the equivalent
  model riggen can read. The variant, its fields and the refusal site
  (`mjcf_in.rs:406`) are unchanged — this is one `Display` arm.
- No crate boundary, no type, no schema, no `.riggen` version moves.

## Steps

- [x] **Step 1 — ADR-0022, and the ⚠ OPEN closed.** Write
  `docs/adr/0022-composite-joints-stay-refused.md` from the Design deltas
  above (context: why ADR-0015 §5 is being asked again; decision: refused,
  and A2 is the shape if reopened; consequences: five directories stay
  shut, loudly; alternatives: A, A2, B, C, each with the cost that sank
  it). Add its row to `docs/adr/README.md` and amend 0015's row. Replace
  the ⚠ OPEN paragraph in `docs/03-roadmap.md` §v0.4 with one sentence
  citing ADR-0022. `docs(adr):` commit, no code.
- [ ] **Step 2 — the refusal tells the user what to do.** One `Display`
  arm in `crates/riggen-export/src/import.rs`; extend
  `the_shapes_the_document_cannot_hold_are_refused_by_name`
  (`mjcf_in.rs:2100`) — or a sibling test next to it — to assert the
  rendered string names the body, both joints, and the nested-bodies
  workaround, and that it is a single line. Update
  `docs/02-data-model.md` §745's "shapes it refuses" sentence to cite
  ADR-0022 beside ADR-0015 §5, in the same commit.

## Acceptance

`cargo test -p riggen-export` is green, with the refusal test asserting the
message contains the body name, both joint names and the workaround, and
contains no `\n`; `cargo fmt --check` and `cargo clippy --all-targets -- -D
warnings` pass; `grep -c "⚠ OPEN" docs/03-roadmap.md` returns 0 for the
v0.4 section; `docs/adr/0022-*.md` exists and is listed in
`docs/adr/README.md`. Importing a composite Menagerie file — e.g.
`hello_robot_stretch/stretch.xml` — still fails, and now says what to do.

## Docs to update on completion

- `docs/03-roadmap.md` §v0.4 — the ⚠ OPEN paragraph replaced by the
  decision and its ADR (done in step 1); the cycle's status line unchanged
  otherwise.
- `docs/02-data-model.md` §745 — the "shapes it refuses" sentence cites
  ADR-0022 for the composite case (done in step 2).
- `docs/adr/README.md` — the 0022 row; 0015's row notes the re-affirmation
  (done in step 1).
- `AGENTS.md` current state — the ADR is written, so "Next: an ADR
  (composite joints), then the file half" becomes the file half alone.
- `docs/BACKLOG.md` — a line for the reopen conditions (a blocked user, a
  second corpus, or MuJoCo relaxing `mjMINVAL`), pointing at ADR-0022 for
  the shape (A2). Not a "Rejected" line: the idea was accepted, its answer
  was *no*.

## Open questions

None. The idea's three decisions were answered by the human on 2026-09-06:
refuse as today; write the ADR; the sharper message gets its own step here
rather than riding in the composition plan.
