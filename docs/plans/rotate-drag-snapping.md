# Plan: rotate-drag-snapping

- Started: 2026-09-12
- Milestone: v0.5 — the viewport and the camera (the last unlanded bullet)
- Idea (verbatim from the human): "next thing on roadmap" —
  `docs/ROADMAP.md` §v0.5: "**Snapping during a rotate gizmo drag.**
  ADR-0019 §5 left the drag snap to translation because a rotation about a
  named axis has nothing in the ladder to land on. The answer is aligning
  the dragged frame's axis to a snapped feature's (a circle's, a face
  normal), which needs a rule for *which* of the three axes aligns and a
  second overlay idiom — an ADR if the rule turns out to be contested."

## Goal

A rotate gizmo drag lands on features the way a translate drag already
does. While the drag is in flight the snap ladder runs under the cursor, in
a **direction-only** form — a fitted circle's axis, or a face normal;
a vertex and a box corner say nothing about direction and are skipped — and
the previewed rotation is corrected so that one of the dragged frame's own
axes lies exactly along that direction. Which axis is not a new question the
user answers: a ring drag has one degree of freedom, about the ring's own
axis, so only the two frame axes perpendicular to it can move at all, and
the one that lands is whichever of their four signed directions the drag has
already brought nearest. The overlay says so before the release commits it —
a cyan spoke at the gizmo's radius along the direction being landed on, and
a readout that names the axis. One drag, one command, as before. Roadmap
acceptance: "a rotate drag lands the dragged frame's axis on a bore's."

## Non-goals

- **No angle quantum.** 5° / 15° increments under a modifier stay out: the
  wheel over a ring is riggen's rotation quantum (ADR-0019 §2) and the
  translate half of that idea stays the backlog line it is.
- **No fourth candidate from the document.** For a `GizmoTarget::Joint` the
  joint's own `axis` is often not a frame axis; landing *it* on a bore is
  what the Place joint tool already does in one click, and the rule here
  stays "one of the three frame axes".
- **The view ring is not claimed**, for ADR-0019 §3's reason: the document
  has no name for the camera's forward axis, so a drag about it has no
  frame axis worth reporting. A drag there behaves as it does today.
- **No new switch.** The pointer policy's five rows are untouched; a rotate
  drag simply joins `snapping()`, which is what sets
  `set_select_suppressed` and what `set_pick_excluded` keys off.
- **Nothing in the document changes** — no schema field, no command, no
  export. `docs/DATA-MODEL.md` is untouched.
- **Align is not replaced.** Two clicks making two circles concentric stays
  the tool for a part that came out of CAD at the wrong origin.

## Design deltas

- **ADR-0029 (new)** — *a rotate drag lands a frame axis on a feature;
  amends ADR-0019 §5*. It decides: the drag plane is the ring's, the
  candidate set is the four signed directions of the two frame axes
  perpendicular to the ring, the target is the feature axis projected into
  that plane (no projection worth the name → no snap), the winner is the
  nearest by angle, the ladder runs direction-only, the view ring is out,
  and the marker is a spoke plus an axis-prefixed readout.
  `docs/adr/0019-*.md` gains an "Amended by" line in the index table.
- **`app/snap.rs`** — a pure `align_in_plane(ring_axis, frame_rotation,
  feature_axis) -> Option<Alignment>` beside `choose` and `nearest_within`,
  unit-tested without a GPU; `Tool::snaps` unchanged; `translate_dragging`
  gains a `rotate_dragging` sibling and `snapping()` takes both;
  `dragged_instances` keys off a **link** drag rather than a translate one,
  so a rotate drag also looks through the subtree it is turning;
  `compute_snap` skips the vertex and box rungs while rotating, and its
  early return narrows from `!translate_dragging()` to `!gizmo_dragging()`.
- **`app/gizmo.rs`** — `GizmoState` latches the ring (and the world axis it
  turns about) at drag start, because `hovered_ring` is gated on
  `over_handle` and goes `None` the moment the cursor leaves the band; the
  result arm applies the correction to `pose.r` the way it already replaces
  `pose.t` for a translate drag, and `end_gizmo_drag` clears the latch.
- **`app/snap.rs::push_snap_overlay`** — the second idiom: while a rotate
  drag is aligning, a cyan segment from the gizmo's pivot along the target
  direction at the ring's own world radius (`ring_scale · gizmo_size`, the
  numbers `ring_under_cursor` already measures with), and the label at its
  tip prefixed with the axis — `+z → circle r 12.0 mm · 24 seg · res 0.01
  mm`. `SnapCandidate::readout()` itself is unchanged; `placed_status` reads
  it.
- **`debug/mod.rs`** — `SnapDebug` gains `align: Option<AlignDebug { axis,
  target, degrees }>` behind `skip_serializing_if = "Option::is_none"`, so
  every existing JSON golden is byte-identical and a headless test can
  assert the alignment rather than the pixels.
- **`docs/ARCHITECTURE.md` §Picking and snapping** — the three gestures
  become four, "a *rotate* drag does not snap" is replaced by the rule, and
  the `set_pick_excluded` sentence stops saying "translate".

## Steps

Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [x] **[2]** Step 1 — **ADR-0029 and the rule as a pure function.**
  Write the ADR (decision, consequences, alternatives: a fixed +Z, a
  tolerance band, the joint's own axis, the view ring, an axis-coloured
  spoke) and add `align_in_plane` to `app/snap.rs` with its unit tests: the
  nearest of the four wins; the correction is never more than a quarter
  turn; a feature axis parallel to the ring axis gives `None`; a frame that
  is already aligned gives a zero correction and still names the axis.
  Index row in `docs/adr/README.md`. No behaviour changes yet, nothing
  visible.
- [ ] **[3]** Step 2 — **the drag lands the axis.** `rotate_dragging`,
  `snapping()`, the direction-only ladder, the pick exclusion for a link
  drag, the ring latched at drag start, the correction applied to the
  preview, the release committing it unchanged. Tests: `a_rotate_drag_does_not_snap`
  is retired and replaced by `a_rotate_drag_lands_an_axis_on_a_bore` (the
  cylinder fixture `cylinder_stl` already builds, a second link dragged by
  its X ring, the frame's +Z on the bore's axis mid-drag) and
  `a_snapped_rotate_drag_commits_the_alignment` (one history entry, the
  committed pose agreeing with what the preview showed to 0.5°). Regression
  watch: a *click* on a selected frame under Rotate still places it
  (`placing_frame`), because both gestures live on the same tool and
  selection.
- [ ] **[2]** Step 3 — **the overlay says what it landed on.** The spoke,
  the axis-prefixed readout, `SnapDebug::align`, and the golden
  `gizmo_rotate_drag_snaps_to_a_bore` captured mid-drag with the button
  still down — the marker on the wall, the spoke on the ring, the part
  already turned, nothing committed. Show the human the image.

## Acceptance

`cargo test --workspace` green, and in particular the three new tests of
steps 2 and 3: a rotate drag over a fitted bore lands the dragged frame's
+Z on the bore's axis within 0.5°, previews it, and commits it as one
history entry — the roadmap's own line for this bullet. Every existing JSON
golden unchanged (the new debug field is skipped when absent); the one new
PNG golden reviewed by the human before it is committed.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Picking and snapping — four gestures ask for the
  ladder, not three; the rotate rule and the direction-only ladder; the
  `set_pick_excluded` sentence; the `set_select_suppressed` row's "translate
  drag" → "a gizmo drag"; the test list (`a_rotate_drag_does_not_snap` out,
  the three new names in, `gizmo_rotate_drag_snaps_to_a_bore` in the golden
  set).
- `docs/adr/README.md` — ADR-0029's row, and ADR-0019's status becomes
  "Accepted, §5 amended by 0029".
- `docs/ROADMAP.md` §v0.5 — the bullet marked *Landed
  (plans/rotate-drag-snapping, ADR-0029)*, as the other five are. Every line
  of v0.5 is then in, so `/close-cycle` is what follows this plan.
- `AGENTS.md` "Current state" — the v0.5 paragraph loses "**Left:**
  rotate-drag snapping" and gains the rule in a clause; still under ~15
  lines.
- `docs/BACKLOG.md` — nothing removed (the snap-quantum line is explicitly
  out of scope and stays); a new line if step 2 leaves one behind.

## Open questions

- ~~OPEN 1: **unconditional, or only within a tolerance?**~~ Decided at
  step 1, **unconditional** (the recommendation): ADR-0029 §6. Free
  rotation is the background, a vertex, or a ring the feature is parallel
  to; a tolerance band buys back a freedom that is already a centimetre
  away, at the price of a constant and a marker that has to explain when it
  is live.
- ~~OPEN 2: **does a face normal count, or only a fitted circle?**~~
  Decided at step 1, **both** (the recommendation): ADR-0029 §5. The ladder
  runs direction-only — circle > point — and the readout names which one
  landed, so a cardinal normal on a box that produces a no-op is legible
  rather than mysterious.
