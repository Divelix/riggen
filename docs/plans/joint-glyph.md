# Plan: joint-glyph

- Started: 2026-09-11
- Milestone: v0.5 — the viewport and the camera (`docs/ROADMAP.md`, the
  sixth bullet: "View's joint glyph loses its legacy pieces and gains an
  opaque band")
- Idea (verbatim from the human): "/plan joint-glyph". No idea file: the
  roadmap bullet is the human's own report — the band's three-layer alpha
  stack "reads wrong at a grazing camera angle", and View draws "the axis
  segment, the pivot dot, the origin triad and the actuator ring in View
  exactly as it does in Edit".

## Goal

In **View** a joint's glyph is the thing the user scrubs and nothing else:
a hinge's band, a slide's bars, and the white tick at `q`. The axis
segment, the pivot dot, the origin triad and the actuator ring stay in
**Edit**, which keeps the full legacy glyph. The band itself stops being
one amber at three alphas and becomes three **opaque** shades of the
glyph's own colour, so an annulus sector that foreshortens at a grazing
camera angle and overlaps itself in screen space cannot double-cover — the
overlap is the same colour as the sector. Because a hidden thing answers
nothing and its converse (ADR-0020 §5, ADR-0021's second amendment),
**View's hover target becomes exactly what View draws**: the band's disc
for a hinge, the bars for a slide, and the axis segment for neither — a
prismatic joint, picked by its axis line alone today, is picked by its
bars instead. An actuated joint keeps a mark in View that is not the ring.

## Non-goals

- **Edit's glyph composition.** The axis, pivot dot, triad and actuator
  ring are drawn there exactly as today, and Edit's hover target stays the
  axis segment alone (ADR-0021 §6). Only the band's and bars' colours
  change in Edit, because `push_arc` / `push_slide` are one code path.
- **The `Overlay::JointNames` bug** — that the toggle draws the mimic
  leader's name and the actuator preset string rather than the joint's own
  name (`driven_marks`). A separate backlog line about *which text*, not
  about which shapes; the View cue this plan adds is a shape and is not
  gated by the names toggle.
- The rest of v0.5: ViewCube, fly camera, ground grid, MSAA, rotate-drag
  snapping. In particular this plan does **not** re-baseline the goldens
  for MSAA; it refreshes only the goldens its own change moves.
- The glyph's *size*, its depth policy (ADR-0020), the tick's colour and
  overshoot, the mimic muting rule (ADR-0013), and the scrubber quantum.

## Design deltas

- **`docs/adr/0027-…`** (step 1): what View draws, the opaque shade ramp,
  the actuated cue, and "View's target is what View draws". It **amends
  ADR-0014** (an actuated joint's mark is the ring at the pivot — true in
  Edit, replaced in View), **narrows ADR-0021 §1** (only joint glyphs
  answer the cursor in View — and now only the part of a glyph View draws
  does), and leaves **ADR-0013** intact (the muting is on the colour the
  ramp shades, so a follower's band is muted at all three shades).
- **`crates/riggen-app/src/app/glyphs.rs`** — `RANGE_ALPHA` /
  `LIMIT_ALPHA` / `VALUE_ALPHA` and `layered()` (an alpha that lands a
  stack on a target alpha) become `RANGE_SHADE` / `LIMIT_SHADE` /
  `VALUE_SHADE` and `shade()` (an RGB scale, alpha left at 255). Three
  constants either way, and the mimic-muted and hot variants keep coming
  from the one `axis_color()` — the ramp shades whatever colour it is
  handed, so the count does not multiply by four. `glyph_overlay()` gains
  a `self.mode` branch over which pieces it pushes; `push_arc` and
  `push_slide` change colours only. `JointGlyph` gains `bar_points()`
  beside `band_points()` — the slide's hover ring, the analogue of the
  band's centreline — and `glyph_distance()`'s View branch drops the axis.
- **`crates/riggen-app/src/debug/mod.rs`** — `GlyphDebug` gains
  `drawn: Vec<&'static str>`, the pieces this glyph contributed this frame
  (`"axis"`, `"pivot"`, `"triad"`, `"actuator"`, `"band"`, `"bars"`,
  `"tick"`), so a scenario asserts the *composition* as JSON and not only
  as pixels (ADR-0003), and `bar: Option<(f64, f64)>` beside `band` for a
  slide's travel extent.
- **`crates/riggen-viewport`** does not change. ADR-0021 §6 holds once
  more: this is which primitives the app pushes, in which mode.

## Steps

Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [x] **[2]** Step 1 — **ADR-0027: View's glyph is the band alone, and the
      band is opaque.** Docs only, no code. Settles the four open questions
      below: the actuated cue in View, what an unlimited prismatic joint
      shows, what a selected `Fixed` joint shows, and that View's hover
      target is exactly what View draws. Records the amendment to ADR-0014
      and the narrowing of ADR-0021 §1. Commit `docs(adr): …`.
      Landed as `docs/adr/0027-views-glyph-is-what-view-draws-and-the-band-is-opaque.md`;
      the human took all three recommendations.
- [x] **[2]** Step 2 — **The band goes opaque, in both modes.** `shade()`
      replaces `layered()`; `push_arc` draws its three sectors and
      `push_slide` its bars in the three opaque shades; the slide gains the
      range bar step 1 decided on. New scenario **`glyph_band_grazing`** —
      the camera brought into the joint's plane, which is where the alpha
      stack double-covered — and a refresh of every golden that carries a
      band (`glyph_revolute`, `glyph_prismatic`, `glyph_driven_joint`,
      `glyph_hover`, `glyph_behind_part`, `view_*`, `zen_view`,
      `gizmo_rotate_joint`, `joint_tree_chain`, and whatever else the run
      names). The human sees the images, and answers OPEN 4 from
      `gizmo_rotate_joint` — whether an opaque band under the rotate
      gizmo's rings now competes with them. `snapshots:` is part of this
      commit, with the reason, per `.agents/rules/git.md`.
      Landed: 59 goldens refreshed and 2 added; the scenario needed a
      camera the harness could not set, so `RiggenApp::look_from(yaw,
      pitch, distance_scale)` joins `fit_view_now` in `debug/mod.rs`.
- [x] **[2]** Step 3 — **View draws the band and the tick and nothing
      else.** `glyph_overlay()` branches on the mode: no axis segment, no
      pivot dot, no origin triad, no actuator ring in View; Edit unchanged.
      The actuated cue from step 1 lands here. `GlyphDebug::drawn` and
      `bar` land here too, and the View scenarios assert the composition
      (`["band", "tick"]` for a hinge, `["bars", "tick"]` for a slide,
      plus the cue) as JSON. New scenario **`view_glyph_actuated`**;
      refreshed View goldens. Landed: the branch is one list,
      `glyph_pieces()`, which `glyph_overlay()` draws from and
      `GlyphDebug::drawn` reports, so the picture and the dump cannot
      disagree; the tick moved out of `push_arc` / `push_slide` into
      `JointGlyph::tick_ends()` to become a piece of its own. The cue is
      `"bore"` in `drawn`, the ring stays `"actuator"`.
- [ ] **[2]** Step 4 — **View's hover target is what View draws.**
      `glyph_distance()` in View drops the axis branch and gains the bars:
      `JointGlyph::bar_points()` and the same polygon-distance-or-interior
      test the band gets. `the_band_is_the_hover_target_in_view_and_not_in_edit`
      inverts its last assertion (the axis is Edit's alone now), and the
      View scenarios that aim at `glyph_axis_point` (`view_wheel_on_glyph`
      and the zen and overlay-row ones, four call sites) move to
      `glyph_band_point`. New harness helper `glyph_bar_point` and a new
      scenario **`view_glyph_hover_bar`**: the pointer on a slide's bar,
      off its axis, has the glyph hot and the joint named in the status
      bar. Unit tests for `bar_points()` beside the existing
      `the_band_sits_between_the_actuator_ring_and_the_old_arc`.

## Acceptance

`cargo test -p riggen-app` green, and within it:

- **`glyph_band_grazing`** — the golden at a grazing camera angle shows a
  band of three flat colours with no lighter or darker seam where the
  sector overlaps itself, which is the bug the human reported.
- **In View**, `debug_state().glyphs[0].drawn` is exactly `["band",
  "tick"]` for the pendulum's hinge and `["bars", "tick"]` for the same
  joint made prismatic — plus the cue for an actuated one — while **in
  Edit** the same joint's `drawn` still carries `"axis"`, `"pivot"` and
  `"triad"`.
- **`view_glyph_hover_bar`** — a prismatic joint in View is hovered from a
  point on its bar and *not* from a point on its axis; the hinge's
  `view_glyph_hover_band` still passes unchanged.
- Every refreshed golden has been shown to the human and approved
  (ADR-0003), and `cargo clippy --all-targets -- -D warnings` is clean.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Panels and menus, the **Joint glyphs** bullet —
  the band is three **opaque shades of the glyph's own colour** (`shade`),
  not one amber at three alphas (`layered`); **what View draws** (band or
  bars, and the tick) against what Edit draws (all of it); the actuated
  cue in View beside ADR-0014's ring in Edit; the slide's range bar and
  what an unlimited slide shows.
- `docs/ARCHITECTURE.md` §Panels and menus, the same bullet's "**the
  target depends on the mode**" paragraph, and the §Picking and snapping
  sentence that repeats it (~line 801) — in View the target is the band's
  disc or the bars; the axis segment is Edit's alone.
- `docs/BACKLOG.md` — delete the absorbed lines: "In View mode,
  `glyphs.rs`'s `glyph_overlay()` draws a line along the joint's axis …
  drop the axis line"; and both lines under **### From the glyph band
  (plans/joint-glyph-range-and-value, 2026-09-05)** — the rotate-gizmo
  band (answered by OPEN 4) and the low-contrast alphas (answered by the
  opaque ramp) — which empties that heading, so the heading goes too.
- `AGENTS.md` "Current state" — the v0.5 line, once `/retire-plan` runs.
  `docs/ROADMAP.md`'s v0.5 section is not edited: it has no status line
  until `/close-cycle`, and its sixth bullet is the section's own record
  of what this plan did.

## Open questions

- **ANSWERED (human, step 1 → ADR-0027 §3): fill the bore.** **The actuated cue in View.** The ring at
  `ACTUATOR_RING_RADIUS` is one of the pieces going, and ADR-0014 made the
  ring the mark. Recommendation: **fill the bore** — a disc at the same
  radius in the glyph's own colour instead of a stroked ring. It is a fill
  like the band rather than a stroke, it sits inside the clear bore where
  it cannot read as part of the band, and "solid hub = driven" reads at a
  glance. Alternatives: the tick in a second colour; nothing at all in
  View, leaving it to the joint tree's row. *Human, by step 1.*
- **ANSWERED (human, step 1 → ADR-0027 §4): the range bar.** **An unlimited prismatic joint in View.** `push_slide` takes
  `limits.unwrap_or((0.0, 0.0))`, so a slide with no limits has
  zero-length bars — today the axis segment carries it, and in View that
  segment is gone, leaving a tick and nothing to aim at. Recommendation:
  give the slide the **range bar** it never had, spanning the axis
  segment's own extent (`±AXIS_HALF_LENGTH · size`) in the range shade —
  the slide's analogue of the hinge's full circle, drawn in both modes,
  which restores the symmetry the three shades want and gives View
  something to hit. *Answered as recommended.*
- **ANSWERED (human, step 1 → ADR-0027 §5): accept.** **A selected `Fixed` joint in View.** It has neither band nor
  bars, so under this plan it draws nothing at all. Recommendation:
  accept — a weld has nothing to pose, View's joint tree does not list it,
  and it can only be selected in View by carrying the selection over from
  Edit (ADR-0021 §3). *Answered as recommended.*
- **ANSWERED (human, step 2, from the image): it stays as it is.** The
  rings draw over the band and still read as the handles; a fill and a
  stroke are different things, and the band is not grabbable in Edit.
  **The band under the rotate gizmo's rings, now opaque.** The
  backlog line from plans/joint-glyph-range-and-value left this to be
  revisited if the fill and the stroked handles ever read as competing
  handles; an opaque fill is exactly the change that could make them.
  Either it stays as it is, or the band drops to a stroke while a rotate
  gizmo is on that joint. *Answered: it stays.*
- **ANSWERED (human, step 2, from the images): 0.34 / 0.62 / 1.0.** The
  agent showed three ramps face-on, with the joint selected and not; the
  middle one was chosen. Noted from the same images: the *hot* colour is
  near-white, so its shades read khaki rather than amber — a consequence
  of one `axis_color()` feeding the ramp, not a bug, and left alone.
  **The three shade factors themselves.** The alphas 0.2 / 0.5 /
  0.9 were settled by eye on the goldens; their opaque replacements have
  to be too, and the range sector is the one to watch — at 0.2 alpha it
  was barely there, and opaque it is a full amber annulus over the scene
  at all times. *Answered: 0.34 / 0.62 / 1.0.*
