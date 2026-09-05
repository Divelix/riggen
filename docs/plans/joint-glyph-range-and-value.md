# Plan: joint-glyph-range-and-value

- Started: 2026-09-05
- Milestone: v0.4, the window half (03 §The window: "The glyph shows the
  range and the value")
- Idea: docs/ideas/joint-glyph-range-and-value.md (absorbed)
- Idea (verbatim from the human): "we should rework joint visualization: it
  should depict both joint limits and current value. I suggest to make
  translucent ring chart with limits as less translusent sector plus current
  value on top of it, e.g. ring 20%, limits 30%, value: 50% — thus value
  sums up to 100% opaque"

## Goal

A revolute joint's glyph is a **band** — an annulus in the joint's plane at
the radius the limit arc has today — drawn as three filled sectors: the
full circle faint, the limits over it at a middle opacity, the run from the
zero position to `q` on top at a high one, the white spoke at `q` kept so a
joint at zero still points. The three opacities are what *results* on
screen, not layers. A `Continuous` joint's full circle is its limit band; a
prismatic joint gets the same three as bars beside its axis, end stops
kept. Everything is depth-tested as before (ADR-0020): a hidden part of a
band is dimmed, never dropped. The pivot triad, the actuator ring
(ADR-0014) and the bore inside the band stay visible, and the mimic amber
(ADR-0013) survives. The band's centreline is exposed so the View-mode plan
can make it the hover target.

## Non-goals

- The hover target and the wheel claim: `glyph_at` keeps hit-testing the
  axis segment; growing the target to the band and its interior is
  mode-dependent and belongs to the View-mode plan (the idea's hover
  section, 03 §The window).
- The rotate gizmo's rings (ADR-0007, ADR-0010) — not touched; step 2 only
  looks at the two stacked.
- Frame glyphs, the `Fixed` glyph, the labels, the depth readback's
  resolution (ADR-0020 §3), any new colour beyond the alphas.
- A prismatic "full range" band: a slide has no circle to be faint.

## Design deltas

- **`riggen-viewport/src/overlay.rs`**: `OverlayItem::Strip { pairs:
  Vec<(DVec3, DVec3)>, color }` — a filled quad strip in world space, each
  pair one rung (inner, outer), the strip's fill carrying its alpha in
  `color`. `Overlay::sector(center, axis, start, inner, outer, sweep,
  color)` tessellates an annulus sector into a strip at `ARC_STEP`; a bar
  is a two-rung strip. Depth for a fill: each rung's midpoint is classified
  against the `DepthImage`, and a quad is dimmed (`HIDDEN_STRENGTH`) when
  **both** its rungs are hidden, so a crossing lands on a rung and the
  visible and dimmed quads meet without a gap — the strip analogue of
  `split_runs`, pure and unit-tested without a GPU. Two rungs far apart
  on screen (a bar's two) are subdivided in world space first, as a
  path's segment is, so a bar dims where it enters a part. Painted as one
  `egui::Mesh` per visibility run.
- **`docs/01-architecture.md` §Layer map**, the overlay paragraph: the
  per-item depth rule gains its fill half — a stroke is split at its
  crossings, a fill is dimmed quad by quad.
- **`riggen-app/src/app/glyphs.rs`**: `push_arc` becomes the band —
  `BAND_INNER` (a new fraction of `size`, inside `ARC_RADIUS`, outside
  `ACTUATOR_RING_RADIUS`), the three resulting alphas as constants
  `RANGE_ALPHA` / `LIMIT_ALPHA` / `VALUE_ALPHA` (start 0.2 / 0.5 / 0.9,
  settled by eye at step 2); the value sector runs from `reference()` (the
  zero position) to `q`, signed. `push_slide` becomes two bars plus the
  stops and tick. `JointGlyph::band_points()` returns the centreline's
  world points for a later hit-test. The mimic colours multiply into the
  band's alphas as they do into the stroke today.
- **`riggen-app/src/debug/mod.rs`** `GlyphDebug`: `value_sweep: f64` (the
  signed sweep of the value sector, `q` for a revolute) and `band: Option<
  (f64, f64)>` (inner, outer radius), so a scenario asserts the shape
  rather than the pixels alone.
- **`docs/01-architecture.md` §Joint glyphs**: the glyph's description
  rewritten for the band (same commit as step 2), the prismatic bars (step
  3).
- No ADR: the idea recorded why — the shape is a drawing, the fill rule is
  ADR-0020 §5 extended, nobody's pointer ownership changes.

## Steps

- [x] Step 1 — `OverlayItem::Strip` and `Overlay::sector` in
  `riggen-viewport`, with the quad-wise depth rule and its painter; unit
  tests for the sector's tessellation (rung count, inner/outer radii, sweep
  sign) and for the run split on a strip crossing a hidden stretch (pure,
  like `split_runs`'s tests). The overlay paragraph of 01 §Layer map gains
  the fill rule. Nothing in the app draws one yet, so no golden changes.
- [x] Step 2 — The revolute / continuous band in `glyphs.rs`: three
  sectors, spoke kept, `BAND_INNER` and the three alphas as constants,
  `band_points()`, `GlyphDebug::{value_sweep, band}`. Look at it before the
  numbers are fixed (`visual-debug`): `glyph_revolute`, `glyph_hover`,
  `glyph_behind_part` (a band half inside a part is where the quad-wise dim
  is judged against the stroke split), `glyph_driven_joint` (the amber and
  the actuator ring inside the band) and `gizmo_rotate_joint` (a band under
  a gizmo ring). Refresh every golden with a movable joint in view;
  `snapshots:` in the message and the images shown to the human. A
  scenario assertion: in `glyph_revolute`, `value_sweep` equals the
  joint's `q` to 1e-9 and `band` sits inside `ARC_RADIUS` and outside
  `ACTUATOR_RING_RADIUS`.
- [ ] Step 3 — The prismatic bars in `push_slide`: the limits as a bar at
  `LIMIT_ALPHA`, zero to `q` on top at `VALUE_ALPHA`, stops and tick kept,
  `value_sweep` = `q` in metres. `glyph_prismatic` refreshed and shown; 01
  §Joint glyphs updated for the slide.

## Acceptance

`cargo test` green, the six goldens above refreshed once each and reviewed
by the human; `glyph_revolute` asserting `value_sweep == q` and the band's
radii; the strip's run split unit-tested with a crossing that lands on a
rung. On the arm (`--example arm`) at rest, which end of each hinge is the
lower limit, how much of the range is used, and whether a joint is near a
stop read without finding the arc's start.

## Docs to update on completion

- `docs/03-roadmap.md` §v0.4 §The window — the glyph bullet's ⚠ OPEN (idiom
  and hover target) reduced to the hover half, which the View-mode plan
  owns; the bullet itself stays until the cycle closes.
- `docs/01-architecture.md` §Layer map (overlay paragraph) and §Joint glyphs
  — written in steps 1–3; verify against the code at retirement.
- `docs/BACKLOG.md` — nothing expected; add whatever step 2's look at the
  gizmo stack turns up.
- `AGENTS.md` current state — unchanged (no milestone lands).

## Open questions

- ⚠ OPEN: the three resulting alphas (0.2 / 0.5 / 0.9 to start) and
  `BAND_INNER` — the human confirms on step 2's snapshots; the agent
  proposes from `visual-debug` captures first. *Step 2 proposes 0.2 /
  0.5 / 0.9 and `BAND_INNER = 0.42` as landed: on `glyph_revolute` the
  three read as three, and the faint range is still visible over the
  dark background; over an amber part the band is low-contrast, which
  the colour non-goal accepts.*
- ⚠ OPEN: a band under the rotate gizmo's ring in `gizmo_rotate_joint` — if
  the two read as competing handles, the band on a gizmo'd joint drops to a
  stroke-weight rendering while the gizmo is on it; the human decides at
  step 2 from the image, the agent does not pre-empt it. *Step 2's
  `gizmo_rotate_joint` shows the band under the three thin rings and the
  white view-plane ring; the agent's reading is that a filled band and
  stroked rings do not compete, but the decision is the human's.*
