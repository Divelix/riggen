# Idea: joint-glyph-range-and-value

- Status: Open
- Raised: 2026-09-05
- Prompt (verbatim from the human): "we should rework joint visualization:
  it should depict both joint limits and current value. I suggest to make
  translucent ring chart with limits as less translusent sector plus current
  value on top of it, e.g. ring 20%, limits 30%, value: 50% — thus value
  sums up to 100% opaque (you can suggest something else). Such rings would
  be tedious to hover, probably we need invisible spheres to easily mouse
  hover them (debatable, you can argue)"

## Problem

A revolute glyph today is an axis segment, a limit arc stroked at 1.5 px,
and a white spoke at `q` (01 §Joint glyphs, `app/glyphs.rs::push_arc`). On
the arm's `glyph_driven_joint` snapshot the three arcs are thin loops at
three orientations; which end is the lower limit, how much of the range is
used, and whether the joint is near a stop cannot be read without finding
the arc's start and comparing it to the spoke. The value is a *line*, and a
line does not read as a quantity.

In v0.4's View mode (03 §The window) the glyph stops being decoration: it
is the thing the wheel drives and the only thing under the cursor. A
handle that is turned by hovering it has to say, at a glance, how far it
has turned and how far it can — the slider's own two facts, in the
viewport — and it has to be aimable with a wheel-driving hover that lasts
seconds, not a click.

## Constraints it runs into

- **ADR-0020, every part of a joint glyph is depth-tested**, and §5 — a
  hidden run is dimmed to ~35 % in the same colour and width, because the
  drawn shape and `glyph_at`'s hit target must not disagree. The overlay
  has no fill primitive: `OverlayItem` is Segment / Polyline / Arc / Point /
  Label (`riggen-viewport/src/overlay.rs`), and depth is applied by
  `split_runs` on a *path*. A filled sector needs a new item and a rule for
  how a fill meets depth — an extension of §5, not a reversal.
- **The user's opacity arithmetic does not compose.** Three translucent
  layers at 20 / 30 / 50 % painted over each other give
  1 − 0.8 · 0.7 · 0.5 = 72 %, not 100 %. What the proposal *means* — a
  three-step ramp ending solid — is specified as the resulting alpha of
  each band, not as layers.
- **ADR-0019 §1**, the wheel is claimable by a rotate ring under the
  cursor. View mode wants a hovered glyph to claim it the same way
  (`set_wheel_claimed`, `app/mod.rs`), which makes the hover target a
  *dwell* target: the cursor sits on it while notches arrive.
- **ADR-0013 / ADR-0014 marks must survive**: the muted amber of a mimic
  follower, the full-amber actuator ring at 0.16 × size around the pivot
  and its label. Whatever fills the range must not paint over that ring.
- **ADR-0007 / ADR-0010**: the rotate gizmo's rings sit on the same pivot
  when the Rotate tool has a joint, drawn by `transform-gizmo-egui` on
  top of the overlay. A filled band at `ARC_RADIUS` under a gizmo ring is a
  stack of two rings; Edit mode has to look at that.
- **`glyph_at` hit-tests in screen points** (01 §Joint glyphs): the target
  is what the user can see, at a fixed pixel tolerance, deliberately *not*
  a world-space volume that shrinks with distance. An invisible sphere at
  the pivot is exactly that volume.
- **ADR-0003**: `glyph_revolute`, `glyph_prismatic`, `glyph_hover`,
  `glyph_behind_part`, `glyph_driven_joint` and every scenario with a
  movable joint in view change their golden; the rotate-gizmo scenarios
  too. That is the price of any option but "do nothing".
- **SEED §What not to spend agent time on** lists theming; a glyph's shape
  is not a theme. No layer rule is touched: the primitive is the
  viewport's, the glyph builder the app's, as now.

## Options

### A — A stacked ring: range, limits, value as filled bands

The human's proposal, made composable. One new `OverlayItem::Sector`
(centre, axis, start, inner and outer radius, sweep, fill), tessellated as
an annulus fan. The band, not a disc: the inner radius keeps the pivot
triad, the actuator ring and the bore the joint was placed on visible
inside it. Three sectors per revolute joint, drawn in order: the full
circle at a low alpha, the limit sector over it at a middle one, the value
sector from the **zero position to `q`** on top at a high one, plus the
existing spoke at `q` (so a joint at zero, whose value sector is empty,
still shows where it points). Alphas stated as what results — say 0.2,
0.5, 0.9, settled by looking at the snapshot — not as layers.

Depth: a fan is classified per vertex against the depth image and each
triangle drawn at the dimmed alpha when its vertices are hidden — the same
"dim, never drop" as §5, coarser than a path split, which for a band a
few pixels wide is invisible. A hidden band reads as a fainter band, which
is right.

`Continuous`: no limits, so the full circle *is* the limit band; value from
zero to `q`, wrapping. `Prismatic`: the same three, flattened — there is no
"full range" for a slide, so a translucent bar over the limits and a
denser bar from zero to `q`, with the end stops; `push_slide`'s offset
keeps it beside the axis. `Fixed`, when selected: unchanged.

Trade-offs: the strongest read of the three; a real quantity, and the same
shape the tree's scrubber draws in 1D. Costs a primitive and a depth rule
for fills, and every glyph golden. A band across a bore hides a little of
what is behind it, at 0.2 alpha — the reason for the annulus and for
keeping the alphas low until the value band.

Cost: ~4 plan steps — the primitive with its per-vertex depth (01 §Layer map's
overlay paragraph gains the rule), the revolute / continuous glyph with its snapshots, the
prismatic one, and the hover target below. Forecloses nothing.

### B — Weighted strokes: the value as a heavier run of the arc

No new primitive. The limit arc stays a 1.5 px stroke; the run from zero to
`q` is stroked again at ~4 px in the tick colour; short end stops at both
limits. Everything is a path, so ADR-0020's split applies unchanged.

Trade-offs: cheapest by far and readable — a thick run against a thin arc
*is* a bar chart. But at 4 px the value never becomes a surface, the full
circle of a limited joint has nowhere to appear (a third stroke weight is
not a third band), and the hover story is unchanged: the target is still a
line. It is the one-step version of A, and would be replaced by A the
first time the View mode is used in anger.

Cost: 1–2 plan steps.

### C — Limits as a band, value as the spoke plus a heavy run

A hybrid: one filled sector (the limits, low alpha) from a new primitive,
and the value as B's heavy stroke from zero to `q`. No stacking arithmetic,
one fill.

Trade-offs: pays for the primitive and gets one band out of it; the value
still reads as a line. Cheaper than A by a step and worse than A at
exactly the thing the prompt asks for — the value as a quantity.

Cost: ~3 plan steps.

### The hover target (orthogonal to A / B / C)

Today: nearest *axis segment* within `GLYPH_HOVER_RADIUS` = 8 pt. The
prompt's worry is right for the ring — a thin loop is a bad dwell target —
and wrong about the remedy:

- **An invisible world-space sphere** shrinks with distance and swallows
  the mesh hover under it in Edit, where the mesh is what the user is
  usually aiming at, and it breaks the rule that the target is the drawn
  shape.
- **The drawn shape itself**: hit-test the axis segment *and* the band's
  projected polyline (`split_runs` already produces it), at the same 8 pt,
  and in **View mode only** the band's interior as well — the ellipse the
  ring projects to, inside which there is nothing else to hover. A band
  a dozen pixels wide plus its interior is a target the size of a coin,
  which is what a dwell-and-wheel gesture wants, and in Edit the target
  stays what can be seen.

Mode-dependence puts this line in the View-mode plan, not the glyph's; the
glyph plan only has to expose the projected band.

### Do nothing

The spoke and the thin arc stay. View mode arrives with a wheel that drives
a glyph whose value is a line, and the tree's scrubber becomes the only
place the range reads — the user looks away from the robot to pose it,
which is the thing the mode exists to stop.

## Recommendation

**A**, as the annulus with the value from zero, the spoke kept, and the
alphas set by eye on the snapshot — with the hover target as "the drawn
shape, plus the interior in View". B loses because it is A's rough draft;
C loses because it spends the primitive and keeps the weak half. No ADR:
the depth rule for fills is a paragraph beside ADR-0020's §5 in 01
§Layer map's overlay paragraph, the shape is a drawing, and nothing in it changes who owns the
pointer — the wheel claim is View mode's ADR, already on the roadmap.

What would change my mind: if a filled band under the rotate gizmo's ring
reads as two conflicting handles in Edit, the band drops to B's weights
while a gizmo is on that joint. And if the per-vertex fill dimming looks
worse than the path split in `glyph_behind_part`, the band is drawn as a
stack of thin arcs (a fill by stroking) — uglier code, the same depth rule.

## Decision for the human

1. **Annulus band or full disc?** Band (the pivot triad, the actuator ring
   and the bore stay visible inside it).
2. **The value sector from the zero position or from the lower limit?**
   From zero: it shows the displacement the mesh actually made and its
   sign; the tree's scrubber already shows the position within the range.
3. **Hover target: the drawn band plus its interior in View, or an
   invisible sphere?** The drawn shape; no sphere.
4. **Alphas as resulting opacities (about 0.2 / 0.5 / 0.9), settled on the
   snapshot rather than fixed here?** Yes.
5. **An ADR?** No — a paragraph in 01 (§Layer map, the overlay) for how a fill meets
   depth.
