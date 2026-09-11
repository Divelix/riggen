# ADR-0027: In View the glyph is the band alone, the band is opaque, and what View draws is what View answers

- Status: Accepted
- Date: 2026-09-11
- Amends: [ADR-0014](0014-actuators-on-the-joint-mjcf-only-three-presets.md)
  — an actuated joint's mark is the ring at the pivot, which stays true in
  Edit and is replaced in View; narrows
  [ADR-0021](0021-two-modes-and-edit-is-the-zero-configuration.md) §1 —
  only joint glyphs answer the cursor in View, and now only the part of a
  glyph View draws
- Leaves intact: [ADR-0013](0013-mimic-joints-as-urdf-mimic-and-mjcf-equality.md)
  (the muting is on the colour the ramp shades),
  [ADR-0020](0020-the-overlay-reads-the-scenes-depth-back.md) (the depth
  policy), [ADR-0003](0003-headless-visual-snapshots.md)

## Context

The glyph grew one piece at a time, and every piece was added in a window
that had one mode. The axis segment and the pivot dot came with M2, so a
joint the tree knew about could be seen and aimed at. The origin triad
came with it, because a joint frame's orientation is two number fields
otherwise. The actuator ring came with ADR-0014, so a driven hinge does
not draw like a free one. The band came last (plans/joint-glyph-range-and-
value), and it is the only one of them the user *operates*: it is the
range, the limits and the value, and in View it is the disc they aim the
wheel at (ADR-0021 §1).

ADR-0021 then split the window in two, and the glyph did not split with
it. View draws the axis segment, the pivot dot, the origin triad and the
actuator ring exactly as Edit does — four pieces that answer questions
about *where a joint frame is*, which is Edit's question. In View they are
clutter around the one piece that matters, and the triad's three colours
in particular compete with the band for the eye at the very moment the
user is reading a value off it.

The band has a second problem, reported by the human from the v0.4 build.
It is one amber at three alphas, stacked so the resulting opacities are
`RANGE_ALPHA` / `LIMIT_ALPHA` / `VALUE_ALPHA` (`layered` derives each
layer's own alpha from the one below). Translucency is exact only while
the layers cover each other once. Bring the camera into the joint's plane
and the annulus sector foreshortens until its own far half is drawn over
its near half in screen space: a translucent sector then **double-covers**
itself, and the band shows a lighter seam where nothing about the joint
changed. It reads as a defect in the model rather than in the drawing.
The two faintest shades are also barely there — the range sector at 0.2
is close to invisible against the scene, which was already on the backlog
from the band's own plan.

Both problems point the same way: in View the glyph should be the thing
the user scrubs, drawn in colours that cannot interfere with themselves.

One rule constrains the answer. ADR-0021's second amendment settled that
**a hidden thing answers nothing** — the drawing and the pointer target
go together, because a target the user can hit but cannot see is the trap
ADR-0020 §5 refused. Its converse is what this decision needs: a glyph
that stops drawing its axis segment in View must stop answering from it
too, or View gains an invisible line that steals the pick from the disc
beside it. A prismatic joint is the case that makes this real — it has no
band, so today it is picked by its axis line alone, and dropping the axis
from View leaves it with nothing to aim at at all.

## Decision

**In View a joint glyph is the band (a hinge) or the bars (a slide), the
tick at `q`, and the driven cue — nothing else. The band's three shades
are opaque. What View draws is exactly what View answers.**

1. **View's composition.** `glyph_overlay()` branches on the mode. In
   View it pushes the band or the bars, the white tick at `q`, and — for
   an actuated joint — the cue of §3. It does **not** push the axis
   segment, the pivot dot, the origin triad or the actuator ring. In
   **Edit** every one of those is pushed exactly as v0.4 left it: Edit is
   where a joint frame is placed, and all four answer that question. The
   labels (`driven_marks`) are unchanged in both modes and stay gated on
   the `joint names` toggle — a name is a decoration, not a piece of the
   glyph.

2. **The band is opaque.** The three sectors are drawn in three
   **shades of the glyph's own colour** at full alpha, not in one colour
   at three alphas: `shade()` scales the colour's RGB and leaves alpha at
   255, replacing `layered()`. `RANGE_SHADE` / `LIMIT_SHADE` /
   `VALUE_SHADE` keep the ramp's order and meaning — the full circle
   darkest, the limits over it, the run from the zero position to `q`
   brightest — and the exact three factors are settled by eye on the
   goldens, as their alphas were. An opaque sector drawn over itself is
   the same colour as itself, so the grazing-angle seam cannot exist;
   there is no arrangement of the camera that makes an opaque fill
   double-cover.

   The ramp shades **whatever colour it is handed**, so the one
   `axis_color()` still carries the mimic muting (ADR-0013) and the hot
   brightening: a follower's band is muted at all three shades, and the
   constants stay three rather than multiplying by four.

   This applies in **both** modes. `push_arc` and `push_slide` are one
   code path and the bug is not View's: an Edit camera can graze the same
   plane. Edit's glyph is otherwise untouched.

3. **An actuated joint's cue in View is a filled bore.** A disc at
   `ACTUATOR_RING_RADIUS` in the glyph's own colour, where Edit strokes
   ADR-0014's ring. It is a fill like the band rather than a stroke, so it
   does not read as a second handle; it sits inside the clear bore that
   `BAND_INNER` already guarantees, so it cannot read as part of the band;
   and "solid hub = driven" is legible at a glance without a label.
   ADR-0014's decision that the mark is the ring is amended to: the mark
   is the ring in Edit and the filled bore in View. That an actuated joint
   keeps the **full amber** — an actuator holds a joint the user can still
   pose, where a mimic takes the posing away — is unchanged.

4. **A slide gains a range bar, in both modes.** `push_slide` draws a
   third, dimmest bar spanning the axis segment's own extent
   (`±AXIS_HALF_LENGTH · size`) in `RANGE_SHADE`, under the limit bar and
   the value bar. The hinge's full circle has always been the travel the
   limits are *cut out of*; a slide had no such thing, because it has no
   travel beyond its limits to draw — but it has the axis segment's
   extent, which is the length the glyph already claims on screen. This
   restores the three-shade symmetry between the two kinds, and it is what
   an **unlimited** prismatic joint shows: `limits.unwrap_or((0.0, 0.0))`
   gives it zero-length limit and value bars, so without the range bar a
   slide with no limits would draw a tick and nothing else in View, with
   nothing to aim at.

5. **A `Fixed` joint draws nothing in View.** It has neither band nor
   bars, and under §1 that is the whole of it. Accepted rather than
   special-cased: a weld has nothing to pose, View's joint tree does not
   list it, and it can only be selected in View by carrying the selection
   over from Edit (ADR-0021 §3). Edit still draws it — the axis, the
   pivot and the triad are exactly what a weld has to show while it is
   being placed.

6. **View's hover target is what View draws.** `glyph_distance()` in View
   drops the axis branch entirely: the target is the **band and its
   interior** for a hinge (unchanged — the pointer inside the projected
   centreline or within `GLYPH_HOVER_RADIUS` of it) and the **bars** for
   a slide, by the same polygon-distance-or-interior test over
   `JointGlyph::bar_points()` — the bars' outline, the analogue of the
   band's centreline. A joint with neither answers nothing. In
   **Edit** the target stays the axis segment alone (ADR-0021 §6): the
   mesh behind a glyph is what Edit's tools aim at, and a target that
   swallowed it would take the hover pick away.

   This is ADR-0021 §1 narrowed by its own second amendment's rule. "Only
   joint glyphs answer the cursor in View" becomes "only the parts of a
   glyph View draws answer the cursor in View", for the reason the
   amendment gave: the drawing and the target go together in both
   directions. A prismatic joint is picked by its bars where it used to
   be picked by its axis line, which is also the better target — a bar is
   a quad and a line is a line.

7. **The viewport crate does not change.** ADR-0021 §6 holds once more:
   this is which primitives the app pushes, in which mode, and in which
   colour. `Overlay::sector`, `strip`, `segment` and `point` are what
   they were.

## Consequences

- View is the robot and its joints. A hinge is a ringed disc with a white
  spoke; a slide is a striped bar with a white tick; a driven joint has a
  solid hub. Nothing else amber is on screen, and the triad's red-green-
  blue no longer sits inside the band the user is reading.
- The grazing-angle seam is gone by construction, not by tuning, and the
  range sector is legible instead of nearly invisible — which closes both
  of the backlog lines the band's own plan left behind.
- **An opaque range sector is a full amber annulus over the scene at all
  times**, where a 0.2-alpha one was a suggestion. That is the cost of
  §2, it is paid in every golden with a hinge in it, and the three shade
  factors are where it is controlled. If the human's judgement from the
  images is that the range shade has to go so dark it stops reading as
  amber, the fallback is to drop the range sector's *fill* in favour of
  its two edges — a new amendment, not a return to alpha.
- The ramp scales RGB toward black, so it is calibrated against the dark
  viewport ground (`0.09, 0.10, 0.12`). In the light theme the same three
  factors read as three browns on a pale ground: still monotone, still
  ordered, higher contrast than the dark theme's. Accepted — the goldens
  are dark, the ramp's *order* is what carries the meaning, and a
  per-theme ramp would double the constants for a difference the user
  never sees both sides of.
- Every golden that carries a band moves, and the View goldens lose four
  pieces of glyph. That refresh is this decision's largest cost and is
  taken once, with the images shown to the human (ADR-0003).
- `GlyphDebug` gains `drawn` — the pieces the glyph contributed this
  frame — so the composition is asserted as JSON and not only as pixels,
  and a future mode branch that forgets a piece fails a scenario instead
  of a golden's eye.
- A user who wants to see a joint's frame in View cannot; `Tab` is the
  answer, and it is the same `Tab` that answers every other "I want to
  edit this" in the window.

## Alternatives considered

- **Keep every piece in View and only fix the alphas.** The seam is the
  reported bug and opacity fixes it alone. But the four pieces are not
  drawn for View's question, and leaving them would keep the triad
  competing with the band for the eye — and would leave the prismatic
  joint's pick on an axis line that is a poor target in a mode where it
  is the *only* target. The two changes are separable and are taken
  together because the second is what makes the first worth refreshing
  the goldens for.
- **Drop the axis segment in View but keep answering from it.** Cheapest
  by far — no `bar_points()`, no new test. Rejected outright: it is
  precisely the invisible-target trap ADR-0021's second amendment and
  ADR-0020 §5 both refused, and the slide would be pickable only on a
  line nothing draws.
- **Sort the band's sectors back-to-front and keep them translucent.**
  A depth sort would make the stack correct at a grazing angle without
  giving up translucency. It puts a per-frame sort of overlay primitives
  into the app for one shape, it is exact only while the sectors do not
  interpenetrate, and it does nothing for the second complaint — the
  faint shades are faint because they are faint, not because they are
  mis-ordered.
- **A second colour for the driven cue instead of a filled bore**, e.g.
  the tick in cyan. No new shape at all, but it overloads the one piece
  that means "the value is here" with a second meaning, and it collides
  with the mimic muting, which is already the colour channel's job
  (ADR-0013).
- **No driven cue in View at all**, leaving it to the joint tree's row.
  Tempting — View's composition would then be two pieces for every hinge.
  Rejected because zen (ADR-0021, third amendment) draws no tree at all,
  and "is this joint driven" would become a question the window cannot
  answer in the mode built for looking at the robot.
- **Give the slide bars a fixed screen length instead of the axis
  segment's extent.** Constant on-screen size is a better target, but
  every other measure in the glyph is a fraction of `size` (§`glyph_size`)
  so that a glyph is the size of the part it belongs to; one piece
  measured in points would breathe against the rest under a zoom.
