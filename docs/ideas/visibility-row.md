# Idea: visibility-row

- Status: Open
- Raised: 2026-09-05
- Prompt (verbatim from the human): "visibbility row"

## Problem

Everything riggen draws over the robot is drawn always. On the sample arm
that is three joint glyphs with bands, three driven-joint labels, two frame
triads with names, and the meshes under all of it — and the bands are
sized from the parts, so a dense assembly is a haystack of amber. There is
exactly one thing the user can turn off today: View › Collision geometry,
a checkbox in a menu (`app/mod.rs::menu_bar`, `SHOW_COLLISION_KEY`).

Two moments hurt. **Looking at the robot**: View exists to look at the
posed model, and its own glyphs are what stand in front of it — the very
thing zen mode on `Z` is being added for, except zen is all-or-nothing and
hides the panels too. **Working on one joint in Edit**: the neighbouring
glyphs and frame labels sit over the part being aimed at, and the Place
joint / Align tools aim at meshes through them.

The roadmap already owns the line (03 §The window): *"One row of icons,
Blender's overlay toggles for riggen's things: joints, joint names, links,
frames, collision geometry — View › Collision geometry moves out of the
menu into it, and the corner is the one the Joints window vacates."* This
is the brainstorm for what the row means, not whether to have it.

## Constraints it runs into

- **ADR-0021 §1 and §6**: in View, *only joint glyphs answer the cursor* —
  `set_pick_suppressed` is on unconditionally, a click on a glyph selects
  its joint, and the wheel over a hovered glyph poses it. Turn joints off
  and View has **no pointer target left at all**. The row cannot be
  specified without saying what that means; it is an amendment to
  ADR-0021's switch table, not a UI detail.
- **ADR-0020 §5's own reasoning**, which is the strongest in-repo
  precedent here: a run behind geometry is dimmed rather than dropped
  because *"a run drawn narrower or not at all would be a target the user
  can hit but cannot see."* A toggle that hides drawing while leaving the
  target live contradicts it.
- **ADR-0003**: every visible UI state gets a snapshot. Five independent
  toggles is 32 states; the row needs a scenario budget, not a matrix.
- **`.riggen` is schema 3** (02): visibility is a view setting and must
  not enter the document. `show_collision` already shows the shape —
  eframe storage, not serde.
- **01 §Panels, `mode.rs::viewport_chrome`**: the corner chrome is one
  `toolbar_rect`, and under it the camera is blocked and picks suppressed.
  A second chrome rect wants the same treatment and `toolbar_rect` is
  currently a single `Option<Rect>`.
- **A live conflict over the corner.** `docs/BACKLOG.md` (from
  plans/panels-and-numbers) says the Materials window *"opens over the
  corner chrome … anchor it at the top-right instead, the corner the
  Joints window vacated"* — the same corner the roadmap gives the row.
  One of the two lines has to move.
- **There is no icon set.** egui bundles `NotoEmoji-Regular` and
  `emoji-icon-font` and nothing designed for small monochrome UI toggles;
  `glyphs.rs` already records the lesson (`»` and not `↳`, *"a mark that
  renders as a tofu box says nothing at all"*). "A row of icons" is not
  free.
- No SEED non-goal is touched. The layer rule is untouched: the viewport
  already has `set_instance_visible`, and glyphs are built app-side.

## Options

### A — A row of self-drawn toggles; hidden means gone, target and all

Five toggle buttons in the viewport's top-right, drawn with `egui`'s
painter as riggen's own idiom at icon size — a band-and-spoke for joints,
a box for links, a small triad for frames, a dashed box for collision, and
a text `A` for joint names. Hiding filters at the source: `joint_glyphs()`
and `frame_glyphs()` return nothing, so drawing *and* `glyph_at` go dark
together and there is never a target that cannot be seen; links and
collision go through `set_instance_visible`, as collision already does.
The state is app state persisted like `SHOW_COLLISION_KEY`, shared by both
modes, and the status bar says what is hidden so an empty viewport is
never a mystery.

Trade-offs: the icons are ours to draw and ours to keep legible at 16 px,
which is real work and real snapshot surface — but they are also the only
option that shows the user the exact mark they will see in the scene, and
they cost no new asset, no font licence and no wasm payload. Turning
joints off in View leaves the mode with nothing to click, which is
coherent (that *is* "just show me the robot") but must be stated in
ADR-0021 rather than discovered.

Cost: ~5 plan steps — the row and its chrome rect; the five toggles
plumbed to the two seams; the ADR-0021 amendment; goldens; the menu item
and the Materials-window corner moved.

Forecloses nothing; a real icon font can replace the drawings later
without changing the seam.

### B — The same row, but hiding is drawing-only

A hidden joint is still hoverable, clickable and wheel-poseable; the row
is purely an overlay declutter. Cheaper by a step (nothing touches
picking, no ADR amendment) and it keeps View's one gesture alive with the
glyphs off.

Trade-offs: it is the thing ADR-0020 §5 argued against, one layer up — the
user hides the joints, mouses over the arm, and the wheel starts turning a
shoulder they cannot see. Recoverable but bewildering, and it makes
"hidden" mean two different things depending on which toggle it is
(a hidden *link* cannot be picked, because the pick buffer has nothing in
it; only the app-side glyph test would keep answering).

Cost: ~4 plan steps.

### C — No row: grow the View menu

Four more checkboxes beside Collision geometry. Costs one step, needs no
icons, no corner, no chrome rect, and no ADR.

Trade-offs: a toggle two clicks deep in a menu is a toggle nobody flips
while looking at the thing they want decluttered — which is the whole
gesture. It also leaves the top-right corner empty and the roadmap line
unspent, and Blender's row is the idiom researchers already have in hand.

### Do nothing

Zen on `Z` lands anyway and covers the extreme case ("show me the robot
alone"), so the cost is the middle: no way to keep the joint tree and drop
the frame labels, no way to work in Edit with the neighbouring bands off.
The roadmap line stays open into the next cycle and the corner stays empty.

## Recommendation

**A.** The row is worth the corner, and "hidden means gone, target and
all" is the only rule that stays consistent with the reason ADR-0020 dims
instead of dropping. Filtering at `joint_glyphs()` / `frame_glyphs()`
makes it one seam rather than five, which is why A costs barely more than
B; the difference is the ADR amendment, and that amendment is the point —
View losing its pointer target when joints are off is a rule, not a bug to
find in a snapshot later.

B loses because it invents a second meaning of "hidden" that differs per
toggle. C loses because the gesture is "declutter what I am looking at",
and a menu is the wrong distance from the viewport.

What would change my mind: if the self-drawn icons look like mud at 16 px
in the first golden, the row should fall back to short text labels
(`joints · names · links · frames · collision`) in the same corner rather
than acquire an icon font — the row is five items, and text that reads
beats an icon that doesn't.

Two details taken as decided unless you say otherwise: the toggles are
remembered across sessions the way collision already is; and a hidden
link still *sizes* its joint's glyph (`glyph_size` reads bounds, not
visibility), because the part has not stopped existing.

## Decision for the human

1. **What does a toggle turn off — drawing and the pointer target
   together (A), or drawing only (B)?** Preferred: **both**, with the
   consequence written into ADR-0021 as an amendment (§1 and §6 gain "and
   nothing answers the cursor when joints are hidden"). This is the one
   question that needs an ADR touch; the rest is UI.
2. **The top-right corner: the row, or the Materials window?**
   Preferred: **the row** — it is chrome, one row tall, present always;
   the Materials window is occasional and can anchor under it. The
   backlog line from plans/panels-and-numbers gets rewritten rather than
   removed.
3. **Icons drawn by us, or short text labels?** Preferred: **drawn by
   us**, with text as the named fallback if the first golden is mud. No
   icon font either way.
4. **Is "links" a toggle in Edit too, or View-only?** Preferred: **both
   modes** — hiding links in Edit means the mesh-aiming tools have nothing
   to hit, which is the same self-consistent rule as (1), and the status
   bar says so. Say View-only if you would rather Edit never lose its
   targets.
5. **One plan with zen on `Z`, or two?** Preferred: **two** — the row
   first, zen after, since zen's panel list has to name the row. They are
   both small; one plan is defensible if you would rather spend one cycle
   of review on the pair.
