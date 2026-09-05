# Idea: view-edit-modes

- Status: Open
- Raised: 2026-09-05
- Prompt (verbatim from the human): "2 modes: View and Edit, user can switch
  between them by pressing Tab; when user opens model - View mode activated
  … view/edit split with view as default - I think it is the first thing we
  need to implement before anything else"

## Problem

The window has one mode, and it is the editor's. `riggen robot.urdf`, a
dropped `.riggen`, the web demo's sample arm — every one opens into five
placement tools, a link tree, a properties panel, and a floating Joints
window over the viewport that is the only way to pose the robot. SEED §3
says the common case is editing an existing model rather than building
one; before either, it is *looking* at it — does the imported arm move the
way the file says, are the limits right, which way does joint 3 turn. That
question is answered today by finding a slider in a window that covers the
robot, while a click on the robot selects a mesh and the wheel zooms away
from the joint under the cursor.

The two activities also want opposite pointers. Posing wants the joint to
be the only thing under the cursor and the wheel to turn it; editing wants
the mesh under the cursor, the gizmo to claim the left drag, and — because
every frame-rewriting command works in the zero configuration (01 §Panels,
plans/m2-placement-ux OPEN 1) — the robot *un*-posed. Today the second wins
by default and the first is a window. Everything else in 03 §The window
(the joint tree, joints-only picking, the wheel on a glyph, the glyph plan's
hover target, the visibility row, zen) needs to know which mode it is in,
which is why this is the first line.

## Constraints it runs into

- **The switch table** (01 §Picking and snapping; ADR-0010, amended by
  ADR-0018 and ADR-0019): five switches, one channel each, set by the app
  every frame. A mode is a *policy* on them, not a sixth switch: View sets
  `set_pick_suppressed` unconditionally (both picks off, camera live — the
  glyph hover is the app's own screen-space test and is untouched) and
  `set_wheel_claimed` while a glyph is hovered, the way a rotate ring
  claims it. The viewport crate does not change; the layer rule holds.
- **The zero-configuration rule** (01 §Panels, 02 §Commands): the four
  editing tools reset `q` on entry, not as a history entry, with a status
  line. A mode whose whole purpose is frame editing collides with that rule
  head-on — either Edit *is* the zero configuration or it keeps the per-tool
  snap. `Reparent` is the one command allowed off zero, decided as a
  paragraph in 02 on 2026-09-02 because "a tree edit is made while posing".
- **ADR-0019 §2**: the wheel over a ring is a *committing* gesture, one
  history entry per burst. Over a glyph it would pose `q`, which is derived
  state and never a command (01 §The document is the only state) — so no
  history entry, like the sliders today. Two wheels, two semantics, told
  apart by mode.
- **Tool keys** `V G R J B` (ADR-0019, `tool.rs`) and `Esc` back to Select:
  Edit's vocabulary. `Tab` is egui's focus traversal and goes through
  `consume_key` in `handle_shortcuts`, yielding to a focused `TextEdit` as
  every bare key does.
- **The Joints window's open-itself rule** (plans/panels-and-numbers OPEN 3,
  01 §Panels) exists because posing had no home; the joint tree gives it
  one and the rule is deleted with the window.
- **ADR-0003**: nearly every golden opens a document with a movable joint
  and would now open in View; every tool scenario has to `Tab` first. This
  is the single largest cost and the reason the human's ordering is right —
  the glyph plan's step 2 refreshes the same goldens and should land after
  this, not before (its step 1, the viewport primitive, is independent).
- **The web demo** (01 §The web build) opens the sample arm — the first
  thing a visitor sees becomes View, which is the point.
- **SEED §What not to spend agent time on**: docking UI. A mode is not
  docking — the panel set per mode is fixed, nothing is dragged.
- **ADR-0006**: a mesh drop is a link. A drop is building, not looking.

## Options

### A — Two modes; Edit is the zero configuration (Blender's armature split)

**View** poses: the left panel is the joint tree (movable joints in
kinematic order, fixed joints collapsed through, a follower's row read-only
at its resolved `q` with the rule under it, ADR-0013), each row a scrubber
that drags and steps by the wheel at the ring's 5° / 1° quantum
(ADR-0019 §2 — one rotation quantum everywhere), showing value and both
limits; the properties panel and the toolbar are gone; only joint glyphs
answer the cursor, a click selects the joint, the wheel over it steps it.
**Edit** is the v0.3 editor at `q = 0`: `Tab` into it stashes `q` and
rewinds to zero (the same non-history rewind `set_tool` does today), `Tab`
back restores it. The per-tool reset and its status line become dead code
and are removed; `Reparent`'s `at` is the zero configuration again in the
GUI, its off-zero form kept for the SDK. A `View | Edit` segmented control
sits at the viewport's top-left (the toolbar, in Edit, to its right), with
`Tab` in its tooltip — a shortcut nobody can find is folklore (01). The
status bar names the mode. Opening a document, an import, the demo → View;
File › New and a mesh drop → Edit, because there is nothing to pose yet
(the open-itself rule's judgement, moved to the mode).

Trade-offs: the cleanest contract — "View is the posed robot, Edit is the
rest pose" is a sentence, and it is the one Blender users already know
(Pose Mode shows the pose, Edit Mode the rest position). The robot visibly
snaps to zero on `Tab`, once, instead of snapping on every tool entry with
a status line explaining it. The pose is never lost: it comes back with
View. It changes a standing decision (the per-tool reset, the 02 paragraph
on `Reparent` at `q`) — which is what makes this an ADR. Cost: ~7 plan
steps plus the ADR and a snapshot-suite migration step.

### B — Two modes; Edit keeps the pose, `q` locked

Same View. Edit shows the robot *as posed* with the joint rows read-only,
and keeps today's per-tool reset: entering Move snaps to zero, `Esc` back
to Select leaves it there, and the pose is gone until View re-poses it by
hand. `Reparent` at the current `q` stays meaningful.

Trade-offs: no motion on `Tab`, which is gentler; but Edit then has two
states (posed, then zero after the first tool) and a robot that jumps for a
reason the user has to read. The pose is lost on the first tool. This is
the human's wording read literally ("value is not editable"), and it keeps
the "why did it jump" minute that v0.3 was about removing. Cost: ~6 steps
plus the ADR (the mode contract is still a decision).

### C — No mode: a Joints tab in the left panel and a pose lock

The left panel gains a second tab, Joints, with the tree of scrubbers; the
floating window goes; a hovered glyph claims the wheel always; meshes stay
pickable; a toggle locks `q`.

Trade-offs: cheapest (~4 steps), no ADR, no golden migration beyond the
panel. But it does not deliver the part that matters — joints as the only
thing under the cursor, the toolbar and properties out of the way, the
demo opening as a viewer — and the wheel-claim-by-glyph then competes with
the rotate ring's claim in the same mode. It is the window relocated.

### Do nothing

The Joints window stays, the demo opens an editor, and each of the other
five lines in 03 §The window has to decide for itself what "in View" means
— which is the mode, written five times.

## Recommendation

**A.** Edit as the zero configuration is what every editing tool already
requires; making the mode carry it removes the per-tool snap and its
explanation rather than adding a second one, and it is the split the
reference tool uses. B is A with the pose lost on the first tool. C does
not change what the cursor does, which is the request.

Order: this plan before the glyph plan's step 2 and before every other line
of 03 §The window, since they all ask "which mode". The glyph plan's step 1
can go either way.

What would change my mind: if the visible snap to zero on `Tab` into Edit
reads as breakage in the by-hand run — then B's "posed until a tool is
picked" is the fallback, with A's stash-and-restore kept so the pose still
comes back on `Tab`.

## Decision for the human

1. **Edit is the zero configuration, `q` stashed on `Tab` and restored on
   the way back?** Yes (A); or Edit keeps the pose and the per-tool reset
   (B).
2. **In View, the properties panel and the toolbar are hidden — the joint
   tree is the only panel?** Yes; the joint's numbers are on its row and
   the visibility row (03 §The window) is the only viewport control.
3. **Open rule: a document, an import, the demo open in View; File › New
   and a mesh drop open in Edit?** Yes; or always View.
4. **Tool keys in View do nothing but hint "Tab to edit" in the status
   bar?** Yes; or they switch to Edit and pick the tool.
5. **A visible `View | Edit` control top-left with `Tab` in its tooltip?**
   Yes.
6. **An ADR?** Yes — ADR-0021, the mode contract: what each mode owns
   (picks, wheel, panels, `q`), Edit as the zero configuration superseding
   the per-tool reset paragraph and narrowing 02's `Reparent`-at-`q`
   paragraph to the SDK.
