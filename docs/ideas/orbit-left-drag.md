# Idea: orbit-left-drag

- Status: Open
- Raised: 2026-09-02
- Prompt (verbatim from the human): "/idea orbit-left-drag" — the v0.3 line
  "Orbit on left-drag with click-to-select still working (an idea first: the
  rule is the hard part)".

## Problem

The camera orbits on **middle**-drag only (`viewport/mod.rs`
`handle_input`: middle = orbit, shift+middle = pan, wheel = zoom). A
left-drag over the viewport does nothing at all. Three groups hit that:

- **Trackpad users.** A MacBook has no middle button. On the web demo —
  the first thing most people now see — such a visitor can zoom the sample
  arm and select its parts but cannot turn it. The README's "orbit with the
  middle mouse button" is the only hint, and the demo page is not the
  README.
- **People arriving from MuJoCo.** `simulate` orbits on left-drag and pans
  on right-drag, as do rerun, three.js `OrbitControls`, Sketchfab and every
  browser viewer the audience has met. Riggen's own layout — a viewer with
  a tree beside it — reads as one of those, so the first gesture is a
  left-drag, and the puzzled minute begins there.
- **Every by-hand run since M2** listed it; the roadmap opens v0.3 with it.

The reason it was never a one-liner: the left button already means
*select* (`response.clicked()` → the select pick), *place* (a snapping
tool's click), and *grab a handle* (the gizmo, ADR-0010). The rule has to
say who owns a left press before anyone knows whether it will become a
drag.

## Constraints it runs into

- **ADR-0010, the pointer policy.** The gizmo's widget senses **clicks
  only**, precisely so that a *drag* from a handle falls through to the
  viewport and orbits — `orbit_works_from_a_gizmo_handle` pins it. That was
  right while "drag" meant middle-drag. Once a left-drag orbits, the same
  fall-through means a left-drag on a handle orbits the camera *and* moves
  the part: the gizmo reads the raw pointer and never looks at its widget.
  The three switches (`set_pick_suppressed`, `set_select_suppressed`,
  `set_pointer_blocked`) each say something about picks or about the whole
  pointer; none says "the primary drag is spoken for, the camera may still
  take the middle one". Whatever the rule is, it amends that table.
- **The one-frame lag** (ADR-0010 §Consequences): the gizmo cannot say it
  owns the cursor until it has run, and the viewport runs first. Any
  "is a handle under the press?" test the viewport makes is last frame's
  answer. Benign for hover, and benign here too *only if* the press frame
  is handled from the flag the previous frame set — which it is whenever the
  cursor was resting on the handle before the press, i.e. always for a
  human. A scripted test that teleports the cursor and presses in one frame
  would see the lag; the harness already drives drags in steps.
- **egui's click/drag arbitration.** egui decides "click" versus "drag" for
  us: `clicked()` fires on release only if the pointer stayed within
  `max_click_dist` (6 points by default) and under `max_click_duration`;
  `dragged_by(Primary)` fires once either is exceeded. So click-to-select
  survives a left-drag orbit with no code of ours: a still press selects, a
  moved press orbits, and a press that jitters seven points orbits without
  selecting — the same trade every web viewer makes. The plan should pin
  those two constants in a test rather than trust the defaults. (Verify the
  values against the pinned egui 0.36; they are `InputOptions` fields.)
- **Snapping tools** (01 §Picking and snapping): a click means "put it
  here" and the select pick is suppressed. A left-drag there today does
  nothing; orbiting is strictly better and needs no rule.
- **Joint and frame glyphs**: they set `set_pick_suppressed` while under
  the cursor so the click reaches the glyph, not the geometry. A drag from a
  glyph should orbit like a drag from anywhere — so "pick suppressed" must
  *not* be what withholds the left drag; only the gizmo's claim may.
- **What it forecloses.** Left-drag on empty space is the CAD idiom for
  **box select** (Onshape, Fusion, Blender). Riggen has single selection
  only, and SEED.md's non-goals list no multi-select; nothing in the
  roadmap asks for it. Giving the left drag to the camera closes that door
  unless a modifier reopens it later (shift+left-drag is free).
- **Touch** is a backlog line (demo gap). A one-finger drag arrives in egui
  as a primary drag, so this rule is also the touch orbit for free; it does
  not decide pinch or two-finger pan, which stay backlog.
- **Layer rule**: `riggen-viewport` decides camera input from egui's
  response alone and knows nothing of gizmos; the claim must reach it as a
  switch set by `riggen-app`, as the three existing ones do.

## Options

### A — Left-drag orbits, right-drag pans, middle keeps everything it has; a handle claims the left button

The MuJoCo / rerun / three.js mapping, added on top of today's rather than
replacing it: left = orbit, right = pan, shift+left = pan (trackpads with
no right button), middle and shift+middle unchanged, wheel unchanged.
Click-to-select is egui's click threshold. The gizmo, while a handle is
under the cursor or its drag is in flight, tells the viewport the primary
drag is claimed; the viewport then ignores `dragged_by(Primary)` but not
middle — `orbit_works_from_a_gizmo_handle` keeps passing as written.
Nothing under the cursor withholds a right-drag.

Trade-offs: matches the audience's muscle memory; one new switch in the
ADR-0010 table (a fourth row, "the primary drag; middle still orbits");
forecloses box select on a bare left-drag. Right-click has no meaning in
the viewport today, so right-drag pan conflicts with nothing; a future
context menu would have to be click-only, which is how egui context menus
already behave.

Cost: ~3 plan steps (the switch and the viewport rule; the bindings and
their snapshot scenarios; README + 01 + the ADR-0010 amendment).

### B — Left-drag orbits only over empty space; over a part it does nothing (or box-selects later)

The CAD idiom kept half-open: a drag that starts on geometry is reserved.
Trade-offs: the decision depends on the hover pick, which is asynchronous
and one frame late, so the same gesture would sometimes orbit and sometimes
not depending on where the readback was — exactly the kind of inconsistency
v0.3 exists to remove. And the demo visitor's first drag is on the arm, not
beside it. Cost: ~4 steps and a rule nobody can state in one line. Loses.

### C — Modifier orbit: alt+left-drag (Maya, Fusion's "emulate")

Zero conflicts with select, place or gizmo. Trade-offs: undiscoverable —
the trackpad visitor still cannot turn the arm until they read something,
which is the problem restated; alt is the browser's menu key on some
platforms. Cost: 1 step. Loses on the problem statement.

### D — A preference: "left-drag orbits" on/off (Blender's "emulate 3 button mouse")

Option A behind a toggle, default on. Trade-offs: riggen has no persisted
preferences at all today; a settings surface for one boolean is a new kind
of thing, and the demo has no place to persist it. Cost: A + 2 steps for
the surface. Loses until a second preference exists.

### Do nothing

The demo stays unorbitable on a trackpad; every by-hand run keeps listing
it; v0.3's first bullet stays open. Costs nothing in code and the first
impression in full.

## Recommendation

**A.** It is the mapping the audience already knows, it costs three steps,
and it is additive: nobody who orbits with the middle button today loses
anything. The one real design content is the fourth switch — the gizmo
claims the *primary drag*, not the pointer and not the picks — and that is
a one-paragraph amendment to ADR-0010's table, not a new architecture.
Box select is the only thing given up, and it is not on any list.

What would change my mind: a decision that multi-select is coming in v0.3
or v0.4. Then the bare left-drag should stay reserved and the answer is
"C now, revisit" — orbit on alt+left plus a visible hint in the viewport
corner.

## Decision for the human

1. **Left-drag orbits the camera, right-drag pans, and egui's click
   threshold is the select/orbit arbiter?** Preferred: yes (A). The
   alternative worth naming is C, if box select must stay possible.
2. **Shift+left-drag also pans?** Preferred: yes — it is the only pan a
   one-button trackpad has, and shift already means pan on the middle
   button.
3. **Is an ADR needed?** Preferred: yes, a short one — ADR-0018 amending
   the ADR-0010 pointer table with the fourth switch and recording that the
   bare left-drag belongs to the camera, which is the decision box select
   would later have to overturn. The roadmap already budgets for exactly
   this ADR and no other in v0.3.
