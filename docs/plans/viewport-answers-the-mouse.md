# Plan: viewport-answers-the-mouse

- Started: 2026-09-03
- Milestone: v0.3 — the hand-feel debt
- Idea (verbatim from the human): "the remaining mouse items" — the three
  left in `docs/03-roadmap.md` §v0.3 under **The viewport answers the
  mouse**: "Keyboard shortcuts for the five tools; the rotate gizmo on the
  wheel; snapping *during* a gizmo drag, not only in the Align tool."

## Goal

The five tools have keys, the rotate gizmo answers the wheel, and a Move
drag snaps to the same features the Align tool does. Pressing `V G R J B`
switches tool without reaching for the toolbar (and the toolbar says so);
hovering one of the three rotate rings and turning the wheel steps the
target's rotation about that ring's axis by 5° (1° with shift), with the
whole burst of notches landing as **one** history entry; and while a link
is being dragged by a translate handle the snap ladder runs under the
cursor, so the drag can be dropped onto a vertex, a box corner or a bore
centre with the cyan marker and readout that placement already draws. When
this is done the only item left in the v0.3 bullet list is **The overlay
tells the truth**.

## Non-goals

- **Rotate-drag snapping** — a rotate drag keeps behaving exactly as it
  does; only translation snaps (the human's call at planning time).
- **Grid / angle increments as a general facility.** The wheel's step is
  the wheel's step; there is no document-wide snap quantum, and none of
  the export formats grows a field.
- The other v0.3 bullet, **the overlay tells the truth** (depth testing,
  driven/actuated badges): a separate plan.
- Box select, touch gestures, a ViewCube, WASD fly — backlog, and ADR-0018
  already foreclosed the first.
- Any change to what a gizmo drag *commits*: still one `SetJoint` /
  `MoveJointFrame` / `SetFrame` per gesture, through `commit_gizmo`.

## Design deltas

- **`riggen-viewport`, the pointer contract.** Two channels the app cannot
  express today, both recorded in ADR-0019 (step 2):
  - `Viewport::set_wheel_claimed(bool)` — a fifth switch beside ADR-0010's
    three and ADR-0018's fourth. Turns off **zoom only**; the camera drags,
    both picks and the standard-view keys stay live. Set while a rotate
    ring is under the cursor.
  - `pointer_blocked` narrows to the camera. Today one flag means "no
    camera **and** no picks", and a gizmo drag sets it — which is exactly
    what kills the hover pick the snap ladder needs. It becomes
    `set_camera_blocked(bool)`, and pick suppression is left to the two
    pick switches that already exist; the toolbar sets camera-blocked
    *and* `set_pick_suppressed`, a gizmo drag sets camera-blocked alone.
    No behaviour changes for the toolbar.
- **`riggen-app/src/app/gizmo.rs`.** A pure `ring_under_cursor(...)
  -> Option<RingAxis>`: `Gizmo::pick_preview` answers only *whether* a
  handle is hit and the crate's subgizmos are private, so which ring is
  under the cursor is ours to compute — a ray/plane test per local axis
  against the same arc radius and focus distance the crate uses
  (`scale_factor = mvp[15] / proj.x.x / rect.width * 2`, radius =
  `scale_factor * gizmo_size`, tolerance = `scale_factor * (stroke/2 + 5)`;
  `transform-gizmo-0.11.0/src/subgizmo/rotation.rs`). Plus `GizmoState`
  bookkeeping for the wheel gesture, on the `WHEEL_BURST` idiom the
  Properties scrubbers already use.
- **`riggen-app/src/app/snap.rs`.** `compute_snap` stops bailing on
  `gizmo_state.captured` while a **translate** drag is in flight, and drops
  a hit that belongs to the dragged link's own subtree — a part must not
  snap to itself. `Tool::snaps()` gains the drag case through
  `RiggenApp::snapping()`, not through the tool.
- **`riggen-app/src/app/tool.rs` + `shortcuts.rs`.** `Tool::shortcut() ->
  egui::Key`, consumed in `handle_shortcuts` with `Modifiers::NONE` and
  yielding to a focused text field like every other bare key. `V` Select,
  `G` Move, `R` Rotate, `J` Place joint, `B` Align — Blender's `G` = grab
  and `R` = rotate, with `V` and `B` standing in for the initials because
  **`W A S D E Q` are reserved for the fly camera** the backlog wants
  ("WASD fly mode", from the M2 exit gate); a tool must not spend one of
  them. The keys also avoid what the viewport already owns — `Num1/3/5/7/0`
  (standard views), `P` (projection), `Home` (fit) — which is what rules
  out the digits.
- **`debug_state`**: `gizmo.hovered_ring` (`"x" | "y" | "z"`), and
  `input.wheel_claimed` beside the other four switches, absent while off so
  existing goldens are untouched.
- **ADR-0019** — the wheel can be claimed, and a translate drag keeps the
  hover pick. Amends ADR-0010 §Decision 3 and ADR-0018 §Decision 3.

## Steps

- [x] Step 1 — **Which ring is under the cursor.** `ring_under_cursor` in
      `gizmo.rs` as a pure function of (view, projection, rect, gizmo world
      pose, visuals, cursor), unit-tested with hand-built matrices and no
      GPU — cursor on the X ring, on the Y ring, in the middle, far outside,
      and on a ring seen edge-on. Reported as `gizmo.hovered_ring` in
      `debug_state`, pinned by a scenario that parks the cursor on a ring of
      `gizmo_rotate_joint`'s gizmo, and by a unit asserting `None` on the
      crate's outer **view** ring, whose radius is only `stroke + 5` larger
      than a face-on local ring and which the wheel must never claim.
      Nothing yet reacts to it. *This is the
      plan's one real unknown — a reimplementation of crate-private geometry
      that has to agree with the ring the crate draws hot — so it retires
      first.*
- [x] Step 2 — **ADR-0019.** The two contract rows above, their
      alternatives (Ctrl+wheel instead of the bare wheel, which
      `raw_wheel_delta_y` deliberately skips today and which the Properties
      scrubbers have already spent; a sixth switch instead of narrowing
      `pointer_blocked`), and the consequence that zoom stops working over a
      ~15 px band of the viewport while Rotate is active. Docs only.
- [ ] Step 3 — **The wheel steps the hovered ring.**
      `Viewport::set_wheel_claimed` and its skip in `handle_input`; the app
      claims the wheel while `ring_under_cursor` is `Some` and a rotate
      gizmo has a target — the three local rings only, never the view ring,
      where the wheel goes on zooming. Each notch rotates the target's world
      pose about that ring's **local** axis by **5°**, or **1°** while shift
      is held, and commits through the same `commit_gizmo` a drag uses; notches within `WHEEL_BURST` coalesce into
      one history entry via `History::apply_in_gesture`, so one burst is one
      undo. Tests: a scenario turning the wheel three notches on a joint's
      Z ring asserts the joint's axis/origin and `history` depth **1**, one
      more that a pause between notches makes two entries, and one that the
      wheel still zooms with the cursor a ring-radius away.
- [ ] Step 4 — **A translate drag snaps.** `set_camera_blocked` replaces
      `set_pointer_blocked` (toolbar behaviour unchanged); during a
      translate gizmo drag the hover pick stays live, `compute_snap` runs,
      hits inside the dragged subtree are ignored, and the previewed pose's
      translation is replaced by the snap point — so it is the **gizmo's own
      origin**, the link frame's origin, that lands on the snapped feature,
      the same anchor `place_frame` uses — with its rotation untouched, so
      the release commits the snapped pose. The existing cyan marker and
      readout draw as they do for Place joint. Tests: a scenario dragging a
      link's translate handle to within `SNAP_PIXEL_RADIUS` of another
      part's vertex asserts the committed origin equals that vertex to 1e-6
      and that it is one history entry; a snapshot of the marker mid-drag;
      and `snapping_is_off_outside_the_placement_tools` extended to say that
      a *rotate* drag still does not snap.
- [ ] Step 5 — **Tool shortcuts.** `V` Select, `G` Move, `R` Rotate, `J`
      Place joint, `B` Align, consumed in `handle_shortcuts`
      before the panels, ignored while a text field has focus, and shown in
      each toolbar button's tooltip so the binding is discoverable rather
      than folklore. `Esc` keeps its meaning (back to Select) unchanged.
      Tests: a scenario pressing each key asserts `debug_state.tool` and
      that entering an editing tool still rewinds `q` (`ZERO_CONFIG_STATUS`);
      one that a focused rename field swallows them; a refreshed `toolbar`
      snapshot.

## Acceptance

`cargo test -p riggen-app` green with the new scenarios
(`ring_under_cursor` units, `hovered_ring`, `wheel_steps_the_hovered_ring`,
`a_wheel_burst_is_one_history_entry`, `gizmo_drag_snaps_to_a_vertex`,
`a_rotate_drag_does_not_snap`, `tool_shortcuts_switch_tools`), plus the
v0.3 gate: the M2 arm build run by hand end to end adds no new entry to the
hand-feel list for these three gestures.

## Docs to update on completion

- `docs/adr/0019-*.md` — written in step 2, nothing to do at retirement.
- `docs/01-architecture.md` §Frame loop — the five switches and who sets
  them; §Picking and snapping — the snap ladder runs during a translate
  drag and skips the dragged subtree; §Panels and menus — the tool keys.
- `docs/03-roadmap.md` §v0.3 — fold the three items into the status
  paragraph, leaving **The overlay tells the truth** as the last bullet,
  and correct the section's closing line, which currently says ADR-0018 is
  the only ADR this cycle expects.
- `README.md` §The first minute step 1 — the tool keys and the ring wheel
  beside the mouse bindings ADR-0018 put there.
- `AGENTS.md` current state — what is left of v0.3.

## Open questions

None left open at planning time. The four this plan opened were put to the
human on 2026-09-03 and answered:

- **The keys** are `V G R J B` — Blender's `G`/`R`, with `V` and `B` for
  Select and Align, because `W A S D E Q` are reserved for the fly camera
  the backlog wants and a tool key must not spend one of them.
- **The view ring** is not claimed: the wheel keeps zooming over it. A step
  about the camera's forward axis is not one the user can predict or repeat.
- **The step** is 5° per notch, 1° with shift. Ctrl is unavailable as the
  fine modifier — egui's zoom modifier is stripped from the viewport's wheel
  by `raw_wheel_delta_y`, and Ctrl+wheel already means "step this number" in
  Properties.
- **The snap anchor** is the gizmo's origin, matching `place_frame`.

One decision the agent took rather than asking: this cycle gets a **second
ADR** (ADR-0019, step 2) even though §v0.3 of the roadmap says ADR-0018 is
the only one it expects. The switch table is a published contract that two
ADRs already state and amend; a third change to it belongs in the same
place, and the roadmap sentence is corrected at retirement (it is in the
docs-to-update list). Say so if you would rather it stayed prose in
`docs/01-architecture.md`, and step 2 drops.
