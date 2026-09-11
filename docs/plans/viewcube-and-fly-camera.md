# Plan: viewcube-and-fly-camera

- Started: 2026-09-11
- Milestone: v0.5 — the viewport and the camera
- Idea: `docs/ideas/viewcube-and-fly-camera.md` (absorbed)
- Idea (verbatim from the human): "next thing on roadmap"

## Goal

You always know which way you are looking, and you can get inside an
assembly instead of only orbiting round it. A **ViewCube** sits in the
viewport's bottom-right — RoboCAD's, ported into `riggen-app` as an egui
painter widget over the 26 `ViewOrientation` variants `orientation.rs` has
held since M0 — and clicking a facet animates to that view, dragging it
orbits, its Home button fits, and its projection button takes over the
`persp` / `ortho` text that used to be painted there. **`W A S D E Q`**,
reserved off the tool keys since M2, translate the turntable's pivot along
the camera basis at a speed proportional to `distance`, so a joint buried
in a shell can be put in front of the camera and then orbited locally. The
camera stays **one turntable** — yaw, pitch, distance, target — so every
consumer of `view_proj`, `cursor_ray`, `frame_bounds` and the animations is
untouched (ADR-0028). And the pivot those gestures turn around is **drawn**
while a camera gesture is live: a small cross at `target`, no fade, gone the
frame the gesture ends.

## Non-goals

- No second camera kind. No first-person mode, no roll, no mouse-look
  (ADR-0028 §1 records why, and what would reopen it).
- No change to ADR-0018's drag table: the cube's drag is on the cube's own
  rect, and bare left over the geometry still orbits.
- The numpad keeps working. This plan makes it unnecessary, not absent.
- Not the rest of v0.5: no ground grid, no MSAA, no rotate-drag snapping.
  Those are the pass under the camera and the gizmo, and are their own work.
- The axes triad stays where it is, bottom-left, in the viewport's own pass.

## Design deltas

- **ADR-0028** (new): navigation is one turntable — the cube aims it, the
  keys move its pivot. Records: the turntable is the only camera model; the
  fly keys translate `target` and nothing else; the cube is **app-side**
  interactive chrome that writes the camera, the pivot cue is **viewport-side**
  feedback no document knows about; the cue has no fade, so no golden carries
  a clock (ADR-0003, and ADR-0021's refusal of a timed cue).
- `riggen-viewport/src/camera/orbit.rs`: `OrbitCamera::fly(dir, dt, boost)`.
  `dir` is (forward, right, up) and **all three are the camera's own basis**
  (`OrbitCamera::basis`, so the pole heuristic applies at top and bottom
  views): `E` / `Q` rise and fall along the view's up, not along world Z, and
  a pitched camera flies in the direction it is looking in all six. Moves
  `target` only; `yaw`, `pitch`, `distance` untouched; cancels an animation
  like every other gesture. Speed = `FLY_SPEED * distance * boost`.
- `riggen-viewport/src/viewport/mod.rs`: `handle_input` reads the six keys
  with `key_down` (held, not pressed) while the pointer is over the viewport
  and `!ctx.wants_keyboard_input()` — an inline rename must not fly the
  camera. `dt` is `stable_dt.min(0.1)`; deterministic under kittest, which
  leaves `RawInput::time` unset so dt is exactly `step_dt`.
- `riggen-viewport/src/viewport/mod.rs`: a viewport-owned pivot `Overlay`,
  built when a camera gesture is live (orbit, pan, fly, or an animation in
  flight) and painted **after** the app's overlay, so `set_overlay` cannot
  clobber it. `Occlusion::Always` — it is cursor-class feedback (ADR-0020).
  The viewport's `persp` / `ortho` corner text is **removed** in step 5; the
  cube's button is the readout.
- `riggen-app/src/app/viewcube/` (new): `facets.rs`, `projection.rs`,
  `widget.rs`, `tests.rs`, ported from
  `robocad-ui/src/viewcube/` — cgmath → glam, `robocad_viewport::{Projection,
  ViewOrientation}` → `riggen_viewport::{…}` (same variants, same
  `normal()` / `yaw_pitch()` / `label()` / `as_standard_view()`), and
  RoboCAD's `is_sketch_mode` parameter dropped.
- `riggen-app/src/app/mode.rs::viewport_chrome`: a third piece of corner
  chrome. `chrome_rects` becomes three rects, and the cube is not drawn in
  zen for the same reason the others are not.
- `riggen-app/src/debug/camera.rs`: `CameraDebug` gains `pivot_visible`, so
  the cue is a JSON golden and not only a PNG one.

## Steps

Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [x] **[1]** Step 1 — **ADR-0028**, "navigation is one turntable: the cube
      aims it, the keys move its pivot". The decision, the three rejected
      options (a second camera kind, a free camera, a six-ball gizmo), the
      layer split (cube app-side, pivot viewport-side), the no-fade rule, and
      the note that ADR-0018 is untouched. Row in `docs/adr/README.md`.
      Same commit deletes `docs/ideas/viewcube-and-fly-camera.md` — already
      gone, absorbed by the plan's own commit (db92fab).
- [x] **[2]** Step 2 — **The fly keys.** `OrbitCamera::fly` plus the gating
      in `handle_input`; `Shift` fast, `Ctrl` slow. Tests in
      `camera/tests.rs`: forward moves `target` along the view direction by
      `speed·dt` and leaves yaw/pitch/distance alone; `E`/`Q` move along the
      camera's up, which at a pitched camera is not world Z; at a pole the
      basis's up hint is the one `basis()` already picks; boost multiplies. Harness gains `hold_key` /
      `release_key`; scenario `fly_into_the_assembly` flies into the sample
      arm and snapshots it. **Checkpoint for the human:** a by-hand run —
      if flying with the pivot locked to the eye reads as pushing the scene
      rather than walking into it, that is the one thing that reopens
      ADR-0028 §1.
- [x] **[2]** Step 3 — **The orbit pivot is drawn.** The viewport's own
      overlay, live exactly while a camera gesture is, sized from `distance`,
      no fade. `CameraDebug::pivot_visible`. Scenario
      `orbit_shows_the_pivot` snapshots mid-drag and asserts the cue is gone
      after the release.
- [x] **[2]** Step 4 — **The cube's math.** `facets.rs` + `projection.rs` +
      `tests.rs` ported (chamfered cube → 26 facets, project, depth-sort,
      backface-cull, hit-test, face text). No UI yet, so no golden: the
      ported unit tests are the step's evidence, and `cargo test -p
      riggen-app` runs them.
- [x] **[2]** Step 5 — **The cube in the corner.** `widget.rs` ported and
      wired: `Select` → `animate_to_orientation`, `Orbit` → `camera.orbit`,
      `Home` → `animate_frame_scene`, `ToggleProjection` →
      `toggle_projection`; bottom-right, its rect in `chrome_rects`; the
      viewport's `persp` / `ortho` text removed. New scenarios
      `viewcube_corner` and `viewcube_click_snaps_to_top`; the existing zen
      scenario now proves the cube is absent there. **This step refreshes the
      whole snapshot suite** — every scenario shows that corner — so its
      commit message is `snapshots:` and says why, and the images go in front
      of the human before it lands.

## Acceptance

`cargo test -p riggen-viewport` and `cargo test -p riggen-app --test visual`
green with `fly_into_the_assembly`, `orbit_shows_the_pivot`,
`viewcube_corner` and `viewcube_click_snaps_to_top` in the suite; and the
milestone's own half, run by hand once: the sample arm is opened cold,
every named view is reached by clicking the cube, the gripper is flown into
and orbited locally, and the numpad is never touched.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Layer map — the ViewCube in the app's box, the
  pivot cue in the viewport's overlay.
- `docs/ARCHITECTURE.md` §The document is the only state — `W A S D E Q`
  are the fly camera now, not reserved for one.
- `docs/ARCHITECTURE.md` §Panels and menus — corner chrome is three rects;
  the cube is bottom-right and gone in zen.
- `docs/ARCHITECTURE.md` §Frame loop (the camera paragraphs) — the fly keys
  and their gating, the pivot cue and when it is live, the projection
  readout having moved onto the cube.
- `docs/ARCHITECTURE.md` §Testing — the new scenarios and the `hold_key` /
  `release_key` helpers.
- `docs/ROADMAP.md` v0.5 — the ViewCube and fly-camera lines are done; the
  grid, MSAA and rotate-snap lines are what is left.
- `AGENTS.md` current state — one line.

## Open questions

None left; both were answered 2026-09-11, before step 1, and ADR-0028
records them:

- **`E` / `Q` follow the camera's own up**, not world Z (human). All six keys
  are then one rule — you fly where you are looking — and nothing has to
  special-case a pitched camera.
- **Zen has no projection readout** (agent). ADR-0021's zen rule is that
  nothing is drawn there, and a label that survived would be the one piece of
  chrome that did; `P` and `Num5` still toggle in zen, `debug_state().camera`
  still names the projection, so no golden loses the fact.
