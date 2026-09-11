# ADR-0028: Navigation is one turntable — the ViewCube aims it, the fly keys move its pivot

- Status: Accepted
- Date: 2026-09-11
- Amends: nothing. [ADR-0018](0018-left-drag-orbits-gizmo-claims-the-primary-drag.md)'s
  drag table is untouched — the cube's drag is on the cube's own rect, and a
  bare left-drag over the geometry still orbits.
- Leaves intact: [ADR-0020](0020-the-overlay-reads-the-scenes-depth-back.md)
  (the pivot cue is cursor-class feedback, drawn unconditionally),
  [ADR-0021](0021-two-modes-and-edit-is-the-zero-configuration.md) (the cube
  joins `chrome_rects` like every other corner widget and is gone in zen,
  and its refusal of a timed cue is what forbids a fade here),
  [ADR-0003](0003-headless-visual-snapshots.md)

## Context

The camera is M0's, and M0's is a turntable: `yaw`, `pitch`, `distance`,
`target` (`riggen-viewport/src/camera/orbit.rs`). Everything downstream
speaks those four — `view_proj`, `cursor_ray`, picking, the overlay's
projection, `frame_bounds`, `StandardView`, `animate_to`, the gizmo's
matrices, `debug_state().camera`, and the 500-odd lines of
`camera/tests.rs`. v0.5's goal sentence is two complaints about it.

**You do not always know which way you are looking.** The bottom-left axes
triad names the world axes and nothing else. The only way to a named view
is the numpad (`Num1/3/7/0`, `Ctrl` for the opposite face), which is
folklore on a laptop without one and invisible on the web demo — the first
riggen most people meet (ADR-0017). `ViewOrientation` has held all 26
canonical orientations since M0 with a comment saying they are "kept whole
for the ViewCube port later"; twenty of them have never been called.

**You can orbit round an assembly but not get into it.** Every gesture is
about one pivot: orbit turns round it, pan slides it, zoom approaches it.
A joint inside a closed shell — a gripper between fingers, a hip inside a
shroud — cannot be put in front of the camera at all, which in View is
exactly where the wheel and the band want it (ADR-0021 §1, ADR-0027).
`W A S D E Q` have been reserved off the tool keys since M2 against this
day.

The obvious reading of "fly camera" is a second camera kind — rerun's
`Eye3DKind::FirstPerson`, a free eye with its own drag semantics. That is
the question this ADR settles, because it recurs: it is asked once per
viewport feature, and each time the answer has to be re-derived from the
same list of consumers above.

Four constraints shape the rest:

- **The layer rule.** The viewport owns the camera and the projection; the
  app owns selection, chrome and document overlays. A ViewCube is
  interactive chrome that *writes* the camera. The pivot cue is camera
  feedback no document knows about.
- **ADR-0021 made the corners chrome.** A widget that does not register in
  `chrome_rects` steals drags from the camera and lets picks through under
  itself.
- **ADR-0003, and ADR-0021's zen amendment**, which refused a timed cue
  outright ("never a timed toast") because a golden would then carry a
  clock.
- **Bare letter keys yield to a focused `TextEdit`** (`shortcuts.rs`), and
  `W`, `A`, `S`, `D` are letters: an inline link rename must not fly the
  camera.

## Decision

**There is one camera and it is the turntable. The ViewCube aims it; the
fly keys translate its pivot; while any camera gesture is live the pivot is
drawn, without a fade.**

1. **One camera model.** `OrbitCamera` stays yaw / pitch / distance /
   target, and nothing gains a "which camera kind is live" branch. There is
   no first-person mode, no roll, no mouse-look. Roll is wrong for a Z-up
   document, and every named view, `from_direction`, `animate_to(yaw,
   pitch)` and the cube's own hit test are expressed in yaw and pitch.

   What would reopen this: a by-hand run in which flying with the pivot
   locked rigidly to the eye reads as *pushing the scene* rather than
   *walking into it*. The fix would then be a second `kind`, and none of
   this work is wasted — a first-person kind is §2's translation with the
   drag rebound.

2. **The fly keys move `target` and nothing else.** `OrbitCamera::fly(dir,
   dt, boost)` adds `dir · FLY_SPEED · distance · boost` to `target`,
   leaving `yaw`, `pitch` and `distance` untouched, so the eye follows
   rigidly and arrives inside the assembly with the pivot — after which the
   same left-drag orbits *locally*, which is the thing that was missing.
   Speed is proportional to `distance` so the same key crosses the same
   fraction of the view whatever the scale of the robot; `Shift` is fast
   and `Ctrl` is slow.

   `dir` is expressed in **the camera's own basis in all six directions**
   (`OrbitCamera::basis`): `W`/`S` along forward, `A`/`D` along right,
   `E`/`Q` along the view's **up** — not world Z. One rule, *you fly where
   you are looking*, rather than four keys in the view and two in the
   world; and because it is `basis()`, the pole heuristic that already
   stands in Y for Z at the exact top and bottom views applies here for
   free, instead of a second special case.

   A fly cancels a running camera animation, like every other camera
   gesture. The keys are read as **held** (`key_down`, not `key_pressed`)
   while the pointer is over the viewport and `!wants_keyboard_input()`,
   which is what keeps an inline rename from flying the camera. `dt` is
   the frame's `stable_dt`, clamped, so a stalled frame cannot teleport the
   pivot across the scene — and under kittest, which leaves `RawInput::time`
   unset, `dt` is exactly the step, so a scenario that holds a key for *n*
   frames is deterministic.

   The numpad is not removed. This decision makes it unnecessary, not
   absent.

3. **The ViewCube is app-side chrome; it is RoboCAD's, ported.**
   `robocad-ui/src/viewcube/` — the chamfered cube's 26 facets, the
   projection with its depth sort, backface cull and hit test, and the
   widget — becomes `riggen-app/src/app/viewcube/`. The port is cgmath →
   glam and `robocad_viewport::{Projection, ViewOrientation}` →
   `riggen_viewport::{…}`: the same enums with the same variants, which is
   why the port is a port and not a rewrite, and why its own tests come
   with it. It is **painter-drawn** — no wgpu pass, no shader, nothing for
   a later MSAA change to be taught about.

   It sits **bottom-right** and registers its rect in `chrome_rects`, so
   the camera holds still and picks are suppressed under it; it is not
   drawn in zen, for the same reason the other two corners are not. Its
   outputs write the camera through the interface that already exists:
   `Select(ViewOrientation)` → `animate_to_orientation`, `Orbit{dy, dp}` →
   `camera.orbit`, `Home` → `animate_frame_scene`, `ToggleProjection` →
   `toggle_projection`. That last one **takes over the `persp` / `ortho`
   text** the viewport used to paint in that corner: the button is the
   readout, and the viewport's own label is removed rather than duplicated.

   The bottom-left **axes triad stays** where it is, in the viewport's own
   pass. It names the axes in the gizmo's colours; the cube names the faces
   and is clickable. They answer different questions.

4. **The pivot is drawn while a camera gesture is live, with no fade.** A
   small cross at `target`, sized from `distance`, pushed by the
   **viewport** into an overlay of its own — painted after the app's, so
   `set_overlay` cannot clobber it — at `Occlusion::Always`, the
   cursor-feedback class of ADR-0020. It is live exactly while an orbit, a
   pan, a fly key or a camera animation is, and it is gone the frame that
   ends.

   **No fade.** rerun fades its one and it is nicer for it; a fade is a
   clock, and ADR-0021's amendment already refused one for this reason. A
   binary cue is a golden that asserts by value, and the same fact reaches
   `debug_state()` as `CameraDebug::pivot_visible`, so a scenario can
   assert the cue's absence after a release without reading pixels.

5. **Zen has no projection readout.** Zen draws nothing in the corners
   (ADR-0021, third amendment), and under §3 the projection's readout is
   the cube's button — so in zen there is none. A label that survived would
   be the one piece of chrome that did. `P` and `Num5` still toggle in zen
   and `debug_state().camera` still names the projection, so no golden and
   no test loses the fact.

## Consequences

- The four camera lines of v0.5 become one small addition to one struct
  plus a widget in a corner. Nothing downstream of the camera changes at
  all: no consumer learns a new question, and `camera/tests.rs` keeps
  every assertion it has.
- Twenty `ViewOrientation` variants stop being dead code five milestones
  after they were written, and the numpad stops being the only door to a
  named view — which is what makes the web demo navigable.
- A joint inside a shell is reachable: fly the pivot to it, then orbit
  round it. That is the gesture View was built for and could not perform.
- **The turntable's pivot can now be left anywhere**, including outside the
  scene entirely, and nothing re-centres it but `Home` and a zoom-to-fit.
  A user who flies off into the void gets an empty viewport and an orbit
  around nothing; `Home` is the answer, and the drawn pivot of §4 is what
  makes the state legible rather than mysterious.
- The bottom-right corner gains a widget that is ~25 % larger than the text
  it replaces, and **every golden shows that corner** — so this decision
  costs one whole-suite snapshot refresh, taken once, with the images shown
  to the human (ADR-0003).
- `E` / `Q` following the view's up means that at a top view they move the
  pivot in the world XY plane, not in Z. That is the price of the single
  rule, it is what "fly where you are looking" means at a camera pointing
  straight down, and the basis's existing pole heuristic is what keeps it
  defined there at all.
- Speed proportional to `distance` means flying while zoomed far out
  crosses the scene in a few frames. Intended — it is the same behaviour
  the wheel already has — but it is the constant to reach for
  (`FLY_SPEED`) if the hand-feel run of §1 complains.

## Alternatives considered

- **A second camera kind, `Orbit | FirstPerson`** (rerun's `Eye3DKind`).
  The truer "fly camera": the wheel dollies, the drag turns the head, the
  pivot sits one unit ahead. Rejected for now, and named in §1 as the thing
  this reopens into. It costs an amendment to ADR-0018's drag table, a
  branch in every consumer listed in Context, an answer for `Home`,
  `StandardView` and `frame_bounds` in first person, and a mode indicator —
  a fourth thing in a corner — and it buys a second mental model for a
  window that has just learned its first two (ADR-0021). What the
  researcher actually asked for is a joint in front of the camera, which
  moving one `Vec3` delivers.
- **A free camera: eye plus quaternion.** Rejected on sight. It replaces
  the turntable outright, and `MAX_PITCH`, `StandardView`,
  `ViewOrientation::from_direction`, `animate_to(yaw, pitch)`, the cube's
  hit test and every camera test are yaw/pitch. Roll is a non-feature for a
  Z-up document.
- **`E` / `Q` along world Z.** Defensible — "up is up" is what a walking
  metaphor implies, and it needs no pole heuristic because Z is never
  degenerate. Rejected by the human: two keys in the world and four in the
  view is two rules, and a pitched camera then rises out of its own frame
  in a direction the view does not show.
- **A flat six-ball axis gizmo** (Blender's) instead of the cube. ~150
  lines, no port, legible smaller. But it answers "which way is +X", which
  the triad already answers, rather than "which face am I on"; it reaches 6
  orientations where the cube reaches 26; and it discards a tested port
  from the codebase AGENTS.md names as the ancestor.
- **Draw the cube as a second wgpu pass**, as the axes triad already is.
  Rejected: picking a facet needs a screen-space hit test regardless, the
  widget must live where `chrome_rects` and the pointer are (app side), and
  a third pass is one more thing a later MSAA change would have to be
  taught.
- **Top-right for the cube**, the convention in Fusion and SolidWorks.
  It is taken by the visibility row, and moving that row to make space is a
  change to a corner ADR-0021 settled. Bottom-right holds only the text the
  cube's own button replaces, and is where Onshape puts its cube.
- **Fade the pivot cue out over ~200 ms.** What every other tool does, and
  it looks better. It is a clock in a golden, which is the thing ADR-0003
  and ADR-0021's amendment both refuse; the cue is binary so that its
  absence is assertable.
