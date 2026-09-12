# Plan: viewcube-corner-axes

- Started: 2026-09-12
- Milestone: v0.6 — taken ahead of the section's import lines by the
  human's call; chrome, not an import line (see *Docs to update*)
- Idea: docs/ideas/viewcube-corner-axes.md (absorbed; option A, lettered)
- Idea (verbatim from the human): "I just noticed in onshape, XYZ axes are
  attached to view cube corner - do the same for our ViewCube and remove
  bottom-left gizmo"
- Answers (verbatim from the human, 2026-09-12): "1. Make axes opaque with
  letters on the ends, but cube itself 50% transparent, while letters on
  cube opaque 2. Remain cube the same size. Axes must just stick to cube
  corners and rotate with cube like it is part of it 3. hide all in zen:
  viewcube and axes"
- Then, over an Onshape screenshot: "this is how it looks in onshape - do
  the same. Also these arrows around it we need to add too". On the
  screenshot: "on picture you can see full green axis that is behind the
  cube so it is transparent". The curved pair is left out for now, and a
  triangle steps 15°, animated.

## Goal

The ViewCube looks and works like Onshape's, minus the roll arrows.

**The axes.** Three thin, opaque arms, `X` red, `Y` green and `Z` blue,
start just outside the cube's (−X, −Y, −Z) corner. They run parallel to
the three edges that meet there, overshoot the cube, and carry an opaque
letter past each end. They are projected through the cube's own
`camera_basis`, so they turn with the cube as a part of it.

**The cube.** It keeps today's size and facets, filled at 50 % alpha, so
an arm behind it — `Y`, at the home view — shows through along its whole
length. The face labels stay opaque.

**The arrows.** Four faint triangles sit on a ring around the cube: up,
down, left and right. A click on one animates the camera 15° that way,
changing yaw or pitch only.

**Everything else.** The bottom-left axes triad and its whole wgpu surface
are gone. Zen hides the cube, its arms and its arrows together. One
whole-suite snapshot refresh carries every visible change.

## Non-goals

- **The two curved arrows** (Onshape's roll). They need roll, which
  ADR-0028 §1 refuses; it is a backlog line, left out by the human.
- **Onshape's view dropdown** under the cube. The projection button stays
  where it is and does what it does.
- **The cube itself.** Its size (`SIZE` 92 in `mode.rs`, fit `radius /
  1.75`), facets, facet hit test, home icon's look, drag rate and
  projection button all stay (the icon and the button move; see
  `widget.rs`). The arms are paint only: not clickable, and they change
  nothing about what a click selects.
- **The back facets.** None are drawn through the translucent front ones,
  and the backface cull stays, so a mirrored `BACK` never shows through
  `FRONT`. What shows through is the scene and the arms.
- **Zen.** No axes indicator of any kind (human's answer 3).
- **The viewport's other passes.** No change to scene, grid, hover, select,
  pick, blit or depth resolve, nor to MSAA.
- **The glyphs and the gizmo.** No change to the frame-glyph or joint-glyph
  triads, or to the gizmo's handle colours, beyond where their shared
  colour constant lives.
- **The web build.** No web-only work: it draws the same widget.

## Design deltas

- **ADR-0030** (new — ADRs are never edited after acceptance,
  `docs/adr/README.md`): *the ViewCube carries the world axes and four
  step arrows, and the viewport's triad is removed; amends ADR-0028 §3.* It
  records:
  - what §3 did not weigh: the triad carries no letters, so it named the
    axes only to a user who already knew the colours;
  - Onshape as the reference;
  - the corner, the translucent cube with opaque arms and labels, and why
    a hidden arm shows through the fill rather than being dimmed or cut;
  - that a letter is dropped entirely while its arm's end is behind the
    cube;
  - the arrows' 15° step, and that they only ever change yaw or pitch;
  - that the roll arrows stay out because §1 refuses roll;
  - the chrome rect growing around a cube of unchanged size;
  - that zen loses its only axes indicator, by the human's call.

  The `docs/adr/README.md` index gains a row for 0030, and 0028's row
  gains "§3 amended by 0030".
- **`riggen-app/src/app/viewcube/axes.rs`** (new) — a pure function,
  `project_corner_axes(rect, yaw, pitch) -> [CornerAxis; 3]`. A
  `CornerAxis` has an axis index, a screen-space tail and tip, its runs
  split into *behind the cube* and *in front*, and its letter's anchor.
  It is tested like `projection.rs`, with no egui context, and uses the
  cube's own fit, so the arms cannot drift from the facets.
  - **Geometry, from the screenshot.** The origin is the min corner pushed
    a small `AXES_GAP` outward along the (−1, −1, −1) diagonal, which puts
    the arms just off the cube's edges. Each arm runs along +X, +Y or +Z
    for `ARM_LENGTH`, about 1.3 cube edges, so it clears the far corner,
    as Onshape's `Z` clears the top and its `X` clears `Right`. No
    arrowheads.
  - **At the home view** (yaw −45°, pitch 0.5), the corner is the
    silhouette's lower-left vertex: `X` along the bottom, `Z` up the left
    side, and `Y` into the screen along the hidden Bottom/Left edge.
  - **Front or behind.** The cube is convex, so a sample point is behind
    it iff both hold:
    - its projection falls inside a front-facing facet;
    - it lies behind that facet's plane along `eye_dir`.

    Runs come from sampling the arm at a fixed step. The split decides
    paint order only: both are drawn in the same opaque colour.
  - **Letters.** Each letter sits past its arm's end along the arm's screen
    direction. **A letter whose arm end is behind the cube is not drawn at
    all**, as in the screenshot, where the green `Y` arm shows through the
    cube and has no letter. `CornerAxis` carries `letter_visible`, which is
    the front/behind test at the arm's end point. The arm is still drawn
    through the fill; only its letter goes. When an arm is foreshortened below a few points (it points
    at the eye, e.g. `Z` at the Top view), its letter moves out along the
    corner's direction from the cube centre — turning by angle as the arm
    shortens below `FORESHORTENED`, so the letter never jumps. No two
    *drawn* letters overlap at any of the 26 orientations.
  - **Letters cover letters** (found in step 1). Six edge views look along
    `X − Y`, `X − Z` or `Y − Z` — BackRight, FrontLeft, BottomRight,
    TopLeft, FrontBottom, BackTop — and there two arms land on one screen
    line, both outside the silhouette, so both letters would be drawn on top
    of each other. The nearer letter covers the farther one, as the cube
    covers a letter behind it: `letter_visible` is also false while the
    letter's square overlaps a nearer drawn one. `CornerAxis::depth` (the
    tip's) orders them, and step 4 paints the in-front runs by it too.
- **`viewcube/arrows.rs`** (new) — the four step arrows, as pure layout
  and hit test.
  - `arrow_rects(cube_rect) -> [(StepArrow, Rect); 4]`: small isosceles
    triangles on a ring outside the cube's circle, pointing outward at 12,
    3, 6 and 9 o'clock. **Tuned at step 2:** `ARROW_RING_GAP` 28 pt. An arm
    can reach ~53 pt in any screen direction, and at the FrontTopRight iso
    `Z` points straight up at the `Up` arrow, so the ring sits past the
    farthest a letter reaches; `ARM_LENGTH` came down 2.6 → 2.4 (1.2 edges,
    still clearing the far corner) to keep the ring close.
  - `hit_test_arrows(…, pos) -> Option<StepArrow>`.
  - `StepArrow::delta() -> (yaw, pitch)` at `ARROW_STEP` = 15°.
  - **Sign.** An arrow turns the view the way dragging the cube towards
    that arrow does (`widget.rs`: `delta_yaw = -dx·k`, `delta_pitch =
    dy·k`), so `Right` is `-15°` yaw and `Up` is `-15°` pitch. Arrows and
    drag can never disagree.
  - **The action.** `ViewCubeAction` gains `Step { delta_yaw, delta_pitch
    }`. `mode.rs` turns it into `camera.animate_to(yaw + dy, (pitch +
    dp).clamp(-FRAC_PI_2, FRAC_PI_2))` (`arrows::step_camera`). The clamp
    is the face views' own range, so `Down` at the Top view is a no-op, not
    a flip. (`Down`, not `Up`: by the sign above, `Up` lowers pitch and
    turns *off* the pole — corrected at step 2.) Stepping from a
    running animation starts from where the camera is now, like any
    facet click.
- **`viewcube/widget.rs`** — paints back to front:
  1. arm runs behind the cube;
  2. facets at 50 % alpha (hover fill too, so hovering never hides an
     arm);
  3. face labels, opaque;
  4. arm runs in front;
  5. arm letters, opaque, only those whose end is in front of the cube;
  6. arrows: faint grey, brighter when hovered, pointing-hand cursor, as
     the home icon does;
  7. home icon and projection button, unchanged.

  - **Seams.** If the translucent facets show a doubled-feather seam along
    shared edges, the facets become one `Mesh` with feathering off, not
    one `PathShape` each.
  - **Hit order.** Arrows, home, projection, then facets. An arrow click
    is never read as a cube drag or a facet select.
  - **Layout.** The projection button moves below the down arrow.
  - **Home icon** (found at step 2). On the cube rect's corner it sat on
    the `Z` arm and letter at the home, Top and Right views. It moves to
    `home_icon_rect`, the top-left corner of `corner_axes_extent`, which no
    arm or letter reaches at any orientation.
  - **Rect.** The painter clip and the returned `rect` become the union of
    the cube rect, the arms' reach at any orientation
    (`corner_axes_extent`), the arrows and the button. `chrome_rects`
    therefore claims everything the widget paints (ADR-0021).
- **`mode.rs` placement.** `SIZE` stays 92, so the cube is the same size.
  The block is inset by the new reach, so no letter or arrow is clipped by
  the viewport's edge. The cube moves inward; it does not shrink.
- **One home for the axis colours.** Today the three `Color32`s are written
  out in `glyphs.rs` (`TRIAD_COLORS`, doc-commented as `AxesTriadMesh`'s),
  in `gizmo.rs` (`gizmo_visuals`), and as floats in `AxesTriadMesh::new`.
  They become one `pub(crate) const AXIS_COLORS` in the app, used by the
  cube, the glyphs and the gizmo. Every comment saying "the axes triad's
  colours" now says the cube's.
- **`riggen-viewport` loses the triad:**
  - `gpu_mesh.rs`: `AxesTriadMesh` and `ColorVertex` (only the triad uses
    it), with their `lib.rs` re-exports;
  - `shaders/axes.wgsl` and `pipelines.rs::build_axes_pipeline`;
  - `gpu_state.rs`: `AXES_GIZMO_SIZE`, `AXES_GIZMO_MARGIN`,
    `axes_pipeline`, `axes_uniform_buffer`, `axes_uniform_bind_group`,
    `axes_mesh`;
  - `render_pass.rs`: `ViewportCallback`'s `axes_*` fields, the
    corner-viewport draw and the uniform upload;
  - `viewport/mod.rs`: their construction and the corner rect;
  - `camera/orbit.rs`: `axes_gizmo_view_proj`;
  - `viewcube/projection.rs`: the "like the axes triad" comment.
- **Test hooks**, beside `viewcube_facet_center`, both `None` in zen:
  - `RiggenApp::viewcube_axis_tips() -> Option<[(Pos2, bool); 3]>` — each
    tip, and whether its letter is drawn (`false` when the tip is behind
    the cube);
  - `RiggenApp::viewcube_arrow_center(StepArrow) -> Option<Pos2>`.

  `debug_state()` gains nothing: arms and arrows are paint, like facets,
  and the camera fields already report a step's result.

## Steps

Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [x] **[3]** Step 1 — `feat(app)`: the corner axes' geometry, not yet
  painted. Add `viewcube/axes.rs`: origin, arms, front/behind split,
  letter placement, `corner_axes_extent`. Unit tests in
  `viewcube/tests.rs`:
  - At the home view, `X` points screen-right, `Z` screen-up, and `Y`
    is mostly behind while `X` and `Z` are wholly in front. `X` and `Z`
    have `letter_visible`; `Y` does not.
  - At the opposite iso (the eye mirrored through the centre) `Y`'s letter
    comes back and no arm is behind: `X` and `Z` ran outside the
    silhouette at home, so they still do — the roles do not simply swap.
  - At the Top view, `X` points right and `Y` up, and `Z` is
    foreshortened with its letter moved outward.
  - At all 26 orientations and a sampled orbit, no two drawn letters
    overlap, everything lies inside `corner_axes_extent(rect)`, the runs
    cover each arm end to end, and some letter is always drawn.
  - At BackRight, `X` and `Y` coincide on screen and only the nearer `X`
    keeps its letter.
  - The tail is the corner triangle's projected centre scaled out along
    the diagonal to the gap — checked against `project_viewcube`, which
    pins "part of the cube".
- [x] **[2]** Step 2 — `feat(app)`: the step arrows' layout and action,
  not yet painted or hit by the widget. Add `viewcube/arrows.rs` and
  `ViewCubeAction::Step`, handled in `mode.rs`. Unit tests:
  - each arrow's delta matches a drag towards it;
  - the rects sit outside the cube's circle (inside the block moved to
    step 4's `viewcube_corner`, the block being the union `widget.rs`
    gains there);
  - no arm run and no drawn letter touches an arrow at any sampled view;
  - the hit test finds each arrow's centre and nothing at the cube's
    centre.

  A camera test covers the handler: `Down` at the Top view leaves pitch at
  90° and `Up` then turns one step off it, and `Right` from yaw 0 lands at
  −15° with target and distance untouched.

  Before committing, the agent paints arms, arrows and the 50 % cube
  locally (uncommitted) and captures a scratch board through
  `visual-debug` against the screenshot: home, Top, Front, Right and the
  iso opposite home, dark and light. It tunes `AXES_GAP`, `ARM_LENGTH`,
  the letter size and the arrow ring to it, checks for facet seams, and
  shows the human the board. *Done:* no seams at 5× in either theme, so
  the facets stay one `PathShape` each; the letter stays 11 pt.
- [x] **[1]** Step 3 — `docs(adr)`: ADR-0030, amending ADR-0028 §3, with
  the human's answers and step 2's tuned constants, plus the index rows.
- [ ] **[2]** Step 4 — `snapshots(app,viewport)`: paint and wire it all,
  and delete the triad, in one commit so the suite is refreshed once.
  - `widget.rs` paints in the order above, hit-tests the arrows first and
    returns the grown rect. `mode.rs` insets the block, `AXIS_COLORS` is
    unified, and the viewport's triad surface is deleted. Add
    `viewcube_axis_tips` and `viewcube_arrow_center`.
  - `viewcube_corner` asserts the three tips and four arrow centres lie
    inside the registered chrome rect, that `Y`'s letter is not drawn at the
    home view while `X`'s and `Z`'s are, and that the cube's rect is still 92 points square.
  - A new scenario, `viewcube_arrow_steps`, clicks `Right`, waits for the
    flight to land, and asserts yaw moved −15° while pitch, target and
    distance stayed. Its golden is taken after landing, as
    `viewcube_click_snaps_to_top`'s is.
  - The zen scenario asserts both hooks answer `None`.
  - Refresh every golden with `UPDATE_SNAPSHOTS=1`. The agent reads the
    whole suite's diff before staging. Allowed changes: the bottom-left
    corner emptied, and the bottom-right block moved inward, now with a
    translucent cube, arms, arrows and the button lower. Any other pixel
    moving is a regression, not a refresh.
  - Show the human `empty_app`, `viewcube_corner`, `viewcube_arrow_steps`
    and one zen golden, before and after (ADR-0003), next to the Onshape
    screenshot. The message says `snapshots:` and why.

## Acceptance

- `cargo test --workspace` green with the refreshed suite, including steps
  1–2's unit tests and step 4's scenarios.
- `grep -rn "AxesTriad\|axes_pipeline\|AXES_GIZMO\|axes_gizmo_view_proj\|axes.wgsl" crates`
  finds nothing.
- By hand once, with `riggen --example arm`, placed next to the Onshape
  screenshot:
  - the home view reads like it: the `Y` arm is visible through the cube
    and has no letter;
  - orbiting a full turn and over both poles by dragging the cube, a
    letter shows exactly while its arm's end is in front of the cube, and
    the arms never come loose from the corner;
  - clicking each arrow steps 15° the way the arrow points;
  - clicking a facet still flies to it;
  - the bottom-left corner is empty;
  - `Z` takes cube, arms and arrows together.

## Docs to update on completion

- `docs/ARCHITECTURE.md` §ViewCube (≈ line 400):
  - the 50 % facets with opaque labels;
  - replace "The bottom-left **axes triad** stays where it is…" with the
    arms — corner, paint order, letters — and the arrows: step, sign,
    clamp, hit order;
  - the grown chrome rect;
  - the arms are the only axes indicator, and zen has none;
  - `viewcube_axis_tips` and `viewcube_arrow_center` beside
    `viewcube_facet_center`.
- `docs/ARCHITECTURE.md` render-pass paragraph (≈ line 703): drop "the axes
  triad draws last in its own corner viewport with a rotation-only camera".
- `docs/ARCHITECTURE.md` ground-grid paragraph (≈ line 753): furniture is
  now just the background and the grid.
- `docs/ARCHITECTURE.md` joint glyphs (≈ 448) and frame glyphs (≈ 568):
  "in the axes triad's colours" becomes "in the axis colours the ViewCube
  names".
- `README.md` first-run paragraph: the ViewCube's corner names X, Y and Z,
  and its arrows step the view 15°.
- `docs/ROADMAP.md` v0.6: one line in the section's in-list — "**The
  ViewCube names the axes and steps the view**, and the bottom-left triad
  goes (ADR-0030) — chrome, taken first by the human's call". The status
  line names ADR-0030 when the cycle closes. The closed M0/M2 sections'
  mentions of the triad stay.
- `AGENTS.md` current state: "Next" names this as v0.6's first landed line.
- Doc comments naming the triad — `glyphs.rs` (module doc, `AXIS_COLOR`,
  `TRIAD_COLORS`), `gizmo.rs::gizmo_visuals`, the `render_pass.rs` and
  `gpu_state.rs` furniture comments — go in step 4's diff. The drift check
  confirms none are left.

## Open questions

- **Left / Right at the exact Top and Bottom views** (found at step 2).
  There `OrbitCamera::basis` stands Y in for Z as the up hint, so the
  picture does not depend on yaw: a side arrow changes `yaw` by 15° and
  the view does not visibly turn. Onshape spins the top view there. Doing
  so means the pole heuristic reading yaw, which is ADR-0028's camera, not
  this plan's; left as it is, and named in ADR-0030.

Otherwise none. The human answered the fill, size, zen, roll-arrow and step
questions, and the letters: "you turn off axis letter visibility
completely when it goes behind cube - onshape on screenshot does exactly
that". The answers are in the design above.
