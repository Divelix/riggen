# Plan: m0-skeleton-viewport

- Started: 2026-08-29
- Milestone: M0
- Idea (verbatim from the human): "M0" — straight from `docs/03-roadmap.md §M0`, no idea file.

## Goal
`riggen part.stl` (and `riggen a.stl b.obj c.stl`, and dropping files onto the
window) opens a native window with each file as one orbitable, pickable
instance; the Cargo workspace of 01-architecture exists with five crates and a
green CI (fmt, clippy `-D warnings`, test, wasm build check); `riggen-mesh` has
`TriMesh`, STL (binary + ASCII) and OBJ loaders, `Aabb` and ray/triangle;
`riggen-viewport` is `robocad-viewport` ported to glam and keyed by
`InstanceId`, with the sketch-plane code and the edge/vertex pick passes gone;
the `egui_kittest` snapshot suite and `debug_state()` are in place and a
`startup` scenario passes on the CPU adapter.

## Non-goals
- Any document, `MeshAsset`, unit scaling, or Y-up fix — a dropped file is shown
  in file units as-is (M1 owns `MeshAsset.scale`/`fix_up`).
- Any panel beyond the status bar; no tree, no properties, no menus beyond
  File › Open / Quit.
- Mass properties (`mass.rs`), convex hull, primitive fits — M3.
- MSAA and a ground grid: neither exists in robocad (see Open questions).
- Vertex welding / large-mesh decimation; the pick-id encoding caps a mesh at
  2^20 − 1 triangles and the loader rejects bigger ones with a clear error.
- The web *page* (`index.html`, trunk); only the wasm *build* is checked.
- Ports of the ViewCube, shortcuts manager, settings menu, shortcuts-help
  window, mass-properties panel — their milestones come later.

## Design deltas
All present-tense text lands in the design docs at retirement; the deltas:

- `docs/01-architecture.md §Cargo workspace` — as built: `crates/riggen-{mesh,
  core,export,viewport,app}`; `riggen-core` and `riggen-export` are empty
  `lib.rs` placeholders that already carry the "no egui/wgpu" rule. `assets/
  fixtures/` holds the tiny test meshes. Workspace profile: `[profile.dev]
  opt-level = 1`, `[profile.dev.package."*"] opt-level = 3`.
- `docs/01-architecture.md §Frame loop / Picking` — the ID buffer encodes
  `(instance slot: 12 bits, triangle + 1: 20 bits)`, `0` = miss; readback is a
  5×5 region resolved nearest-to-cursor (robocad's vertex > edge > face ladder
  is gone). Hover picks are memoised on `(pixel, view_proj)`.
- `docs/01-architecture.md §Testing` — the harness facts that must not be
  rediscovered: kittest's `step()` does no GPU work, so clicks are one raw
  event per rendered frame; scenarios serialise on a global `Mutex` (parallel
  lavapipe devices segfault); `UPDATE_SNAPSHOTS=1` refreshes PNG **and** JSON
  goldens; `visual_scratch` is `test = false` and run by name; every float in
  `debug_state()` is rounded to 6 decimals and `-0.0` normalised.
- Camera math: glam `Mat4::perspective_rh` / `orthographic_rh` produce wgpu's
  `[0,1]` depth directly, so robocad's `OPENGL_TO_WGPU_MATRIX` is dropped and
  `view_proj = proj * view`. `Rad<f32>` fields become bare `f32`; `Point3` /
  `Vector3` collapse to `Vec3`. f32 in the camera and GPU path, f64 `DVec3` in
  `TriMesh` (02-data-model: f32 only past the GPU boundary).
- New types: `riggen_mesh::{TriMesh, Aabb, Ray, ray_triangle}`;
  `riggen_viewport::{InstanceId, Viewport, Scene, OrbitCamera, PickHit
  {instance, triangle}}`. `Viewport::ui(ui)` loses the `sketch_mode` flag.
- The frame-time HUD moves from the viewport painter into the status bar (the
  roadmap's wording); `set_frame_hud_visible(false)` still exists for the
  harness.
- No ADR needed: every decision above is either the roadmap's or a mechanical
  consequence of glam; the grid/MSAA question is answered in the plan, not an
  ADR.

### Port map (robocad → riggen-viewport)
| robocad file | LOC | riggen | change |
|---|---|---|---|
| `camera/{orbit,orientation,animation,tests}.rs` | 379+368+181+434 | `camera/` | cgmath→glam; drop `sketch_up`, `orient_to_plane`, `lock_to_sketch_plane`; drop `OPENGL_TO_WGPU_MATRIX`; 19 tests kept |
| `scene.rs` | 414 | `scene.rs` | `BodyId`→`InstanceId`, `RenderMesh`→`&TriMesh`, `Aabb`→`riggen_mesh::Aabb`, `Matrix4`→`DMat4`; 6 tests kept |
| `picking.rs` | 74 | `pick_id.rs` | `(instance, tri)` packing; 2 tests rewritten |
| `mesh.rs` | 457 | `gpu_mesh.rs` | keep `Vertex`, `PickVertex`, `ColorVertex`, `AxesTriadMesh`, face upload; drop edge/vertex buffers, `build_vertex_marks`, `VERTEX_MARK_STRIDE`, `FaceOutline` (outline pass: see Open questions) |
| `viewport/{gpu_state,pipelines,render_pass,picking}.rs` | 180+332+380+104 | same | pipelines cut 14→8 (background, scene, pick, hover, select, outline, axes, blit); pick pass keeps the face loop only |
| `viewport/mod.rs` | 1066 | `viewport/mod.rs` | drop `SketchCameraLock`, `enter/exit_sketch_mode`, `sketch_screen_pos`, `sketch_uv_at`, the `sketch_mode` gates; keep `raw_wheel_delta_y`, `last_pick` memo, keyboard views, Home = fit |
| `shaders/*.wgsl` | — | same | keep background, scene, pick, hover, select, outline, axes, blit; delete grid, hover_line, select_line |
| `sketch_plane.rs` | 143 | — | deleted |
| `robocad-app/src/main.rs` | 31 | `riggen-app/src/main.rs` | keep `LOW_LATENCY` + `AutoNoVsync` verbatim with its comment; add `std::env::args` file list |
| `robocad-app/src/lib.rs` (`WebHandle`) | 48 | `riggen-app/src/lib.rs` | as-is |
| `robocad-app/src/app/file_io.rs` (`rfd` open) | ~40 of 191 | `riggen-app/src/app/file_io.rs` | Open only; STL/OBJ filter; wasm arm = status "no filesystem" |
| `robocad-app/src/debug/{mod,camera}.rs` | 184+96 | `riggen-app/src/debug/` | `DebugState { camera, instances, selection, viewport_rect }`; `round()`/`round32()` verbatim |
| `robocad-app/tests/visual/{main,harness}.rs`, `tests/visual_scratch.rs`, `kittest.toml` | — | same paths | `eval_idle` settle loop becomes "no pending pick and no camera animation for 4 frames" |
| `robocad-ui/src/status_bar.rs` | 72 | `riggen-app/src/app/status_bar.rs` | `riggen | units: file | hover: i3/t120 | selected: … | 4.10 ms (244 fps)` |
| `.github/workflows/ci.yml`, `.githooks/`, `.gitignore` snapshot lines | — | same | drop the size/wasm-lint jobs; keep `mesa-vulkan-drivers` install with its comment |

## Steps
- [x] Step 1 — Workspace skeleton and CI. Root `Cargo.toml` (resolver 3,
  edition 2024, profiles, `[workspace.dependencies]`: egui/eframe/egui-wgpu
  0.36, egui_kittest 0.36.1 `[wgpu, snapshot, eframe]`, glam 0.30 `[serde]`,
  stl_io 0.11, tobj 4, bytemuck, serde/serde_json, rfd 0.15, web-time,
  pollster dev), `rust-toolchain.toml`, five crates (`mesh`, `core`, `export`,
  `viewport` as empty libs; `app` as `cdylib + rlib` + bin `riggen` opening an
  empty eframe window titled "riggen"), `WebHandle` wasm entry,
  `.github/workflows/ci.yml` (fmt, clippy, test with mesa, wasm build),
  `.gitignore` snapshot artefacts. Test: `cargo test --workspace` (zero tests,
  builds), `cargo build --target wasm32-unknown-unknown -p riggen-app`, the
  pre-commit hook now runs for real.
- [x] Step 2 — Snapshot harness and `debug_state()` on the empty shell.
  `RiggenApp::new(cc)` requiring the wgpu render state; status bar (bottom
  panel, one frame of lag by design) + empty `CentralPanel`; `kittest.toml`
  (`threshold = 0.6`, `max_failed_pixels = 64`); `tests/visual/harness.rs`
  with `adapter_available`, `gpu_lock`, `scenario`, `settle`,
  `pump_rendered`, `click_at`, `scratch`, JSON golden compare;
  `tests/visual/main.rs` with `startup`; `tests/visual_scratch.rs`
  (`test = false`); `DebugState` with `viewport_rect` only (`camera` joins in
  step 8 — there is no camera to report before step 6 lands). Test:
  `cargo test -p riggen-app --test visual` renders `startup.png` +
  `startup.json` on lavapipe; show the human the PNG. Retires the
  "does kittest + eframe 0.36.1 + lavapipe work on this machine" unknown
  before any port code exists.
- [x] Step 3 — `riggen-mesh` core: `pub use glam`; `TriMesh { positions:
  Vec<DVec3>, normals: Vec<DVec3>, indices: Vec<u32> }` with
  `triangle_count()`, `triangle(i) -> [DVec3; 3]`, `flat_normals()`,
  `validate()` (index range, non-multiple-of-3); `Aabb { min, max }` with
  `of_points`, `union`, `transformed(&DMat4)`, `center`, `half_diagonal`;
  `Ray { origin, dir }` + Möller–Trumbore `ray_triangle(ray, tri) ->
  Option<f64>`; `TriMesh::cube(half)` test helper. Tests: unit cube AABB,
  ray hits/misses/parallel/backface, degenerate-index rejection.
- [x] Step 4 — STL loader: `load_stl(path) -> Result<TriMesh, MeshError>`
  handling binary and ASCII (sniff: "solid" prefix *and* parseable facets, since
  some binary files start with "solid"), unwelded vertices, normals recomputed
  from winding (file normals are unreliable), `MeshError` grows `Io` / `Parse`
  variants (the enum itself landed in step 3 with the three `validate()`
  variants; `thiserror`-free with `Display`). Fixtures in `assets/fixtures/`: `cube_binary.stl`,
  `cube_ascii.stl` (generated once by a test helper, committed, < 2 KB).
  Tests: both fixtures give 12 triangles, identical AABB, `validate()` ok;
  garbage bytes → error.
- [x] Step 5 — OBJ loader via `tobj` (`triangulate = true`, `single_index =
  true`), all shapes merged into one `TriMesh`, normals used when present else
  `flat_normals()`, `mtl` ignored. Fixture `assets/fixtures/cube.obj`. Tests:
  12 triangles, same AABB as the STLs; `load_mesh(path)` dispatches on
  extension (case-insensitive) and returns `MeshError::UnsupportedFormat`
  otherwise.
- [x] Step 6 — `riggen-viewport::camera` ported to glam: `OrbitCamera`,
  `Projection`, `StandardView`, `ViewOrientation` (26-way, kept for the
  ViewCube later), `CameraAnimation`, `shortest_angular_delta`; `[0,1]`-depth
  projections, `OPENGL_TO_WGPU_MATRIX` gone, sketch fields gone. Test: the 19
  camera tests ported verbatim plus one new: `proj_matrix` maps the near plane
  to depth 0 and the far plane to depth 1 in both projections.
- [x] Step 7 — `riggen-viewport::{scene, pick_id, gpu_mesh}`: `InstanceId(u32)`
  newtype, `Scene<M: InstancePayload>` keyed by `InstanceId` with
  `set_instance(&TriMesh)`, `remove`, `set_visible`, `set_model(DMat4)`,
  `bounds() -> Option<(DVec3 center, f64 radius)>`; `pick_id::{encode,
  decode}` with the 12/20 split, `decode(0) = None`; `GpuMesh::upload(device,
  slot, &TriMesh)` producing `Vertex` + `PickVertex` buffers (f64→f32 at
  upload). Tests: scene tests ported (GPU-free `TestPayload`), pick-id
  round-trip and saturation, `upload` rejects `> 2^20 − 1` triangles.
- [ ] Step 8 — Render path: `GpuState` (8 pipelines), `OffscreenTarget`,
  `ModelUniforms` (dynamic offsets, grows by `next_power_of_two`),
  `ViewportCallback: egui_wgpu::CallbackTrait` with the offscreen colour pass
  (background, instances, axes triad in its own viewport rect) and the blit;
  `Viewport::new / ui / set_instance / remove_instance / set_instance_model /
  instance_states`; orbit (middle drag), pan (shift + middle), zoom via
  `raw_wheel_delta_y` + zoom-to-cursor, Numpad views, Home = fit, `P`
  projection toggle; repaint requested only while animating. App wires
  `Viewport` into the `CentralPanel` and gains `open_path(&Path)`. Test:
  snapshot `cube` (`open_path("assets/fixtures/cube_binary.stl")`, fit, settle)
  + `debug_state().camera` (eye, target, up, distance, yaw/pitch/fov,
  projection, animating, aspect, view, proj — robocad's `CameraDebug`) and
  `debug_state().instances` listing one instance with 12 triangles and the
  right bounds. No picking yet.
- [ ] Step 9 — Picking and restyle: pick pass on its own encoder + `R32Uint`
  target + 5×5 readback + `map_async` into `Arc<Mutex<..>>`; `device.poll`
  at the top of `ui`; `PendingPick`, `last_pick` memo, click = select beats
  hover, `PointerGone` clears; `hover` / `select` / `outline` highlight draws
  over the hit instance's triangle range… (see Open question 2 for whether the
  outline pass survives); `hovered() / selected() -> Option<PickHit>`;
  `DebugState.selection`. Tests: snapshots `hover_cube` (hover at viewport
  centre) and `select_cube` (`click_at` centre) with their JSON asserting the
  hit instance and a triangle index; a pick memo unit test (same pixel + same
  camera → no second pick issued).
- [ ] Step 10 — Files in: `riggen a.stl b.obj` CLI args, `egui` file drop
  (`hovered_files` → tinted overlay + "drop to open", `dropped_files` → one
  instance per file, placed in file units at the origin), File › Open via
  `rfd` (native only) with STL/OBJ filter, load errors shown in the status
  bar, zoom-to-fit after every load, status bar readouts (hover/selected
  `i3/t120`, instance count, frame-time HUD behind `set_frame_hud_visible`).
  Loading is synchronous in M0 (the `jobs` thread comes with M3's hull
  work). Tests: snapshot `three_parts` (the three fixtures via `open_path`,
  offset so they don't overlap) + JSON with three instances; a unit test that
  a bad path yields a status message and no instance.

## Acceptance
- `cargo test --workspace` green, including `startup`, `cube`, `hover_cube`,
  `select_cube`, `three_parts` on the CPU adapter (`vulkaninfo` shows llvmpipe;
  the scenarios print SKIPPING otherwise and that counts as a failure of the
  environment, not a pass).
- `cargo build --target wasm32-unknown-unknown -p riggen-app` succeeds.
- Manual, once: `cargo run -p riggen-app -- assets/fixtures/cube_binary.stl
  assets/fixtures/cube_ascii.stl assets/fixtures/cube.obj` and drop three
  STLs onto the window → three orbitable, pickable parts; the human confirms.
  Then the human tags `m0` (`.agents/rules/git.md`).

## Docs to update on completion
- `docs/01-architecture.md §Cargo workspace` — layout as built, `assets/fixtures/`.
- `docs/01-architecture.md §Frame loop / §Picking and snapping` — pick-id
  encoding, 5×5 nearest-to-cursor readback, `(pixel, view_proj)` memo, the
  `[0,1]`-depth note on the camera.
- `docs/01-architecture.md §Testing` — harness facts listed under Design deltas;
  `UPDATE_SNAPSHOTS`, `visual_scratch`, the `GPU` mutex, one event per frame.
- `docs/03-roadmap.md §M0` — status line "done <date>, tag m0"; rewrite the
  bullet so it stops claiming grid/MSAA were ported (they were not in robocad).
- `docs/BACKLOG.md` — add: "Ground grid at z = 0 (new; robocad never had one)",
  "MSAA for the offscreen colour pass (new; robocad had none)", "Vertex
  welding + `is_closed` for STL (M3 needs it)", "Meshes > 2^20 triangles:
  decimate or widen the pick id", "Async mesh loading via `jobs`".
- `AGENTS.md` "Current state" — "M0 done: workspace, `riggen-mesh` loaders,
  ported viewport, snapshot harness. Next: M1 document/tree/joints."
- `SEED.md` is frozen; its MSAA claim stays and the roadmap carries the truth.

## Open questions
- ⚠ OPEN: Ground grid — robocad has none. Ship M0 with the gradient
  background only and put the grid in the backlog (recommended: keeps M0 a
  pure port and the roadmap out-list honest), or add a "Step 11 — ground grid
  pipeline" to this plan? **Human decides before step 8.**
- ⚠ OPEN: Selected-face outline pass — robocad outlines the selected B-Rep
  face; on an STL "face" = one triangle, so the outline would trace a single
  triangle. Recommended: drop `outline.wgsl` and the outline pipeline (7
  pipelines, not 8) and tint the whole selected *instance* in the select
  pass, hover-tinting the instance too, with the hit triangle index kept in
  `PickHit` for M2's snapping. **Agent decides at step 9; human may override
  earlier.**
- Frame-time HUD location — **decided (human, 2026-08-29): status bar.**
  Landed in step 2 as the right-aligned readout behind `set_frame_hud_visible`.
- `TriMesh` positions — **decided (human, 2026-08-29): f64**, per
  02-data-model ("f32 only past the GPU boundary"); M3's mass properties want
  it. The memory cost (72 MB per million unwelded triangles) is accepted.
- Finding (step 3): `flat_normals()` **unwelds** — a vertex shared by two
  faces cannot carry both their normals, and the pick pass needs per-corner
  vertices anyway (step 7's `PickVertex`). STL is unwelded already; an OBJ
  without normals gets tripled. `ray_triangle` is two-sided: the ID buffer
  chose the triangle, the CPU test only recovers the point on it.
- Finding (step 4): `stl_io` sniffs on the `solid ` prefix alone and keeps
  its binary reader private, so `riggen-mesh::stl` parses binary itself
  (size-checked against the facet count) and uses `stl_io` for ASCII only.
  The `MAX_TRIANGLES = 2^20 − 1` cap is enforced by every loader
  (`finish_loaded`); step 7's `upload` check becomes a defensive assert.
- Finding (step 7): the 12-bit field in the pick id is a **slot** the
  `Scene` allocates from a free-list (`MAX_INSTANCES = 4096` live at once,
  `SceneFull` otherwise), not the `InstanceId`: ids are never reused over a
  session, slots are. `Scene::instance_at_slot` maps a readback to an id and
  answers `None` for a stale hit. Pick vertices are built per triangle
  corner (3 per triangle, drawn non-indexed) so a welded OBJ picks
  correctly; the shaded pass stays indexed. Scene `model` is `DMat4`,
  narrowed at uniform upload in step 8.
