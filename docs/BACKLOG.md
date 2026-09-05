# Backlog

One line per raw idea. Picking one up means `/idea` (needs thinking) or
`/plan` (obvious); the line is removed then — as is a line a roadmap cycle
has committed to, which now lives in `docs/03-roadmap.md` instead. Rejected ideas keep one line
below with the reason, so the same idea is not re-brainstormed.

- `validate` does not check that geom poses or an `Override` inertial's numbers are finite (joint origins, joint limits, frame poses and densities are); a NaN typed into a geom pose reaches the export
- `MoveJointFrame` re-expresses a link's visual geom poses but not `CollisionPolicy::Meshes` / `Primitives` poses, so a link with imported collision meshes or hand-placed primitives moves its collision in the world when its pivot moves
- Things that *reference* a site now that frames exist (ADR-0012): MJCF sensors, actuators on a site, equality constraints, cameras, `<touch>`/`<force>`
- Frames as a snap source — placing a joint or another frame onto an existing frame, and frame-relative geom poses (a frame's parent is a link, always, today)
- Live joint-state link from a running Python script to the GUI (file or socket)
- A web worker for `jobs`, so a convex decomposition does not freeze the browser tab: `Jobs` has no thread on wasm and runs the job inline, and the demo asks before starting one rather than fixing it (ADR-0017, 01 §Jobs and threads; RoboCAD's `InlineEval` has the same gap)
- A WebGL2 fallback for the demo, for browsers without WebGPU: needs a second picking mechanism, because the ID-buffer readback is `copy_texture_to_buffer` on an `R32Uint` target and wgpu's GL backend will not do it (ADR-0017 §7)
- Touch and a narrow-screen layout for the demo: it is a desktop-browser UI today, and a phone gets the desktop panels. A one-finger drag already orbits, because it reaches egui as a primary drag (ADR-0018); pinch-zoom and two-finger pan have nothing behind them and are untested
- A directory drop on the web, with real relative mesh paths: a plain drop gives only file names, so two `base.stl` in one gesture collide (ADR-0017 §Consequences)
- A document surviving a reload of the demo page: eframe's `persistence` keeps the UI layout and the import-units choice, as on native, and the document is lost — the meshes would have to be kept too, so it is browser storage rather than a serde change
- Snapping during a *rotate* gizmo drag: align the dragged frame's axis to a snapped feature's axis (a circle's, a face normal). Needs a rule for which of the three axes aligns and a second overlay idiom, which is why ADR-0019 §5 left the drag snap to translation
- A snap quantum for a translate drag — mm / degree increments under a modifier, the CAD idiom beside the feature snap (ADR-0019's out-list; the wheel's 5° step is the rotation half of it and the document has nowhere to keep a general one)
- Ground grid at z = 0 in the viewport (new; robocad never had one — M0 ships the gradient background only)
- MSAA for the offscreen colour pass (new; robocad had none)
- Meshes over 2^20 triangles: decimate at load or widen the pick id (loaders reject them today)
- Manual split planes for collision geometry: cut a part by hand where V-HACD's automatic split is wrong (the convex-decomposition idea's option E, not taken — ADR-0011 chose the algorithm; this is the escape hatch for the parts it gets wrong)
- Async mesh loading via `jobs` (M0 loads synchronously on the UI thread; the thread itself exists since ADR-0011)
- Per-drop import-units dialog for mixed-unit batches (M1 has one app-wide setting, ADR-0006)
- Publish the workspace to crates.io so `cargo install riggen` installs the app: publish `riggen-mesh`, `-core`, `-export`, `-viewport`, rename `riggen-app` to `riggen` over the 0.0.1 reservation, `cargo publish --workspace` in `release.yml` (plans/m4-distribution OPEN 1; the README says `cargo install --git` until then)
- A 30-second screencast for the README, recorded after the GUI polish and before the announced release (plans/m4-distribution OPEN 2; the README ships with the hero PNG)
- macOS code signing / notarization, if the clean-VM run of the wheel hits Gatekeeper (pip-installed files carry no quarantine attribute, so an unsigned binary should run from a terminal — unverified until the human's macOS run)

### From the M2 exit gate (the by-hand arm build, 2026-08-29)

- A ViewCube in the viewport corner with the persp/ortho toggle on it (robocad has one; M0 ships the axes triad and a text label)
- WASD fly mode, and draw the orbit pivot while the camera moves (rerun's viewer is the reference; M0 ships turntable orbit only)

### From the M3 exit gate (the export run, 2026-08-29)

The by-hand half was done headlessly: both exports of the arm (`arm.riggen`,
and `arm.urdf` imported) load in MuJoCo with zero warnings, agree with our
FK, and swing under gravity for 10 s without a NaN; the interactive
`mujoco.viewer` look is the human's. What was annoying on the way:

- Interpenetrating shells (the fixture parts are a box plus a shaft, not a boolean) count the overlap twice in `mass_properties`; a note in the Inertial readout ("N geoms, overlaps counted twice") would save a puzzled minute
- No `PackageMap` UI: `package://` on import is resolved beside the file or up the tree; a "packages" table in Import URDF… for the cases that heuristic misses
- An imported link without `<inertial>` has no material and `Computed` cannot run until one is assigned — a default material for imports, or a one-click "assign PLA to every link"
- The export dialog re-resolves (hulls included) on every option change; fine for the arm, and `riggen-app::jobs` now exists to move it off the UI thread for the first big mesh (decomposition already goes through it; hulls stay synchronous and cached per `MeshId`)
- Oriented (PCA) primitive fits; today every fit starts from the AABB in the link frame and the user rotates it
- MuJoCo's joint limits are soft: a freely swinging arm overshoots `range` by a few degrees with default `solref` — not an export bug, but a "joint limits are soft in MuJoCo" note in the export dialog would pre-empt the question
- The `#[ignore]`d fixture generators (`write_arm_fixtures`, `write_arm_sample`) live in the visual test binary and need lavapipe to build; a `cargo xtask fixtures` would be lighter

### From the M4 exit gate (the wheel, 2026-08-30)

The by-hand half was done headlessly: the manylinux wheel installed into
`python:3.12-slim` with no Rust and no checkout, `--version` and
`--export` ran; the window on a clean VM, the TestPyPI dispatch and the
`v0.1.0` push are the human's. What was annoying on the way:

- The NVIDIA Vulkan device creation is ~200 ms of the ~400 ms launch; creating the wgpu device on a thread while winit creates the window (`WgpuSetup::Existing`) would overlap them, at the cost of choosing the adapter without a surface
- `WGPU_BACKEND=gl` fails on X11 + NVIDIA with `incompatible_surface_backends: GL` (pre-existing, eframe's default did the same), so the GL escape hatch is only a hatch on machines where wgpu's GL surface works
- The linux aarch64 wheel is built and its ELF checked, but nothing in the pipeline *runs* it: the release smoke matrix has no ARM runner (ubuntu-24.04-arm exists on GitHub now — add it)
- `--example arm` overwrites `<temp>/riggen-example-arm/` on every run, so a document saved there is lost next time; save should nudge the user to Save As
- The wheel carries a CycloneDX SBOM maturin adds — 566 KB for the app in 0.1, 48 KB for `riggen-py` since 0.2 (the binary is wheel data now, ADR-0009); fine, worth a look when size matters
- `python -m riggen` on Windows is `subprocess.call`, so Ctrl-C reaches the child through the console, not through the parent — good enough, but `riggen` on `PATH` is the real entry there
- `load_files` starts a camera animation, so a harness test that opens a document through it never settles; the tests open through `open_path` + `fit_view_now` and the difference is only in the harness's head
- `cargo build --release` had never been run before M4: egui's `Style::debug` is `cfg(debug_assertions)` and the Debug menu did not compile; CI now builds release through the wheel job, which is the only reason it stays caught

### From the v0.2 SDK (plans/python-sdk, 2026-08-30)

- Free-threaded CPython (3.13t / 3.14t): the abi3 wheel does not install there. Needs either per-version wheels (five targets × every Python) or PyO3's free-threaded support once it covers what the module uses; nobody has asked yet (ADR-0009).
- `riggen.show()` leaves its `riggen-show-*` temp directory behind (`viewer.path` is the user's to reopen); a `Viewer.close()` that removes it, or cleanup at interpreter exit, once someone minds
- `Robot.validate()` / `check()` are always empty in the SDK — every way to obtain a document validates; drop them or find them a purpose (a document assembled from `to_json` edits?)
- The `_riggen` layer carries a joint's `parent` / `child` on read and ignores them on write; a `Frame` API (`robot.frames()`) is read-only until frames exist in the app
- The SDK suite's `cli` fixture skips silently when no binary is found outside CI; a `pytest -rs` in the wheel job would show a skip as a skip
- A live link between a running script and the window (streaming `q`) stays a file-based loop (`show()` / `wait()`); revisit only if it becomes the headline feature (ADR-0009 closed ADR-0002's question as "no")

### From SDF export (plans/sdf-export, 2026-09-01)

- A **Gazebo model package** — `model.config`, a `<world>`, `<include>`, `<plugin>` — was a non-goal: that is a distribution format, not an export, and ADR-0008's directory is what the other two writers fill. Worth doing if a user asks for one.
- **SDF import.** The reading direction is URDF and MJCF; `libsdformat` is a CI test dependency (ADR-0016 §6) and never a runtime one. A third import would extend the one vocabulary ADR-0015 §4 built rather than invent a third.
- `pybullet` reads riggen's SDF wrong — it ignores `//pose/@relative_to` in silence and is f32 (measured, ADR-0016 §Context). Nothing is planned: its users want the `.urdf` the same export writes. Revisit only if SDF-for-pybullet becomes a real request, and then as a second pose convention behind an option, never as the default.
- `test_sdf_load.py` cannot see which frame an `<xyz>` is expressed in, because every joint in the fixtures is axis-aligned with the model at q = 0; the convention is pinned in `sdf.rs`'s golden instead. A fixture with a rotated joint origin would close it numerically.
- The `sdf` CI job adds `packages.osrfoundation.org`, the workflow's only third-party apt repository, and pins nothing: `gz-jetty-sdformat-python` follows the repo. A version pin, or a mirror, if it ever breaks a build.

### From the panels (plans/panels-and-numbers, 2026-09-03)

- One egui widget quirk is worked around in place, not upstream: `DragValue` stashes the edited text and parses it again the frame after the editor closes (a second commit, and a commit after Escape — the field clears the stash). Check it on the next egui bump, or file it. (The `Slider` half of this line went with the Joints window, ADR-0021.)
- The Materials window opens over the corner chrome — the `View | Edit` control and the toolbar, all anchored at the viewport's top-left; anchor it at the top-right instead, the corner the Joints window vacated
- Collision › "Add file…" and "Add mesh to this link…" open no dialog in the browser — the seam reads a dropped file, but there is no picker; `rfd::AsyncFileDialog` would give the web build one
- A scrubber's speed is one percent of the value with a per-unit floor (`STEP_M`, `STEP_DEG`, …); a field cannot say its own step beyond those constants, and a joint limit in degrees near zero scrubs slowly

### From the overlay (plans/overlay-tells-the-truth, 2026-09-04)

- The depth readback is full-resolution; at 4K it is 33 MB and about 5 ms of memcpy per copy, against 0.28 ms at 1440×900 (ADR-0020 §3). Downsample — which needs a second pass, and would misclassify a thin glyph's ends by a pixel — only if a 4K viewport actually bites

### From the human's GUI notes (the View / Edit split, 2026-09-05)

The notes themselves — two modes with Tab between them, the joint tree with
scrubbers in place of the Joints window, joints as the only pick in View,
the wheel driving a hovered joint, a glyph that shows range and value, a
visibility row top-right — are v0.4's second half now (03 §The window). What
that half deliberately leaves alone:

- **Custom window chrome, the way rerun draws it.** The native title bar (close / minimise / maximise) replaced by our own top bar with the menu in it: rerun's `re_ui::viewport_with_window_chrome` — `with_decorations(false)` plus a transparent surface for the rounded corners on Windows and Linux, macOS keeping its native buttons over a fullsize content view, a Wayland `xdg-decoration` probe deciding the default — and `native_window_buttons_ui` for the buttons. The surface's alpha mode is fixed at window creation, so it is a startup decision, not a toggle; the web build has no chrome to replace. Not v0.4
- **Edit beyond locking `q`.** The mode's gesture is "move the joint relative to its parent", and the gizmo on a *link* today moves its parent joint's origin with the subtree (01 §Toolbar, ADR-0007) — a second way to move the same frame that the reading would drop; meshes stay hoverable, movable and rotatable in Edit until this is decided

## Rejected

- `SetRoot` across a movable joint — a URDF always has a root, and the reversed-pivot convention is a design question nothing in M3 needed (plans/m3-sim-ready OPEN 2, 2026-08-29)
