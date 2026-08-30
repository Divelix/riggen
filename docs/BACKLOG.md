# Backlog

One line per raw idea. Picking one up means `/idea` (needs thinking) or
`/plan` (obvious); the line is removed then. Rejected ideas keep one line
below with the reason, so the same idea is not re-brainstormed.

- Named frames / MJCF sites (TCP, sensor mounts)
- Mimic joints; actuator presets for MJCF
- MJCF import; SDF export
- Live joint-state link from a running Python script to the GUI (file or socket)
- Web demo build
- Convex decomposition freezes a wasm build: `Jobs` has no thread there and runs the job inline, so a browser tab would stall for the seconds V-HACD takes. Needs a web worker (RoboCAD's `InlineEval` has the same gap) — only matters if the web demo happens
- Ground grid at z = 0 in the viewport (new; robocad never had one — M0 ships the gradient background only)
- MSAA for the offscreen colour pass (new; robocad had none)
- Meshes over 2^20 triangles: decimate at load or widen the pick id (loaders reject them today)
- Manual split planes for collision geometry: cut a part by hand where V-HACD's automatic split is wrong (the convex-decomposition idea's option E, not taken — ADR-0011 chose the algorithm; this is the escape hatch for the parts it gets wrong)
- Async mesh loading via `jobs` (M0 loads synchronously on the UI thread; the thread itself exists since ADR-0011)
- Per-drop import-units dialog for mixed-unit batches (M1 has one app-wide setting, ADR-0006)
- Open the Joints window automatically when a document has a movable joint (M1 hides it under Window › Joints; the by-hand run missed it)
- Drag feedback in the link tree: a ghost of the row at the cursor and a grab cursor while reparenting (only the drop target highlights today)
- `Reparent { keep_world_pose }` at the current `q`, not the zero configuration (needs `JointState` in the command; a drag with non-zero sliders jumps)
- Clicking empty viewport space with a *joint* selected in the tree does not clear the selection
- Rename a material from the materials table (the name is the key; links reference it by name)
- Snapping *during* a gizmo drag: the handles honour the snap ladder, not just the align tool (M2 keeps the two apart — align is the mouse-only route; the by-hand M2 run asked for it, wanting a joint to land on a parent bore's centre or a corner vertex)
- A depth-tested overlay, so a joint glyph behind a part reads as behind it (M2 draws every overlay on top)
- Publish the workspace to crates.io so `cargo install riggen` installs the app: publish `riggen-mesh`, `-core`, `-export`, `-viewport`, rename `riggen-app` to `riggen` over the 0.0.1 reservation, `cargo publish --workspace` in `release.yml` (plans/m4-distribution OPEN 1; the README says `cargo install --git` until then)
- A 30-second screencast for the README, recorded after the GUI polish and before the announced release (plans/m4-distribution OPEN 2; the README ships with the hero PNG)
- macOS code signing / notarization, if the clean-VM run of the wheel hits Gatekeeper (pip-installed files carry no quarantine attribute, so an unsigned binary should run from a terminal — unverified until the human's macOS run)

### From the M2 exit gate (the by-hand arm build, 2026-08-29)

- A joint gizmo drag previews nothing: the glyph stays on the old pivot until the release commits (`preview_world` covers a link drag only — the glyph should be built from the dragged pose)
- Place joint with a *link* selected, and Align with a *joint* selected, do nothing and say nothing (each tool wants the other kind of selection; say so in the status bar, or grey the button)
- Orbit on left-drag instead of middle-drag (LMB-drag does nothing today; needs a rule that keeps click-to-select working — an idea, not a plan)
- Keyboard shortcuts for the tools (M2 ships the toolbar only)
- Turn a rotate gizmo with the mouse wheel: a fine adjustment that needs no drag
- Properties numbers as drag/scroll fields, Blender-style — wheel to step, drag to scrub with the pointer wrapping at the screen edge (M1 ships text fields with a draft buffer)
- A ViewCube in the viewport corner with the persp/ortho toggle on it (robocad has one; M0 ships the axes triad and a text label)
- WASD fly mode, and draw the orbit pivot while the camera moves (rerun's viewer is the reference; M0 ships turntable orbit only)

### From the M3 exit gate (the export run, 2026-08-29)

The by-hand half was done headlessly: both exports of the arm (`arm.riggen`,
and `arm.urdf` imported) load in MuJoCo with zero warnings, agree with our
FK, and swing under gravity for 10 s without a NaN; the interactive
`mujoco.viewer` look is the human's. What was annoying on the way:

- Properties › Inertial's tensor fields are 56 px wide and show six decimals: a kg·m² value like 2.86e-5 reads as `0.000029` and is clipped — the readout uses scientific notation, the editable fields should too, or be wider
- Interpenetrating shells (the fixture parts are a box plus a shaft, not a boolean) count the overlap twice in `mass_properties`; a note in the Inertial readout ("N geoms, overlaps counted twice") would save a puzzled minute
- `CollisionPolicy::Meshes` is read-only in the panel: per-geom collision editing (pose, remove, add a file)
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

## Rejected

- `SetRoot` across a movable joint — a URDF always has a root, and the reversed-pivot convention is a design question nothing in M3 needed (plans/m3-sim-ready OPEN 2, 2026-08-29)
