# Backlog

One line per raw idea. Picking one up means `/idea` (needs thinking) or
`/plan` (obvious); the line is removed then. Rejected ideas keep one line
below with the reason, so the same idea is not re-brainstormed.

- Convex decomposition for collision (CoACD port vs bundled binary — needs an ADR)
- Named frames / MJCF sites (TCP, sensor mounts)
- Mimic joints; actuator presets for MJCF
- MJCF import; SDF export
- Live joint-state link from a running Python script to the GUI (file or socket)
- Web demo build
- Ground grid at z = 0 in the viewport (new; robocad never had one — M0 ships the gradient background only)
- MSAA for the offscreen colour pass (new; robocad had none)
- Meshes over 2^20 triangles: decimate at load or widen the pick id (loaders reject them today)
- Async mesh loading via `jobs` (M0 loads synchronously on the UI thread)
- Per-drop import-units dialog for mixed-unit batches (M1 has one app-wide setting, ADR-0006)
- Open the Joints window automatically when a document has a movable joint (M1 hides it under Window › Joints; the by-hand run missed it)
- Drag feedback in the link tree: a ghost of the row at the cursor and a grab cursor while reparenting (only the drop target highlights today)
- `Reparent { keep_world_pose }` at the current `q`, not the zero configuration (needs `JointState` in the command; a drag with non-zero sliders jumps)
- Clicking empty viewport space with a *joint* selected in the tree does not clear the selection
- Rename a material from the materials table (the name is the key; links reference it by name)
- `SetRoot` across a movable joint (refused today; M3 decides the pivot convention)
- Snapping *during* a gizmo drag: the handles honour the snap ladder, not just the align tool (M2 keeps the two apart — align is the mouse-only route; the by-hand M2 run asked for it, wanting a joint to land on a parent bore's centre or a corner vertex)
- A depth-tested overlay, so a joint glyph behind a part reads as behind it (M2 draws every overlay on top)

### From the M2 exit gate (the by-hand arm build, 2026-08-29)

- The gizmo swallows **all** viewport pointer input, not just its own drag: with Move or Rotate active, zoom, pan, orbit and click-to-select stop working (two causes — `set_input_suppressed` is all-or-nothing, and `transform-gizmo-egui::interact` registers a click-sensing widget at the cursor *every* frame, which egui's hit test prefers over the viewport; reported three ways: dead camera, laggy-feeling zoom, clicks that only flicker the hover tint)
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
- The export dialog re-resolves (hulls included) on every option change; fine for the arm, `riggen-app::jobs` for the first big mesh (plans/m3-sim-ready non-goals: hulls synchronous and cached per `MeshId`)
- Oriented (PCA) primitive fits; today every fit starts from the AABB in the link frame and the user rotates it
- MuJoCo's joint limits are soft: a freely swinging arm overshoots `range` by a few degrees with default `solref` — not an export bug, but a "joint limits are soft in MuJoCo" note in the export dialog would pre-empt the question
- The `#[ignore]`d fixture generators (`write_arm_fixtures`, `write_arm_sample`) live in the visual test binary and need lavapipe to build; a `cargo xtask fixtures` would be lighter

## Rejected

(none yet)
