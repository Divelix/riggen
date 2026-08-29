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
- `is_closed` in the STL loader, and a warning when a mesh is not (M3's mass properties need a closed mesh; `feature::adjacency` computes it since M2)
- Meshes over 2^20 triangles: decimate at load or widen the pick id (loaders reject them today)
- Async mesh loading via `jobs` (M0 loads synchronously on the UI thread)
- Per-drop import-units dialog for mixed-unit batches (M1 has one app-wide setting, ADR-0006)
- Open the Joints window automatically when a document has a movable joint (M1 hides it under Window › Joints; the by-hand run missed it)
- Drag feedback in the link tree: a ghost of the row at the cursor and a grab cursor while reparenting (only the drop target highlights today)
- `Reparent { keep_world_pose }` at the current `q`, not the zero configuration (needs `JointState` in the command; a drag with non-zero sliders jumps)
- Clicking empty viewport space with a *joint* selected in the tree does not clear the selection
- Rename a material from the materials table (the name is the key; links reference it by name)
- `SetRoot` across a movable joint (refused today; M3 decides the pivot convention)
- Snapping *during* a gizmo drag: the handles honour the snap ladder, not just the align tool (M2 keeps the two apart — align is the mouse-only route)
- A depth-tested overlay, so a joint glyph behind a part reads as behind it (M2 draws every overlay on top)

## Rejected

(none yet)
