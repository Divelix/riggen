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
- Vertex welding + `is_closed` for STL (M3's mass properties need a closed mesh)
- Meshes over 2^20 triangles: decimate at load or widen the pick id (loaders reject them today)
- Async mesh loading via `jobs` (M0 loads synchronously on the UI thread)

## Rejected

(none yet)
