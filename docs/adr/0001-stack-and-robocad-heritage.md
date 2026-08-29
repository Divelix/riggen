# ADR-0001: egui + own wgpu viewport + glam, ported from RoboCAD

- Status: Accepted
- Date: 2026-08-29

## Context

Riggen narrows RoboCAD (`~/Documents/code/pet/cad/robocad`) from a parametric
CAD to a mesh assembler after that project's kernel proved unable to boolean
curved solids (its ADR-0013). RoboCAD's layers above the kernel — an own wgpu
renderer injected into egui via `egui_wgpu` callbacks (its ADR-0002), a
kernel-free inertia integrator, the egui UI shell and a headless snapshot
harness — are exactly what an assembler needs and took three weeks to build
and tune. The seed document originally listed `three-d`, `kiss3d` or
`bevy_egui` for rendering and `nalgebra` for math.

Rerun ships on eframe 0.36 + egui-wgpu + wgpu 30 + glam + maturin — the same
versions RoboCAD is on — and is the existence proof that this stack delivers
a fast, Python-distributed GPU viewer.

## Decision

1. UI is egui/eframe 0.36 from crates.io; the local egui checkout is reading
   material, not a dependency.
2. The 3D viewport is RoboCAD's renderer, ported: `robocad-viewport` minus the
   sketch-plane and edge/vertex pick passes, keyed by instance instead of
   B-Rep body. `three-d`, `kiss3d` and `bevy_egui` are rejected.
3. `glam` is the single math library, f64 in the document and f32 at the GPU
   boundary. The port replaces RoboCAD's `cgmath`.
4. Also ported: `mass.rs`, the UI panels that survive (mass properties,
   settings, status bar, shortcuts), eframe low-latency setup, the wasm
   scaffolding, the snapshot harness, and the docs/ADR process. Left behind:
   sketcher, kernel facade, monstertruck, the parametric document.

## Consequences

- M0 is a port, not a build; the viewport arrives already tuned for latency
  and already tested headlessly.
- We own picking, instancing and overlays. That is the price of a viewport
  that lives inside egui's frame instead of the other way around.
- `cgmath` → `glam` touches every line of viewport math once; done in the
  port, never incrementally.
- egui's known limits (no docking in-tree, egui-drawn menus, single window on
  web) are accepted; none matter for a focused single-window tool.

## Alternatives considered

- **`bevy_egui`** — Bevy owns the loop, the device and the asset system; the
  app would be a Bevy app with egui panels, the inverse of what we want, and
  a far larger binary in the wheel.
- **`three-d`** — brings its own GL-flavoured context; embedding it in an
  egui-wgpu frame means two renderers sharing a surface. Not worth it when a
  wgpu renderer already exists.
- **`kiss3d`** — dormant, no egui-wgpu integration.
- **Rerun's `re_renderer`** — the right design at industrial scale, but not a
  stable published API and dragging most of Rerun's utils with it. Borrow
  ideas (instancing, outlines), not the crate.
- **`nalgebra`** — fine, but the wgpu/egui/Rerun ecosystem speaks glam, and
  glam's f64 types cover kinematics.
