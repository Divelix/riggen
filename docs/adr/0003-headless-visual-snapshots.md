# ADR-0003: headless visual snapshots from day one

- Status: Accepted
- Date: 2026-08-29
- Carries over RoboCAD's ADR-0014, which verified the mechanism.

## Context

This repo is developed by an AI agent that can read source and run tests but
cannot see the window. In RoboCAD, layout regressions, overlay projection bugs
and render-state bugs were all reported by a human describing the screen — a
slow, lossy channel — until `egui_kittest` was wired to drive the real
`eframe::App` through wgpu on a CPU adapter (lavapipe) and diff PNGs. It was
verified there that an `egui_wgpu` paint callback with an offscreen colour
pass, an ID-buffer pick pass with async readback and a blit renders correctly
headlessly, and that the CPU adapter makes local and CI agree.

Riggen's M2 (gizmos, snapping, joint glyphs) is precisely the class of work
that is invisible to an agent without this.

## Decision

The `egui_kittest` snapshot suite and the `debug_state()` JSON dump ship in
M0, before any feature. Every UI milestone adds scenario snapshots; the M2
acceptance test is a scripted snapshot scenario. A snapshot that changes is
reviewed by the human as an image, not described.

## Consequences

- `riggen-app` carries `egui_kittest` (wgpu + snapshot + eframe features) as a
  native-only dev-dependency and `pollster` to probe for an adapter.
- Committed PNGs in `tests/snapshots/`; a `visual_scratch` test that renders
  on demand and compares against nothing, for "show me the app right now".
- CI needs `mesa-vulkan-drivers`; the RTX 5090 on the dev machine is not what
  the tests render with, by design.
