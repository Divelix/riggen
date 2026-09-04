# ADR-0020: The overlay reads the scene's depth back, and hidden runs dim rather than vanish

- Status: Accepted
- Date: 2026-09-04

## Context

A joint has no geometry. Its glyph — an axis segment through the pivot, an
origin triad, a limit arc with a tick at `q` — is the only place it exists
in the viewport, and it is drawn with egui's painter over the finished
wgpu frame (`riggen-viewport/src/overlay.rs`). egui's painter has no depth
buffer, so until now every overlay item was unconditionally on top.

That reads as a lie in two directions. A pivot *inside* a link draws over
the link, so it looks like it is floating in front of it. Two hinges on
opposite sides of the same part draw identically, so the near one and the
far one cannot be told apart, and the arc of the far one appears to wrap
around the outside of the geometry it is actually buried in. On the M2 arm
this is the difference between "the shoulder is where I put it" and "the
shoulder is somewhere along this line".

The overlay is also the only thing in the viewport that can answer this
question at all. The scene is one wgpu pass with a real depth attachment;
the gizmo is drawn by `transform-gizmo-egui`, which is not an `Overlay`
and stays on top by its own rules (ADR-0007, ADR-0010).

## Decision

**The viewport copies its own depth attachment back and hands it to the
overlay, and a run that is behind geometry is drawn dimmed, not dropped.**

1. **The offscreen depth texture gains `COPY_SRC`, and the frame copies it
   to a mapped buffer** — the same shape as the ID-buffer pick
   (docs/01-architecture.md §Picking and snapping): recorded during the
   paint callback, mapped with `map_async`, taken whenever wgpu has filled
   it in, never waited on. A readback that never lands is abandoned after
   eight frames, exactly as `MAX_PICK_FRAMES` abandons a pick, so one lost
   surface cannot wedge the overlay for the session.

   Two things differ from the pick. The copy goes on **egui's** encoder,
   after the scene pass, so it sees the depth this frame wrote rather than
   last frame's; that means `map_async` cannot be called from inside the
   callback (a submit that touches a mapping buffer is invalid) and instead
   waits for the next `ui()`, gated on a flag the callback sets. And the
   copy is only asked for on a frame that will actually render — a
   logic-only harness pass (`tests/visual/harness.rs::settle`) would
   otherwise queue a request nobody ever answers.

2. **The matrix travels with the image.** A `DepthImage` carries the
   `view_proj`, rect and `pixels_per_point` it was rendered with, and a
   glyph is classified by projecting it through *those*, not through this
   frame's camera. The readback is at best one frame old; carrying its own
   matrix makes a stale image still self-consistent — the answer is "was
   this point behind geometry when those texels were written", off by one
   frame of camera motion, instead of "where is this point now, against
   depth from somewhere else", which is off by however far the camera
   moved. During an orbit the glyph's dimming lags the geometry by a frame;
   at rest it is exact, which is when it is read.

3. **Full resolution, memoised on what it depends on.** The image is
   copied at the viewport's own pixel size, and re-copied only when
   `(view_proj, size, Scene::revision)` changes — the same memo the hover
   pick keeps on `PickInputs`. `Scene` gains that revision counter, bumped
   by every change to what is drawn, where it sits, or whether it is
   visible. A resting camera over an unchanging scene therefore reads the
   depth buffer back **once**, not once a frame.

   Measured on the dev machine (NVIDIA discrete, Vulkan), per readback:

   | Viewport | Bytes | Record + submit | Memcpy out | Blocking round trip |
   |---|---|---|---|---|
   | 1440×900 | 5.3 MB | 19 µs | 0.28 ms | 0.49 ms |
   | 2560×1440 | 14.8 MB | 13 µs | 0.76 ms | 1.11 ms |
   | 3840×2160 | 33.2 MB | 18 µs | 5.3 ms | 6.0 ms |

   Only the first column is on the render thread; the rest is spread over
   the following frames, and none of it blocks. In the browser (the release
   `web` profile on the same machine, Chromium/WebGPU/Vulkan) the demo
   holds 60 fps at rest and 62 fps through a sustained left-drag orbit —
   the case where the memo misses and the readback fires every frame — with
   a clean console throughout. That is a vsync-bound reading, so it shows
   no dropped frames rather than how much headroom is left. What the measurement did
   change: widening every texel into a `Vec<f32>` cost about a millisecond
   a readback on its own, so `DepthImage` keeps the **mapped bytes** and a
   lookup steps over the 256-byte row padding. 4K is the known ceiling and
   a half-resolution image is the lever if it ever bites; it is not paid
   for now, because a glyph is a one-pixel-wide line whose ends a half-res
   image would misclassify.

4. **Depth is per item, and `Always` is the default.** `OverlayItem`
   carries an `Occlusion` — `Always` or `Test` — and nothing is
   depth-tested by accident. Joint glyphs and frame-glyph triads ask for
   `Test`: they claim to be somewhere in the scene, so they have to look
   it. Snap markers, the align pick, readout labels and a frame's name stay
   `Always`: they answer "where is the pointer", and hiding a marker behind
   the part the pointer is aiming at would answer nothing. The presence of
   any `Test` item is also what makes the viewport read the depth buffer
   back at all.

5. **A hidden run is dimmed, not dropped.** A path is split at its depth
   crossings and the hidden runs are stroked in the same colour at ~35%
   alpha, the same width. A glyph inside a part still has to be *visible*
   and still has to be *aimable* — `glyph_at` hit-tests the screen-space
   line, and a run that vanished would be a target the user can hit but
   cannot see. Same width for the same reason: the drawn line and the hit
   target must not disagree.

6. **A relative depth bias.** A glyph lying exactly on a surface — a pivot
   placed by a snap, a frame triad on a face — projects to the depth the
   geometry wrote, and without slack it would flicker on rounding alone.
   The slack is a fraction of the NDC distance left to the far plane
   (`DEPTH_BIAS * (1 - stored)`), which under a perspective projection is a
   constant *relative* margin in eye space: about a part in a thousand of
   the distance to the camera, wherever the part is. A constant NDC bias
   would be far too small near the camera and far too large at the far
   plane.

7. **A point the image cannot answer for reads as visible.** Behind the
   camera, off the rect, no image yet: the overlay draws at full strength.
   An overlay that is unsure must not dim, or a glyph fades for a reason
   the user cannot see on screen.

## Consequences

- Only the *opaque* scene pass writes depth. The background, the axes
  triad, the hover and select restyles and every `Translucent` instance
  draw with `depth_write_enabled: false`, so a collision preview
  (View › Collision geometry, ADR-0011) does not hide the glyphs inside it.
  That is the wanted behaviour and it falls out of the existing pipelines
  rather than being arranged.
- `Viewport::is_settled()` is false while a depth readback is in flight, so
  the snapshot harness waits for it; a scenario that wants the dimming in
  its golden has to `pump_rendered`, because the copy rides on the paint
  callback and `settle` never reaches one.
- A glyph's dimming lags an orbit by one frame (§2). During the drag the
  camera is what the eye is following; at rest the answer is exact.
- `GlyphDebug` reports whether it was drawn against a depth image, so a
  scenario asserts the policy and not only the pixels (ADR-0003).
- The far-side box-target filter in the snap ladder keeps its behaviour but
  loses half its stated reason: an AABB corner floats off the surface
  whether or not the overlay is depth-tested.
- WebGPU allows `copyTextureToBuffer` from `depth32float` with the
  depth-only aspect, so the web build (ADR-0017, WebGPU only) needs no
  second path. wgpu's GL backend does not — and neither does the pick's
  `R32Uint` readback, which is why the demo is WebGPU only already.

## Alternatives considered

- **A CPU ray cast per glyph point.** No GPU readback, no staleness, exact.
  Rejected because `riggen-mesh` has no BVH: the snap ladder can afford a
  ray cast only because the GPU pick already names the triangle to test.
  Casting a ray per point of every glyph, every frame, against every
  triangle in the scene is the one shape of the problem that gets *worse*
  as the robot gets bigger, and building an acceleration structure for it
  is a larger change than reading back a buffer the GPU already filled in.
- **Draw the overlay in wgpu, as a depth-tested line pass.** Exact, no
  readback, no staleness. Rejected because wgpu has no line width: every
  stroke would need quad expansion in a shader, arcs would need
  tessellation on the GPU or a vertex buffer rebuilt per frame, and the
  labels could not follow at all — text is egui's, and half the overlay
  drawn in one system and half in another is two idioms to keep in step.
  It also throws away the property that makes the overlay trustworthy: it
  goes through `Viewport::project`, the same matrix the pass rasterized
  with, and so cannot disagree about where a point is.
- **Hide the run entirely, as a proper hidden-line renderer would.**
  Rejected in §5: a joint inside a part still has to be reachable, and the
  hit test measures against the whole line.
- **Dash the hidden run, the drafting convention.** Unambiguous, and it
  needs a dash idiom the egui painter does not have — a second splitter on
  top of the depth splitter, and points and labels could not follow it.
  Dimming survives both the light scene and the dark one and costs one
  alpha.
- **A half-resolution or quarter-resolution depth image.** Cheaper by 4×,
  and it needs a second pass to produce. Rejected at the measurements in
  §3: full resolution is 19 µs on the render thread at the dev machine's
  window size, and a glyph is a thin line whose ends a downsampled image
  would misclassify by a pixel. Recorded as the lever for 4K.
- **Read the depth back every frame, no memo.** Simpler by one struct.
  Rejected because it puts a megabyte-scale copy in the steady-state frame
  budget for an image that is bit-for-bit identical to the one already
  held; the pick made the same call one plan earlier for the same reason.
