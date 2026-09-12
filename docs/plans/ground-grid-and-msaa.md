# Plan: ground-grid-and-msaa

- Started: 2026-09-12
- Milestone: v0.5
- Idea: none — both items are already specified in `docs/ROADMAP.md`'s v0.5
  section ("A ground grid at z = 0", "MSAA on the offscreen colour pass"),
  small and uncontested enough to skip `/idea`.

## Goal

Two of the six v0.5 viewport bullets land: the offscreen scene pass
multisamples (an edge reads smooth, not jagged, at any window size) and a
grid at z = 0 gives the viewport a floor, so a part dropped at the origin
reads as standing on something instead of floating in the gradient
background. Both are viewport furniture — no document field, no ADR, no
change to `riggen-core`/`riggen-export`.

## Non-goals

- Rotate-drag snapping (the third "Left" item in AGENTS.md) — still
  contested (which axis aligns, a second overlay idiom), stays a backlog
  line for its own plan.
- Any new format, importer, writer, or distribution work — v0.5's own "Out"
  list.
- A snap quantum, docking, theming, a second renderer — `§What not to spend
  agent time on`.
- wasm/WebGL2: the offscreen target and its formats are unchanged by this
  plan; MSAA support is queried per-adapter and falls back to 1 wherever
  the adapter can't do it (including, if it ever comes to it, a WebGL2
  fallback — not exercised here since the wasm check only builds, it
  doesn't render).

## Design deltas

`docs/ARCHITECTURE.md` §Frame loop is the only design doc that changes —
both features live entirely in `riggen-viewport` plus a `debug_state()`
field in `riggen-app`.

- **MSAA.** The offscreen colour texture and `Depth32Float` texture
  (`OffscreenTarget`, `viewport/gpu_state.rs`) become multisampled at a
  count chosen once, in `Viewport::new`, by querying
  `wgpu::Adapter::get_texture_format_features` for both the offscreen
  colour format and `DEPTH_FORMAT`; 4 if both report
  `TextureFormatFeatureFlags::MULTISAMPLE_X4`, else 1 (lavapipe, the
  reference adapter per ADR-0003, is expected to say yes, but the query
  replaces an assumption with a fact). Every pipeline that draws into the
  scene pass (`background`, `scene`, `translucent`, `hover`, `select`,
  `axes`) is built with that `sample_count`; `pick` and `blit` stay at 1 —
  the pick pass already stays single-sampled by roadmap decision (an
  `R32Uint` id buffer cannot be resolved), and the blit's *source* is
  necessarily single-sampled already (below).
  - **Colour**: the multisampled colour texture gets a `resolve_target` —
    a new single-sampled texture, `TEXTURE_BINDING | RENDER_ATTACHMENT` —
    set on the scene pass's colour attachment; `blit_bind_group` samples
    that resolve texture instead of the multisampled one it samples
    today (a multisampled texture cannot be `textureSample`d in `blit.wgsl`
    the way a regular one is).
  - **Depth**: WebGPU/wgpu has no depth-resolve attachment, and
    `copy_texture_to_buffer` cannot read a multisampled texture at all —
    so the overlay depth readback (ADR-0020, `viewport/depth.rs`) cannot
    point at the multisampled depth texture the way it points at today's
    single-sampled one. A small resolve pass (new `depth_resolve.wgsl`, a
    fullscreen triangle) reads the multisampled depth texture as
    `texture_depth_multisampled_2d` via `textureLoad(t, coord, 0)` —
    sample 0, not an average: `depth.rs`'s existing `DEPTH_BIAS` slack
    already exists for exactly this kind of near-surface rounding, and a
    blended depth would invent a value no sample actually wrote, the same
    reasoning the roadmap already gives for why the pick pass stays
    single-sampled — and writes it via `@builtin(frag_depth)` into a new
    single-sampled `Depth32Float` texture with `COPY_SRC`. `depth_copy`
    (`render_pass.rs`) copies from *that* texture; `depth.rs` itself does
    not change — the resolved texture is still one `Depth32Float` texel
    per pixel; `DepthImage` and everything the overlay does with it is
    untouched.
  - Falling back to `sample_count = 1` degrades to exactly today's
    pipeline shape (no resolve texture, no resolve pass, `blit_bind_group`
    samples the plain colour texture) — no feature flag, the branch is the
    adapter's own answer.
  - `debug_state()` reports the chosen sample count (a new field, e.g. on
    `DebugState` or a small struct beside `CameraDebug`), so a scenario or
    `visual-debug` capture can assert MSAA is actually on rather than
    inferring it from pixels.
- **Ground grid.** A new draw inside the existing `scene_pass` (the same
  `wgpu::RenderPass` the background, opaque/translucent instances, hover/
  select and the axes gizmo already share), after the translucent
  instances and before hover/select — depth-tested `LessEqual` against
  the depth the opaque pass wrote, no depth write, alpha-blended: the same
  pipeline shape `build_highlight_pipeline` already builds for the
  translucent collision pass, minus the per-instance model bind group
  (group 0 only, like `background`). A new fullscreen-triangle shader
  (`grid.wgsl`) unprojects each pixel through `inv_view_proj` — the same
  near/far unprojection `background.wgsl`'s perspective branch already
  does, which needs no ortho/perspective split to be correct (that split
  in `background.wgsl` is a *stylistic* choice for its directional colour
  blend, not a mathematical necessity) — intersects the ray with the
  world z = 0 plane, discards where the plane is behind the camera or
  beyond the far clip, and shades a two-scale line pattern (metre lines,
  a coarser subdivision) antialiased with `fwidth`, fading to transparent
  toward the horizon. Writes its own `@builtin(frag_depth)` from the
  intersection so the depth test against real geometry is correct: a part
  in front of the grid hides it, the grid hides the background behind it.
  Two palettes (light/dark) matching `background.wgsl`'s existing
  `is_dark_mode` uniform. Not an instance, not pickable, no `PickHit` —
  furniture like the background and the axes triad, and like them drawn
  unconditionally, **including zen** (it is not in `chrome_rects`).
  Reuses whatever `sample_count` the MSAA step established, since it
  draws in the same pass with the same pipeline shape.
- No ADR for either — both are mechanism, not policy, within decisions
  already on the books (ADR-0003's snapshot mandate, ADR-0020's depth
  readback). ⚠ OPEN below covers the one genuinely new mechanism (the
  manual depth resolve) in case it turns out to need one anyway.

## Steps

Complexity: **[1]** routine — the design says exactly what to write, the
tests are mechanical; **[2]** careful — a case to get right within a given
design; **[3]** unproven — behaviour that has to be established here.

- [x] **[3]** Step 1 — MSAA on the offscreen scene pass. Adapter
  format-feature query with the 1/4 fallback (unit-testable as a pure
  function over `TextureFormatFeatureFlags`); multisample every scene-pass
  pipeline; colour resolve target feeding the blit; the manual
  `depth_resolve.wgsl` pass feeding the existing depth-copy/overlay path
  unchanged; the sample count in `debug_state()`. Regenerate every
  snapshot golden in this same commit (`UPDATE_SNAPSHOTS=1`) — the whole
  suite moves at once because antialiasing changes every pixel near an
  edge — and commit it typed `snapshots:` per the roadmap's own accept
  note ("that refresh is one `snapshots:` commit that says so and nothing
  else").
- [x] **[2]** Step 2 — the ground grid. `grid.wgsl`, its pipeline (built
  once, at the sample count Step 1 chose), the draw call in `scene_pass`,
  the light/dark palette. At least one new or extended snapshot scenario
  that puts a part above the grid next to one resting on it, so the depth
  cueing is asserted rather than merely visible; every existing scenario's
  goldens refresh alongside it (the grid is now in frame everywhere the
  ground is). Commit typed `snapshots:` for the same reason as Step 1.

## Acceptance

`cargo test --workspace` passes with both features on and every snapshot
golden regenerated. By hand (`visual-debug`, per AGENTS.md — never asked of
the human): the sample arm reopened shows an antialiased silhouette at
1440×900 and `debug_state()` reports a sample count > 1 on this adapter;
dropping a part at the origin next to one lifted above it shows the first
sitting on drawn grid lines and the second's shadow-less gap above them,
in both light and dark mode; the pick pass still resolves the right
instance/triangle under both features (existing pick tests unaffected,
since `pick`/`blit` formats and sample counts are untouched by the colour/
depth resolve change).

## Docs to update on completion

- `docs/ARCHITECTURE.md` §Frame loop — the offscreen colour+depth pair
  becomes colour+depth (multisampled) + colour-resolve + depth-resolve;
  the new grid draw and its "furniture, not chrome" status.
- `docs/ROADMAP.md` v0.5 — mark "A ground grid at z = 0" and "MSAA on the
  offscreen colour pass" `*Landed (plans/ground-grid-and-msaa)*`, matching
  the two bullets already marked that way.
- `AGENTS.md` "Current state" — drop "ground grid, MSAA" from the "Left:"
  line, keeping "rotate-drag snapping" (the one item this plan doesn't
  touch).

## Open questions

- ~~⚠ OPEN: the manual depth-resolve pass~~ — **resolved in Step 1, as
  designed, and no ADR.** `texture_depth_multisampled_2d` +
  `textureLoad(t, coord, 0)` + `@builtin(frag_depth)` compiles and runs on
  lavapipe with no validation complaint, and the resolved image answers
  identically to the single-sampled one it replaces: every JSON golden's
  depth-derived value (`pivot_hidden`, the glyph occlusion classes) came
  back byte-identical across the refresh, only the new `sample_count` line
  changed. Nothing about ADR-0020's policy moved — the overlay still reads
  one `Depth32Float` texel per pixel and `depth.rs` is untouched — so this
  stays implementation, recorded in `docs/ARCHITECTURE.md` §Frame loop at
  retirement.
- Step 1 refined one detail of *Design deltas*: the sample-count query also
  requires `MULTISAMPLE_RESOLVE` on the colour format, not just
  `MULTISAMPLE_X4` on both. The blit samples the resolve target, so a
  format that multisamples but cannot be resolved is no use to this
  pipeline; the spec guarantees the two together for every renderable
  colour format, so the extra term never costs a working adapter anything.
- Step 2 note, for the docs at retirement: the two lattices are fixed at
  **1 m and 10 m**, which is the plan's "metre lines, a coarser
  subdivision" taken literally. It suits a robot; it means the floor goes
  blank when the camera is close enough to a 5 cm part that no metre line
  is in frame. A decade-adaptive spacing would fix that and is a candidate
  backlog line, not a change this plan authorised.
- The `sample_count == 1` fallback has no test of its own and no way to be
  forced — the plan's *Design deltas* rules out a feature flag, so the
  branch is only ever taken by an adapter that cannot multisample, and this
  machine's can. It is structurally the pre-MSAA pipeline shape.
