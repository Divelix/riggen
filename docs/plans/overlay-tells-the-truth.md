# Plan: overlay-tells-the-truth

- Started: 2026-09-04
- Milestone: v0.3 — the hand-feel debt (the last open bullet)
- Idea (verbatim from the human): "plan it" — `docs/03-roadmap.md` §v0.3,
  "**The overlay tells the truth.** A depth-tested overlay, so a glyph
  behind a part reads as behind it; a badge or tint on a joint glyph that
  is driven (ADR-0013) or actuated (ADR-0014), which today look like free
  joints." No idea file: the bullet is the roadmap's own, already argued.

## Goal

A joint glyph stops lying about two things. **Depth:** the part of an axis
segment, limit arc or triad arm that is behind geometry is drawn dimmed
instead of at full strength, so a pivot inside a link reads as inside it
and the eye can tell a near hinge from a far one — while staying visible
and aimable, which is why hidden runs dim rather than disappear.
**Authority:** a joint that a mimic drives (ADR-0013) or an actuator holds
(ADR-0014) is marked on the glyph itself, so the viewport says what the
Joints window already says in words (`panels/joints.rs::mimic_rule`) — that
this hinge is not free to move on its own. Folded in from the backlog,
because it is the same lie: a joint gizmo drag previews on the glyph, as a
frame gizmo drag already does.

## Non-goals

- The transform gizmo (`transform-gizmo-egui`, ADR-0007/0010) draws itself
  and is not an `Overlay`; it stays on top, undepthed.
- Snap markers, the align pick and the readout labels are cursor feedback
  and stay unconditionally on top; only glyphs are depth-tested (OPEN 4).
- The far-side box-target filter in the snap ladder (01 §Picking and
  snapping) stays as it is — an AABB corner still floats off the surface
  whether or not the overlay is depth-tested. Only that paragraph's stated
  reason changes.
- No dashed-line idiom, no hidden-line removal on the scene itself, no
  ground grid, no new format, importer or writer.
- The Joints window and Properties are already truthful about mimics and
  actuators; nothing there changes.

## Design deltas

- **`riggen-viewport`**: the offscreen depth texture gains
  `COPY_SRC`; after the scene pass, when the overlay asks for it, the frame
  copies `Depth32Float` to a mapped buffer and resolves it asynchronously
  exactly as the pick readback does (`pending_pick` / `resolve_pending_pick`,
  never a blocking poll). The viewport keeps one `DepthImage { pixels,
  size, view_proj, rect }` — **the matrix it was rendered with travels with
  it**, so a one-frame-stale image is still self-consistent and a glyph is
  classified against the depth that actually existed when those texels were
  written. `ready_for_snapshot()` accounts for a depth readback in flight,
  or the snapshot suite races it.
- **`OverlayItem`**: a per-item occlusion policy (`Occlusion::Test` /
  `Occlusion::Always`, default `Always` so nothing changes by accident) —
  landed in step 1, because it is also how an item *asks* for the depth
  image; `paint_overlay` splits a projected path at depth crossings and
  strokes the hidden runs dimmed (step 3). The split is a pure function
  over (points, depth lookup) and gets unit tests without a GPU.
- **`riggen-app/src/app/glyphs.rs`**: `JointGlyph` gains what drives it
  (`mimic: Option<JointId>`, `actuator: Option<&'static str>` — the preset's
  name, not the gains), the overlay marks it, and `joint_glyphs` takes the
  dragged pivot when a joint gizmo drag is live, the way `frame_glyphs`
  already takes `dragged_frame`.
- **`debug_state`**: `GlyphDebug` gains the driven/actuated fields and
  whether the glyph was drawn against a depth image, so a scenario asserts
  the *policy* and not only the pixels (ADR-0003).
- **ADR-0020** (step 2): how an egui-painter overlay learns depth. Records
  the readback, the staleness rule, the dimmed-not-dropped idiom, and the
  two rejected alternatives — a CPU ray-cast (no BVH in `riggen-mesh`: the
  snap ladder only affords one because the GPU pick names the triangle) and
  a wgpu line pass (wgpu has no line width; every stroke would need quad
  expansion, and labels could not follow).

## Steps

- [x] Step 1 — **The viewport keeps a depth image.** `COPY_SRC` on the
  offscreen depth texture; copy-to-buffer after the scene pass, mapped
  async like the pick, kept with the `view_proj` and rect it was rendered
  with; skipped entirely when no overlay item asks for it.
  `is_settled()` waits for it. Measure the copy at the dev
  machine's window size and in the web build, and note the numbers in the
  ADR (OPEN 1 decides full-res vs downsampled from those). Test: a headless
  kittest scenario where a point known to be inside the cube reads a nearer
  depth than its own projected z, and one outside reads the far plane.
- [x] Step 2 — **ADR-0020** from step 1's measurements: the readback, the
  staleness rule, dimmed-not-dropped, the per-item policy, the rejected
  alternatives.
- [ ] Step 3 — **Hidden runs read as hidden.**
  `paint_overlay` splits every path at crossings and strokes
  hidden runs at the dimmed strength (OPEN 2); a depth bias so a glyph
  lying exactly on a surface does not flicker. Unit tests on the splitter;
  snapshot `glyph_behind_part` — a two-link arm posed so one pivot is
  inside its link and another is in front of it.
- [ ] Step 4 — **A driven joint looks driven.** Mimic followers and
  actuated joints marked on the glyph (OPEN 3); `GlyphDebug` carries it;
  snapshot `glyph_driven_joint` over a document that has both (the
  `bracket`/sample arm already carries a mimic and two actuators — reuse
  it rather than inventing a fixture).
- [ ] Step 5 — **A joint gizmo drag previews on the glyph**
  (`docs/BACKLOG.md` line "A joint gizmo drag previews nothing"):
  `joint_glyphs` prefers the dragged pivot, so the axis, arc and triad move
  with the drag and the release changes nothing visible. Snapshot or
  `debug_state` assertion mid-drag; the backlog line is deleted in the same
  commit.

## Acceptance

`cargo test -p riggen-app --test visual` passes with `glyph_behind_part`
and `glyph_driven_joint` as new goldens, both shown to the human as images
before they are committed (ADR-0003); the splitter's unit tests pass with
no GPU; `wasm32` still builds and the web demo's console stays clean with
the readback in the loop. Then the cycle's own gate: the M2 arm built by
hand end to end produces no new entry for the v0.3 list — after which
`/retire-plan`, then `/close-cycle` for v0.3.

## Docs to update on completion

- `docs/01-architecture.md` §Layer map — the paragraph that says the
  overlay is "**not** depth-tested ... for a joint glyph inside a part that
  is the wanted behaviour" is now wrong in its first half and only half
  right in its second: hidden runs dim, they do not vanish.
- `docs/01-architecture.md` §Frame loop — the depth readback in the loop,
  beside the pick's.
- `docs/01-architecture.md` §Picking and snapping — the far-side box-target
  filter keeps its behaviour; its stated reason ("the overlay is not
  depth-tested and a bounding box floats around the geometry") loses its
  first clause.
- `crates/riggen-viewport/src/overlay.rs` module doc — the "**Not
  depth-tested**" paragraph, replaced by the policy and its default.
- `docs/02-data-model.md` — only if `JointGlyph`'s new fields need saying
  there; they are derived, not stored, so probably not.
- `docs/BACKLOG.md` — delete the joint-gizmo-preview line (step 5).
- `docs/03-roadmap.md` §v0.3 — the overlay bullet lands; "Left: the
  overlay" becomes the cycle's closing status.
- `AGENTS.md` current state — v0.3's last bullet done.

## Open questions

- ✔ OPEN 1 (agent) — **decided at step 1: full resolution.** Measured on
  the dev machine (NVIDIA discrete, Vulkan) at 1440×900: 19 µs to record
  and submit the copy, 0.28 ms to memcpy the mapped rows out, 0.49 ms for
  the whole blocking round trip — and none of it blocks, because the
  readback is spread over frames like the pick's. It is also *memoised* on
  `(view_proj, size, Scene::revision)`, so a resting camera over an
  unchanging scene pays nothing at all. What the measurement did change:
  widening every texel to `f32` cost ~1 ms a readback, so the image keeps
  the mapped bytes and a lookup steps over the row padding. The known
  ceiling is 4K (33 MB, 5.3 ms of memcpy); downsampling is the lever if it
  ever bites. Recorded in ADR-0020 at step 2; the browser figure is still
  to take, at the acceptance run.
- ⚠ OPEN 2 (human, at step 3): how a hidden run reads. Recommendation: the
  same colour at ~35% alpha, same width — the CAD idiom, and it survives
  both the light scene and the dark one. Alternatives are a thinner stroke
  or a dashed one (dashes need a new painter idiom; that is why they are
  not the recommendation).
- ⚠ OPEN 3 (human, at step 4): the driven/actuated idiom. Recommendation: a
  mimic follower's amber goes muted and its label reads `↳ <leader>`; an
  actuated joint keeps full amber and gains a small ring at the pivot with
  the preset's name (`position` / `velocity` / `motor`). A joint that is
  both shows both. Alternative: colour alone, no text — quieter, but a
  screenshot then cannot be read without the panel.
- ⚠ OPEN 4 (agent, at step 3): confirmed by the snapshot — snap markers,
  the align pick and readout labels stay `Occlusion::Always`. If the
  `glyph_behind_part` golden shows a snap marker floating confusingly, this
  reopens as an ADR amendment rather than a silent change.
