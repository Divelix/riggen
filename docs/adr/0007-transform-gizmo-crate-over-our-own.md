# ADR-0007: The gizmo comes from `transform-gizmo-egui`, bridged through `mint`

- Status: Accepted
- Date: 2026-08-29

## Context

M2 needs a transform gizmo: a link or a joint frame that can be dragged and
turned in the viewport, with drag previewing and release committing one
command. A gizmo is a lot of fiddly screen-space geometry — arrow and plane
handles, an arcball, ray-to-ray closest-point picking, per-handle
highlighting, correct behaviour at grazing angles — and none of it is
riggen's subject.

`transform-gizmo-egui` 0.11 is the obvious candidate. Three things had to be
checked before depending on it, and plans/m2-placement-ux OPEN 3 named them
as the conditions under which we would write our own instead:

1. **Pointer coexistence with the ID buffer.** Both the gizmo and the
   viewport want the mouse, and the viewport's picking is an asynchronous
   ID-buffer readback that must not fire under a gizmo drag.
2. **Two glam versions.** The crate pins `glam ^0.32`; the workspace is on
   0.30 (ADR-0001, "glam is the one math library").
3. **The wasm build check and snapshot determinism.**

## Decision

Use `transform-gizmo-egui` 0.11, behind `riggen-app/src/app/gizmo.rs`, which
is the only file in the workspace that names it.

1. **Pointer.** The gizmo's interaction widget is registered *after* the
   viewport's rect in the same layer, and egui's hit test prefers the
   widget registered last — so the gizmo takes the click. The hover is a
   separate matter: a widget underneath still reports `hovered`, so the
   viewport would keep issuing picks under an active gizmo.
   `Viewport::set_input_suppressed(bool)` turns the viewport's pointer
   handling off wholesale, and the app sets it from `Gizmo::is_focused()`
   — one frame late, which is the same lag egui's own interaction has and
   is not perceptible. The toolbar is registered after the gizmo, so the
   precedence is viewport < gizmo < toolbar.
2. **Two glam versions.** The crate's public API is entirely `mint` —
   `mint::RowMatrix4<f64>`, `mint::Vector3<f64>`, `mint::Quaternion<f64>`.
   Our glam 0.30 gains the `mint` feature and converts at the call site, so
   glam 0.32 exists in the tree but no riggen type is ever built from it and
   neither version appears in the other's signatures. `mint` is exactly the
   crate that exists for this.
3. **wasm.** `transform-gizmo` depends on `ahash` with default features,
   which include `runtime-rng` → `getrandom`; Cargo's feature unification is
   additive, so nothing we declare can turn it back off, and `getrandom` 0.3
   refuses to compile for `wasm32-unknown-unknown` until it is told which
   backend the target has. `.cargo/config.toml` sets
   `--cfg getrandom_backend="wasm_js"` for that target and riggen-app takes
   a wasm-only `getrandom` with the `wasm_js` feature. Native builds are
   untouched.
4. **Snapshots.** The gizmo draws an egui mesh from the camera matrices and
   the cursor position, all of which the scenarios already control; with the
   pointer sent away (`PointerGone`) nothing is highlighted and the frame is
   reproducible. `gizmo_move_link` and `gizmo_rotate_joint` pin it.

The adapter is thin on purpose: it converts matrices and a `Pose` in, a
`Pose` out, and owns the drag lifecycle (`preview_world` while dragging, one
`SetJoint` or `MoveJointFrame` on release). Replacing the crate later means
rewriting one file.

## Consequences

- Two `glam` versions in `Cargo.lock`, 0.30 and 0.32. `cargo tree` is
  noisier and the wasm binary carries both; neither costs correctness,
  because they never meet. ADR-0001's "glam is the one math library" still
  holds for *our* code: no crate but `riggen-mesh` names glam, and nothing
  names 0.32 at all.
- One `.cargo/config.toml` rustflag, scoped to `wasm32-unknown-unknown`.
  The web build is a build check, not a product (01 §Testing), and a
  browser build wanting a browser RNG backend is not a compromise.
- `Viewport::set_input_suppressed` is a general facility, not a gizmo hack:
  the align and place-joint tools will want the same off-switch.
- The gizmo is not depth-tested — it draws over the geometry it edits, like
  every other overlay in the viewport.

## Alternatives considered

- **Write our own gizmo.** The escape hatch OPEN 3 kept open. Rejected: the
  three risks above all resolved cheaply, and the crate is ~2000 lines of
  screen-space geometry we would otherwise write, snapshot and debug
  ourselves, for a part of the tool with no riggen-specific behaviour. The
  adapter keeps the option open at the cost of one file.
- **Vendor the crate at glam 0.30.** A fork to maintain for a version bump
  that `mint` already makes unnecessary.
- **Make the gizmo a native-only dependency and `cfg` it out on wasm.**
  Would keep `Cargo.lock` and the wasm build clean, at the cost of
  `#[cfg]`s through the app's frame loop and a wasm check that no longer
  compiles the code it is meant to guard.
