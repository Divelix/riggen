# Architecture decision records

One file per decision, numbered, never edited after acceptance — a change of
mind is a new ADR that supersedes the old one. Format: Context, Decision,
Consequences, Alternatives considered.

| # | Title | Status |
|---|---|---|
| [0001](0001-stack-and-robocad-heritage.md) | egui + own wgpu viewport + glam, ported from RoboCAD | Accepted |
| [0002](0002-binary-in-wheel-before-pyo3.md) | Ship the binary in the wheel; PyO3 only for the headless SDK | Accepted, amended by 0009 |
| [0003](0003-headless-visual-snapshots.md) | Headless visual snapshots from day one | Accepted |
| [0004](0004-mjcf-acceptance-target-resolved-robot.md) | MJCF is the acceptance target; exporters read a convention-neutral `ResolvedRobot` | Accepted, §4 amended by 0014 |
| [0005](0005-ids-as-counters-joints-as-edges.md) | Ids are per-document counters; joints are the edges of the link tree | Accepted |
| [0006](0006-drops-are-links-removal-takes-the-subtree.md) | A dropped mesh is a link; removal takes the subtree; import scale is an app setting | Accepted |
| [0007](0007-transform-gizmo-crate-over-our-own.md) | The gizmo comes from `transform-gizmo-egui`, bridged through `mint` | Accepted, amended by 0010 |
| [0008](0008-export-conventions.md) | Export conventions: meshes baked to meters as STL, `fullinertia`, a headless CLI export | Accepted |
| [0009](0009-one-wheel-abi3-extension-plus-binary-as-data.md) | One wheel: a PyO3 abi3 extension module plus the binary as wheel data | Accepted |
| [0010](0010-gizmo-egui-glue-is-ours.md) | The gizmo's egui glue is ours; the pointer is shared per handle | Accepted, §3 amended by 0018 and 0019, a mode policy over it in 0021 |
| [0011](0011-convex-decomposition-from-parry-vhacd.md) | Convex decomposition from `parry3d-f64`'s V-HACD; the merge step is ours; the document stores parameters, not pieces | Accepted |
| [0012](0012-frames-as-mjcf-sites-and-urdf-dummy-links.md) | A frame is an MJCF `<site>` and a URDF massless dummy link; the import does not reverse the second; frames and links share one namespace | Accepted |
| [0013](0013-mimic-joints-as-urdf-mimic-and-mjcf-equality.md) | A mimic joint is URDF's `<mimic>` and an MJCF `<equality><joint polycoef>`; no chains; a removed leader frees its followers | Accepted, "no chains" amended by 0025 |
| [0014](0014-actuators-on-the-joint-mjcf-only-three-presets.md) | An actuator lives on its joint, is MJCF-only, is named after the joint, and comes in three presets; amends 0004 §4 | Accepted, amended by 0023 and 0024 |
| [0015](0015-mjcf-import-subset-and-one-import-vocabulary.md) | MJCF import reads the subset the document can hold; `<default>` is resolved, not stored; one import vocabulary with URDF | Accepted, §5's composite-joint bullet re-affirmed by 0022, §5's composition bullet amended by 0026 |
| [0016](0016-sdf-export-conventions.md) | SDF at 1.11: `relative_to` poses, native `<mimic>`, `<capsule>` and `<frame>`; libsdformat's Python bindings prove it in CI | Accepted |
| [0017](0017-web-io-bytes-in-downloads-out.md) | Web IO: one `FileSource` seam in, downloads out, a dropped set resolved by file name, WebGPU only | Accepted |
| [0018](0018-left-drag-orbits-gizmo-claims-the-primary-drag.md) | The bare left-drag belongs to the camera; the gizmo claims the primary drag; amends 0010 §3 | Accepted, §3 amended by 0019 |
| [0019](0019-the-wheel-is-claimable-and-a-drag-keeps-the-hover-pick.md) | The wheel can be claimed by a rotate ring, and a gizmo drag keeps the hover pick; amends 0010 §3 and 0018 §3 | Accepted |
| [0020](0020-the-overlay-reads-the-scenes-depth-back.md) | The overlay reads the scene's depth back, and hidden runs dim rather than vanish | Accepted |
| [0021](0021-two-modes-and-edit-is-the-zero-configuration.md) | The window has two modes, `Tab` between them; View poses through the joint tree and the glyphs alone, Edit is the zero configuration; supersedes the per-tool `q` reset | Accepted, amended three times 2026-09-05: §4 (an all-fixed document opens in View too), §1/§6 (a hidden thing answers nothing) and zen is orthogonal to the mode (`Esc` leaves it, `chrome_rects` empty) |
| [0022](0022-composite-joints-stay-refused.md) | A `<body>` with several `<joint>`s stays refused: the corpus values the synthesis at five directories and MuJoCo rejects a massless moving body; if reopened, the writer collapses by inspection | Accepted |
| [0023](0023-actuators-are-a-model-level-table-named-in-their-own-namespace.md) | Actuators move off the joint into `Robot::actuators`, a model-level table; an actuator carries its own name, and several may target one joint; amends 0014 | Accepted, `ActuatorTarget` amended by 0025 |
| [0024](0024-general-actuator-escape-hatch.md) | `<general>` is a fourth `ActuatorSpec` variant, read iff no preset can express it; ranges move onto the actuator as a tri-state; amends 0014 | Accepted |
| [0025](0025-couplings-chains-qpos-ref-and-fixed-tendons.md) | Mimic chains resolve topologically and only a cycle is refused; `<joint ref>` is `Joint::qpos_ref`, an MJCF offset over a `q` that stays the deviation from the authored pose; `<tendon><fixed>` is `Robot::tendons` in MJCF's own `qpos` terms, with `ActuatorTarget::Tendon`; amends 0013 and 0023 | Accepted |
| [0026](0026-composition-is-resolved-at-import-never-stored.md) | Composition is resolved at import, never stored: `<include>` and `<frame>` are flattened by a pre-pass with MuJoCo's rules and errors, `<replicate>` and `<attach>` stay refused by name; amends 0015 §5 | Accepted |
