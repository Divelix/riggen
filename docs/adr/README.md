# Architecture decision records

One file per decision, numbered, never edited after acceptance — a change of
mind is a new ADR that supersedes the old one. Format: Context, Decision,
Consequences, Alternatives considered.

| # | Title | Status |
|---|---|---|
| [0001](0001-stack-and-robocad-heritage.md) | egui + own wgpu viewport + glam, ported from RoboCAD | Accepted |
| [0002](0002-binary-in-wheel-before-pyo3.md) | Ship the binary in the wheel; PyO3 only for the headless SDK | Accepted, amended by 0009 |
| [0003](0003-headless-visual-snapshots.md) | Headless visual snapshots from day one | Accepted |
| [0004](0004-mjcf-acceptance-target-resolved-robot.md) | MJCF is the acceptance target; exporters read a convention-neutral `ResolvedRobot` | Accepted |
| [0005](0005-ids-as-counters-joints-as-edges.md) | Ids are per-document counters; joints are the edges of the link tree | Accepted |
| [0006](0006-drops-are-links-removal-takes-the-subtree.md) | A dropped mesh is a link; removal takes the subtree; import scale is an app setting | Accepted |
| [0007](0007-transform-gizmo-crate-over-our-own.md) | The gizmo comes from `transform-gizmo-egui`, bridged through `mint` | Accepted, amended by 0010 |
| [0008](0008-export-conventions.md) | Export conventions: meshes baked to meters as STL, `fullinertia`, a headless CLI export | Accepted |
| [0009](0009-one-wheel-abi3-extension-plus-binary-as-data.md) | One wheel: a PyO3 abi3 extension module plus the binary as wheel data | Accepted |
| [0010](0010-gizmo-egui-glue-is-ours.md) | The gizmo's egui glue is ours; the pointer is shared per handle | Accepted |
