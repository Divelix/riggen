# Architecture decision records

One file per decision, numbered, never edited after acceptance — a change of
mind is a new ADR that supersedes the old one. Format: Context, Decision,
Consequences, Alternatives considered.

| # | Title | Status |
|---|---|---|
| [0001](0001-stack-and-robocad-heritage.md) | egui + own wgpu viewport + glam, ported from RoboCAD | Accepted |
| [0002](0002-binary-in-wheel-before-pyo3.md) | Ship the binary in the wheel; PyO3 only for the headless SDK | Accepted |
| [0003](0003-headless-visual-snapshots.md) | Headless visual snapshots from day one | Accepted |
| [0004](0004-mjcf-acceptance-target-resolved-robot.md) | MJCF is the acceptance target; exporters read a convention-neutral `ResolvedRobot` | Accepted |
| [0005](0005-ids-as-counters-joints-as-edges.md) | Ids are per-document counters; joints are the edges of the link tree | Accepted |
