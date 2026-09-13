# Plan: move-joint-frame-collision

- Started: 2026-09-13
- Milestone: v0.6 — the import gap's last mile
- Idea (verbatim from the human): "MoveJointFrame" — the roadmap line
  "`MoveJointFrame` re-expresses collision geometry too"; small and
  obvious, so no idea file.

## Goal

Moving a pivot changes no world pose in the zero configuration, collision
geometry included. `Command::MoveJointFrame` re-expresses the child link's
`CollisionPolicy::Meshes` geom poses and `CollisionPolicy::Primitives`
poses through `origin_new⁻¹ ∘ origin_old`, as it already does for the
visuals, the child joints' origins, the frames and an `Override` inertial.
Every caller gets the fix, because every caller issues that one command:
the gizmo on a joint, click-the-bore, and the SDK's `joint.move_frame`. An
imported link's collision meshes stay where they were in the world, in
the viewport and in the export.

## Non-goals

- **No repair of saved documents.** A `.riggen` whose collision was
  already shifted by an earlier pivot move stays as saved: nothing records
  where the collision used to be.
- **No change to the derived policies.** `SameAsVisual`, `ConvexHull` and
  `ConvexDecomposition` are computed from the visuals at their poses
  (ADR-0011), so they already follow. `None` has nothing to move.
- **Not the other v0.6 lines:** `validate` checking geom poses and
  `Override` numbers, and a `PackageMap` UI.
- **No other command.** `MoveJointFrame` is the only command in core that
  re-expresses a link's contents (checked at planning). `Reparent` rewrites
  a joint origin and leaves everything on the link in link coordinates.

## Design deltas

- **`riggen-core::Command::MoveJointFrame`** (DATA-MODEL §Commands and
  history). Its re-expression list gains `CollisionPolicy::Meshes` geom
  poses and `CollisionPolicy::Primitives` poses. The sentence "are not
  re-expressed and do move — a backlog line" goes. The doc comment on the
  variant says "visual and collision geom poses".
- **`riggen-core::Primitive` gains `pose()` / `pose_mut()`.** The four
  variants share the field, and core now needs to write it. The app's
  `document::primitive_pose` becomes a call to `Primitive::pose`.
- **SDK.** `Joint.move_frame`'s docstring lists collision geometry among
  what stays put. No API change.
- No ADR: this completes the invariant the command already states ("no
  world pose at `q = 0` changes"), and no decision is left open.

## Steps

Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [x] **[1]** Step 1 — The re-expression in core.
  - `Primitive::pose` / `pose_mut`; `document::primitive_pose` reads it.
  - `MoveJointFrame` composes `delta` onto every `Meshes` geom pose and
    every `Primitives` pose of the child link.
  - Tests in `command.rs`:
    - The child link carries `Meshes` with a posed geom, and another child
      carries `Primitives` holding all four shapes, each posed. After a
      move that both turns and translates, every collision world pose is
      unchanged to `EPS`, and the shape parameters (size, radius, length)
      are bitwise unchanged.
    - A `SameAsVisual` link's stored policy is unchanged.
    - One undo restores a document equal to the one before.
    - `moving_a_joint_frame_changes_no_world_pose_at_zero` still passes,
      its `world_geoms` extended to collision geoms.
  - DATA-MODEL §Commands, the variant's doc comment and the SDK docstring
    change in this commit.
- [x] **[2]** Step 2 — The app sees it.
  - A behaviour test in `tests/visual/main.rs`: import
    `assets/fixtures/arm/arm.urdf`, where `fore` carries a `Meshes`
    collision (`fore_hull.stl`) and `base` a box `Primitives`. Turn on
    the collision view, then commit a gizmo drag on `fore`'s parent joint
    and on `base`'s. One history entry each. Every `debug_state()`
    instance with `collision: true` keeps its position, as the existing
    `gizmo_drag_on_a_joint_moves_only_the_pivot` checks for visuals.
  - A new snapshot, `pivot_move_keeps_collision`: the arm with the
    collision view on, after the move, the hull on the forearm. Shown to
    the human.
  - Confirm at the step which joints to drag. If the fixture's links do
    not import as described, pick the pair that carries one policy each
    and say so here.
  - *At the step:* the links import as described and the pair is
    `base_joint` and `fore_joint`. But `fore_joint` mimics `upper_joint`
    with offset 0.1, so at `q = 0` the forearm is turned 0.1 rad and a
    pivot move swings it, visual and hull alike (≈0.8 mm here). The test
    frees that mimic with one `SetJoint` before the drags. The gap belongs
    to the command, not to collision: a backlog line, see *Open questions*.

## Acceptance

- Roadmap v0.6's clause: moving a pivot on a link with imported collision
  meshes leaves that collision where it was in the world. Step 2's
  behaviour test is that check, run on the imported `arm.urdf`.
- `cargo test --workspace` passes, including step 1's command tests and
  the `pivot_move_keeps_collision` snapshot.
- `pytest python/tests/sdk` passes (the docstring only; `move_frame`'s
  existing test is unchanged).

## Docs to update on completion

- `docs/DATA-MODEL.md` §Commands and history: the `MoveJointFrame`
  paragraph lists collision poses, and the backlog sentence is gone
  (step 1; confirm at retirement).
- `docs/ARCHITECTURE.md`:
  - the snapshot list gains `pivot_move_keeps_collision`;
  - check the gizmo and click-the-bore paragraphs, and the SDK table's
    `move_frame` row, for "geoms" meaning visuals only.
- `docs/ROADMAP.md` v0.6: the `MoveJointFrame` line marked *Landed*.
- `docs/BACKLOG.md`: remove any line still naming this gap.
- `AGENTS.md` current state: move the line from **Next** to landed.

## Open questions

None for this plan: it completes the command's stated invariant.

Found in step 2, left out of scope: `MoveJointFrame` re-expresses
through the origins as if the moved joint sat at zero, but a mimic
follower with a non-zero `offset` does not sit at zero when `q = 0`, so its
child moves in the world. Recorded in `docs/BACKLOG.md`. Fixing it means
composing the follower's resolved motion into `delta`, which touches every
re-expressed pose, not just collision.
