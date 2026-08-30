# ADR-0012: A frame is an MJCF `<site>` and a URDF massless dummy link; the import does not reverse the second; frames and links share one namespace

- Status: Accepted
- Date: 2026-08-31

## Context

`Frame { name, parent, pose }` has been in the schema since M1 and always
empty (`docs/02-data-model.md` §Core types). It is the TCP, the sensor
mount, the grasp pose — a named pose on a link that carries no mass and no
geometry, and that downstream tools are supposed to be able to name.

The two formats disagree about whether such a thing exists.

**MJCF has it.** `<site>` is exactly this: a named frame inside a body,
with `pos` and `quat`, no mass, no collision. `mj_forward` fills
`data.site_xpos` / `data.site_xmat`, and every MuJoCo sensor, tendon and
equality constraint names sites. There is nothing to decide but the
attributes.

**URDF does not.** There is no site, no frame, no marker element. What the
ROS ecosystem does instead — REP-120's `tool0` and `flange`, every
MoveIt `<xacro>` that defines a TCP, every `robot_state_publisher` setup —
is a `<link>` with no `<visual>`, no `<collision>` and no `<inertial>`,
attached with `type="fixed"`. `tf` then publishes the frame, MoveIt can use
it as an end-effector parent, and KDL treats the zero-mass leaf as
massless. This is a convention, not a spec, but it is *the* convention, and
a consumer that does not follow it still gets a well-formed URDF.

Two consequences need deciding rather than discovering:

1. **The import direction.** `urdf_in` reads URDF back. Should a massless
   childless link become a `Frame` again?
2. **Names.** MJCF keeps sites and bodies in separate namespaces; a site
   and a body may both be called `tcp`. URDF cannot: both are `<link>`.

## Decision

**MJCF gets a bare `<site name pos quat/>` inside its link's body, written
after the geoms, with no `size`, `group`, `rgba` or `type`.** MuJoCo's own
default is a 0.005 m sphere, so an unadorned site is a small visible dot in
`mujoco.viewer` — a TCP marker one can actually see is a feature, not an
oversight. A user who wants it bigger, invisible or in a group adds a
`<default class>` to the model; a `size` we invented would be a number
nobody asked for that overrides theirs.

**URDF gets a massless dummy link plus a fixed joint**: `<link
name="tcp"/>` and `<joint name="tcp_fixed" type="fixed">` with the frame's
pose as the `<origin xyz rpy>` and its link as the `<parent>`. All the
dummy links are written after every real link, and all the fixed joints
after every real joint, so the file still reads root-first.

**A URDF import does not turn a massless childless link back into a
frame.** The import keeps it as a `Link`.

**Frames and links are one namespace**, enforced in `validate`: a frame
name must be a valid XML name, unique among frames, and different from
every link name. The fixed joint a frame exports to, `<name>_fixed`, must
likewise not be an existing joint's name.

## Consequences

- Round-tripping our own URDF through `urdf_in` gains two links per two
  frames: the frames come back as ordinary massless links, and re-exporting
  writes them as links. This is the accepted cost of the import decision,
  and it is recorded in `docs/02-data-model.md` §URDF import so nobody
  reports it as a bug.
- The MJCF route round-trips exactly, once MJCF import exists — a `<site>`
  is unambiguous.
- CI can prove the numbers: `--fk-samples` writes each frame's world pose
  per sampled configuration and `test_mjcf_load.py` compares it against
  `data.site(name)` from `mj_forward`, to 1e-6, the same way M3 proved
  joints (ADR-0004).
- Two names the user could otherwise have chosen are now rejected up front,
  with an error naming the collision, rather than silently renamed at
  export time — the export writes what the document says or refuses.
- A frame is a pose with a name and nothing else. Anything that
  *references* a site — sensors, actuators, cameras, equality constraints —
  is later work that now has something to point at.

## Alternatives considered

- **A massless URDF link becomes a `Frame` on import.** Nothing
  distinguishes our dummy from a real massless link — a bracket the user
  has not weighed yet, a mount plate imported without an `<inertial>`, a
  `base_footprint`. Guessing wrong deletes a link, and the deletion only
  becomes visible on the next export. A heuristic ("childless *and*
  massless *and* geometry-less") narrows it without closing it, and the
  failure it leaves is the silent kind. Keeping every URDF link a link is
  wrong in one direction only, visibly, and the user can delete the link
  and add a frame in ten seconds.
- **Renaming at export time** — `tcp` the frame becomes `tcp_frame` the
  URDF link when `tcp` is already a link. The file then disagrees with the
  document and with the MJCF written beside it, and the name a downstream
  launch file hardcodes changes when an unrelated link is renamed. One rule
  checked once, at the moment the name is typed, is cheaper for everyone.
- **Separate namespaces, and a URDF-only check at export.** The same rule,
  discovered later, in the dialog, after the work — and only when the user
  exports URDF at all.
- **An MJCF `<site>` with our own `size` and `group="4"`.** Invents two
  numbers, hides the marker by default, and puts them where a `<default>`
  cannot cleanly override without repeating the class.
- **Frames as MJCF bodies with no mass.** MuJoCo would need
  `mocap="true"` or an inertia to accept a body; `<site>` exists precisely
  so this is not necessary.
