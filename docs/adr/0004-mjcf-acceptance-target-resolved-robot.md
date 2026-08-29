# ADR-0004: MJCF is the acceptance target; exporters read a convention-neutral `ResolvedRobot`

- Status: Accepted
- Date: 2026-08-29

## Context

The audience is RL researchers, most of whom simulate in MuJoCo (and MJX);
URDF is the interchange format everyone else (ROS, Isaac's importer, Rerun)
reads. MJCF's conventions differ from URDF's in ways that bite silently:
angles default to degrees, quaternions are `w x y z`, inertials are given
as principal moments in a principal frame, effort/velocity limits live on
actuators rather than joints, and collision meshes are convexified by the
compiler. RoboCAD's robotics design deferred this with "MJCF's body-frame
conventions differ enough that `ResolvedRobot` must stay convention-neutral;
design it then". It is now.

## Decision

1. The MVP exports both formats, from a single `resolve(&Robot) ->
   ResolvedRobot` (02-data-model) that fixes conventions — joint frame equals
   child link frame, radians, meters, inertials about CoM in link axes — and
   from which each writer is a dumb serialiser.
2. **MJCF is the acceptance target** for M3: the milestone closes when
   `mujoco.MjModel.from_xml_path` loads the sample robot with zero compiler
   warnings and `mj_forward` body poses match `riggen-core::fk`. URDF is
   verified by the `urdf-rs` round-trip FK test.
3. `<compiler angle="radian"/>` is always written; the `w x y z` quaternion
   conversion and the principal-axes decomposition live in one tested module.
4. Anything MJCF cannot express from the document without an actuator model
   (effort, velocity) is written as an XML comment naming the dropped value,
   never dropped silently.

## Consequences

- The joint-frame convention is chosen once, in favour of the URDF rule,
   because it maps one-to-one onto MJCF's nested bodies with `joint pos="0 0
   0"`; no re-rooting in either writer.
- SDF or USD later means one more writer against the same `ResolvedRobot`.
- CI runs a Python job with `mujoco` installed; that is the only Python in
  the MVP's test matrix.

## Alternatives considered

- **URDF-first, MJCF via a converter** — the converters are exactly the
  fragile step the seed document complains about, and they cannot see the
  document's intent (collision policy, hybrid inertials).
- **Model the document in MJCF's terms** — would put actuators, sites and
  defaults into the core model before the GUI needs them, and make URDF the
  lossy direction.
