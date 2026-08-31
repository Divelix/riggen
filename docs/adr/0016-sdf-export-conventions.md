# ADR-0016: SDF is written at version 1.11 with `relative_to` poses, a native `<mimic>` and a native `<capsule>`, and libsdformat itself is what proves it

- Status: Accepted
- Date: 2026-09-01

## Context

`docs/03-roadmap.md`'s last v0.2 line is "MJCF import; SDF export"; the
import half landed as ADR-0015. The export half is the third dumb
serialiser of `ResolvedRobot` that ADR-0004 §Consequences promised —
"adding SDF later is a new writer, not a new resolve" — so what needed
deciding is not architecture but *conventions*: which of SDF's five living
spec versions we declare, which of its two pose conventions we write, how
ADR-0013's mimic and ADR-0014's actuator survive the trip, and, before any
of that, **whether CI can tell us we are wrong**. A writer nobody validates
is a text generator.

SDF is not URDF with different tags. Three things separate them, and all
three are opportunities rather than obstacles:

- **A pose graph.** SDF 1.7 introduced `//pose/@relative_to`, so a link's
  pose can be expressed in its parent's frame instead of the model's. A
  document may use either convention, and consumers disagree about which
  they understand.
- **Native constructs we have been apologising for.** SDF has a
  `<capsule>`, where the URDF writer emits a cylinder and a comment; a
  `<frame attached_to>`, where the URDF writer emits a massless dummy link
  on a fixed joint (ADR-0012); and, from spec 1.11, an `<axis><mimic>`
  whose algebra is exactly ADR-0013's.
- **No reference implementation in Rust, and none on PyPI.** `libsdformat`
  is the spec's own parser and is not in Ubuntu's archive.

The three questions the plan left open — the version, the pose convention,
and the checker — turned out to be one question, because the checker
decides the other two. So they were **measured** before being decided: a
hand-written three-link SDF carrying a mimic, a capsule, a frame, a
rotated link and a `world` joint, run through every candidate in an
`ubuntu:24.04` container.

| Candidate | What it did |
|---|---|
| `pybullet` (`loadSDF`, one `uv run --with`) | **Silently ignored `//pose/@relative_to`**: put the third link at 0.1 m where 0.2 m belonged and reported success. Its diagnostics are `b3Warning` lines on stdout, not exceptions; its numbers are **f32** (0.2 came back 3e-9 off), so the 1e-9 bar this project holds MJCF's round trip to is unreachable. |
| `gz sdf -k` (`libsdformat` + `gz-tools2`) | The reference parser, and it caught every deliberate break — a joint naming a missing link, a mimic naming a missing joint, a duplicate link name. But it **exits 0 regardless**, so CI would have to scrape `Error Code` out of its output. |
| **`gz-jetty-sdformat-python`** | The same parser through its own bindings. `Root.load()` **raises** `SDFErrorsException` on any error; `semantic_pose().resolve("__model__")` returns exact **f64**; `JointAxis.resolve_xyz`, `MimicConstraint`, `Frame.attached_to` and `GeometryType.CAPSULE` all read back what was written. |
| `pinocchio` from PyPI | `buildModelsFromSdf` is in the Python layer but the wheel is built without the C++ SDF parser: `AttributeError` on the first call. |

Neither `libsdformat` package is in Ubuntu's archive; both come from
`packages.osrfoundation.org`, four lines of CI and about nine seconds.

## Decision

### 1. `<sdf version="1.11">`, and the mimic is native

1.11 is the first spec with `<axis><mimic>`. Its algebra —
`follower = multiplier · (leader − reference) + offset` — is ADR-0013's
`q_follower = k·q_leader + o` exactly when `reference` is zero, which is
also the note the spec itself makes about URDF's `<mimic>`. Measured:
`libsdformat14` (Gazebo Harmonic, the LTS) and `libsdformat15` both accept
a 1.11 document and round-trip the constraint, so the newer spec costs no
audience.

The alternative was 1.10 and a comment, and it was rejected on ADR-0013's
own grounds: a coupling that survives as prose is a coupling silently
dropped, which is the failure that whole plan existed to avoid. Riggen now
writes the mimic natively in all three formats — URDF's `<mimic>`, MJCF's
`<equality><joint polycoef>`, SDF's `<axis><mimic>`.

Both optional numbers are written out rather than left to their defaults,
including `<reference>0</reference>`, so the file states the whole rule
the way the URDF writer's `multiplier` and `offset` do.

### 2. Poses are `relative_to`, and the joint has none at all

Every non-root link carries `<pose relative_to="«parent link»">`; the root
link carries no `<pose>`. The joint carries **no `<pose>` element**,
because SDF's own default is the one riggen already has: *"By default, the
pose of a joint is expressed in the child link frame"* — ADR-0004's joint
frame, spelled by omission. `<axis><xyz>` carries **no `expressed_in`**,
because its default is the joint frame, which is that same child link
frame, which is the frame `ResolvedJoint::axis` is already in.

The result is that the writer stays a serialiser with no arithmetic in it:
`ResolvedJoint::origin` is the child link's `<pose>`, `ResolvedJoint::axis`
is the `<xyz>`, `ResolvedSite::pose` is the `<frame>`'s `<pose>`, and
nothing is composed on the way out. It is also what a human reading the
file wants — the numbers still show the tree.

The alternative was flattening every link pose into the model frame, which
`pybullet` requires. It was rejected twice over: it would put an
FK-at-zero pass inside a dumb writer, and it would buy compatibility with
a reader that is already better served by the URDF the same export writes.
The consequence is stated plainly rather than hidden: **`pybullet` reads
riggen's SDF wrong.** Its users want the `.urdf`.

### 3. What each thing is written as

| `ResolvedRobot` | SDF |
|---|---|
| the robot | `<sdf version="1.11"><model name>` |
| a link | `<link name>`; non-root gets `<pose relative_to="«parent»">` = `ResolvedJoint::origin` |
| an inertial | `<inertial><pose>`(CoM)`<mass><inertia><ixx>…<izz>` — element bodies, not attributes |
| a link with `inertial: None` | no `<inertial>`; the consumer's default stands, as in URDF |
| a visual / collision geom | `<visual name="«link»_visual_«i»">` / `<collision name="«link»_collision_«i»">` with `<pose>` and `<geometry>` |
| a mesh | `<geometry><mesh><uri>` (§4) |
| a primitive | `<box><size>` (full extents), `<cylinder><radius><length>`, `<sphere><radius>`, and **`<capsule><radius><length>`** — the one place SDF beats URDF, so the "written as a cylinder" comment has no counterpart here. `length` is the cylindrical part, exactly `Primitive::Capsule::length` |
| a joint | `<joint name type>` with `<parent>` / `<child>`, no `<pose>` (§2) |
| `JointKind` | `fixed`, `revolute`, `continuous`, `prismatic` — SDF has all four by those names, so no kind is approximated |
| the axis | `<axis><xyz>`, no `expressed_in` (§2) |
| `Limits` | `<axis><limit><lower><upper>`, and `<effort>` / `<velocity>` **only when non-zero** |
| `Limits: None`, and every `continuous` joint | no `<limit>` element; SDF's defaults are ±inf, which is what "unlimited" means |
| `Dynamics` | `<axis><dynamics><damping><friction>`, written only when either is non-zero |
| `Dynamics::armature` | a comment naming it, as in URDF: SDF's `<dynamics>` has `spring_reference` and `spring_stiffness` and no armature |
| `ResolvedMimic` | `<axis><mimic joint="«leader»"><multiplier><offset><reference>0` (§1) |
| `ActuatorSpec` | a comment naming the preset and its gains (§5) |
| a `ResolvedSite` | `<frame name attached_to="«link»"><pose>` — `<pose>`'s default `relative_to` *is* `attached_to`, so the pose goes out in the link frame verbatim (ADR-0012's third spelling) |
| `floating_base: false` | `<joint name="world_joint" type="fixed"><parent>world</parent><child>«root»</child></joint>` |
| `floating_base: true` | that joint omitted; an SDF model with nothing holding it is free, as MJCF's `<freejoint/>` makes it |

A zero `effort` or `velocity` is the *unfilled* value (ADR-0014), and SDF's
default for both is infinity while a literal `0` means a joint that can
exert nothing. So they are omitted rather than written, the same rule the
MJCF writer follows for `forcerange` and `ctrlrange` — and unlike the URDF
writer, whose `<limit>` requires all four.

### 4. Mesh URIs

`MeshPathStyle` maps onto SDF's `<uri>` without a new variant:

| `MeshPathStyle` | `<uri>` |
|---|---|
| `Relative` | `meshes/«stem».stl` |
| `Package(n)` | `model://«n»/meshes/«stem».stl` |
| `Absolute` | `file:///«absolute path»` |

`model://` is SDF's own resolution scheme and the direct analogue of
URDF's `package://`, so the one dialog control keeps meaning one thing in
all three formats. The written directory is unchanged (ADR-0008): one
`.sdf` beside the same `meshes/` the other two writers already fill.

### 5. SDF has no actuator either, so ADR-0014's URDF reasoning applies verbatim

An `ActuatorSpec` becomes a comment naming the preset and its gains, the
same sentence the URDF writer emits. Gazebo *does* drive joints — through
`<plugin>` blocks naming `gz::sim::systems::JointPositionController` and
its kin — but a plugin is a simulator configuration, not a robot
description: it names a C++ class, a shared library and a version of
Gazebo, none of which are in our document, and inventing one is precisely
the fragile-exporter behaviour ADR-0014 exists to refuse. The plan's
non-goals already exclude `<plugin>`, `<gazebo>` and `model.config`, and
this ADR does not reopen them.

### 6. `libsdformat`'s Python bindings are what prove it, in a `sdf` CI job

CI installs `gz-jetty-sdformat-python` from `packages.osrfoundation.org`
and runs `python/tests/test_sdf_load.py`, the mirror of
`test_mjcf_load.py`:

1. `Root.load()` — any `SDFErrorsException` is the verdict. This is the
   "is it legal SDF" half, answered by the spec's own parser.
2. `link.semantic_pose().resolve("__model__")` for every link and
   `frame.semantic_pose().resolve("__model__")` for every frame — the
   parser's own reading of the pose graph, in f64. The test then applies
   each sampled `q` along `axis.resolve_xyz(…)` itself, the way
   `urdf.rs`'s `independent_fk` does, and compares against `arm.fk.json`.
   This is the "does it mean what `fk` means" half.
3. `axis.mimic()` against the sampled `q`, the way `check_equalities` does
   for `mjEQ_JOINT`: a pair of joints the samples show as exactly coupled
   must carry a `<mimic>`, so a dropped one fails rather than agreeing
   with itself.

The bar is **1e-9**, not the `mujoco` job's 1e-6: nothing here is a
simulator with its own integrator and its own float width, only the
reference parser's arithmetic against ours.

`pybullet` is not in CI. It cannot read what §2 decided, and a checker
that reports success on a robot it has misassembled is worse than no
checker.

## Consequences

- `riggen-export` gains `sdf.rs` beside `mjcf.rs` and `urdf.rs`, reading
  the same `ResolvedRobot` with no new field, no new resolve, and no
  arithmetic — the promise ADR-0004 made, kept.
- `xml.rs` grows text elements (`Xml::text`, `xml::pose6`), because SDF
  puts its numbers in element bodies where the other two use attributes.
  The writing half stays escaping and indentation and nothing else.
- `Format` stops being a three-valued enum: `Both` is a lie once there are
  three writers, so it becomes a set of three booleans, with `both` kept
  as a `FromStr` spelling for the two it always meant.
- CI gains one apt repository. It is the only third-party repository in
  the workflow, and it exists because the reference parser for the format
  we are writing is not packaged anywhere else.
- `pybullet` reads riggen's SDF wrong, by our choice and not by accident.
  The README and `docs/02-data-model.md` say so; the answer for a pybullet
  user is the `.urdf` that the same export writes.
- Three of riggen's apologies lose their SDF counterpart: the capsule is a
  capsule, the frame is a frame, and the mimic is a constraint. Only the
  actuator comment survives into all three formats.

## Alternatives considered

- **SDF 1.10, the version Gazebo Harmonic ships.** Safer on paper, and
  measured to be unnecessary: Harmonic's `libsdformat14` reads 1.11.
  Choosing it would have cost the mimic.
- **Flattened model-frame poses.** Every reader including `pybullet`
  understands them. Rejected in §2: an FK pass inside a dumb writer, a
  file whose numbers no longer show the tree, bought for a reader whose
  own best path through riggen is URDF.
- **`pybullet` as the CI checker**, or as a second one beside
  `libsdformat`. It ignores `relative_to` without a word, so it would have
  dictated §2 rather than checked it.
- **`gz sdf -k` rather than the Python bindings.** The same parser, but
  exit code 0 on hard errors and no numbers to do FK with; CI would scrape
  strings out of stdout and still need a second tool for the poses.
- **A Gazebo model package** — `model.config`, `<world>`, `<include>`,
  `<plugin>`. That is a distribution format, not an export, and ADR-0008's
  directory is what the other two writers fill. Backlog if a user asks.
- **SDF import.** Explicitly not in this plan. The reading direction stays
  URDF and MJCF; §6's use of `libsdformat` is a test dependency, never a
  runtime one.
