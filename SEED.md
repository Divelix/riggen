# Project Seed Document: Riggen

## 1. Project Overview
**Name:** Riggen
**Tagline:** The blazingly fast, lightweight robot assembler for RL researchers.
**Core Concept:** `Riggen` is a native GUI tool plus Python SDK for Reinforcement Learning and Deep Learning roboticists. It lets users import mesh files (STL/OBJ), visually assemble them into a kinematic tree, define and test joints, compute sim-ready inertials and collision geometry, and export the result as MuJoCo MJCF or URDF. It also imports existing URDF models so they can be inspected, fixed and converted.

`Riggen` is the deliberate narrowing of RoboCAD (`~/Documents/code/pet/cad/robocad`), a robotics-first parametric CAD. That project hit a wall in its B-Rep kernel (see its ADR-0013); everything it built *above* the kernel — the wgpu viewport, mass properties, UI shell, test harness — is exactly what an assembler needs, and is carried over (§8).

## 2. The Naming Rationale
The name **"Riggen"** is derived from the 3D animation and mechanical term "Rig" (the process of adding a kinematic skeleton and joints to static meshes so they can move). The `-en` suffix gives it an action-oriented, verb-like quality (like *fasten* or *sharpen*). It is short, lowercase-friendly (`uv run riggen`), and fits alongside modern developer tools like `ruff`, `uv`, and `rye`.

Name availability (checked 2026-08-29): `riggen` is free on both PyPI and crates.io. Register both early.

## 3. The Problem Statement
Assembling a custom robot for simulation is a recurring friction point for RL researchers:
* **CAD is Overkill:** SolidWorks, Onshape and Fusion are built for manufacturing, not simulation. They are heavy, licensed, and their URDF exporter plugins (`sw_urdf_exporter`, `fusion2URDF`, `onshape-to-robot`) are a fragile extra step that regularly mangles frames, inertials or mesh paths.
* **Manual XML is Tedious:** Writing URDF/MJCF by hand is slow. Tuning joint anchors, axes, limits and origins in a text editor is trial and error against a simulator that fails cryptically.
* **"Loads" is not "sim-ready":** A model that parses is not a model that simulates well. MuJoCo needs convex collision geometry, positive-definite inertia tensors that satisfy the triangle inequality, and consistent frame conventions. Most tools stop at emitting XML.
* **The common case is editing, not building:** Most researchers start from an existing description (MuJoCo Menagerie, a vendor URDF) and need to tweak, fix or convert it — not assemble from scratch.

## 4. Competitive Landscape (as of August 2026)
The "GUI that turns meshes into URDF/MJCF" category is no longer empty. Riggen has to justify itself against these:

| Tool | What it is | What it lacks |
|---|---|---|
| [URDF_kitchen](https://github.com/Ninagawa123/URDF_kitchen) (beta2, ~250★) | PyQt node-graph assembler: load STL/OBJ/DAE, mark connection points, connect nodes, export URDF + MJCF, colliders, inertia | Qt; node-graph rather than direct 3D manipulation; no programmatic API; no import of existing models |
| [URDF-Studio](https://github.com/OpenLegged/URDF-Studio) (OpenLegged / D-Robotics, Jun 2026) | Web modeler with skeleton/detail/hardware workflow, motor library, MuJoCo export, AI assistant | Browser only; no local Python workflow; backed by a hardware vendor |
| [URDF Studio (deyuf)](https://urdf.deyuf.org/docs/) + VS Code extension | Browser import/edit/convert of URDF, Xacro, MJCF, SDF, USD | Editor for existing files, not an assembler from meshes |
| [mujoco-viewer (VS Code)](https://github.com/julien-blanchon/mujoco-viewer), [mujoco-scene-editor](https://github.com/markusgrotz/mujoco-scene-editor) | Live MJCF preview/edit with gizmos and inspectors | MJCF-text-first; no inertia/collision pipeline from raw meshes |
| [onshape-to-robot](https://github.com/Rhoban/onshape-to-robot), [fusion2URDF](https://github.com/Adriaeik/fusion2URDF) | Mature CAD-plugin exporters (URDF/SDF/MJCF) | Require the CAD subscription and the CAD's assembly semantics; the "fragile exporter" step itself |
| [Rerun ≥ 0.24](https://rerun.io/docs/howto/logging-and-ingestion/urdf) | Built-in URDF loader; animates joints from logged angles | View-only by design. Complementary: riggen's output is what you log to Rerun |
| Blender/Phobos, Isaac Sim URDF importer | Full DCC / full simulator | Bloated, steep learning curve, not the tool you open for a five-minute fix |

**Why Riggen anyway — the five differentiators, in priority order:**
1. **Python SDK + GUI, Rerun-style.** RL researchers live in Python. `riggen.Robot().add_link(...).add_joint(...).export_mjcf()` in a script, with the GUI for the parts that are miserable in text (axis placement, limits, eyeballing collisions). No competitor has both.
2. **A git-friendly document as source of truth.** A `.riggen` file (JSON) that is diffable, hand-editable and regenerable. URDF and MJCF are *projections* of it, never the thing you edit.
3. **Sim-ready is a feature, not a claim.** Convex hulls / decomposition for collision, inertia sanity checks that block export with an explanation, frame conventions verified mechanically by a round-trip test (export → `urdf-rs` parse → FK → compare against our FK) in CI.
4. **Import existing URDF, edit, export MJCF.** Meets users where they actually are.
5. **`pip install`, native GPU window, sub-second launch.** No Qt, no browser, no Electron. True but thin on its own — it is the *floor*, not the pitch.

**Explicit non-goals (v1):** USD/Isaac export, physics/dynamics, collision *checking* (visual interference only), parametric modelling (mesh primitives may return later without a B-Rep kernel), a web build as a product (it stays a CI build check), closed kinematic loops.

## 5. The Solution & User Experience (UX)
`Riggen` offers a "Rerun.io-like" developer experience:
* **Zero Friction Distribution:** A Python wheel. `pip install riggen` / `uv add riggen`.
* **Instant Launch:** `riggen` or `python -m riggen` in a terminal. A native, GPU-accelerated window opens instantly. `riggen robot.urdf` opens an existing model.
* **Focused UX:** Drag and drop meshes, snap them together, define joints with gizmos, drag sliders to test joint limits, click "Export MJCF".
* **Scriptable (v0.2):** `import riggen` in a notebook or training script; `riggen.show(robot)` spawns the GUI on the current document, like `rr.spawn()`.

## 6. Core Features
### MVP (v0.1)
1. **3D Canvas:** Drag-and-drop STL/OBJ meshes into a lightweight, low-latency wgpu viewport with picking, orbit camera and standard views.
2. **Kinematic Tree Builder:** Parent/child links in a tree panel; links group one or more meshes that move rigidly together.
3. **Joint Configuration:** Fixed, revolute, continuous, prismatic. Axis and origin placed with a transform gizmo, snapped to mesh features (bounding-box faces/centers, circle-fit on selected geometry), or typed numerically.
4. **Visual Joint Testing:** One slider per non-fixed joint respecting limits; FK drives the viewport with instance transforms only (no mesh reprocessing).
5. **Snapping & Alignment:** Align origins / bounding boxes / picked points between meshes.
6. **Inertials:** Computed from the closed mesh at a user-set density (volume, CoM, full tensor), with `Override` and `Hybrid` (computed tensor scaled to a weighed mass) modes. Validation at export: non-zero mass, positive-definite, triangle inequality.
7. **Collision Geometry:** Per link: same-as-visual, convex hull, or fitted primitives (box/cylinder/sphere/capsule). Convex decomposition is post-MVP.
8. **Export:** MJCF (acceptance target — the RL audience and the harder frame conventions) and URDF from one convention-neutral `ResolvedRobot`. Meshes written next to the XML with configurable path style.
9. **Import:** URDF (via `urdf-rs`), so the edit → convert workflow works from day one. MJCF import is post-MVP.
10. **Document:** `.riggen` JSON file, save/open, undo/redo.

### Post-MVP
* Python SDK (headless core: build, validate, export, FK) and `riggen.show()` — v0.2.
* Convex decomposition (CoACD-style), mesh decimation for visual/collision tiers.
* MJCF import; SDF export.
* Named frames (TCP, sensor mounts) exported as fixed links / MJCF sites.
* Mimic joints, actuators/motor presets for MJCF.
* Web build as a shareable demo.

## 7. Tech Stack (decided)
The stack is the one Rerun ships on today (eframe 0.36 + egui-wgpu + wgpu 30 + glam + PyO3/maturin) — the existence proof that this exact combination delivers a fast, Python-distributed GPU viewer — and the one RoboCAD already runs on.

| Layer | Choice | Why / notes |
|---|---|---|
| Language | Rust, edition 2024 | Speed, safety, one binary, WASM-capable |
| UI | `egui` / `eframe` 0.36 (crates.io) | Immediate mode, native + web. The local egui checkout (`~/Documents/code/rust/egui`, clean upstream 0.36.1) is for reading source, not a path dependency |
| 3D rendering | **Own wgpu renderer** injected through `egui_wgpu` paint callbacks, ported from `robocad-viewport` | Already exists: MSAA, ID-buffer picking, orbit camera, low-latency presentation (`LOW_LATENCY` + `AutoNoVsync`, raw wheel events, pick throttling). `three-d` / `kiss3d` / `bevy_egui` rejected: they want to own the device or the loop and fight egui-wgpu |
| Gizmos | `transform-gizmo-egui` (0.11, Aug 2026) | Tracks egui releases; evaluate and replace with own if it fights the viewport's picking |
| Math | `glam` — f64 (`DVec3`, `DMat4`, `DQuat`) in the document and kinematics, f32 at the GPU boundary | What Rerun and the wgpu ecosystem use. Replaces RoboCAD's `cgmath` (unmaintained) during the port |
| Mesh I/O | `stl_io`, `tobj` (OBJ); glTF later | Trivial loaders; own mesh type |
| Mass properties | Ported from `robocad-kernel/src/mass.rs` | Kernel-free signed-tetrahedra integration with a self-consistency check |
| Robot formats | `quick-xml` writer for MJCF and URDF; `urdf-rs` 0.9 for URDF import and the round-trip test | No usable MJCF crate exists — own serializer, minimal reader later |
| Document | `serde` + `serde_json`, versioned schema | The `.riggen` file |
| Python packaging | `maturin` with **`bindings = "bin"`**: the native `riggen` executable ships inside the wheel with a console-script entry | Rerun's pattern (`rerun_cli/__main__.py` is a 30-line `subprocess.call` to the bundled binary). Zero PyO3 in the MVP; no GIL / event-loop / macOS-main-thread problems |
| Python SDK (v0.2) | `PyO3` 0.28 extension module over `riggen-core` (headless), pure-Python `riggen.show()` that writes the document and spawns the binary | The `rr.spawn()` model. The egui window is never opened from inside a PyO3 call |
| Testing | `egui_kittest` headless visual snapshots (RoboCAD ADR-0014), unit tests per crate, URDF round-trip FK comparison in CI | Snapshots are what make agent-driven UI work checkable |
| Web | `cfg(wasm32)` scaffolding kept; CI build check only | Not a v1 deliverable |

### Workspace layout
```
riggen-core      document model (links, joints, frames, inertials, collision policy),
                 FK, validation, serde, undo — no GPU, no egui
riggen-mesh      STL/OBJ loading, own mesh type, mass properties, convex hull, primitive fits
riggen-export    ResolvedRobot, MJCF + URDF writers, URDF import, round-trip tests
riggen-viewport  wgpu renderer (ported), camera, picking, instance transforms
riggen-app       eframe shell, panels, gizmos, drag-drop, snapshot suite  ← maturin bin
riggen-py        (v0.2) PyO3 module over riggen-core
```
Five crates, not eight. Layer rule carried from RoboCAD: lower crates never name types from upper ones; `riggen-core` and `riggen-export` compile without wgpu or egui so the SDK can reuse them.

## 8. Heritage from RoboCAD
**Port (adapting to glam and the new document):**
* `robocad-viewport` (~4.5k LOC): renderer, camera, picking, sketch-plane code dropped.
* `robocad-kernel/src/mass.rs`: inertia integration.
* `robocad-ui`: mass-properties panel, settings, status bar, shortcuts help, the `Area`-inside-viewport layout.
* `robocad-app`: eframe setup, low-latency surface config, `rfd` dialogs, wasm scaffolding, `egui_kittest` visual snapshot suite and the `debug_state()` JSON dump.
* `docs/05-robotics.md`: the Link / RobotJoint / Frame / `InertialSpec` / `GeomPolicy` / `ResolvedRobot` design — minus geometry-anchored `TopoRef`s.
* Process: `docs/` + ADR convention, `AGENTS.md` / `CLAUDE.md`, workspace profile settings.

**Leave behind:** the sketcher (`robocad-sketch`), the B-Rep kernel facade and monstertruck (`robocad-kernel`), the parametric document and expression language (`robocad-doc`). Roughly 35k of the 58k lines.

## 9. Decisions taken in this seed
| Question | Decision | Reason |
|---|---|---|
| MJCF-first or URDF-first? | **MJCF is the acceptance target; both exported from `ResolvedRobot` in the MVP** | The RL audience; MJCF's body-frame conventions are the harder case, so getting them right first keeps `ResolvedRobot` honest |
| Python API in MVP? | **No — v0.2.** MVP ships the binary in the wheel only | Removes PyO3 from the critical path; the GUI's UX is the risk to retire first |
| Import existing models in MVP? | **URDF import yes; MJCF import later** | `urdf-rs` makes it cheap and it doubles as the round-trip test parser |
| Start from zero or port RoboCAD? | **Port** the viewport, mass code, UI shell and test harness | It is the hard, already-working half of the product |

## 10. Directives for the AI Agent
With this seed agreed, produce — in the RoboCAD `docs/` style, as separate numbered documents plus ADRs where a choice is non-obvious:
1. **Architecture Document:** crate boundaries and the layer rule; how the document, FK, viewport and exporters interact; how the future PyO3 module reuses `riggen-core` and `riggen-export`; the `.riggen` schema and versioning.
2. **Data Structures:** the core Rust types for `Robot`, `Link`, `Joint`, `Frame`, `InertialSpec`, `CollisionPolicy`, `ResolvedRobot`, and the convention notes (meters, radians, right-handed, Z-up; MJCF vs URDF frame differences) that every exporter must respect.
3. **Development Roadmap:** 4–5 milestones from "ported viewport shows a dropped STL" to "wheel on PyPI", each with an acceptance test — the MJCF milestone's being "loads and simulates in `mujoco` with no warnings, and the URDF round-trip FK test passes".
4. **Prototype the joint-axis placement UX first.** It is the one part with real product risk; iterate on it with visual snapshots before building panels around it.
