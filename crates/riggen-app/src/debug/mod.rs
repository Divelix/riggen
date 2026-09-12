//! What the app thinks it drew, as inspectable data (ADR-0003).
//!
//! The 3D viewport is one wgpu paint callback, so it contributes no AccessKit
//! nodes: a harness can photograph it but cannot ask it anything. This module
//! is the text side of that pair — a snapshot scenario writes a PNG *and*
//! this JSON, and a mismatch between the two localises the bug immediately.
//!
//! It is also the small public surface the out-of-crate snapshot suite needs
//! (`tests/visual/`), since the app's own fields are `pub(crate)`.
//!
//! **Every float is rounded** ([`round`]) before it is serialised. The JSON is
//! a committed golden, and an unrounded `f32`-to-`f64` widening churns in the
//! last digits between runs for reasons that have nothing to do with the code
//! under test.

mod camera;

pub use camera::CameraDebug;

use serde::Serialize;

use crate::app::{DecompState, RiggenApp, Selection};

/// Decimal places every serialised float is rounded to.
///
/// Internal units are meters (AGENTS.md), so six places is a micrometre — far
/// below anything a modelling or projection bug hides in, and far above the
/// noise floor that would make a golden churn. Screen coordinates are points,
/// where it is well under a pixel.
const PRECISION: i32 = 6;

/// Rounds to [`PRECISION`] decimal places. See the module doc for why.
pub fn round(x: f64) -> f64 {
    let scale = 10f64.powi(PRECISION);
    let r = (x * scale).round() / scale;
    // `-0.0` and `0.0` serialise differently and compare equal, which would
    // make a golden flip for no reason.
    if r == 0.0 { 0.0 } else { r }
}

/// Rounds an `f32` the same way, widening first.
pub fn round32(x: f32) -> f64 {
    round(x as f64)
}

/// Whether a rounded field is zero, for the `skip_serializing_if` that keeps
/// a field a file never set out of the goldens written before it existed.
fn is_zero(x: &f64) -> bool {
    *x == 0.0
}

/// A whole frame's worth of app state, as JSON.
#[derive(Debug, Clone, Serialize)]
pub struct DebugState {
    pub camera: CameraDebug,
    /// The document as the app holds it: what the instances are derived
    /// from.
    pub document: DocumentDebug,
    /// Transient panel state.
    pub ui: UiDebug,
    /// Every instance in draw order, hidden ones included.
    pub instances: Vec<InstanceDebug>,
    pub selection: SelectionDebug,
    /// The viewport's pointer policy, when any of it is on. Omitted while
    /// all three switches are off, which is most frames — so the goldens
    /// that never suppress anything are unchanged.
    #[serde(skip_serializing_if = "InputDebug::is_off")]
    pub input: InputDebug,
    /// The transform gizmo, when one is drawn (ADR-0007).
    pub gizmo: Option<GizmoDebug>,
    /// One entry per joint glyph the overlay drew, in joint id order.
    pub glyphs: Vec<GlyphDebug>,
    /// One entry per frame glyph, in `FrameId` order. Omitted for a
    /// document with no frames, which is most goldens.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub frame_glyphs: Vec<FrameGlyphDebug>,
    /// What the cursor is pointing at, for the placement tools (`snap.rs`).
    pub snap: Option<SnapDebug>,
    /// The status bar's one-off message — a load error, a load summary.
    pub status: Option<String>,
    /// `[min_x, min_y, max_x, max_y]` of the viewport in egui logical points.
    /// `None` before the first frame has laid it out.
    pub viewport_rect: Option<[f64; 4]>,
    /// Samples per pixel in the offscreen scene pass — 4 where the adapter
    /// multisamples, 1 where it does not. Here because antialiasing is
    /// otherwise only assertable as "the pixels look softer", and because a
    /// golden PNG taken at one sample count means nothing at another: this
    /// line says which one the image beside it was rendered at.
    pub sample_count: u32,
    /// Wall-clock numbers. Absent whenever the frame-time HUD is off — the
    /// snapshot suite turns it off, so the goldens never see them.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub timing: Option<TimingDebug>,
}

/// The startup budget's readout (docs/ROADMAP.md §M4).
#[derive(Debug, Clone, Serialize)]
pub struct TimingDebug {
    /// Milliseconds from the start clock (`main`, or `new` in a harness)
    /// to the end of the first `ui` pass. `None` before the first frame.
    pub first_frame_ms: Option<f64>,
    /// Seconds between the last two frames.
    pub frame_dt: Option<f64>,
}

/// What the panels are in the middle of.
#[derive(Debug, Clone, Serialize)]
pub struct UiDebug {
    /// `"View"` or `"Edit"` (ADR-0021).
    pub mode: &'static str,
    /// Zen: every panel and all three pieces of corner chrome hidden
    /// (ADR-0021, amended). Omitted when false, so no golden taken with
    /// the chrome up gains a line.
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub zen: bool,
    /// The active tool, by its toolbar label.
    pub tool: &'static str,
    /// The link an inline rename is editing, as `"l3"`, and the text so far.
    pub renaming: Option<(String, String)>,
    /// Floating windows currently open, by name.
    pub windows: Vec<&'static str>,
    /// The modal being shown, if any: `"unsaved_changes"`.
    pub modal: Option<&'static str>,
    /// What the OS window title reads.
    pub title: String,
    /// The visibility row: the names of the classes that are switched
    /// **off**, in the row's order, omitted when everything is shown
    /// (`app/overlays.rs`). Collision is off by default, so the quiet
    /// state reads `["collision"]`.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub overlays: Vec<&'static str>,
    /// A tree row being dragged, if one is; omitted otherwise.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub drag: Option<TreeDragDebug>,
}

/// A link row mid-drag: what, over which row, and whether a drop there
/// would be accepted (the cursor reads `Grabbing` or `NotAllowed`).
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct TreeDragDebug {
    pub link: String,
    pub over: Option<String>,
    pub allowed: bool,
}

/// The `Robot` and the derived state around it.
#[derive(Debug, Clone, Serialize)]
pub struct DocumentDebug {
    /// `name.riggen` of the file the document came from, `None` for a new
    /// document.
    pub file: Option<String>,
    pub name: String,
    pub dirty: bool,
    /// `MeshAsset::scale` a dropped mesh gets.
    pub import_scale: f64,
    /// In id order, which is creation order.
    pub links: Vec<LinkDebug>,
    pub joints: Vec<JointDebug>,
    /// In `FrameId` order. Omitted when the document has no frames, so a
    /// golden written before frames existed is unchanged.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub frames: Vec<FrameDebug>,
    /// The fixed tendons the document holds (ADR-0025 §4). Omitted when
    /// there are none, so every golden written before them is unchanged.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub tendons: Vec<TendonDebug>,
    /// `"link l3"` / `"joint j7"` / `"frame f2"`, or `None`.
    pub selection: Option<String>,
}

/// A fixed tendon, as the joint properties panel reads it out: what it is
/// over, and what drives it. The actuators are here and not in the joint's
/// own list because a tendon actuator is on the *tendon* (ADR-0023).
#[derive(Debug, Clone, Serialize)]
pub struct TendonDebug {
    pub id: String,
    pub name: String,
    /// Joint name and coefficient, in the order the tendon holds them.
    pub joints: Vec<(String, f64)>,
    /// `"<name> (<preset>)"` per actuator driving it, in `ActuatorId` order.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub actuators: Vec<String>,
}

/// A named frame as the document holds it (ADR-0012).
#[derive(Debug, Clone, Serialize)]
pub struct FrameDebug {
    pub id: String,
    pub name: String,
    /// The link it hangs on.
    pub parent: String,
    /// Its pose in that link's frame.
    pub pos: [f64; 3],
    /// `w x y z`, MuJoCo order, as the export writes it.
    pub quat: [f64; 4],
}

/// A frame glyph: the triad the overlay drew and where it landed.
#[derive(Debug, Clone, Serialize)]
pub struct FrameGlyphDebug {
    pub frame: String,
    pub name: String,
    /// The frame's **world** pose origin: `world(parent) ∘ pose`.
    pub origin: [f64; 3],
    /// Length of a triad arm.
    pub size: f64,
    /// Where the origin lands in the viewport, in egui points.
    pub screen: Option<[f64; 2]>,
    /// Drawn brighter: the hovered frame, else the selected one.
    pub active: bool,
    /// The pointer is on this glyph, or on its row in the tree.
    pub hovered: bool,
}

#[derive(Debug, Clone, Serialize)]
pub struct LinkDebug {
    pub id: String,
    pub name: String,
    /// The joint whose child this is; `None` for the root.
    pub parent_joint: Option<String>,
    pub geoms: usize,
    pub material: Option<String>,
}

#[derive(Debug, Clone, Serialize)]
pub struct JointDebug {
    pub id: String,
    pub name: String,
    pub kind: String,
    pub parent: String,
    pub child: String,
    /// The current joint value (`JointState`), radians or meters.
    pub q: f64,
    /// MJCF's `<joint ref>`: `qpos = q + qpos_ref` (ADR-0025 §3). Omitted
    /// when zero, which is every joint riggen authored itself.
    #[serde(skip_serializing_if = "crate::debug::is_zero")]
    pub qpos_ref: f64,
}

/// What the ID buffer resolved: the hovered and the selected triangle.
#[derive(Debug, Clone, Serialize)]
pub struct SelectionDebug {
    pub hovered: Option<HitDebug>,
    pub selected: Option<HitDebug>,
}

/// Which of the viewport's pointer switches are on (ADR-0010, ADR-0018).
///
/// The bug this exists for was a *policy* bug — the gizmo took the whole
/// pointer instead of the handle under it — and a policy is asserted here
/// rather than inferred from whichever tint it did or did not produce.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct InputDebug {
    /// Neither pick runs; the camera is still live.
    pub pick_suppressed: bool,
    /// A click means "place here", not "select what is under the cursor".
    pub select_suppressed: bool,
    /// The camera ignores the pointer: something else owns the gesture.
    /// The picks have their own two switches (ADR-0019 §4).
    pub camera_blocked: bool,
    /// The *left* drag is spoken for — a gizmo handle has it — while the
    /// middle and right drags, the wheel and both picks stay live
    /// (ADR-0018).
    pub primary_drag_claimed: bool,
    /// The **wheel** is spoken for — a rotate ring under the cursor steps
    /// on it — while everything else stays live (ADR-0019).
    pub wheel_claimed: bool,
}

impl InputDebug {
    /// Nothing suppressed — the plain viewport.
    fn is_off(&self) -> bool {
        !self.pick_suppressed
            && !self.select_suppressed
            && !self.camera_blocked
            && !self.primary_drag_claimed
            && !self.wheel_claimed
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
pub struct HitDebug {
    pub instance: u32,
    pub triangle: u32,
}

impl From<riggen_viewport::PickHit> for HitDebug {
    fn from(hit: riggen_viewport::PickHit) -> Self {
        Self {
            instance: hit.instance.0,
            triangle: hit.triangle,
        }
    }
}

/// The transform gizmo: what it is attached to and where it sits.
///
/// `screen` goes through the same `camera.view_proj` the wgpu pass uses, so
/// a gizmo that has drifted from the geometry it edits shows up here as two
/// numbers rather than as a picture that looks slightly wrong.
#[derive(Debug, Clone, Serialize)]
pub struct GizmoDebug {
    /// `"link l3"` / `"joint j7"`.
    pub target: String,
    /// `"translate"` or `"rotate"`.
    pub mode: &'static str,
    /// World position of the frame the gizmo is on.
    pub origin: [f64; 3],
    /// Where that lands in the viewport, in egui points.
    pub screen: Option<[f64; 2]>,
    /// Whether a drag is in flight (the document is not yet edited).
    pub dragging: bool,
    /// Whether the gizmo owns the cursor: a handle is under it, or a drag
    /// is in flight. Suppresses the viewport's own input while it holds.
    pub captured: bool,
    /// Which rotate ring the cursor is on — `"x"`, `"y"` or `"z"`. Absent
    /// off the rings, and on the crate's fourth, view-axis one, which the
    /// wheel does not claim.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub hovered_ring: Option<&'static str>,
}

/// One joint glyph: where its pivot and axis are, and how big it is drawn.
///
/// `screen` is the pivot through `Viewport::project` — the same projection
/// the overlay strokes with, so a glyph that is drawn in the wrong place
/// shows up here as a number rather than as a picture one has to measure.
#[derive(Debug, Clone, Serialize)]
pub struct GlyphDebug {
    pub joint: String,
    pub name: String,
    pub kind: String,
    /// The pivot in world coordinates: `world(parent) ∘ origin`.
    pub origin: [f64; 3],
    /// The joint axis in world coordinates, unit length.
    pub axis: [f64; 3],
    /// Half-length of the axis segment; every other measure is a fraction
    /// of it.
    pub size: f64,
    /// Current joint value, radians or meters.
    pub q: f64,
    /// The signed sweep of the value sector — from the zero position to
    /// `q`, radians for a hinge — so a scenario asserts the band's shape
    /// and not only the pixels (plans/joint-glyph-range-and-value).
    pub value_sweep: f64,
    /// The band's inner and outer radius in metres, for a revolute or
    /// continuous joint. Omitted for a joint without a band.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub band: Option<(f64, f64)>,
    /// A slide's bars along its axis, in metres from the pivot: the range
    /// bar's extent (ADR-0027 §4). The band's counterpart, and omitted
    /// for a joint that has no bars.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bar: Option<(f64, f64)>,
    /// The pieces this glyph contributed this frame, in draw order:
    /// `"axis"`, `"pivot"`, `"triad"`, `"actuator"`, `"bore"`, `"band"`,
    /// `"bars"`, `"tick"`. Edit draws all of them; View draws the band or
    /// the bars, the tick, and a filled bore for an actuated joint
    /// (ADR-0027 §1) — so the *composition* is asserted here as JSON and
    /// not only as pixels (ADR-0003).
    pub drawn: Vec<&'static str>,
    /// Where the pivot lands in the viewport, in egui points.
    pub screen: Option<[f64; 2]>,
    /// Drawn brighter and thicker: the hovered joint, else the selected one.
    pub active: bool,
    /// The pointer is on this glyph, or on its row in the tree.
    pub hovered: bool,
    /// The joint this one follows, as `"j12"` — the glyph is drawn muted
    /// and labelled `» <leader>` (ADR-0013; `↳` is a tofu box in egui's
    /// bundled fonts, see `driven_marks`). Omitted for a free joint.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub mimic: Option<String>,
    /// The preset of every actuator driving it, in `ActuatorId` order — the
    /// glyph gains a ring at the pivot (ADR-0014), one ring however many
    /// drive it (ADR-0023). Omitted for an unactuated joint.
    #[serde(skip_serializing_if = "Vec::is_empty")]
    pub actuators: Vec<&'static str>,
    /// Whether the scene's depth image says the pivot is behind geometry,
    /// so a scenario asserts the *policy* and not only the pixels
    /// (ADR-0020, ADR-0003). `null` — omitted — before any depth image has
    /// landed, which is every logic-only scenario.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub pivot_hidden: Option<bool>,
}

/// The snap target under the cursor: which kind won the priority ladder,
/// where it is, and — for a circle — the fit's own confidence numbers, the
/// same ones the viewport readout shows.
#[derive(Debug, Clone, Serialize)]
pub struct SnapDebug {
    /// `"vertex"`, `"box corner"`, `"box face"`, `"circle"`, `"point"`.
    pub kind: &'static str,
    /// Where a click would place something, in world coordinates.
    pub point: [f64; 3],
    /// The hit triangle's normal.
    pub normal: [f64; 3],
    /// The axis a joint would take: the circle's, else the normal.
    pub axis: [f64; 3],
    /// The exact ray/triangle hit, whatever kind won.
    pub hit: [f64; 3],
    pub link: String,
    /// `None` unless the kind is `circle`. Millimetres, as the readout.
    pub radius_mm: Option<f64>,
    pub segments: Option<usize>,
    pub residual_mm: Option<f64>,
    /// The readout drawn beside the marker.
    pub readout: String,
    pub screen: Option<[f64; 2]>,
    /// What a **rotate** drag is landing on it (ADR-0029). Omitted — and
    /// absent from the JSON — every other time, which is every scenario
    /// that is not mid-rotate-drag.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub align: Option<AlignDebug>,
}

/// The alignment a rotate drag is previewing: which of the dragged frame's
/// own axes lands, the direction it lands along, and how far the snap had
/// to turn it. The rule, not the pixels — a headless scenario asserts on
/// this and the golden PNG shows the spoke that says it.
#[derive(Debug, Clone, Copy, Serialize)]
pub struct AlignDebug {
    /// `"+z"` / `"-x"`: the frame axis, signed.
    pub axis: &'static str,
    /// The feature's axis projected into the ring's plane, normalised.
    pub target: [f64; 3],
    /// The correction the snap applied, in degrees. Never more than 45.
    pub degrees: f64,
}

/// One viewport instance: identity, visibility, size and where it is.
#[derive(Debug, Clone, Serialize)]
pub struct InstanceDebug {
    pub id: u32,
    /// The `(LinkId, GeomId)` this instance draws, as `"l3"` / `"g5"`.
    pub link: Option<String>,
    pub geom: Option<String>,
    pub visible: bool,
    pub triangles: u32,
    /// Model-space `[min, max]`, before `model`.
    pub bounds: Option<[[f64; 3]; 2]>,
    /// Translation column of the model matrix: the geom's world position
    /// at the current joint values.
    pub position: [f64; 3],
    /// Linear RGBA tint: the link's material colour.
    pub color: [f64; 4],
    /// A translucent collision shape (the visibility row's `collision`), drawn
    /// after the opaque instances and skipped by the pick pass. Omitted
    /// when false, so the M0–M2 goldens are unchanged.
    #[serde(skip_serializing_if = "std::ops::Not::not")]
    pub collision: bool,
}

impl RiggenApp {
    /// Snapshot of everything this module reports. Cheap enough to call per
    /// frame; the snapshot suite calls it once per scenario.
    pub fn debug_state(&self) -> DebugState {
        let robot = self.robot();
        let document = DocumentDebug {
            file: self
                .file()
                .and_then(|p| p.file_name())
                .map(|n| n.to_string_lossy().into_owned()),
            name: robot.name.clone(),
            dirty: self.history().is_dirty(),
            import_scale: round(self.import_scale()),
            links: robot
                .links
                .iter()
                .map(|(id, link)| LinkDebug {
                    id: id.to_string(),
                    name: link.name.clone(),
                    parent_joint: robot.parent_joint(*id).map(|j| j.to_string()),
                    geoms: link.visuals.len(),
                    material: link.material.clone(),
                })
                .collect(),
            joints: robot
                .joints
                .iter()
                .map(|(id, joint)| JointDebug {
                    id: id.to_string(),
                    name: joint.name.clone(),
                    kind: format!("{:?}", joint.kind),
                    parent: joint.parent.to_string(),
                    child: joint.child.to_string(),
                    q: round(self.joint_value(*id)),
                    qpos_ref: round(joint.qpos_ref),
                })
                .collect(),
            frames: robot
                .frames
                .iter()
                .map(|(id, frame)| {
                    let [w, x, y, z] = riggen_export::xml::quat_wxyz(frame.pose.r);
                    FrameDebug {
                        id: id.to_string(),
                        name: frame.name.clone(),
                        parent: frame.parent.to_string(),
                        pos: [
                            round(frame.pose.t.x),
                            round(frame.pose.t.y),
                            round(frame.pose.t.z),
                        ],
                        quat: [round(w), round(x), round(y), round(z)],
                    }
                })
                .collect(),
            tendons: robot
                .tendons
                .iter()
                .map(|(id, tendon)| TendonDebug {
                    id: id.to_string(),
                    name: tendon.name.clone(),
                    joints: tendon
                        .joints
                        .iter()
                        .map(|tj| {
                            let name = robot
                                .joints
                                .get(&tj.joint)
                                .map_or_else(|| tj.joint.to_string(), |j| j.name.clone());
                            (name, round(tj.coef))
                        })
                        .collect(),
                    actuators: robot
                        .actuators_driving(*id)
                        .map(|(_, a)| format!("{} ({})", a.name, a.spec.kind_name()))
                        .collect(),
                })
                .collect(),
            selection: match self.selection() {
                Selection::None => None,
                other => other.describe(),
            },
        };
        DebugState {
            camera: CameraDebug::capture(self),
            document,
            ui: UiDebug {
                drag: self.tree.drag.map(|d| TreeDragDebug {
                    link: d.link.to_string(),
                    over: d.over.map(|l| l.to_string()),
                    allowed: d.allowed,
                }),
                mode: self.mode().label(),
                zen: self.zen(),
                tool: self.tool().label(),
                renaming: self
                    .tree
                    .renaming
                    .as_ref()
                    .map(|(l, text)| (l.to_string(), text.clone())),
                windows: [self.materials_window_open().then_some("materials")]
                    .into_iter()
                    .flatten()
                    .collect(),
                modal: if self.export_dialog().open {
                    Some("export")
                } else {
                    self.pending_action().map(|_| "unsaved_changes")
                },
                title: self.window_title(),
                overlays: self.overlays().hidden(),
            },
            instances: self
                .viewport
                .instance_states()
                .map(|state| InstanceDebug {
                    id: state.id.0,
                    link: self
                        .link_of_instance(state.id)
                        .or_else(|| self.collision_link_of_instance(state.id))
                        .map(|l| l.to_string()),
                    geom: self.geom_of_instance(state.id).map(|g| g.to_string()),
                    visible: state.visible,
                    triangles: state.triangle_count,
                    bounds: state.bounds.map(|b| {
                        [
                            [round(b.min.x), round(b.min.y), round(b.min.z)],
                            [round(b.max.x), round(b.max.y), round(b.max.z)],
                        ]
                    }),
                    position: {
                        let t = state.model.w_axis;
                        [round(t.x), round(t.y), round(t.z)]
                    },
                    color: state.color.map(round32),
                    collision: state.group == riggen_viewport::RenderGroup::Translucent,
                })
                .collect(),
            selection: SelectionDebug {
                hovered: self.viewport.hovered().map(HitDebug::from),
                selected: self.viewport.selected().map(HitDebug::from),
            },
            input: {
                let (
                    pick_suppressed,
                    select_suppressed,
                    camera_blocked,
                    primary_drag_claimed,
                    wheel_claimed,
                ) = self.viewport.pointer_policy();
                InputDebug {
                    pick_suppressed,
                    select_suppressed,
                    camera_blocked,
                    primary_drag_claimed,
                    wheel_claimed,
                }
            },
            glyphs: {
                let active = self.active_joint();
                self.joint_glyphs()
                    .into_iter()
                    .map(|glyph| GlyphDebug {
                        joint: glyph.joint.to_string(),
                        name: robot
                            .joints
                            .get(&glyph.joint)
                            .map(|j| j.name.clone())
                            .unwrap_or_default(),
                        kind: format!("{:?}", glyph.kind),
                        origin: [
                            round(glyph.pivot.t.x),
                            round(glyph.pivot.t.y),
                            round(glyph.pivot.t.z),
                        ],
                        axis: [
                            round(glyph.axis.x),
                            round(glyph.axis.y),
                            round(glyph.axis.z),
                        ],
                        size: round(glyph.size),
                        q: round(glyph.q),
                        value_sweep: round(glyph.value_sweep()),
                        band: glyph
                            .band()
                            .map(|(inner, outer)| (round(inner), round(outer))),
                        bar: glyph.bar().map(|(from, to)| (round(from), round(to))),
                        drawn: self.glyph_pieces(&glyph),
                        screen: self
                            .project_world(glyph.pivot.t)
                            .map(|p| [round32(p.x), round32(p.y)]),
                        active: active == Some(glyph.joint),
                        hovered: self.hovered_joint() == Some(glyph.joint),
                        mimic: glyph.mimic.map(|m| m.to_string()),
                        actuators: glyph.actuators.clone(),
                        pivot_hidden: self
                            .viewport
                            .depth_image()
                            .map(|image| image.hidden(glyph.pivot.t)),
                    })
                    .collect()
            },
            frame_glyphs: {
                let active = self.active_frame();
                self.frame_glyphs()
                    .into_iter()
                    .map(|glyph| FrameGlyphDebug {
                        frame: glyph.frame.to_string(),
                        name: glyph.name,
                        origin: [
                            round(glyph.pose.t.x),
                            round(glyph.pose.t.y),
                            round(glyph.pose.t.z),
                        ],
                        size: round(glyph.size),
                        screen: self
                            .project_world(glyph.pose.t)
                            .map(|p| [round32(p.x), round32(p.y)]),
                        active: active == Some(glyph.frame),
                        hovered: self.hovered_frame() == Some(glyph.frame),
                    })
                    .collect()
            },
            snap: self.snap().map(|snap| SnapDebug {
                kind: snap.kind.label(),
                point: [
                    round(snap.point.x),
                    round(snap.point.y),
                    round(snap.point.z),
                ],
                normal: [
                    round(snap.normal.x),
                    round(snap.normal.y),
                    round(snap.normal.z),
                ],
                axis: {
                    let axis = snap.axis();
                    [round(axis.x), round(axis.y), round(axis.z)]
                },
                hit: [round(snap.hit.x), round(snap.hit.y), round(snap.hit.z)],
                link: snap.link.to_string(),
                radius_mm: snap.circle.map(|c| round(c.radius * 1000.0)),
                segments: snap.circle.map(|c| c.segments),
                residual_mm: snap.circle.map(|c| round(c.residual * 1000.0)),
                readout: snap.readout(),
                screen: self
                    .project_world(snap.point)
                    .map(|p| [round32(p.x), round32(p.y)]),
                align: self.snap_align().map(|align| AlignDebug {
                    axis: align.landed.axis,
                    target: [
                        round(align.landed.target.x),
                        round(align.landed.target.y),
                        round(align.landed.target.z),
                    ],
                    degrees: round(align.landed.degrees),
                }),
            }),
            gizmo: self.gizmo_target().and_then(|target| {
                let world = self.gizmo_world(target)?;
                Some(GizmoDebug {
                    target: target.describe(),
                    mode: if self.tool() == crate::app::Tool::Rotate {
                        "rotate"
                    } else {
                        "translate"
                    },
                    origin: [round(world.t.x), round(world.t.y), round(world.t.z)],
                    screen: self
                        .project_world(world.t)
                        .map(|p| [round32(p.x), round32(p.y)]),
                    dragging: self.gizmo_dragging(),
                    captured: self.gizmo_captured(),
                    hovered_ring: self.hovered_ring().map(crate::app::RingAxis::label),
                })
            }),
            status: self.status.clone(),
            viewport_rect: self.viewport.viewport_rect().map(|rect| {
                [
                    round32(rect.min.x),
                    round32(rect.min.y),
                    round32(rect.max.x),
                    round32(rect.max.y),
                ]
            }),
            sample_count: self.viewport.sample_count(),
            timing: self.show_frame_hud.then(|| TimingDebug {
                first_frame_ms: self.first_frame_ms.map(round),
                frame_dt: self.last_frame_dt.map(round32),
            }),
        }
    }

    /// [`Self::debug_state`] as pretty-printed JSON — what the snapshot
    /// goldens hold.
    pub fn debug_state_json(&self) -> String {
        serde_json::to_string_pretty(&self.debug_state())
            .unwrap_or_else(|err| format!("{{\"error\": \"{err}\"}}"))
    }

    /// Whether the status bar shows the frame-time readout.
    ///
    /// The snapshot suite turns it off: it reads the wall clock, so it
    /// differs on every frame.
    pub fn set_frame_hud_visible(&mut self, visible: bool) {
        self.show_frame_hud = visible;
    }

    /// Whether a V-HACD run may start (ADR-0011).
    ///
    /// Always true on the desktop; the browser starts at `false` and the
    /// properties panel asks once, because `jobs` has no thread there and
    /// the run freezes the tab (docs/ARCHITECTURE.md §Jobs and threads).
    /// The snapshot suite sets it to see the browser's half of that panel
    /// on a native runner (ADR-0003).
    pub fn set_decomp_consent(&mut self, consent: bool) {
        self.decomp_consent = consent;
    }

    pub fn decomp_consent(&self) -> bool {
        self.decomp_consent
    }

    /// What the overlay classifies `world` against, and `world`'s own
    /// depth, both in NDC: `(stored, own)` — `stored < own` means opaque
    /// geometry stands in front of it (ADR-0020). `None` until a depth
    /// image has landed, and for a point off it or behind the camera.
    pub fn depth_probe(&self, world: riggen_core::glam::DVec3) -> Option<(f32, f32)> {
        self.viewport.depth_probe(world)
    }

    /// Whether a snapshot taken now is reproducible: no pick readback in
    /// flight, no depth readback in flight, and no camera animation reading
    /// the wall clock. The harness pumps frames until this has held for a
    /// few in a row.
    pub fn settled(&self) -> bool {
        // A decomposition still on the job thread would change what the
        // collision view draws, so a snapshot taken now is not the one the
        // scenario asked for.
        self.viewport.is_settled() && !self.decompositions_pending()
    }

    /// Whether any decomposition the document wants is still on the job
    /// thread (`crate::jobs`). Frames keep coming while this is true: the
    /// worker's `wake` is `ctx.request_repaint()`.
    ///
    /// A document that wants one while the browser's consent is withheld
    /// is *not* pending: nothing is running, and nothing will until the
    /// question is answered (ADR-0011, ADR-0017).
    pub fn decompositions_pending(&self) -> bool {
        self.decomp_consent
            && self.wanted_decompositions().iter().any(|key| {
                !matches!(
                    self.decomp.get(key),
                    Some(DecompState::Ready(_) | DecompState::Failed(_))
                )
            })
    }

    /// The convex pieces the job thread produced for `(mesh, params)`:
    /// `None` while it is pending or nothing has asked, `Some(Err(reason))`
    /// when there are none at these parameters. What the properties panel,
    /// the collision view and the snapshot suite read.
    pub fn decomposition(
        &self,
        mesh: riggen_core::MeshId,
        params: riggen_mesh::DecompParams,
    ) -> Option<Result<&[std::sync::Arc<riggen_mesh::TriMesh>], &str>> {
        match self.decomp.get(&(mesh, params))? {
            DecompState::Pending => None,
            DecompState::Ready(pieces) => Some(Ok(&pieces[..])),
            DecompState::Failed(reason) => Some(Err(reason.as_str())),
        }
    }

    /// Frames every visible instance **without** animating there.
    ///
    /// `Home` is the user-facing equivalent, but it animates, and the
    /// animation reads the wall clock — a snapshot taken mid-flight is not
    /// reproducible. This lands on the same view in one frame.
    pub fn fit_view_now(&mut self) {
        self.viewport.frame_scene();
    }

    /// Puts the orbit camera at `yaw` / `pitch` degrees and scales the
    /// distance a fit left it at, keeping the target and **without**
    /// animating — the same reason [`Self::fit_view_now`] exists rather
    /// than `Home`.
    ///
    /// What a scenario that needs a particular angle on a glyph uses: a
    /// grazing view is where an annulus foreshortens onto itself
    /// (ADR-0027 §2), and an orbit drag would land on it only
    /// approximately.
    pub fn look_from(&mut self, yaw_deg: f32, pitch_deg: f32, distance_scale: f32) {
        let camera = &mut self.viewport.camera;
        camera.yaw = yaw_deg.to_radians();
        camera.pitch = pitch_deg.to_radians();
        camera.distance *= distance_scale;
        camera.animation = None;
    }

    /// Centre of the viewport rect, which after [`Self::fit_view_now`] is
    /// over the geometry — where a scenario aims a hover or a click. `None`
    /// before the first frame has laid it out.
    pub fn viewport_center(&self) -> Option<egui::Pos2> {
        self.viewport.viewport_rect().map(|rect| rect.center())
    }
}
