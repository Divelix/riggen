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

use crate::app::{RiggenApp, Selection};

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
    /// The status bar's one-off message — a load error, a load summary.
    pub status: Option<String>,
    /// `[min_x, min_y, max_x, max_y]` of the viewport in egui logical points.
    /// `None` before the first frame has laid it out.
    pub viewport_rect: Option<[f64; 4]>,
}

/// What the panels are in the middle of.
#[derive(Debug, Clone, Serialize)]
pub struct UiDebug {
    /// The link an inline rename is editing, as `"l3"`, and the text so far.
    pub renaming: Option<(String, String)>,
    /// Floating windows currently open, by name.
    pub windows: Vec<&'static str>,
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
    /// `"link l3"` / `"joint j7"`, or `None`.
    pub selection: Option<String>,
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
}

/// What the ID buffer resolved: the hovered and the selected triangle.
#[derive(Debug, Clone, Serialize)]
pub struct SelectionDebug {
    pub hovered: Option<HitDebug>,
    pub selected: Option<HitDebug>,
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
                renaming: self
                    .tree
                    .renaming
                    .as_ref()
                    .map(|(l, text)| (l.to_string(), text.clone())),
                windows: [
                    self.joints_window_open().then_some("joints"),
                    self.materials_window_open().then_some("materials"),
                ]
                .into_iter()
                .flatten()
                .collect(),
            },
            instances: self
                .viewport
                .instance_states()
                .map(|state| InstanceDebug {
                    id: state.id.0,
                    link: self.link_of_instance(state.id).map(|l| l.to_string()),
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
                })
                .collect(),
            selection: SelectionDebug {
                hovered: self.viewport.hovered().map(HitDebug::from),
                selected: self.viewport.selected().map(HitDebug::from),
            },
            status: self.status.clone(),
            viewport_rect: self.viewport.viewport_rect().map(|rect| {
                [
                    round32(rect.min.x),
                    round32(rect.min.y),
                    round32(rect.max.x),
                    round32(rect.max.y),
                ]
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

    /// Whether a snapshot taken now is reproducible: no pick readback in
    /// flight and no camera animation reading the wall clock. The harness
    /// pumps frames until this has held for a few in a row.
    pub fn settled(&self) -> bool {
        self.viewport.is_settled()
    }

    /// Frames every visible instance **without** animating there.
    ///
    /// `Home` is the user-facing equivalent, but it animates, and the
    /// animation reads the wall clock — a snapshot taken mid-flight is not
    /// reproducible. This lands on the same view in one frame.
    pub fn fit_view_now(&mut self) {
        self.viewport.frame_scene();
    }

    /// Centre of the viewport rect, which after [`Self::fit_view_now`] is
    /// over the geometry — where a scenario aims a hover or a click. `None`
    /// before the first frame has laid it out.
    pub fn viewport_center(&self) -> Option<egui::Pos2> {
        self.viewport.viewport_rect().map(|rect| rect.center())
    }
}
