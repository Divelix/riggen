//! The properties panel (right): what the selection is, as editable
//! fields. Numbers are text fields that commit on Enter or lost focus,
//! never per keystroke, so one edit is one command (and one undo step); a
//! commit equal to the document's value goes nowhere (`History` drops it).
//! Angles are degrees here and radians in the document.

use std::collections::HashMap;

use riggen_core::glam::{DMat3, DQuat, DVec3};
use riggen_core::inertial::{Inertial, InertialError, principal_moments};
use riggen_core::{
    ActuatorSpec, CollisionPolicy, Command, FrameId, GestureId, InertialSpec, JointId, JointKind,
    JointState, Limits, LinkId, Mimic, Pose, Primitive, fk,
};
use riggen_mesh::{DecompParams, fit};

use crate::app::{RiggenApp, Selection};

/// Text being typed into numeric / name fields, keyed by the field's
/// widget id, present only while the field has focus.
#[derive(Debug, Clone, Default)]
pub(crate) struct PropertiesState {
    drafts: HashMap<egui::Id, String>,
    /// The scrub in flight this frame: the field being dragged names the
    /// gesture its commands coalesce under (`History::apply_in_gesture`).
    gesture: Option<GestureId>,
    /// A scrub released this frame: the panel ends the gesture after
    /// applying what it produced.
    gesture_ended: bool,
    /// A new gesture starts this frame on a field whose previous one may
    /// still be open (a wheel burst that went quiet, a drag after a burst):
    /// the panel ends it *before* applying.
    gesture_break: bool,
    /// The last Ctrl+wheel notch: which field and when, so a burst of
    /// notches within [`WHEEL_BURST`] coalesces into one entry.
    wheel: Option<(GestureId, f64)>,
}

impl PropertiesState {
    /// Drops every unfinished edit — the selection changed under it.
    pub(crate) fn clear(&mut self) {
        self.drafts.clear();
        self.gesture = None;
        self.gesture_ended = false;
        self.gesture_break = false;
        self.wheel = None;
    }
}

/// Seconds between Ctrl+wheel notches that still count as one gesture.
const WHEEL_BURST: f64 = 0.4;

/// The floor of a scrubber's speed, per unit: what one point of drag is
/// worth when the value is near zero and one percent of it would be
/// nothing. Metres, degrees, kilograms, kg·m², plain numbers, kg/m³, counts.
const STEP_M: f64 = 1e-3;
const STEP_DEG: f64 = 0.1;
const STEP_KG: f64 = 1e-3;
const STEP_KGM2: f64 = 1e-9;
const STEP_UNIT: f64 = 0.01;
const STEP_DENSITY: f64 = 1.0;
const STEP_INT: f64 = 1.0;

/// A number field's resting width; the button grows for a longer number.
const FIELD_WIDTH: f32 = 56.0;

/// Blender's rule: one point of drag is one percent of the value, never
/// less than the field's unit step, and a tenth of that with Ctrl.
fn scrub_speed(value: f64, step: f64, fine: bool) -> f64 {
    let speed = (value.abs() * 0.01).max(step);
    if fine { speed / 10.0 } else { speed }
}

/// What one wheel notch adds: one unit of the last digit the field shows
/// (`1240` → 1, `0.5` → 0.1, `2.86e-5` → 1e-7), or the unit step for a
/// field showing `0`.
fn wheel_increment(value: f64, step: f64) -> f64 {
    let shown = fmt_num(value);
    if shown == "0" {
        return step;
    }
    let (mantissa, exponent) = shown
        .split_once('e')
        .map_or((shown.as_str(), 0), |(m, e)| (m, e.parse().unwrap_or(0)));
    let decimals = mantissa.split_once('.').map_or(0, |(_, f)| f.len() as i32);
    10f64.powi(exponent - decimals)
}

/// This frame's Ctrl+wheel notches, up positive. Read from the raw events
/// like the viewport does (`riggen-viewport` `raw_wheel_delta_y`): egui
/// routes a wheel with its zoom modifier away from scrolling, which is
/// exactly why Ctrl is the stepping wheel — the panel keeps scrolling
/// under a plain one — and no one else consumes it.
fn wheel_notches(ui: &egui::Ui) -> i32 {
    let options = ui.ctx().options(|o| o.input_options);
    ui.input(|input| {
        input
            .raw
            .events
            .iter()
            .filter_map(|event| match event {
                egui::Event::MouseWheel {
                    unit,
                    delta,
                    modifiers,
                    ..
                } if modifiers.matches_any(options.zoom_modifier) => {
                    let lines = match unit {
                        egui::MouseWheelUnit::Line => delta.y,
                        egui::MouseWheelUnit::Point => delta.y / options.line_scroll_speed,
                        egui::MouseWheelUnit::Page => delta.y.signum(),
                    };
                    // A notch is at least one, whatever the platform's
                    // lines-per-notch setting says.
                    Some(lines.abs().max(1.0).round() as i32 * lines.signum() as i32)
                }
                _ => None,
            })
            .sum()
    })
}

/// Document ↔ field unit conversion (radians ↔ degrees, or none).
type Convert = fn(f64) -> f64;

/// The Inertial block's mode combo: the three `InertialSpec` variants by
/// name, without their payloads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum InertialMode {
    Computed,
    Override,
    Hybrid,
}

impl InertialMode {
    const ALL: [Self; 3] = [Self::Computed, Self::Override, Self::Hybrid];

    fn of(spec: &InertialSpec) -> Self {
        match spec {
            InertialSpec::Computed { .. } => Self::Computed,
            InertialSpec::Override { .. } => Self::Override,
            InertialSpec::Hybrid { .. } => Self::Hybrid,
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::Computed => "Computed",
            Self::Override => "Override",
            Self::Hybrid => "Hybrid",
        }
    }
}

/// The Collision block's policy combo: the policies a user picks by hand.
/// `Meshes` (a URDF import) is shown when present but not offered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum CollisionMode {
    None,
    SameAsVisual,
    ConvexHull,
    Primitives,
    Meshes,
    Decomposition,
}

impl CollisionMode {
    const OFFERED: [Self; 5] = [
        Self::None,
        Self::SameAsVisual,
        Self::ConvexHull,
        Self::Decomposition,
        Self::Primitives,
    ];

    fn of(policy: &CollisionPolicy) -> Self {
        match policy {
            CollisionPolicy::None => Self::None,
            CollisionPolicy::SameAsVisual => Self::SameAsVisual,
            CollisionPolicy::ConvexHull => Self::ConvexHull,
            CollisionPolicy::Primitives(_) => Self::Primitives,
            CollisionPolicy::Meshes(_) => Self::Meshes,
            CollisionPolicy::ConvexDecomposition { .. } => Self::Decomposition,
        }
    }

    fn label(self) -> &'static str {
        match self {
            Self::None => "None",
            Self::SameAsVisual => "Same as visual",
            Self::ConvexHull => "Convex hull",
            Self::Primitives => "Primitives",
            Self::Meshes => "Meshes (imported)",
            Self::Decomposition => "Convex decomposition",
        }
    }
}

/// Ceilings on the two integer parameters, so a typo cannot ask for a
/// grid of 10^9 voxels or a thousand collision geoms. Both are far above
/// anything a robot link wants (V-HACD's own `max_convex_hulls` default is
/// 1024, which is not a useful number here).
const MAX_DECOMP_HULLS: u32 = 64;
const MAX_DECOMP_RESOLUTION: u32 = 256;

/// What the Collision block says under a `ConvexDecomposition`'s fields:
/// the job thread's answer, or that it is still running.
enum DecompReadout {
    Pieces(usize),
    Working,
    Failed(String),
    /// No visual mesh, so nothing was ever asked for.
    NoMesh,
    /// The browser: `jobs` has no thread there, so the run would freeze the
    /// tab and is waiting to be asked for (ADR-0011,
    /// docs/01-architecture.md §Jobs and threads).
    NeedsConsent,
}

/// What the button that gives that consent says, and the warning above it.
pub const DECOMP_FREEZE_WARNING: &str =
    "V-HACD runs in this tab: the page will stop responding for a few seconds.";
pub const DECOMP_CONSENT_BUTTON: &str = "Compute anyway";

/// The four primitive kinds, for the add buttons and the fit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum PrimitiveKind {
    Box,
    Cylinder,
    Sphere,
    Capsule,
}

impl PrimitiveKind {
    const ALL: [Self; 4] = [Self::Box, Self::Cylinder, Self::Sphere, Self::Capsule];

    fn label(self) -> &'static str {
        match self {
            Self::Box => "Box",
            Self::Cylinder => "Cylinder",
            Self::Sphere => "Sphere",
            Self::Capsule => "Capsule",
        }
    }

    fn of(p: &Primitive) -> Self {
        match p {
            Primitive::Box { .. } => Self::Box,
            Primitive::Cylinder { .. } => Self::Cylinder,
            Primitive::Sphere { .. } => Self::Sphere,
            Primitive::Capsule { .. } => Self::Capsule,
        }
    }

    /// The primitive of this kind fitted to `points` (link frame): the
    /// AABB-based fits of `riggen_mesh::fit`, with the axial ones' axis as
    /// the pose's Z. A unit shape at the origin when there is nothing to
    /// fit.
    fn fitted(self, points: &[DVec3]) -> Primitive {
        let axial_pose = |center: DVec3, axis: DVec3| {
            Pose::new(center, DQuat::from_rotation_arc(DVec3::Z, axis))
        };
        match self {
            Self::Box => match fit::box_fit(points) {
                Some(f) => Primitive::Box {
                    pose: Pose::from_translation(f.center),
                    size: f.size,
                },
                None => Primitive::Box {
                    pose: Pose::IDENTITY,
                    size: DVec3::splat(0.1),
                },
            },
            Self::Sphere => match fit::sphere_fit(points) {
                Some(f) => Primitive::Sphere {
                    pose: Pose::from_translation(f.center),
                    radius: f.radius,
                },
                None => Primitive::Sphere {
                    pose: Pose::IDENTITY,
                    radius: 0.05,
                },
            },
            Self::Cylinder => match fit::cylinder_fit(points) {
                Some(f) => Primitive::Cylinder {
                    pose: axial_pose(f.center, f.axis),
                    radius: f.radius,
                    length: f.length,
                },
                None => Primitive::Cylinder {
                    pose: Pose::IDENTITY,
                    radius: 0.05,
                    length: 0.1,
                },
            },
            Self::Capsule => match fit::capsule_fit(points) {
                Some(f) => Primitive::Capsule {
                    pose: axial_pose(f.center, f.axis),
                    radius: f.radius,
                    length: f.length,
                },
                None => Primitive::Capsule {
                    pose: Pose::IDENTITY,
                    radius: 0.05,
                    length: 0.1,
                },
            },
        }
    }
}

/// Default limits handed to a joint switched to a kind that needs them.
/// The three presets the combo offers, at MuJoCo's own defaults — `kv` 0
/// on a position servo, `kv` 1 on a velocity one, `gear` 1 on a motor — so
/// picking a kind states nothing we invented (ADR-0014). `kp` has no
/// MuJoCo default worth calling one; 100 is a starting point the user
/// types over.
fn default_actuators() -> [ActuatorSpec; 3] {
    [
        ActuatorSpec::Position { kp: 100.0, kv: 0.0 },
        ActuatorSpec::Velocity { kv: 1.0 },
        ActuatorSpec::Motor { gear: 1.0 },
    ]
}

/// An actuator's gains as `(label, value)`, in the order they are shown.
fn gains(spec: ActuatorSpec) -> Vec<(&'static str, f64)> {
    match spec {
        ActuatorSpec::Position { kp, kv } => vec![("kp", kp), ("kv", kv)],
        ActuatorSpec::Velocity { kv } => vec![("kv", kv)],
        ActuatorSpec::Motor { gear } => vec![("gear", gear)],
    }
}

/// Writes one gain back by the label [`gains`] gave it.
fn set_gain(spec: &mut ActuatorSpec, label: &str, value: f64) {
    match (spec, label) {
        (ActuatorSpec::Position { kp, .. }, "kp") => *kp = value,
        (ActuatorSpec::Position { kv, .. } | ActuatorSpec::Velocity { kv }, "kv") => *kv = value,
        (ActuatorSpec::Motor { gear }, "gear") => *gear = value,
        _ => {}
    }
}

fn default_limits(kind: JointKind) -> Limits {
    match kind {
        JointKind::Prismatic => Limits {
            lower: -1.0,
            upper: 1.0,
            effort: 0.0,
            velocity: 0.0,
        },
        _ => Limits {
            lower: -std::f64::consts::PI,
            upper: std::f64::consts::PI,
            effort: 0.0,
            velocity: 0.0,
        },
    }
}

/// The one number format of the panel, for fields and readouts alike:
/// six significant figures (never fewer than the integer part), scientific
/// notation below `1e-3`, zero below `1e-12`, trailing zeros dropped, no
/// `-0` — `2.86e-5`, `0.001`, `-3`, `1.25`, `1240`.
///
/// Significant figures rather than decimals so a tensor entry in kg·m²
/// keeps its digits: with six *decimals* both `2.86e-5` and `3e-5` showed
/// as `0.000029`, and an edit between them was refused as "no change".
/// [`number_field`] compares through this function, so "differs" means
/// "differs at the displayed precision" and the parser (`str::parse`)
/// accepts both spellings.
pub fn fmt_num(v: f64) -> String {
    if !v.is_finite() {
        return format!("{v}");
    }
    if v.abs() >= 1e6 {
        // Never fewer digits than the integer part has.
        return format!("{v:.0}");
    }
    // Round to six significant figures first, so the branch below sees the
    // rounded value (`0.000999999` is `0.001`, not `1e-3`).
    let rounded: f64 = format!("{v:.5e}").parse().unwrap_or(v);
    if rounded.abs() < 1e-12 {
        // Round-off, not a number: the writers keep twelve decimals
        // (02 §Writers), so nothing smaller could reach a file anyway.
        return "0".to_owned();
    }
    let trim = |s: &str| -> String {
        if s.contains('.') {
            s.trim_end_matches('0').trim_end_matches('.').to_owned()
        } else {
            s.to_owned()
        }
    };
    if rounded.abs() < 1e-3 {
        let sci = format!("{rounded:.5e}");
        let (mantissa, exponent) = sci.split_once('e').unwrap_or((&sci, "0"));
        format!("{}e{exponent}", trim(mantissa))
    } else {
        let magnitude = rounded.abs().log10().floor() as i32;
        let decimals = (5 - magnitude).max(0) as usize;
        trim(&format!("{rounded:.decimals$}"))
    }
}

/// A text field editing a string, committed on Enter / lost focus. The
/// second value is the new text when it was committed and differs from
/// `value`. Escape reverts. `label` names the field for the accessibility
/// tree (`labelled_by`), so a test can find "x" of a row.
fn text_field(
    ui: &mut egui::Ui,
    state: &mut PropertiesState,
    id: egui::Id,
    label: &egui::Response,
    value: &str,
    width: f32,
) -> (egui::Response, Option<String>) {
    let mut text = state
        .drafts
        .get(&id)
        .cloned()
        .unwrap_or_else(|| value.to_owned());
    let response = ui
        .add(
            egui::TextEdit::singleline(&mut text)
                .id(id)
                .desired_width(width),
        )
        .labelled_by(label.id);
    if response.changed() {
        state.drafts.insert(id, text.clone());
    }
    let mut committed = None;
    if response.lost_focus() {
        let draft = state.drafts.remove(&id);
        let escaped = ui.input(|i| i.key_pressed(egui::Key::Escape));
        let text = draft.unwrap_or(text);
        if !escaped && text != value {
            committed = Some(text);
        }
    }
    (response, committed)
}

/// A number field that scrubs. A horizontal drag changes the value at
/// [`scrub_speed`] per point, one `Set…` per frame under the field's
/// gesture ([`PropertiesState::gesture`]) so the whole drag is one undo
/// entry; a click opens the text editor, which commits on Enter or lost
/// focus and reverts on Escape. `None` unless the value changed at the
/// displayed precision ([`fmt_num`]). `step` is the unit floor of the
/// speed; `label` names the field for the accessibility tree.
fn number_field(
    ui: &mut egui::Ui,
    state: &mut PropertiesState,
    id: egui::Id,
    label: &egui::Response,
    value: f64,
    step: f64,
) -> Option<f64> {
    let shown = fmt_num(value);
    let fine = ui.input(|i| i.modifiers.ctrl);
    let speed = scrub_speed(value, step, fine);
    let mut edited: Option<f64> = None;
    let response = ui
        .scope(|ui| {
            // The button's resting width and the text editor's, both.
            ui.spacing_mut().interact_size.x = FIELD_WIDTH;
            // The id `DragValue` takes: nothing allocates between here and
            // its own `next_auto_id`.
            let widget = ui.next_auto_id();
            let response = ui.add(
                egui::DragValue::from_get_set(|new| {
                    if let Some(n) = new {
                        edited = Some(n);
                    }
                    value
                })
                .speed(speed)
                .custom_formatter(|v, _| fmt_num(v))
                .custom_parser(|text| text.trim().parse::<f64>().ok().filter(|v| v.is_finite()))
                .update_while_editing(false),
            );
            if response.lost_focus() {
                // The editor closed this frame — committed on Enter or lost
                // focus, or reverted on Escape — and that is the end of it.
                // `DragValue` also stashes the text and would parse it once
                // more next frame: a second commit, which renormalises an
                // axis twice, and an Escape that commits after all.
                ui.data_mut(|data| data.remove_temp::<String>(widget));
            }
            response
        })
        .inner
        .labelled_by(label.id);
    let gesture = GestureId(id.value());
    if response.drag_started() {
        state.gesture_break = true;
    }
    if response.dragged() {
        state.gesture = Some(gesture);
    }
    if response.drag_stopped() {
        state.gesture_ended = true;
    }
    if response.hovered() {
        let notches = wheel_notches(ui);
        if notches != 0 {
            let increment = wheel_increment(value, step);
            let stepped =
                ((value + f64::from(notches) * increment) / increment).round() * increment;
            edited = Some(stepped);
            let now = ui.input(|i| i.time);
            let burst =
                matches!(state.wheel, Some((g, t)) if g == gesture && now - t < WHEEL_BURST);
            if !burst {
                state.gesture_break = true;
            }
            state.wheel = Some((gesture, now));
            state.gesture = Some(gesture);
        }
    }
    let n = edited?;
    (n.is_finite() && fmt_num(n) != shown).then_some(n)
}

/// Three labelled number fields in a row (`x y z`, `roll pitch yaw`).
/// Returns the vector with the one committed component replaced.
fn vec3_row(
    ui: &mut egui::Ui,
    state: &mut PropertiesState,
    id: egui::Id,
    labels: [&str; 3],
    v: DVec3,
    step: f64,
) -> Option<DVec3> {
    let mut out = None;
    ui.horizontal(|ui| {
        for (i, label) in labels.iter().enumerate() {
            let tag = ui.label(*label);
            if let Some(n) = number_field(ui, state, id.with(i), &tag, v[i], step) {
                let mut w = v;
                w[i] = n;
                out = Some(w);
            }
        }
    });
    out
}

/// One labelled number field on a grid row.
fn number_row(
    ui: &mut egui::Ui,
    state: &mut PropertiesState,
    id: egui::Id,
    label: &str,
    value: f64,
    step: f64,
) -> Option<f64> {
    let tag = ui.label(label);
    let edited = number_field(ui, state, id, &tag, value, step);
    ui.end_row();
    edited
}

fn degrees(rpy: DVec3) -> DVec3 {
    DVec3::new(rpy.x.to_degrees(), rpy.y.to_degrees(), rpy.z.to_degrees())
}

fn radians(rpy: DVec3) -> DVec3 {
    DVec3::new(rpy.x.to_radians(), rpy.y.to_radians(), rpy.z.to_radians())
}

/// `xyz` (meters) and `rpy` (degrees) rows editing a pose; the committed
/// pose if a component changed.
fn pose_rows(
    ui: &mut egui::Ui,
    state: &mut PropertiesState,
    id: egui::Id,
    pose: &Pose,
) -> Option<Pose> {
    let (xyz, rpy) = pose.to_xyz_rpy();
    ui.label("position");
    let new_xyz = vec3_row(ui, state, id.with("xyz"), ["x", "y", "z"], xyz, STEP_M);
    ui.end_row();
    ui.label("rotation °");
    let new_rpy = vec3_row(
        ui,
        state,
        id.with("rpy"),
        ["roll", "pitch", "yaw"],
        degrees(rpy),
        STEP_DEG,
    );
    ui.end_row();
    match (new_xyz, new_rpy) {
        (Some(xyz), _) => Some(Pose::from_xyz_rpy(xyz, rpy)),
        (None, Some(deg)) => Some(Pose::from_xyz_rpy(xyz, radians(deg))),
        (None, None) => None,
    }
}

impl RiggenApp {
    pub(crate) fn properties_panel(&mut self, ui: &mut egui::Ui) {
        let mut commands: Vec<Command> = Vec::new();
        let mut add_mesh_to: Option<LinkId> = None;
        egui::Panel::right("properties_panel")
            .resizable(true)
            .default_size(380.0)
            .show(ui, |ui| {
                ui.heading("Properties");
                ui.separator();
                egui::ScrollArea::vertical().show(ui, |ui| match self.selection {
                    Selection::None => {
                        ui.weak("Nothing selected");
                    }
                    Selection::Link(link) => {
                        self.link_properties(ui, link, &mut commands, &mut add_mesh_to);
                    }
                    Selection::Joint(joint) => self.joint_properties(ui, joint, &mut commands),
                    Selection::Frame(frame) => self.frame_properties(ui, frame, &mut commands),
                });
            });
        // A scrub's commands coalesce under its gesture; release ends it
        // after the last one lands (docs/02-data-model.md §Commands and
        // history: one gesture = one history entry).
        let gesture = self.props.gesture.take();
        let ended = std::mem::take(&mut self.props.gesture_ended);
        if std::mem::take(&mut self.props.gesture_break) {
            self.end_gesture();
        }
        for command in commands {
            let _ = match gesture {
                Some(g) => self.apply_in_gesture(command, g),
                None => self.apply(command),
            };
        }
        if ended {
            self.end_gesture();
        }
        if let Some(link) = add_mesh_to {
            self.add_mesh_dialog(link);
        }
    }

    fn link_properties(
        &mut self,
        ui: &mut egui::Ui,
        link: LinkId,
        commands: &mut Vec<Command>,
        add_mesh_to: &mut Option<LinkId>,
    ) {
        let Some(data) = self.robot.links.get(&link).cloned() else {
            return;
        };
        let state = &mut self.props;
        let base = ui.make_persistent_id(("link", link));

        egui::Grid::new(base.with("grid"))
            .num_columns(2)
            .show(ui, |ui| {
                let tag = ui.label("name");
                if let (_, Some(name)) =
                    text_field(ui, state, base.with("name"), &tag, &data.name, 160.0)
                {
                    commands.push(Command::RenameLink(link, name));
                }
                ui.end_row();

                ui.label("material");
                let mut material = data.material.clone();
                let shown = material.as_deref().unwrap_or("(none)").to_owned();
                egui::ComboBox::from_id_salt(base.with("material"))
                    .selected_text(shown)
                    .show_ui(ui, |ui| {
                        ui.selectable_value(&mut material, None, "(none)");
                        for name in self.robot.materials.keys() {
                            ui.selectable_value(&mut material, Some(name.clone()), name);
                        }
                    });
                if material != data.material {
                    commands.push(Command::SetLinkMaterial(link, material));
                }
                ui.end_row();
            });

        ui.add_space(8.0);
        ui.horizontal(|ui| {
            ui.strong("Meshes");
            ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                if ui.button("Add mesh to this link…").clicked() {
                    *add_mesh_to = Some(link);
                }
            });
        });
        if data.visuals.is_empty() {
            ui.weak("no meshes; drop a file or add one above");
        }
        for geom in &data.visuals {
            let gid = base.with(("geom", geom.id));
            let asset = self.robot.assets.get(&geom.mesh).cloned();
            let file = asset
                .as_ref()
                .and_then(|a| a.path.file_name())
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_else(|| geom.mesh.to_string());
            ui.separator();
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(&file).strong());
                ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                    if ui.small_button("Remove").clicked() {
                        commands.push(Command::RemoveGeom(link, geom.id));
                    }
                });
            });
            egui::Grid::new(gid.with("grid"))
                .num_columns(2)
                .show(ui, |ui| {
                    if let Some(pose) = pose_rows(ui, state, gid.with("pose"), &geom.pose) {
                        commands.push(Command::SetGeomPose(link, geom.id, pose));
                    }
                    if let Some(asset) = asset {
                        let mut edited = asset.clone();
                        if let Some(scale) = number_row(
                            ui,
                            state,
                            gid.with("scale"),
                            "scale",
                            asset.scale,
                            STEP_UNIT,
                        ) && scale > 0.0
                        {
                            edited.scale = scale;
                        }
                        let fix_rpy = asset
                            .fix_up
                            .map(|q| Pose::from_rotation(q).to_xyz_rpy().1)
                            .unwrap_or(DVec3::ZERO);
                        ui.label("fix-up °");
                        if let Some(deg) = vec3_row(
                            ui,
                            state,
                            gid.with("fixup"),
                            ["roll", "pitch", "yaw"],
                            degrees(fix_rpy),
                            STEP_DEG,
                        ) {
                            edited.fix_up = (deg != DVec3::ZERO)
                                .then(|| Pose::from_xyz_rpy(DVec3::ZERO, radians(deg)).r);
                        }
                        ui.end_row();
                        if edited != asset {
                            commands.push(Command::SetAsset(geom.mesh, edited));
                        }
                    }
                });
        }

        ui.add_space(8.0);
        self.inertial_properties(ui, link, &data.inertial, base.with("inertial"), commands);

        ui.add_space(8.0);
        self.collision_properties(ui, link, &data.collision, base.with("collision"), commands);
    }

    /// Every vertex of the link's visual meshes, in the link frame: what a
    /// primitive is fitted to.
    /// What the job thread has to say about `link`'s decomposition: the
    /// piece count over its visuals, a spinner while any of them is still
    /// running, or the first reason there are none.
    fn decomp_readout(&self, link: LinkId, policy: &CollisionPolicy) -> DecompReadout {
        let CollisionPolicy::ConvexDecomposition {
            max_hulls,
            resolution,
            concavity,
        } = *policy
        else {
            return DecompReadout::NoMesh;
        };
        let params = DecompParams {
            max_hulls,
            resolution,
            concavity,
        };
        let Some(data) = self.robot.links.get(&link) else {
            return DecompReadout::NoMesh;
        };
        if data.visuals.is_empty() {
            return DecompReadout::NoMesh;
        }
        if !self.decomp_consent {
            return DecompReadout::NeedsConsent;
        }
        let mut pieces = 0;
        for g in &data.visuals {
            match self.decomposition(g.mesh, params) {
                Some(Ok(ps)) => pieces += ps.len(),
                Some(Err(reason)) => return DecompReadout::Failed(reason.to_owned()),
                None => return DecompReadout::Working,
            }
        }
        DecompReadout::Pieces(pieces)
    }

    fn link_points(&self, link: LinkId) -> Vec<DVec3> {
        let Some(data) = self.robot.links.get(&link) else {
            return Vec::new();
        };
        let mut points = Vec::new();
        for g in &data.visuals {
            if let Some(loaded) = self.mesh_store.get(&g.mesh) {
                points.extend(
                    loaded
                        .mesh
                        .positions
                        .iter()
                        .map(|p| g.pose.transform_point(*p)),
                );
            }
        }
        points
    }

    /// Properties › Collision: the policy combo; for `Primitives` the list
    /// with add (fitted to the meshes on creation) / remove / fit-to-mesh
    /// and each shape's pose and size; `Meshes` read-only. Every commit is
    /// one `SetCollision`.
    fn collision_properties(
        &mut self,
        ui: &mut egui::Ui,
        link: LinkId,
        policy: &CollisionPolicy,
        base: egui::Id,
        commands: &mut Vec<Command>,
    ) {
        let decomposition = self.decomp_readout(link, policy);
        let points = self.link_points(link);
        // Set by the freeze warning's button; applied once the panel's
        // borrow of `self.props` is done with.
        let mut consented = false;
        let assets = &self.robot.assets;
        let state = &mut self.props;

        ui.strong("Collision");
        ui.horizontal(|ui| {
            ui.label("policy");
            let mut mode = CollisionMode::of(policy);
            let before = mode;
            egui::ComboBox::from_id_salt(base.with("policy"))
                .selected_text(mode.label())
                .show_ui(ui, |ui| {
                    for m in CollisionMode::OFFERED {
                        ui.selectable_value(&mut mode, m, m.label());
                    }
                    if !CollisionMode::OFFERED.contains(&before) {
                        ui.selectable_value(&mut mode, before, before.label());
                    }
                });
            if mode != before {
                let next = match mode {
                    CollisionMode::None => CollisionPolicy::None,
                    CollisionMode::SameAsVisual => CollisionPolicy::SameAsVisual,
                    CollisionMode::ConvexHull => CollisionPolicy::ConvexHull,
                    // Starts with a box around the meshes: something to see
                    // and resize, not an empty list.
                    CollisionMode::Primitives => {
                        CollisionPolicy::Primitives(vec![PrimitiveKind::Box.fitted(&points)])
                    }
                    // The algorithm's own defaults, measured at 54–90 ms
                    // a part (plans/convex-decomposition OPEN 2): a
                    // second, not a minute.
                    CollisionMode::Decomposition => {
                        let d = DecompParams::default();
                        CollisionPolicy::ConvexDecomposition {
                            max_hulls: d.max_hulls,
                            resolution: d.resolution,
                            concavity: d.concavity,
                        }
                    }
                    CollisionMode::Meshes => policy.clone(),
                };
                if next != *policy {
                    commands.push(Command::SetCollision(link, next));
                }
            }
        });

        match policy {
            CollisionPolicy::Primitives(prims) => {
                ui.horizontal(|ui| {
                    ui.label("add");
                    for kind in PrimitiveKind::ALL {
                        if ui.small_button(format!("+ {}", kind.label())).clicked() {
                            let mut next = prims.clone();
                            next.push(kind.fitted(&points));
                            commands.push(Command::SetCollision(
                                link,
                                CollisionPolicy::Primitives(next),
                            ));
                        }
                    }
                });
                for (i, prim) in prims.iter().enumerate() {
                    let pid = base.with(("prim", i));
                    ui.separator();
                    ui.horizontal(|ui| {
                        ui.label(egui::RichText::new(PrimitiveKind::of(prim).label()).strong());
                        ui.with_layout(egui::Layout::right_to_left(egui::Align::Center), |ui| {
                            if ui.small_button("Remove").clicked() {
                                let mut next = prims.clone();
                                next.remove(i);
                                commands.push(Command::SetCollision(
                                    link,
                                    CollisionPolicy::Primitives(next),
                                ));
                            }
                            if ui.small_button("Fit to mesh").clicked() {
                                let mut next = prims.clone();
                                next[i] = PrimitiveKind::of(prim).fitted(&points);
                                commands.push(Command::SetCollision(
                                    link,
                                    CollisionPolicy::Primitives(next),
                                ));
                            }
                        });
                    });
                    let mut edited = prim.clone();
                    egui::Grid::new(pid.with("grid")).num_columns(2).show(
                        ui,
                        |ui| match &mut edited {
                            Primitive::Box { pose, size } => {
                                if let Some(p) = pose_rows(ui, state, pid.with("pose"), pose) {
                                    *pose = p;
                                }
                                ui.label("size m");
                                if let Some(v) = vec3_row(
                                    ui,
                                    state,
                                    pid.with("size"),
                                    ["x", "y", "z"],
                                    *size,
                                    STEP_M,
                                ) && v.min_element() > 0.0
                                {
                                    *size = v;
                                }
                                ui.end_row();
                            }
                            Primitive::Sphere { pose, radius } => {
                                if let Some(p) = pose_rows(ui, state, pid.with("pose"), pose) {
                                    *pose = p;
                                }
                                if let Some(r) = number_row(
                                    ui,
                                    state,
                                    pid.with("radius"),
                                    "radius m",
                                    *radius,
                                    STEP_M,
                                ) && r > 0.0
                                {
                                    *radius = r;
                                }
                            }
                            Primitive::Cylinder {
                                pose,
                                radius,
                                length,
                            }
                            | Primitive::Capsule {
                                pose,
                                radius,
                                length,
                            } => {
                                if let Some(p) = pose_rows(ui, state, pid.with("pose"), pose) {
                                    *pose = p;
                                }
                                if let Some(r) = number_row(
                                    ui,
                                    state,
                                    pid.with("radius"),
                                    "radius m",
                                    *radius,
                                    STEP_M,
                                ) && r > 0.0
                                {
                                    *radius = r;
                                }
                                if let Some(l) = number_row(
                                    ui,
                                    state,
                                    pid.with("length"),
                                    "length m",
                                    *length,
                                    STEP_M,
                                ) && l >= 0.0
                                {
                                    *length = l;
                                }
                            }
                        },
                    );
                    if edited != *prim {
                        let mut next = prims.clone();
                        next[i] = edited;
                        commands.push(Command::SetCollision(
                            link,
                            CollisionPolicy::Primitives(next),
                        ));
                    }
                }
            }
            CollisionPolicy::Meshes(geoms) => {
                ui.weak("collision meshes from the import; edit the files to change them");
                for g in geoms {
                    let file = assets
                        .get(&g.mesh)
                        .and_then(|a| a.path.file_name())
                        .map(|n| n.to_string_lossy().into_owned())
                        .unwrap_or_else(|| g.mesh.to_string());
                    ui.label(file);
                }
            }
            CollisionPolicy::None => {
                ui.weak("this link does not collide");
            }
            CollisionPolicy::SameAsVisual => {
                ui.weak("the visual meshes collide (MuJoCo takes their convex hulls)");
            }
            CollisionPolicy::ConvexHull => {
                ui.weak("one convex hull per visual mesh (View › Collision geometry shows them)");
            }
            CollisionPolicy::ConvexDecomposition {
                max_hulls,
                resolution,
                concavity,
            } => {
                ui.weak("V-HACD: convex pieces that keep the part's concavity");
                let params = DecompParams {
                    max_hulls: *max_hulls,
                    resolution: *resolution,
                    concavity: *concavity,
                };
                let mut edited = params;
                egui::Grid::new(base.with("decomp"))
                    .num_columns(2)
                    .show(ui, |ui| {
                        let id = base.with("decomp");
                        // Integers through the same draft-buffer field as
                        // every other number, rounded on commit.
                        if let Some(v) = number_row(
                            ui,
                            state,
                            id.with("max_hulls"),
                            "max pieces",
                            params.max_hulls as f64,
                            STEP_INT,
                        ) && v >= 1.0
                            && v <= MAX_DECOMP_HULLS as f64
                        {
                            edited.max_hulls = v.round() as u32;
                        }
                        if let Some(v) = number_row(
                            ui,
                            state,
                            id.with("resolution"),
                            "voxel grid",
                            params.resolution as f64,
                            STEP_INT,
                        ) && v >= 1.0
                            && v <= MAX_DECOMP_RESOLUTION as f64
                        {
                            edited.resolution = v.round() as u32;
                        }
                        if let Some(v) = number_row(
                            ui,
                            state,
                            id.with("concavity"),
                            "concavity",
                            params.concavity,
                            STEP_UNIT,
                        ) && (0.0..=1.0).contains(&v)
                        {
                            edited.concavity = v;
                        }
                    });
                if edited != params {
                    commands.push(Command::SetCollision(
                        link,
                        CollisionPolicy::ConvexDecomposition {
                            max_hulls: edited.max_hulls,
                            resolution: edited.resolution,
                            concavity: edited.concavity,
                        },
                    ));
                }
                match decomposition {
                    DecompReadout::Pieces(n) => {
                        ui.horizontal(|ui| ui.label(format!("pieces: {n}")));
                    }
                    DecompReadout::Working => {
                        ui.horizontal(|ui| {
                            ui.spinner();
                            ui.weak("computing…");
                        });
                    }
                    DecompReadout::Failed(reason) => {
                        ui.horizontal(|ui| ui.colored_label(ui.visuals().error_fg_color, reason));
                    }
                    DecompReadout::NoMesh => {
                        ui.horizontal(|ui| {
                            ui.weak("nothing to decompose: the link has no visual mesh")
                        });
                    }
                    // Asked once per session, not once per link: saying yes
                    // here starts every decomposition the document wants.
                    DecompReadout::NeedsConsent => {
                        ui.colored_label(ui.visuals().warn_fg_color, DECOMP_FREEZE_WARNING);
                        if ui.button(DECOMP_CONSENT_BUTTON).clicked() {
                            consented = true;
                        }
                    }
                }
            }
        }
        if consented {
            self.decomp_consent = true;
        }
    }

    /// Properties › Inertial: the mode, its fields, and what the meshes
    /// say beside it (docs/02-data-model.md §Inertials). Every committed
    /// field is one `SetInertial`.
    fn inertial_properties(
        &mut self,
        ui: &mut egui::Ui,
        link: LinkId,
        spec: &InertialSpec,
        base: egui::Id,
        commands: &mut Vec<Command>,
    ) {
        let composed = self.link_inertial(link);
        let computed: Option<Inertial> = composed.as_ref().ok().and_then(|c| c.computed);
        // Why the meshes say nothing, when they do not.
        let readout_error: Option<InertialError> = match &composed {
            Err(e) => Some(e.clone()),
            Ok(c) if c.computed.is_none() => self.robot.links.get(&link).and_then(|l| {
                riggen_core::inertial::computed_inertial(
                    l,
                    &crate::app::document::AppMeshes(&self.mesh_store),
                    &self.robot.materials,
                )
                .err()
            }),
            Ok(_) => None,
        };
        let material_density = self
            .robot
            .links
            .get(&link)
            .and_then(|l| l.material.as_ref())
            .and_then(|m| self.robot.materials.get(m))
            .map(|m| m.density);
        let open_mesh_file = |geom: &riggen_core::GeomId| {
            self.robot
                .links
                .get(&link)
                .and_then(|l| l.visuals.iter().find(|g| g.id == *geom))
                .and_then(|g| self.robot.assets.get(&g.mesh))
                .and_then(|a| a.path.file_name())
                .map(|n| n.to_string_lossy().into_owned())
                .unwrap_or_else(|| geom.to_string())
        };
        let state = &mut self.props;

        ui.strong("Inertial");
        egui::Grid::new(base.with("grid"))
            .num_columns(2)
            .show(ui, |ui| {
                ui.label("mode");
                let mut mode = InertialMode::of(spec);
                let before = mode;
                egui::ComboBox::from_id_salt(base.with("mode"))
                    .selected_text(mode.label())
                    .show_ui(ui, |ui| {
                        for m in InertialMode::ALL {
                            ui.selectable_value(&mut mode, m, m.label());
                        }
                    });
                if mode != before {
                    // A new mode starts from what the meshes say, so an
                    // override is a correction, not a blank form.
                    let seed = computed.unwrap_or(Inertial::ZERO);
                    let next = match mode {
                        InertialMode::Computed => InertialSpec::Computed {
                            density_override: None,
                        },
                        InertialMode::Override => InertialSpec::Override {
                            mass: seed.mass,
                            com: seed.com,
                            inertia: seed.inertia,
                        },
                        InertialMode::Hybrid => InertialSpec::Hybrid { mass: seed.mass },
                    };
                    commands.push(Command::SetInertial(link, next));
                }
                ui.end_row();

                match spec {
                    InertialSpec::Computed { density_override } => {
                        let tag = ui.label("density override");
                        ui.horizontal(|ui| {
                            let mut on = density_override.is_some();
                            if ui.checkbox(&mut on, "").changed() {
                                commands.push(Command::SetInertial(
                                    link,
                                    InertialSpec::Computed {
                                        density_override: on
                                            .then(|| material_density.unwrap_or(1000.0)),
                                    },
                                ));
                            }
                            if let Some(d) = density_override
                                && let Some(n) = number_field(
                                    ui,
                                    state,
                                    base.with("density"),
                                    &tag,
                                    *d,
                                    STEP_DENSITY,
                                )
                                && n > 0.0
                            {
                                commands.push(Command::SetInertial(
                                    link,
                                    InertialSpec::Computed {
                                        density_override: Some(n),
                                    },
                                ));
                            }
                            ui.weak("kg/m³");
                        });
                        ui.end_row();
                    }
                    InertialSpec::Override { mass, com, inertia } => {
                        if let Some(m) =
                            number_row(ui, state, base.with("mass"), "mass kg", *mass, STEP_KG)
                        {
                            commands.push(Command::SetInertial(
                                link,
                                InertialSpec::Override {
                                    mass: m,
                                    com: *com,
                                    inertia: *inertia,
                                },
                            ));
                        }
                        ui.label("CoM m");
                        if let Some(c) =
                            vec3_row(ui, state, base.with("com"), ["x", "y", "z"], *com, STEP_M)
                        {
                            commands.push(Command::SetInertial(
                                link,
                                InertialSpec::Override {
                                    mass: *mass,
                                    com: c,
                                    inertia: *inertia,
                                },
                            ));
                        }
                        ui.end_row();
                        // Six independent entries; the tensor is symmetric.
                        let entries = [
                            ("Ixx", inertia.x_axis.x),
                            ("Iyy", inertia.y_axis.y),
                            ("Izz", inertia.z_axis.z),
                            ("Ixy", inertia.y_axis.x),
                            ("Ixz", inertia.z_axis.x),
                            ("Iyz", inertia.z_axis.y),
                        ];
                        let mut edited: Option<[f64; 6]> = None;
                        for (row, chunk) in entries.chunks(3).enumerate() {
                            ui.label(if row == 0 { "inertia kg·m²" } else { "" });
                            ui.horizontal(|ui| {
                                for (col, (label, value)) in chunk.iter().enumerate() {
                                    let tag = ui.label(*label);
                                    let id = base.with(("inertia", row, col));
                                    if let Some(n) =
                                        number_field(ui, state, id, &tag, *value, STEP_KGM2)
                                    {
                                        let mut all = entries.map(|(_, v)| v);
                                        all[row * 3 + col] = n;
                                        edited = Some(all);
                                    }
                                }
                            });
                            ui.end_row();
                        }
                        if let Some([ixx, iyy, izz, ixy, ixz, iyz]) = edited {
                            commands.push(Command::SetInertial(
                                link,
                                InertialSpec::Override {
                                    mass: *mass,
                                    com: *com,
                                    inertia: DMat3::from_cols(
                                        DVec3::new(ixx, ixy, ixz),
                                        DVec3::new(ixy, iyy, iyz),
                                        DVec3::new(ixz, iyz, izz),
                                    ),
                                },
                            ));
                        }
                    }
                    InertialSpec::Hybrid { mass } => {
                        if let Some(m) =
                            number_row(ui, state, base.with("mass"), "mass kg", *mass, STEP_KG)
                            && m > 0.0
                        {
                            commands
                                .push(Command::SetInertial(link, InertialSpec::Hybrid { mass: m }));
                        }
                    }
                }
            });

        // What the meshes say, for comparison — or why they say nothing.
        ui.add_space(4.0);
        ui.weak("computed from the meshes");
        match (computed, readout_error) {
            (Some(c), _) => {
                egui::Grid::new(base.with("readout"))
                    .num_columns(2)
                    .show(ui, |ui| {
                        ui.label("mass");
                        ui.label(format!("{} kg", fmt_num(c.mass)));
                        ui.end_row();
                        ui.label("CoM");
                        ui.label(format!(
                            "{} {} {} m",
                            fmt_num(c.com.x),
                            fmt_num(c.com.y),
                            fmt_num(c.com.z)
                        ));
                        ui.end_row();
                        let [a, b, d] = principal_moments(&c.inertia);
                        ui.label("principal");
                        ui.label(format!(
                            "{} {} {} kg·m²",
                            fmt_num(a),
                            fmt_num(b),
                            fmt_num(d)
                        ));
                        ui.end_row();
                    });
            }
            (None, Some(err)) => {
                let text = match &err {
                    InertialError::OpenMesh { geom } => {
                        format!("open mesh: {} is not closed", open_mesh_file(geom))
                    }
                    other => other.to_string(),
                };
                ui.colored_label(ui.visuals().warn_fg_color, text);
            }
            (None, None) => {
                ui.weak("nothing to compute");
            }
        }
    }

    /// A named frame (ADR-0012): name, the link it hangs on, and its pose
    /// in that link's frame as xyz and RPY. Whatever changed, the panel
    /// commits **one** `SetFrame` — one gesture, one command.
    ///
    /// Changing the link keeps the frame where it is in the world: the
    /// panel re-expresses the pose through `fk` first, so the command stays
    /// dumb and writes what it is given, like `SetJoint`. Like every other
    /// frame-rewriting edit that is done in the **zero configuration**, the
    /// one `Reparent { keep_world_pose }` and `origin_for_world` also work
    /// in.
    fn frame_properties(&mut self, ui: &mut egui::Ui, frame: FrameId, commands: &mut Vec<Command>) {
        let Some(data) = self.robot.frames.get(&frame).cloned() else {
            return;
        };
        let base = ui.make_persistent_id(("frame", frame));
        let mut edited = data.clone();

        egui::Grid::new(base.with("grid"))
            .num_columns(2)
            .show(ui, |ui| {
                let state = &mut self.props;
                let tag = ui.label("name");
                if let (_, Some(name)) =
                    text_field(ui, state, base.with("name"), &tag, &data.name, 160.0)
                {
                    edited.name = name;
                }
                ui.end_row();

                ui.label("link");
                let shown = self
                    .robot
                    .links
                    .get(&data.parent)
                    .map_or("—", |l| l.name.as_str())
                    .to_owned();
                let mut parent = data.parent;
                egui::ComboBox::from_id_salt(base.with("link"))
                    .selected_text(shown)
                    .show_ui(ui, |ui| {
                        for (id, link) in &self.robot.links {
                            ui.selectable_value(&mut parent, *id, &link.name);
                        }
                    });
                if parent != data.parent {
                    let world = fk(&self.robot, &JointState::default());
                    if let (Some(from), Some(to)) = (world.get(&data.parent), world.get(&parent)) {
                        edited.pose = to.inverse().compose(&from.compose(&data.pose));
                    }
                    edited.parent = parent;
                }
                ui.end_row();

                let state = &mut self.props;
                if let Some(pose) = pose_rows(ui, state, base.with("pose"), &data.pose) {
                    edited.pose = pose;
                }
            });

        if edited != data {
            commands.push(Command::SetFrame(frame, edited));
        }
    }

    fn joint_properties(&mut self, ui: &mut egui::Ui, joint: JointId, commands: &mut Vec<Command>) {
        let Some(data) = self.robot.joints.get(&joint).cloned() else {
            return;
        };
        let state = &mut self.props;
        let base = ui.make_persistent_id(("joint", joint));
        let mut edited = data.clone();

        egui::Grid::new(base.with("grid"))
            .num_columns(2)
            .show(ui, |ui| {
                let tag = ui.label("name");
                if let (_, Some(name)) =
                    text_field(ui, state, base.with("name"), &tag, &data.name, 160.0)
                {
                    commands.push(Command::RenameJoint(joint, name));
                }
                ui.end_row();

                ui.label("kind");
                let mut kind = data.kind;
                egui::ComboBox::from_id_salt(base.with("kind"))
                    .selected_text(format!("{kind:?}"))
                    .show_ui(ui, |ui| {
                        for k in [
                            JointKind::Fixed,
                            JointKind::Revolute,
                            JointKind::Continuous,
                            JointKind::Prismatic,
                        ] {
                            ui.selectable_value(&mut kind, k, format!("{k:?}"));
                        }
                    });
                if kind != data.kind {
                    edited.kind = kind;
                    if kind.requires_limits() && edited.limits.is_none() {
                        edited.limits = Some(default_limits(kind));
                    }
                    if !kind.is_movable() {
                        // A fixed joint has no value to drive and no
                        // degree of freedom to actuate, so both go with
                        // the kind rather than being refused by `validate`
                        // after the fact (ADR-0013, ADR-0014).
                        edited.mimic = None;
                        edited.actuator = None;
                    }
                }
                ui.end_row();

                ui.label("parent");
                ui.label(
                    self.robot
                        .links
                        .get(&data.parent)
                        .map_or_else(|| data.parent.to_string(), |l| l.name.clone()),
                );
                ui.end_row();

                if let Some(origin) = pose_rows(ui, state, base.with("origin"), &data.origin) {
                    edited.origin = origin;
                }

                if data.kind.is_movable() {
                    ui.label("axis");
                    if let Some(axis) = vec3_row(
                        ui,
                        state,
                        base.with("axis"),
                        ["x", "y", "z"],
                        data.axis,
                        STEP_UNIT,
                    ) {
                        // Normalised on commit; a zero axis is left for
                        // `validate` to refuse with its message.
                        edited.axis = axis.normalize_or_zero();
                    }
                    ui.end_row();
                }

                if data.kind.requires_limits() {
                    let limits = data.limits.unwrap_or_else(|| default_limits(data.kind));
                    let angular = data.kind == JointKind::Revolute;
                    let (unit, to_ui, from_ui, step): (&str, Convert, Convert, f64) = if angular {
                        ("°", f64::to_degrees, f64::to_radians, STEP_DEG)
                    } else {
                        ("m", |x| x, |x| x, STEP_M)
                    };
                    let mut new_limits = limits;
                    if let Some(v) = number_row(
                        ui,
                        state,
                        base.with("lower"),
                        &format!("lower {unit}"),
                        to_ui(limits.lower),
                        step,
                    ) {
                        new_limits.lower = from_ui(v);
                    }
                    if let Some(v) = number_row(
                        ui,
                        state,
                        base.with("upper"),
                        &format!("upper {unit}"),
                        to_ui(limits.upper),
                        step,
                    ) {
                        new_limits.upper = from_ui(v);
                    }
                    if let Some(v) = number_row(
                        ui,
                        state,
                        base.with("effort"),
                        "effort",
                        limits.effort,
                        STEP_UNIT,
                    ) {
                        new_limits.effort = v;
                    }
                    if let Some(v) = number_row(
                        ui,
                        state,
                        base.with("velocity"),
                        "velocity",
                        limits.velocity,
                        STEP_UNIT,
                    ) {
                        new_limits.velocity = v;
                    }
                    if new_limits != limits {
                        edited.limits = Some(new_limits);
                    }
                }

                // A coupled degree of freedom (ADR-0013). The combo offers
                // exactly the leaders `validate` accepts: a movable joint
                // that is not this one and does not itself follow, so a
                // chain cannot be built by picking one.
                if data.kind.is_movable() {
                    let leaders: Vec<(JointId, String)> = self
                        .robot
                        .joints
                        .iter()
                        .filter(|(id, j)| **id != joint && j.kind.is_movable() && j.mimic.is_none())
                        .map(|(&id, j)| (id, j.name.clone()))
                        .collect();
                    ui.label("mimic");
                    let mut leader = data.mimic.map(|m| m.joint);
                    let shown = leader.map_or_else(
                        || "none".to_owned(),
                        |id| {
                            self.robot
                                .joints
                                .get(&id)
                                .map_or_else(|| id.to_string(), |j| j.name.clone())
                        },
                    );
                    egui::ComboBox::from_id_salt(base.with("mimic"))
                        .selected_text(shown)
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut leader, None, "none");
                            for (id, name) in &leaders {
                                ui.selectable_value(&mut leader, Some(*id), name);
                            }
                        });
                    ui.end_row();
                    if leader != data.mimic.map(|m| m.joint) {
                        // A fresh coupling starts at `q = q(leader)`; the
                        // two fields below it are how it stops being that.
                        edited.mimic = leader.map(|l| Mimic {
                            joint: l,
                            multiplier: data.mimic.map_or(1.0, |m| m.multiplier),
                            offset: data.mimic.map_or(0.0, |m| m.offset),
                        });
                    }
                    if let Some(mimic) = data.mimic {
                        // The offset is in the follower's own unit and the
                        // multiplier is a ratio, so neither is converted
                        // to degrees the way the limits above are.
                        let unit = if data.kind == JointKind::Prismatic {
                            "m"
                        } else {
                            "rad"
                        };
                        if let Some(v) = number_row(
                            ui,
                            state,
                            base.with("multiplier"),
                            "multiplier",
                            mimic.multiplier,
                            STEP_UNIT,
                        ) && let Some(m) = &mut edited.mimic
                        {
                            m.multiplier = v;
                        }
                        if let Some(v) = number_row(
                            ui,
                            state,
                            base.with("offset"),
                            &format!("offset {unit}"),
                            mimic.offset,
                            STEP_UNIT,
                        ) && let Some(m) = &mut edited.mimic
                        {
                            m.offset = v;
                        }
                    }
                }

                // What drives the joint in the exported MJCF (ADR-0014).
                // A follower is left out: its `<equality>` already moves
                // it, and `validate` refuses an actuator beside one.
                if data.kind.is_movable() && data.mimic.is_none() {
                    ui.label("actuator");
                    let mut actuator = data.actuator;
                    egui::ComboBox::from_id_salt(base.with("actuator"))
                        .selected_text(actuator.map_or("none", ActuatorSpec::kind_name))
                        .show_ui(ui, |ui| {
                            ui.selectable_value(&mut actuator, None, "none");
                            for preset in default_actuators() {
                                ui.selectable_value(
                                    &mut actuator,
                                    Some(preset),
                                    preset.kind_name(),
                                );
                            }
                        });
                    ui.end_row();
                    // The combo picks the *kind*; the fields below edit the
                    // gains, so switching kinds and back does not carry the
                    // old ones over.
                    if actuator.map(ActuatorSpec::kind_name)
                        != data.actuator.map(ActuatorSpec::kind_name)
                    {
                        edited.actuator = actuator;
                    }
                    if let Some(spec) = data.actuator {
                        for (label, value) in gains(spec) {
                            if let Some(v) =
                                number_row(ui, state, base.with(label), label, value, STEP_UNIT)
                                && let Some(edit) = &mut edited.actuator
                            {
                                set_gain(edit, label, v);
                            }
                        }
                    }
                    // Beside the thing it copies: seven joints on one arm
                    // usually want one actuator, and clicking each is the
                    // tedium we exist to remove. Mimic followers are
                    // skipped, not refused (ADR-0014).
                    ui.label("");
                    if ui
                        .button("Apply to every movable joint")
                        .on_hover_text(
                            "every joint that is not fixed and does not follow another one",
                        )
                        .clicked()
                    {
                        commands.push(Command::SetActuators(edited.actuator));
                    }
                    ui.end_row();
                }

                let d = data.dynamics;
                if let Some(v) = number_row(
                    ui,
                    state,
                    base.with("damping"),
                    "damping",
                    d.damping,
                    STEP_UNIT,
                ) {
                    edited.dynamics.damping = v;
                }
                if let Some(v) = number_row(
                    ui,
                    state,
                    base.with("friction"),
                    "friction",
                    d.friction,
                    STEP_UNIT,
                ) {
                    edited.dynamics.friction = v;
                }
                if let Some(v) = number_row(
                    ui,
                    state,
                    base.with("armature"),
                    "armature",
                    d.armature,
                    STEP_UNIT,
                ) {
                    edited.dynamics.armature = v;
                }
            });

        if edited != data {
            commands.push(Command::SetJoint(joint, edited));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::fmt_num;

    /// The four spellings the plan names, plus the edges around them.
    #[test]
    fn fmt_num_six_significant_figures() {
        assert_eq!(fmt_num(2.86e-5), "2.86e-5");
        assert_eq!(fmt_num(0.001), "0.001");
        assert_eq!(fmt_num(-3.0), "-3");
        assert_eq!(fmt_num(1.25), "1.25");
        assert_eq!(fmt_num(1240.0), "1240");
        assert_eq!(fmt_num(206.666_666_7), "206.667");
        assert_eq!(fmt_num(0.123_456_789), "0.123457");
        assert_eq!(fmt_num(0.0), "0");
        assert_eq!(fmt_num(-0.0), "0");
        assert_eq!(fmt_num(-1e-9), "-1e-9");
        assert_eq!(fmt_num(1e-10), "1e-10");
        assert_eq!(fmt_num(-6.89317e-20), "0", "round-off reads as zero");
        assert_eq!(fmt_num(0.000_999_999_9), "0.001");
        assert_eq!(fmt_num(1_234_567.0), "1234567");
        assert_eq!(fmt_num(123_456.7), "123457");
    }

    /// A notch is one unit of the last shown digit; a field at zero steps
    /// by its unit floor.
    #[test]
    fn wheel_increment_is_the_last_shown_digit() {
        use super::{STEP_DEG, STEP_M, wheel_increment};
        assert_eq!(wheel_increment(1240.0, STEP_M), 1.0);
        assert_eq!(wheel_increment(0.5, STEP_M), 0.1);
        assert_eq!(wheel_increment(206.667, STEP_M), 0.001);
        assert!((wheel_increment(2.86e-5, STEP_M) - 1e-7).abs() < 1e-20);
        assert_eq!(wheel_increment(-3.0, STEP_DEG), 1.0);
        assert_eq!(wheel_increment(0.0, STEP_DEG), STEP_DEG);
        assert_eq!(wheel_increment(0.0, STEP_M), STEP_M);
    }

    /// Both spellings parse, and round-trip through the format: what the
    /// field shows is what the parser reads back.
    #[test]
    fn fmt_num_round_trips_both_spellings() {
        for text in ["2.86e-5", "0.0000286", "0.001", "1e-3", "-3", "1.25"] {
            let parsed: f64 = text.parse().unwrap();
            let shown = fmt_num(parsed);
            let back: f64 = shown.parse().unwrap();
            assert_eq!(fmt_num(back), shown, "{text}");
            assert!(
                (back - parsed).abs() <= parsed.abs() * 1e-5,
                "{text} → {shown}"
            );
        }
        // The bug this fixes: `2.86e-5` and `3e-5` are different at the
        // displayed precision.
        assert_ne!(fmt_num(2.86e-5), fmt_num(3e-5));
    }
}
