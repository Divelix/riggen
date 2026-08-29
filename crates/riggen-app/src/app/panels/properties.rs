//! The properties panel (right): what the selection is, as editable
//! fields. Numbers are text fields that commit on Enter or lost focus,
//! never per keystroke, so one edit is one command (and one undo step); a
//! commit equal to the document's value goes nowhere (`History` drops it).
//! Angles are degrees here and radians in the document.

use std::collections::HashMap;

use riggen_core::glam::{DMat3, DVec3};
use riggen_core::inertial::{Inertial, InertialError, principal_moments};
use riggen_core::{Command, InertialSpec, JointId, JointKind, Limits, LinkId, Pose};

use crate::app::{RiggenApp, Selection};

/// Text being typed into numeric / name fields, keyed by the field's
/// widget id, present only while the field has focus.
#[derive(Debug, Clone, Default)]
pub(crate) struct PropertiesState {
    drafts: HashMap<egui::Id, String>,
}

impl PropertiesState {
    /// Drops every unfinished edit — the selection changed under it.
    pub(crate) fn clear(&mut self) {
        self.drafts.clear();
    }
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

/// For the readout: small values (a tensor in kg·m²) in scientific
/// notation, the rest like [`fmt_num`].
fn fmt_readout(v: f64) -> String {
    if v != 0.0 && v.abs() < 1e-3 {
        format!("{v:.3e}")
    } else {
        fmt_num(v)
    }
}

/// Default limits handed to a joint switched to a kind that needs them.
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

/// `0.5`, `-3`, `1.25`: six decimals, trailing zeros dropped, no `-0`.
fn fmt_num(v: f64) -> String {
    let r = (v * 1e6).round() / 1e6;
    let r = if r == 0.0 { 0.0 } else { r };
    format!("{r}")
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

/// A number field. `None` unless committed with a parseable value that
/// differs from `value` (at the displayed precision).
fn number_field(
    ui: &mut egui::Ui,
    state: &mut PropertiesState,
    id: egui::Id,
    label: &egui::Response,
    value: f64,
) -> Option<f64> {
    let shown = fmt_num(value);
    let (_, committed) = text_field(ui, state, id, label, &shown, 56.0);
    let parsed = committed?.trim().parse::<f64>().ok()?;
    (parsed.is_finite() && fmt_num(parsed) != shown).then_some(parsed)
}

/// Three labelled number fields in a row (`x y z`, `roll pitch yaw`).
/// Returns the vector with the one committed component replaced.
fn vec3_row(
    ui: &mut egui::Ui,
    state: &mut PropertiesState,
    id: egui::Id,
    labels: [&str; 3],
    v: DVec3,
) -> Option<DVec3> {
    let mut out = None;
    ui.horizontal(|ui| {
        for (i, label) in labels.iter().enumerate() {
            let tag = ui.label(*label);
            if let Some(n) = number_field(ui, state, id.with(i), &tag, v[i]) {
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
) -> Option<f64> {
    let tag = ui.label(label);
    let edited = number_field(ui, state, id, &tag, value);
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
    let new_xyz = vec3_row(ui, state, id.with("xyz"), ["x", "y", "z"], xyz);
    ui.end_row();
    ui.label("rotation °");
    let new_rpy = vec3_row(
        ui,
        state,
        id.with("rpy"),
        ["roll", "pitch", "yaw"],
        degrees(rpy),
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
                });
            });
        for command in commands {
            let _ = self.apply(command);
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
                        if let Some(scale) =
                            number_row(ui, state, gid.with("scale"), "scale", asset.scale)
                            && scale > 0.0
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
                                && let Some(n) =
                                    number_field(ui, state, base.with("density"), &tag, *d)
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
                        if let Some(m) = number_row(ui, state, base.with("mass"), "mass kg", *mass)
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
                            vec3_row(ui, state, base.with("com"), ["x", "y", "z"], *com)
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
                                    if let Some(n) = number_field(ui, state, id, &tag, *value) {
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
                        if let Some(m) = number_row(ui, state, base.with("mass"), "mass kg", *mass)
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
                        ui.label(format!("{} kg", fmt_readout(c.mass)));
                        ui.end_row();
                        ui.label("CoM");
                        ui.label(format!(
                            "{} {} {} m",
                            fmt_readout(c.com.x),
                            fmt_readout(c.com.y),
                            fmt_readout(c.com.z)
                        ));
                        ui.end_row();
                        let [a, b, d] = principal_moments(&c.inertia);
                        ui.label("principal");
                        ui.label(format!(
                            "{} {} {} kg·m²",
                            fmt_readout(a),
                            fmt_readout(b),
                            fmt_readout(d)
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
                    if let Some(axis) =
                        vec3_row(ui, state, base.with("axis"), ["x", "y", "z"], data.axis)
                    {
                        // Normalised on commit; a zero axis is left for
                        // `validate` to refuse with its message.
                        edited.axis = axis.normalize_or_zero();
                    }
                    ui.end_row();
                }

                if data.kind.requires_limits() {
                    let limits = data.limits.unwrap_or_else(|| default_limits(data.kind));
                    let angular = data.kind == JointKind::Revolute;
                    let (unit, to_ui, from_ui): (&str, Convert, Convert) = if angular {
                        ("°", f64::to_degrees, f64::to_radians)
                    } else {
                        ("m", |x| x, |x| x)
                    };
                    let mut new_limits = limits;
                    if let Some(v) = number_row(
                        ui,
                        state,
                        base.with("lower"),
                        &format!("lower {unit}"),
                        to_ui(limits.lower),
                    ) {
                        new_limits.lower = from_ui(v);
                    }
                    if let Some(v) = number_row(
                        ui,
                        state,
                        base.with("upper"),
                        &format!("upper {unit}"),
                        to_ui(limits.upper),
                    ) {
                        new_limits.upper = from_ui(v);
                    }
                    if let Some(v) =
                        number_row(ui, state, base.with("effort"), "effort", limits.effort)
                    {
                        new_limits.effort = v;
                    }
                    if let Some(v) = number_row(
                        ui,
                        state,
                        base.with("velocity"),
                        "velocity",
                        limits.velocity,
                    ) {
                        new_limits.velocity = v;
                    }
                    if new_limits != limits {
                        edited.limits = Some(new_limits);
                    }
                }

                let d = data.dynamics;
                if let Some(v) = number_row(ui, state, base.with("damping"), "damping", d.damping) {
                    edited.dynamics.damping = v;
                }
                if let Some(v) =
                    number_row(ui, state, base.with("friction"), "friction", d.friction)
                {
                    edited.dynamics.friction = v;
                }
                if let Some(v) =
                    number_row(ui, state, base.with("armature"), "armature", d.armature)
                {
                    edited.dynamics.armature = v;
                }
            });

        if edited != data {
            commands.push(Command::SetJoint(joint, edited));
        }
    }
}
