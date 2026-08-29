//! The application state and its per-frame `ui`: menu bar on top, status
//! bar on the bottom, the viewport in the central panel.

mod debug_menu;
mod document;
mod file_io;
mod file_menu;
mod gizmo;
mod glyphs;
mod panels;
mod shortcuts;
mod snap;
mod status_bar;
mod tool;

use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;

use riggen_core::{GeomId, History, JointId, JointState, LinkId, MeshId, Pose, Robot};
use riggen_viewport::{InstanceId, PickHit, Viewport};
use web_time::Instant;

pub use debug_menu::COPIED_STATUS;
pub(crate) use document::LoadedMesh;
pub use document::Selection;
pub use file_menu::PendingAction;
use file_menu::{IMPORT_SCALE_KEY, IMPORT_UNITS};
use gizmo::GizmoState;
pub use gizmo::GizmoTarget;
pub use glyphs::{GLYPH_HOVER_RADIUS, JointGlyph};
use panels::{JointsWindow, MaterialsWindow, PropertiesState, TreeState};
use snap::SnapCache;
pub use snap::{SNAP_PIXEL_RADIUS, SnapCandidate, SnapKind, placed_status};
pub use tool::{Tool, ZERO_CONFIG_STATUS};

/// The eframe app: one `Robot` and what is derived from it
/// (docs/01-architecture.md §The document is the only state).
pub struct RiggenApp {
    robot: Robot,
    history: History,
    /// Where the document lives on disk; `None` until saved or opened.
    file: Option<PathBuf>,
    /// Mesh geometry beside the document, keyed by asset, loaded once per
    /// file and shared across history snapshots.
    mesh_store: HashMap<MeshId, LoadedMesh>,
    /// The viewport's instance per visual geom (docs/02-data-model.md
    /// §Geom): the only map between document and scene.
    instances: BTreeMap<(LinkId, GeomId), InstanceId>,
    /// Current joint values — slider state, never saved.
    q: JointState,
    selection: Selection,
    /// What a viewport gesture means (`tool.rs`).
    tool: Tool,
    /// The transform gizmo and the drag it is in the middle of
    /// (`gizmo.rs`, ADR-0007).
    gizmo_state: GizmoState,
    /// The joint under the pointer: hovered in the tree, or its glyph
    /// hovered in the viewport. Highlights both, both ways (`glyphs.rs`).
    hovered_joint: Option<JointId>,
    /// Set when that hover came from the *glyph* rather than the tree: a
    /// click then selects the joint, and the viewport's own pick is
    /// suppressed so it does not select the part behind it instead.
    glyph_hover: Option<JointId>,
    /// The toolbar's rect, so a glyph behind it is not "hovered" through it.
    toolbar_rect: Option<egui::Rect>,
    /// What the cursor is really pointing at, for the placement tools
    /// (`snap.rs`). Rebuilt every frame from the hovered pick.
    snap_candidate: Option<SnapCandidate>,
    /// The last circle fit, so a resting cursor fits once and not per frame.
    snap_cache: SnapCache,
    /// A link's world pose while a gizmo drag previews it: `sync_scene`
    /// puts the link and its subtree there instead of at the FK pose, and
    /// the document is untouched until the release commits.
    preview_world: Option<(LinkId, Pose)>,
    /// What the viewport reported selected last frame, to notice a click
    /// resolving without mistaking a programmatic selection for one.
    last_viewport_selected: Option<PickHit>,
    /// `MeshAsset::scale` for a dropped mesh. Millimetres by default: that
    /// is what most STL exporters write.
    import_scale: f64,
    /// Transient state of the tree panel (an inline rename in progress).
    pub(crate) tree: TreeState,
    /// Transient state of the properties panel (fields being typed into).
    pub(crate) props: PropertiesState,
    /// The joint sliders window: open or not.
    pub(crate) joints_window: JointsWindow,
    /// The materials table window and its in-progress edits.
    pub(crate) materials_window: MaterialsWindow,
    /// New / Open / Quit waiting on the unsaved-changes answer.
    pub(crate) pending: Option<PendingAction>,
    /// Set once Quit (or the OS close button) passed the dirty check.
    pub(crate) quit_confirmed: bool,
    /// The title last pushed to the OS window.
    pub(crate) last_title: Option<String>,
    pub(crate) viewport: Viewport,
    /// The next [`InstanceId`] to hand out. Never reused within a session.
    next_instance: u32,
    /// A one-off message for the status bar — a load error, an export
    /// destination. `None` reads as "idle".
    pub(crate) status: Option<String>,
    /// Whether the status bar shows the frame-time readout. The snapshot
    /// suite turns it off: it reads the wall clock, so it differs on every
    /// frame.
    pub(crate) show_frame_hud: bool,
    last_frame_instant: Option<Instant>,
    /// Seconds between the last two frames, for the frame-time readout.
    last_frame_dt: Option<f32>,
}

impl RiggenApp {
    /// The import scale a fresh app starts with: millimetres.
    pub const DEFAULT_IMPORT_SCALE: f64 = 0.001;

    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let render_state = cc
            .wgpu_render_state
            .as_ref()
            .expect("riggen-app requires eframe's wgpu backend");
        let viewport = Viewport::new(&render_state.device, render_state.target_format);
        let import_scale = cc
            .storage
            .and_then(|s| s.get_string(IMPORT_SCALE_KEY))
            .and_then(|s| s.parse::<f64>().ok())
            .filter(|s| {
                IMPORT_UNITS
                    .iter()
                    .any(|(_, known)| (known - s).abs() < 1e-12)
            })
            .unwrap_or(Self::DEFAULT_IMPORT_SCALE);

        Self {
            robot: Robot::new("robot"),
            history: History::new(),
            file: None,
            mesh_store: HashMap::new(),
            instances: BTreeMap::new(),
            q: JointState::default(),
            selection: Selection::None,
            tool: Tool::default(),
            gizmo_state: GizmoState::default(),
            hovered_joint: None,
            glyph_hover: None,
            toolbar_rect: None,
            snap_candidate: None,
            snap_cache: SnapCache::default(),
            preview_world: None,
            last_viewport_selected: None,
            import_scale,
            tree: TreeState::default(),
            props: PropertiesState::default(),
            joints_window: JointsWindow::default(),
            materials_window: MaterialsWindow::default(),
            pending: None,
            quit_confirmed: false,
            last_title: None,
            viewport,
            next_instance: 0,
            status: None,
            show_frame_hud: true,
            last_frame_instant: None,
            last_frame_dt: None,
        }
    }

    fn tick_frame_clock(&mut self) {
        let now = Instant::now();
        if let Some(last) = self.last_frame_instant.replace(now) {
            let dt = now.duration_since(last).as_secs_f32();
            self.last_frame_dt = (dt > 0.0).then_some(dt);
        }
    }

    fn menu_bar(&mut self, ui: &mut egui::Ui) {
        egui::Panel::top("menu_bar").show(ui, |ui| {
            egui::MenuBar::new().ui(ui, |ui| {
                ui.menu_button("File", |ui| self.file_menu(ui));
                ui.menu_button("Edit", |ui| self.edit_menu(ui));
                ui.menu_button("Window", |ui| {
                    ui.checkbox(&mut self.joints_window.open, "Joints");
                    ui.checkbox(&mut self.materials_window.open, "Materials");
                });
                ui.menu_button("Debug", |ui| self.debug_menu(ui));
            });
        });
    }

    /// Undo / Redo / Delete, greyed out when there is nothing to do.
    fn edit_menu(&mut self, ui: &mut egui::Ui) {
        if ui
            .add_enabled(self.history.can_undo(), egui::Button::new("Undo"))
            .clicked()
        {
            ui.close();
            self.undo();
        }
        if ui
            .add_enabled(self.history.can_redo(), egui::Button::new("Redo"))
            .clicked()
        {
            ui.close();
            self.redo();
        }
        ui.separator();
        if ui
            .add_enabled(
                self.selection != Selection::None,
                egui::Button::new("Delete"),
            )
            .clicked()
        {
            ui.close();
            self.remove_selected();
        }
    }

    /// Resolves what the pointer is on: a tree row hovered while this frame
    /// was drawn wins, otherwise a glyph under the cursor in the viewport.
    ///
    /// The toolbar and the gizmo are cut out first — both float *over* the
    /// viewport, and a glyph behind them is not something the user is
    /// pointing at.
    fn update_glyph_hover(&mut self, ctx: &egui::Context, glyphs: &[JointGlyph]) {
        let from_tree = self.tree.hovered_joint.take();
        self.glyph_hover = None;
        // A placement tool never lets a glyph take the pointer: the whole
        // gesture is "point at that feature", and the selected joint's own
        // glyph sits exactly where the user is aiming.
        if from_tree.is_none()
            && !self.tool.snaps()
            && self.pending.is_none()
            && !self.gizmo_state.captured
            && let Some(pos) = ctx.pointer_hover_pos()
            && self
                .viewport
                .viewport_rect()
                .is_some_and(|r| r.contains(pos))
            && !self.toolbar_rect.is_some_and(|r| r.contains(pos))
        {
            self.glyph_hover = self.glyph_at(glyphs, pos);
        }
        self.hovered_joint = from_tree.or(self.glyph_hover);
    }

    /// `arm (i1/t120)`: the link and the instance/triangle the ID buffer
    /// resolved.
    fn describe_hit(&self, hit: PickHit) -> String {
        let where_ = format!("i{}/t{}", hit.instance.0, hit.triangle);
        match self.link_name_of_instance(hit.instance) {
            Some(name) => format!("{name} ({where_})"),
            None => where_,
        }
    }
}

impl eframe::App for RiggenApp {
    fn save(&mut self, storage: &mut dyn eframe::Storage) {
        storage.set_string(IMPORT_SCALE_KEY, self.import_scale.to_string());
    }

    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.tick_frame_clock();
        self.handle_close_request(ui.ctx());
        self.handle_file_drops(ui.ctx());
        self.handle_shortcuts(ui.ctx());
        self.update_title(ui.ctx());

        self.menu_bar(ui);

        // A hovered glyph names its joint; otherwise the ID buffer's hit.
        // Both are last frame's — this panel is drawn before the viewport.
        let hovered = match self.hovered_joint.and_then(|j| self.robot.joints.get(&j)) {
            Some(joint) => Some(format!("{} (joint)", joint.name)),
            None => self.viewport.hovered().map(|h| self.describe_hit(h)),
        };
        let selected = self.viewport.selected().map(|h| self.describe_hit(h));
        let document = format!(
            "{}{}",
            self.document_label(),
            if self.history.is_dirty() { "*" } else { "" }
        );
        status_bar::status_bar(
            ui,
            &status_bar::StatusView {
                document: &document,
                import_units: &status_bar::import_units_label(self.import_scale),
                hovered: hovered.as_deref(),
                selected: selected.as_deref(),
                instance_count: self.viewport.instance_count(),
                message: self.status.as_deref(),
                frame_dt: self.show_frame_hud.then_some(self.last_frame_dt).flatten(),
            },
        );

        self.tree_panel(ui);
        self.properties_panel(ui);

        egui::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show(ui, |ui| {
                // Glyphs are derived from the document and the current `q`,
                // so they are rebuilt every frame and handed to the viewport
                // before it paints (docs/01-architecture.md §Frame loop).
                // The hover has to be resolved first: it decides both what is
                // drawn hot and whether the viewport gets the pointer at all.
                let glyphs = self.joint_glyphs();
                self.update_glyph_hover(ui.ctx(), &glyphs);
                self.update_snap(ui.ctx());
                let mut overlay = self.glyph_overlay(&glyphs, self.active_joint());
                self.push_snap_overlay(&mut overlay);
                self.viewport.set_overlay(overlay);
                // One frame behind for the gizmo, which cannot say whether it
                // owns the cursor until it has run, and the viewport runs
                // first.
                self.viewport
                    .set_input_suppressed(self.gizmo_state.captured || self.glyph_hover.is_some());
                // A placement click means "put it here", not "select what is
                // under the cursor" — but the hover pick has to keep running,
                // because it is what the snap is computed from.
                self.viewport.set_select_suppressed(self.tool.snaps());

                let rect = self.viewport.ui(ui).rect;
                // After the viewport, in registration order: egui's hit
                // test prefers the widget registered last, so the gizmo
                // takes the pointer from the viewport and the toolbar from
                // the gizmo.
                self.gizmo_ui(ui, rect);
                self.tool_bar(ui, rect);
                // The viewport's pick is suppressed while a glyph is
                // hovered, so these clicks are unambiguous, and a hovered
                // glyph and a snap are mutually exclusive.
                let clicked = ui.input(|i| i.pointer.primary_clicked());
                if let Some(joint) = self.glyph_hover.filter(|_| clicked) {
                    self.select(Selection::Joint(joint));
                } else if clicked
                    && self.tool == Tool::PlaceJoint
                    && let Selection::Joint(joint) = self.selection
                    && let Some(snap) = self.snap_candidate
                {
                    self.place_joint(joint, &snap);
                }
            });
        self.sync_selection_from_viewport();
        // Windows float over everything, so they go last.
        self.joints_window(ui.ctx());
        self.materials_window(ui.ctx());
        self.unsaved_changes_modal(ui.ctx());
        if self.quit_confirmed {
            ui.ctx().send_viewport_cmd(egui::ViewportCommand::Close);
        }
    }
}
