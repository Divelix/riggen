//! The application state and its per-frame `ui`: menu bar on top, status
//! bar on the bottom, the viewport in the central panel.

mod document;
mod file_io;
mod panels;
mod shortcuts;
mod status_bar;

use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;

use riggen_core::{GeomId, History, JointState, LinkId, MeshId, Robot};
use riggen_viewport::{InstanceId, PickHit, Viewport};
use web_time::Instant;

pub(crate) use document::LoadedMesh;
pub use document::Selection;
use panels::TreeState;

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
    /// What the viewport reported selected last frame, to notice a click
    /// resolving without mistaking a programmatic selection for one.
    last_viewport_selected: Option<PickHit>,
    /// `MeshAsset::scale` for a dropped mesh. Millimetres by default: that
    /// is what most STL exporters write.
    import_scale: f64,
    /// Transient state of the tree panel (an inline rename in progress).
    pub(crate) tree: TreeState,
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

        Self {
            robot: Robot::new("robot"),
            history: History::new(),
            file: None,
            mesh_store: HashMap::new(),
            instances: BTreeMap::new(),
            q: JointState::default(),
            selection: Selection::None,
            last_viewport_selected: None,
            import_scale: Self::DEFAULT_IMPORT_SCALE,
            tree: TreeState::default(),
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
                ui.menu_button("File", |ui| {
                    if ui.button("Open…").clicked() {
                        ui.close();
                        self.open_dialog();
                    }
                    ui.separator();
                    if ui.button("Quit").clicked() {
                        ui.send_viewport_cmd(egui::ViewportCommand::Close);
                    }
                });
            });
        });
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
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.tick_frame_clock();
        self.handle_file_drops(ui.ctx());
        self.handle_shortcuts(ui.ctx());

        self.menu_bar(ui);

        let hovered = self.viewport.hovered().map(|h| self.describe_hit(h));
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

        egui::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show(ui, |ui| {
                self.viewport.ui(ui);
            });
        self.sync_selection_from_viewport();
    }
}
