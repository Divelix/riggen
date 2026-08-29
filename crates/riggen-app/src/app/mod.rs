//! The application state and its per-frame `ui`: menu bar on top, status
//! bar on the bottom, the viewport in the central panel.

mod file_io;
mod status_bar;

use riggen_viewport::{InstanceId, Viewport};
use web_time::Instant;

/// The eframe app. Will own the `Robot` document plus derived, never-saved
/// state (docs/01-architecture.md §The document is the only state); in M0
/// the viewport's instance table *is* the state.
pub struct RiggenApp {
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
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        let render_state = cc
            .wgpu_render_state
            .as_ref()
            .expect("riggen-app requires eframe's wgpu backend");
        let viewport = Viewport::new(&render_state.device, render_state.target_format);

        Self {
            viewport,
            next_instance: 0,
            status: None,
            show_frame_hud: true,
            last_frame_instant: None,
            last_frame_dt: None,
        }
    }

    /// Moves an instance to `position` (file units). M0 has no document to
    /// keep poses in; this exists so a scenario can lay several parts side
    /// by side.
    pub fn place_instance(&mut self, id: InstanceId, position: riggen_mesh::glam::DVec3) -> bool {
        self.viewport
            .set_instance_model(id, riggen_mesh::glam::DMat4::from_translation(position))
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
}

impl eframe::App for RiggenApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.tick_frame_clock();
        self.handle_file_drops(ui.ctx());

        self.menu_bar(ui);

        // `i3/t120`: instance 3, triangle 120 — what the ID buffer resolved.
        let describe =
            |hit: riggen_viewport::PickHit| format!("i{}/t{}", hit.instance.0, hit.triangle);
        let hovered = self.viewport.hovered().map(describe);
        let selected = self.viewport.selected().map(describe);
        status_bar::status_bar(
            ui,
            &status_bar::StatusView {
                hovered: hovered.as_deref(),
                selected: selected.as_deref(),
                instance_count: self.viewport.instance_count(),
                message: self.status.as_deref(),
                frame_dt: self.show_frame_hud.then_some(self.last_frame_dt).flatten(),
            },
        );

        egui::CentralPanel::default()
            .frame(egui::Frame::NONE)
            .show(ui, |ui| {
                self.viewport.ui(ui);
            });
    }
}
