//! The application state and its per-frame `ui`. Status bar plus an empty
//! `CentralPanel` until M0 step 8 puts the viewport in it.

mod status_bar;

use web_time::Instant;

/// The eframe app. Will own the `Robot` document plus derived, never-saved
/// state (docs/01-architecture.md §The document is the only state).
pub struct RiggenApp {
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
    /// The rect the central panel was given this frame, in egui logical
    /// points. Step 8 replaces it with the viewport's own rect; until then it
    /// is what `debug_state()` and the harness aim at.
    pub(crate) central_rect: Option<egui::Rect>,
}

impl RiggenApp {
    pub fn new(cc: &eframe::CreationContext<'_>) -> Self {
        // The viewport (step 8) is built from this device; requiring it now
        // is what makes the snapshot harness prove that `build_eframe`
        // supplies a real `RenderState` before any port code exists.
        cc.wgpu_render_state
            .as_ref()
            .expect("riggen-app requires eframe's wgpu backend");

        Self {
            status: None,
            show_frame_hud: true,
            last_frame_instant: None,
            last_frame_dt: None,
            central_rect: None,
        }
    }

    fn tick_frame_clock(&mut self) {
        let now = Instant::now();
        if let Some(last) = self.last_frame_instant.replace(now) {
            let dt = now.duration_since(last).as_secs_f32();
            self.last_frame_dt = (dt > 0.0).then_some(dt);
        }
    }
}

impl eframe::App for RiggenApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        self.tick_frame_clock();

        status_bar::status_bar(
            ui,
            &status_bar::StatusView {
                hovered: None,
                selected: None,
                message: self.status.as_deref(),
                frame_dt: self.show_frame_hud.then_some(self.last_frame_dt).flatten(),
            },
        );

        let response = egui::CentralPanel::default().show(ui, |_ui| {});
        self.central_rect = Some(response.response.rect);
    }
}
