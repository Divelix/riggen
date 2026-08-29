//! The application state and its per-frame `ui`. An empty window until
//! M0 step 2 adds the status bar and step 8 the viewport.

/// The eframe app. Will own the `Robot` document plus derived, never-saved
/// state (docs/01-architecture.md §The document is the only state).
pub struct RiggenApp {}

impl RiggenApp {
    pub fn new(_cc: &eframe::CreationContext<'_>) -> Self {
        Self {}
    }
}

impl eframe::App for RiggenApp {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        egui::CentralPanel::default().show(ui, |_ui| {});
    }
}
