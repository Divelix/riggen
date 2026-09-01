//! The eframe shell: the bin `riggen` on native, a `WebHandle` cdylib on
//! wasm32 (docs/01-architecture.md §Crates).

mod app;
#[cfg(not(target_arch = "wasm32"))]
pub mod cli;
pub mod debug;
// Downloads are the browser's only way out (ADR-0017 §6), so the module is
// built for wasm — and for `cargo test`, because the test that reads the
// archive back with a zip reader is native.
#[cfg(any(target_arch = "wasm32", test))]
mod download;
pub mod example;
pub mod jobs;

pub use app::{
    ALIGN_PROMPT, ALIGN_WRONG_LINK, COPIED_STATUS, DECOMP_CONSENT_BUTTON, DECOMP_FREEZE_WARNING,
    DroppedSet, ExportDialog, Files, GLYPH_HOVER_RADIUS, GizmoTarget, JointGlyph, PendingAction,
    RiggenApp, SNAP_PIXEL_RADIUS, Selection, SnapCandidate, SnapKind, Tool, ZERO_CONFIG_STATUS,
    align_transform, aligned_status, placed_status,
};

#[cfg(target_arch = "wasm32")]
mod web {
    use wasm_bindgen::prelude::*;

    /// JS-side handle: `new WebHandle()` then `.start(canvas)` from
    /// `web/main.js`, which owns the page, the WebGPU probe and the panic
    /// sheet (docs/01-architecture.md §Cargo workspace, `web/`).
    #[wasm_bindgen]
    pub struct WebHandle {
        runner: eframe::WebRunner,
    }

    #[wasm_bindgen]
    impl WebHandle {
        #[allow(clippy::new_without_default)]
        #[wasm_bindgen(constructor)]
        pub fn new() -> Self {
            console_error_panic_hook::set_once();
            Self {
                runner: eframe::WebRunner::new(),
            }
        }

        #[wasm_bindgen]
        pub async fn start(
            &self,
            canvas: web_sys::HtmlCanvasElement,
        ) -> Result<(), wasm_bindgen::JsValue> {
            self.runner
                .start(
                    canvas,
                    eframe::WebOptions::default(),
                    Box::new(|cc| {
                        // The demo opens on the sample arm, out of the same
                        // `include_bytes!` the desktop's `--example arm`
                        // unpacks — as if it had been dropped on the page
                        // (ADR-0017).
                        let mut app = crate::RiggenApp::new(cc);
                        app.load_dropped(crate::example::Example::Arm.dropped());
                        Ok(Box::new(app))
                    }),
                )
                .await
        }

        /// A panic poisons the runner and the canvas quietly stops
        /// repainting, which reads as a hang. The page polls this so it can
        /// say what happened instead.
        #[wasm_bindgen]
        pub fn has_panicked(&self) -> bool {
            self.runner.has_panicked()
        }

        #[wasm_bindgen]
        pub fn panic_message(&self) -> Option<String> {
            self.runner.panic_summary().map(|s| s.message())
        }

        #[wasm_bindgen]
        pub fn panic_callstack(&self) -> Option<String> {
            self.runner.panic_summary().map(|s| s.callstack())
        }
    }
}
