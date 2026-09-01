//! The eframe shell: the bin `riggen` on native, a `WebHandle` cdylib on
//! wasm32 (docs/01-architecture.md §Crates).

mod app;
#[cfg(not(target_arch = "wasm32"))]
pub mod cli;
pub mod debug;
pub mod jobs;

pub use app::{
    ALIGN_PROMPT, ALIGN_WRONG_LINK, COPIED_STATUS, ExportDialog, GLYPH_HOVER_RADIUS, GizmoTarget,
    JointGlyph, PendingAction, RiggenApp, SNAP_PIXEL_RADIUS, Selection, SnapCandidate, SnapKind,
    Tool, ZERO_CONFIG_STATUS, align_transform, aligned_status, placed_status,
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
                    Box::new(|cc| Ok(Box::new(crate::RiggenApp::new(cc)))),
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
