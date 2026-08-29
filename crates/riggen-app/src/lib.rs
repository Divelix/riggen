//! The eframe shell: the bin `riggen` on native, a `WebHandle` cdylib on
//! wasm32 (docs/01-architecture.md §Crates).

mod app;
pub mod debug;

pub use app::{RiggenApp, Selection};

#[cfg(target_arch = "wasm32")]
mod web {
    use wasm_bindgen::prelude::*;

    /// JS-side handle: `new WebHandle()` then `.start(canvas)` from the host
    /// page. The page itself is out of scope for M0; this is just enough for
    /// the wasm build check to exercise the entry point.
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
    }
}
