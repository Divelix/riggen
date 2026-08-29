#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result<()> {
    // A 3D viewport is an input-latency-first workload, so the surface is
    // configured for the shortest path from cursor to pixels. eframe's
    // default is `HIGH_THROUGHPUT`: vsync plus two queued frames, ~33 ms
    // between the input a frame was drawn from and the moment it lands on
    // screen, which reads as the geometry dragging along behind the cursor.
    // One queued frame, presented without waiting for vsync, trades tearing
    // during a fast orbit for input that tracks the mouse.
    let options = eframe::NativeOptions {
        wgpu_options: egui_wgpu::WgpuConfiguration {
            surface: egui_wgpu::SurfaceConfig {
                present_mode: egui_wgpu::wgpu::PresentMode::AutoNoVsync,
                ..egui_wgpu::SurfaceConfig::LOW_LATENCY
            },
            ..Default::default()
        },
        ..Default::default()
    };
    eframe::run_native(
        "riggen",
        options,
        Box::new(|cc| Ok(Box::new(riggen_app::RiggenApp::new(cc)))),
    )
}

// wasm32 ships as a cdylib loaded by a host page (see `lib.rs`); the bin
// target still needs to compile for the workspace-wide wasm build check.
#[cfg(target_arch = "wasm32")]
fn main() {}
