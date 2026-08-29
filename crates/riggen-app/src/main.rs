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
    // `riggen robot.riggen` opens a document; `riggen a.stl b.obj` drops
    // meshes as links under the root (docs/03-roadmap.md §M1).
    let files: Vec<std::path::PathBuf> = std::env::args_os().skip(1).map(Into::into).collect();
    eframe::run_native(
        "riggen",
        options,
        Box::new(move |cc| {
            let mut app = riggen_app::RiggenApp::new(cc);
            app.load_files(&files);
            Ok(Box::new(app))
        }),
    )
}

// wasm32 ships as a cdylib loaded by a host page (see `lib.rs`); the bin
// target still needs to compile for the workspace-wide wasm build check.
#[cfg(target_arch = "wasm32")]
fn main() {}
