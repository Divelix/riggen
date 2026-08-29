#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result<()> {
    // The startup budget's clock (docs/03-roadmap.md §M4): everything from
    // here to the first painted frame — window, wgpu device, `new`, the
    // first `ui` pass — is what `--timing` reports.
    let started = std::time::Instant::now();
    let args: Vec<std::ffi::OsString> = std::env::args_os().skip(1).collect();
    // `--help`, `--version` and `riggen --export … INPUT` are headless and
    // return before eframe starts (ADR-0008): CI's mujoco job has no display.
    let open = match riggen_app::cli::parse(&args) {
        Ok(riggen_app::cli::Invocation::Help) => {
            print!("{}", riggen_app::cli::help());
            return Ok(());
        }
        Ok(riggen_app::cli::Invocation::Version) => {
            println!("{}", riggen_app::cli::version());
            return Ok(());
        }
        Ok(riggen_app::cli::Invocation::Export(export)) => match riggen_app::cli::run(&export) {
            Ok(written) => {
                for path in written {
                    println!("{}", path.display());
                }
                return Ok(());
            }
            Err(message) => {
                eprintln!("{message}");
                std::process::exit(1);
            }
        },
        Ok(riggen_app::cli::Invocation::Open(open)) => open,
        Err(message) => {
            eprintln!("{message}");
            std::process::exit(2);
        }
    };
    // `riggen robot.riggen` opens a document; `riggen a.stl b.obj` drops
    // meshes as links under the root (docs/03-roadmap.md §M1); `--example
    // arm` unpacks the bundled sample to a temp directory and opens it first.
    let mut files = Vec::new();
    if let Some(example) = open.example {
        match example.extract() {
            Ok(document) => files.push(document),
            Err(message) => {
                eprintln!("cannot unpack the example: {message}");
                std::process::exit(1);
            }
        }
    }
    files.extend(open.files);
    // A 3D viewport is an input-latency-first workload, so the surface is
    // configured for the shortest path from cursor to pixels. eframe's
    // default is `HIGH_THROUGHPUT`: vsync plus two queued frames, ~33 ms
    // between the input a frame was drawn from and the moment it lands on
    // screen, which reads as the geometry dragging along behind the cursor.
    // One queued frame, presented without waiting for vsync, trades tearing
    // during a fast orbit for input that tracks the mouse.
    //
    // The backends are the platform's native ones only (Vulkan, Metal,
    // DX12): eframe's default adds GL, and enumerating it loads the GLX/EGL
    // stack and the vendor's GL driver for an adapter the app never picks —
    // 100–150 ms of the startup budget (docs/03-roadmap.md §M4) on the dev
    // machine. `WGPU_BACKEND=gl` is the escape hatch for a machine with no
    // Vulkan driver at all.
    let mut wgpu_setup = egui_wgpu::WgpuSetupCreateNew::without_display_handle();
    wgpu_setup.instance_descriptor.backends =
        egui_wgpu::wgpu::Backends::from_env().unwrap_or(egui_wgpu::wgpu::Backends::PRIMARY);
    let options = eframe::NativeOptions {
        wgpu_options: egui_wgpu::WgpuConfiguration {
            surface: egui_wgpu::SurfaceConfig {
                present_mode: egui_wgpu::wgpu::PresentMode::AutoNoVsync,
                ..egui_wgpu::SurfaceConfig::LOW_LATENCY
            },
            wgpu_setup: wgpu_setup.into(),
            ..Default::default()
        },
        ..Default::default()
    };
    eframe::run_native(
        "riggen",
        options,
        Box::new(move |cc| {
            let mut app = riggen_app::RiggenApp::with_start(cc, started);
            app.set_print_timing(open.timing);
            app.load_files(&files);
            Ok(Box::new(app))
        }),
    )
}

// wasm32 ships as a cdylib loaded by a host page (see `lib.rs`); the bin
// target still needs to compile for the workspace-wide wasm build check.
#[cfg(target_arch = "wasm32")]
fn main() {}
