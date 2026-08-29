//! Shared scaffolding for the visual snapshot scenarios (ADR-0003).

use egui_kittest::Harness;
use riggen_app::RiggenApp;

/// The window the scenarios render into. Fixed, because the goldens encode
/// it: changing this rewrites every snapshot.
const SIZE: egui::Vec2 = egui::vec2(1440.0, 900.0);

/// Whether this machine can render at all.
///
/// `egui_kittest`'s `create_render_state` panics with "Failed to create render
/// state" when no adapter exists, which reads as a broken test rather than a
/// missing driver. Probing first lets a GPU-less machine skip loudly instead.
/// CI installs `mesa-vulkan-drivers` so it never takes that path — a scenario
/// that silently skips everywhere is worse than no scenario, because it looks
/// like coverage.
fn adapter_available() -> bool {
    let instance = egui_wgpu::wgpu::Instance::new(
        egui_wgpu::wgpu::InstanceDescriptor::new_without_display_handle(),
    );
    let adapters =
        pollster::block_on(instance.enumerate_adapters(egui_wgpu::wgpu::Backends::all()));
    !adapters.is_empty()
}

/// Serialises the scenarios.
///
/// Cargo runs the tests in a binary on parallel threads, and several
/// concurrent lavapipe devices — each with its own offscreen colour, depth and
/// pick targets at 1440x900 — segfault inside the driver. It reproduces under
/// `cargo test --workspace`, where other test binaries add to the load, and
/// not reliably on its own, which is the worst shape a flake can have. One
/// scenario at a time costs a fraction of a second in total and removes it.
fn gpu_lock() -> std::sync::MutexGuard<'static, ()> {
    static GPU: std::sync::Mutex<()> = std::sync::Mutex::new(());
    // A panicking scenario poisons the lock; the ones after it should report
    // their own result, not "poisoned mutex".
    GPU.lock().unwrap_or_else(|poisoned| poisoned.into_inner())
}

/// Runs one scenario end to end: skip if there is no GPU, take the device
/// lock, build the app, hand it to `body`, then capture both goldens.
///
/// Both captures live here so no scenario can accidentally take only one —
/// the picture and the state dump are a pair, and half of one is what makes a
/// snapshot suite untrustworthy (ADR-0003).
pub fn scenario(name: &str, body: impl FnOnce(&mut Harness<'_, RiggenApp>)) {
    if !adapter_available() {
        eprintln!(
            "SKIPPING visual scenario `{name}`: no wgpu adapter on this machine. \
             Install `mesa-vulkan-drivers` (lavapipe) to run the snapshot suite."
        );
        return;
    }

    let _gpu = gpu_lock();
    let mut harness = app_harness();
    body(&mut harness);
    harness.snapshot(name);
    assert_state(&harness, name);
}

/// Builds the real `RiggenApp` over `egui_kittest`'s wgpu renderer and pumps
/// it until it is settled.
///
/// `build_eframe` supplies a genuine `RenderState` on the `CreationContext`,
/// which is what `RiggenApp::new` hard-requires — so the viewport pipelines
/// construct exactly as they do under `run_native`.
fn app_harness() -> Harness<'static, RiggenApp> {
    let mut harness = Harness::builder()
        .with_size(SIZE)
        .wgpu()
        .build_eframe(|cc| RiggenApp::new(cc));
    harness.state_mut().set_frame_hud_visible(false);
    settle(&mut harness);
    harness
}

/// Pumps frames until the app reports itself settled for a few in a row.
///
/// This does **no** GPU work: `Harness::step` only runs egui's logic pass, and
/// the wgpu paint callback executes in `render`. Anything that depends on the
/// GPU having run — the ID-buffer pick above all — needs [`pump_rendered`]
/// instead.
pub fn settle(harness: &mut Harness<'_, RiggenApp>) {
    const MAX_FRAMES: usize = 600;
    const IDLE_FRAMES: usize = 4;

    let mut idle = 0;
    for _ in 0..MAX_FRAMES {
        harness.step();
        if harness.state().settled() {
            idle += 1;
            if idle >= IDLE_FRAMES {
                return;
            }
        } else {
            idle = 0;
        }
    }
    panic!("app never settled within {MAX_FRAMES} frames");
}

/// Steps *and renders* `frames` frames.
///
/// The viewport's hover/selection pick is an ID-buffer pass followed by an
/// async readback, so it needs several real GPU frames before the restyle
/// appears: one to submit the pass, one or more for the readback to resolve,
/// and one to draw the result. `settle` cannot substitute — it never reaches
/// the paint callback at all.
#[allow(dead_code, reason = "used from step 9's picking scenarios on")]
pub fn pump_rendered(harness: &mut Harness<'_, RiggenApp>, frames: usize) {
    for _ in 0..frames {
        harness.step();
        harness.render().expect("render");
    }
}

/// Hovers, presses and releases at `pos`, rendering between each step.
///
/// `Harness::drag_at`/`drop_at` cannot be used for this. `step` drains *every*
/// queued event, running one logic pass each and keeping only the last one's
/// output, so a press and release queued together produce a click frame that
/// is never rendered — and the viewport only schedules a select pick on the
/// frame `response.clicked()` is true. Rendering between each event is what
/// puts the click frame on the GPU.
///
/// The pauses also matter: the viewport allows one pick in flight at a time,
/// so the hover pick has to resolve before the click's is even scheduled.
#[allow(dead_code, reason = "used from step 9's picking scenarios on")]
pub fn click_at(harness: &mut Harness<'_, RiggenApp>, pos: egui::Pos2) {
    fn button(pos: egui::Pos2, pressed: bool) -> egui::Event {
        egui::Event::PointerButton {
            pos,
            button: egui::PointerButton::Primary,
            pressed,
            modifiers: egui::Modifiers::NONE,
        }
    }

    harness.hover_at(pos);
    pump_rendered(harness, 4);
    harness.event(button(pos, true));
    pump_rendered(harness, 2);
    harness.event(button(pos, false));
    pump_rendered(harness, 8);
    // Leaves the frame showing selection alone rather than selection under a
    // hover highlight of the same instance.
    harness.event(egui::Event::PointerGone);
    pump_rendered(harness, 4);
}

/// Runs a throwaway scenario and writes its capture to `target/`, comparing
/// against nothing.
///
/// This is the "show me what the app looks like *right now*" path, as opposed
/// to the regression path [`scenario`] serves. An agent iterating on a UI
/// change needs to see an arbitrary state, and doing that through `scenario`
/// would mint a permanent golden for a state nobody wants pinned. The outputs
/// land under `target/`, which is already gitignored, and the paths are
/// printed so they can be opened straight from the test output.
#[allow(
    dead_code,
    reason = "used by the `visual_scratch` target, which includes this file by path"
)]
pub fn scratch(body: impl FnOnce(&mut Harness<'_, RiggenApp>)) {
    if !adapter_available() {
        eprintln!(
            "SKIPPING scratch capture: no wgpu adapter on this machine. \
             Install `mesa-vulkan-drivers` (lavapipe)."
        );
        return;
    }

    let _gpu = gpu_lock();
    let mut harness = app_harness();
    body(&mut harness);

    let dir = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../target/visual-scratch");
    std::fs::create_dir_all(&dir).expect("create scratch dir");
    // Canonicalised only so the printed paths are clickable rather than
    // `…/riggen-app/../../target/…`.
    let dir = dir.canonicalize().unwrap_or(dir);
    let png = dir.join("scratch.png");
    let json = dir.join("scratch.json");

    let image = harness.render().expect("render");
    image.save(&png).expect("write scratch png");
    std::fs::write(&json, format!("{}\n", harness.state().debug_state_json()))
        .expect("write scratch json");

    // Printed, not asserted: this scenario has no golden and cannot fail.
    // `--nocapture` is what makes these visible.
    println!(
        "scratch capture:\n  {}\n  {}",
        png.display(),
        json.display()
    );
}

/// Directory the goldens live in — the same one `kittest.toml` points the
/// image snapshots at, so a scenario's PNG and JSON sit side by side.
fn snapshot_dir() -> std::path::PathBuf {
    std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/snapshots")
}

/// Whether to rewrite goldens instead of comparing against them. Reads the
/// same `UPDATE_SNAPSHOTS` variable `egui_kittest` does, so one env var
/// updates both halves of a scenario.
fn update_goldens() -> bool {
    match std::env::var("UPDATE_SNAPSHOTS") {
        Err(_) => false,
        Ok(value) => !matches!(value.as_str(), "" | "0" | "false"),
    }
}

/// Compares `debug_state()` against the committed `<name>.json`.
///
/// The state dump is the half of a scenario that says *why* the pixels are
/// what they are, so it is a golden in its own right rather than a debugging
/// aid written to a scratch directory: a camera that drifted or an instance
/// that vanished shows up here as a readable line-level diff, where in the
/// PNG it is a cloud of changed pixels.
fn assert_state(harness: &Harness<'_, RiggenApp>, name: &str) {
    // Trailing newline so the goldens are ordinary text files: without it
    // every diff of one ends in a "\ No newline at end of file" marker.
    let actual = format!("{}\n", harness.state().debug_state_json());
    let path = snapshot_dir().join(format!("{name}.json"));

    if update_goldens() {
        std::fs::write(&path, &actual).expect("write state golden");
        return;
    }

    let expected = std::fs::read_to_string(&path).unwrap_or_else(|err| {
        std::fs::write(path.with_extension("json.new"), &actual).ok();
        panic!(
            "missing state golden {}: {err}. \
             Run `UPDATE_SNAPSHOTS=1 cargo test -p riggen-app --test visual`.",
            path.display()
        )
    });

    if expected != actual {
        std::fs::write(path.with_extension("json.new"), &actual).expect("write .new");
        let diff = first_difference(&expected, &actual);
        panic!(
            "state golden {} does not match.\n{diff}\n\
             Full output written to {}. Run \
             `UPDATE_SNAPSHOTS=1 cargo test -p riggen-app --test visual` if the change is \
             intended.",
            path.display(),
            path.with_extension("json.new").display()
        );
    }
}

/// The first line that differs, which for this JSON is almost always the whole
/// story — dumping two long files into a panic message is not.
fn first_difference(expected: &str, actual: &str) -> String {
    for (i, (e, a)) in expected.lines().zip(actual.lines()).enumerate() {
        if e != a {
            return format!("  line {}:\n  - {}\n  + {}", i + 1, e.trim(), a.trim());
        }
    }
    format!(
        "  same first {} lines, then the files differ in length ({} vs {})",
        expected.lines().count().min(actual.lines().count()),
        expected.lines().count(),
        actual.lines().count()
    )
}
