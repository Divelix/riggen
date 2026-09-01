# Plan: web-demo

- Started: 2026-09-01
- Milestone: v0.2 (the last open line: "Web demo build if the wasm check has stayed green")
- Idea (verbatim from the human): "next logical step in roadmap" — answered as
  the web demo, at "bring-your-own-mesh" scope.

## Goal

riggen runs at a public URL. The page opens a WebGPU canvas with the bundled
sample arm already in it: you orbit, pick, place a joint with the gizmo and
swing the sliders exactly as the native app does. You can drop your own
`.stl` / `.obj` / `.riggen` / `.urdf` / `.xml` onto the page and they open;
Save, Save As and Export hand the browser a download instead of writing to a
filesystem the browser does not have. Underneath, `riggen-mesh`,
`riggen-core` and `riggen-export` grow a byte-oriented half beside their
path-oriented one — `load_mesh_bytes`, a `FileSource` read trait,
`export_files` — so the browser and the native app run the *same* readers and
writers rather than a second, thinner web version of them. CI builds the
wasm-bindgen bundle on every push and GitHub Pages serves it from `main`.

## Non-goals

- **A WebGL2 fallback.** The pick pass is an `R32Uint` target read back with
  `copy_texture_to_buffer`, which wgpu's GL backend will not do. WebGPU or a
  plain-English "this browser has no WebGPU" page. (OPEN 3.)
- **A web worker for `jobs`.** Convex decomposition stays inline on wasm, as
  01 §Jobs and threads already says; step 6 only makes the freeze consented
  to rather than surprising. The worker stays the backlog line it is.
- **A directory drop.** The browser hands us a flat set of files per gesture;
  that set is the resolution scope (ADR-0017), not a tree.
- **Documents surviving a reload.** eframe's `persistence` keeps the UI
  layout and the import-units choice, as on native; the document does not.
- **The SDK on web.** No pyodide, no `import riggen` in a browser.
- **A mobile or touch UI.** The demo is a desktop browser.
- **Any change to `.riggen`.** Schema stays 3; no field is added.

## Design deltas

- **`riggen-mesh`** — `parse_stl` / `parse_obj` (already the real parsers,
  `pub(crate)` today) become `pub`, and `load_mesh_bytes(name: &Path, bytes:
  &[u8])` joins `load_mesh` as the extension dispatcher over them. No new
  dependency; the layer map is untouched.
- **`riggen-core::file`** — a read-only `FileSource` trait (`fn read(&self,
  path: &Path) -> io::Result<Vec<u8>>`) with a `Disk` implementation. `load`
  becomes a thin `Disk` wrapper over `load_from(text, base, &impl
  FileSource)`; `save` a wrapper over `to_json(robot, base) -> String`. This
  is where 01 §File format's "relative mesh paths and a content hash" rule
  stops meaning "relative to a directory on disk".
- **`riggen-export`** — `MeshStore::load`, `urdf_in::load` and
  `mjcf_in::load` take a `&impl FileSource`; `export()` splits into the pure
  `export_files(robot, options) -> Vec<(PathBuf, Vec<u8>)>` and the atomic
  writer over it. The `.tmp`-and-rename discipline stays on the native side
  where it means something.
- **`riggen-app`** — `Example` (and its `include_bytes!` of the 45 KB arm)
  moves out of the native-only `cli` into a target-independent
  `riggen_app::example`, so the web build seeds from the same bytes.
  `open_bytes(name, bytes)` joins `open_path` behind the one extension
  dispatch; `handle_file_drops` takes `DroppedFile.bytes` when there is no
  path. The four `cfg(wasm32)` arms that say *"no filesystem in the browser"*
  (`open_dialog`, `add_mesh_dialog`, `import_dialog`, `save_as_dialog`,
  `choose_export_dir`, `save_debug_state`) become downloads or a pointer at
  the drop gesture.
- **`web/`** — a new top-level directory: `index.html`, the loader module,
  `build.sh` (cargo + a pinned `wasm-bindgen-cli` into `web/dist/`). Listed
  in 01 §Cargo workspace's tree.
- **CI** — the `wasm` job stops being `cargo build` and builds the real
  bundle; a `pages.yml` deploys it.
- **ADR-0017 — web IO: bytes in, downloads out, meshes by file name.** The
  decisions worth a record: the `FileSource` seam rather than a VFS; a
  dropped set resolved by *file name*, ignoring directories, with the
  existing warning vocabulary for what is missing; WebGPU only; downloads in
  place of a save dialog. Written in step 4, when the rule is implemented
  rather than guessed at.

## Steps

- [x] **Step 1 — The page, and the app running in a browser.** `web/index.html`,
  the loader module (WebGPU probe with a plain-English failure page, a
  full-window canvas, a panic overlay) and `web/build.sh` producing
  `web/dist/`. The `wasm` CI job builds the bundle instead of the bin.
  *Observable:* `python3 -m http.server` over `web/dist/` shows the real app
  — menus, panels, status bar, the axes triad in an empty viewport — and
  orbits. *Check:* the agent loads it with the `claude-in-chrome` skill,
  screenshots it, and reads the console clean. This is the step that retires
  the "does egui + wgpu + `transform-gizmo` come up in a browser at all"
  risk, so it goes first and carries no other change. *Done:* it comes up
  — menus, Links, Properties, the toolbar, the status bar at 60 fps, the
  triad, and it orbits. The `claude-in-chrome` extension is not installed
  here, so the Chrome pass runs through `scratch/web/drive.mjs`, a CDP
  driver over the local Chromium; it has to be **headed** (on the X display)
  because headless Chromium's GPU process fails `requestDevice` — a minimal
  clear-to-red WebGPU page fails there too, so it is the environment and not
  riggen. The canvas is read with `toDataURL`, since `Page.captureScreenshot`
  does not composite a WebGPU canvas.
- [x] **Step 2 — Bytes in: the mesh and document readers.**
  `riggen_mesh::load_mesh_bytes`; `riggen_core::file::{FileSource, Disk,
  load_from}`; `MeshStore::load`, `urdf_in::load` and `mjcf_in::load` take
  the source. Native behaviour byte-for-byte unchanged. *Test:* an in-memory
  `FileSource` built from `assets/fixtures/` opens `arm.riggen`, `arm.urdf`
  and `menagerie_style.xml` with no disk access, and each result equals the
  on-disk load. *Done, with two shape changes:* the trait is taken as `&dyn
  FileSource`, not `&impl` — `mjcf_in::Import` holds one for the whole
  conversion, and a generic parameter there buys nothing; and a
  `MemorySource` joins `Disk` in `riggen_core::file` rather than being
  duplicated as a test helper per crate — the browser's source is that
  shape anyway (step 4). The importers keep their path-shaped `load`: the
  source resolves the path, so no text-taking twin is needed.
- [x] **Step 3 — Bytes out: `export_files`.** The pure file list under
  `export()`. *Test:* `export_files` and `export` agree on names and bytes
  for the arm, all three formats and the `meshes/` folder. *Done:*
  `export_files(robot, options, dir)` keeps the `dir` argument — it settles
  what `MeshPathStyle::Absolute` writes into the model files — but reads
  nothing from it and creates nothing in it, which a second test pins
  against a directory that does not exist.
- [x] **Step 4 — Drops open on the web; the sample arm is there at startup.**
  `open_bytes` beside `open_path`; the drop handler prefers `bytes` when
  `path` is `None`; a `DroppedSet` `FileSource` resolving a mesh reference by
  file name against the same gesture's files, warning through the existing
  `file::Warning` / `ImportWarning` vocabulary for each miss. `Example` moves
  to `riggen_app::example` and `WebHandle::start` opens the arm. **ADR-0017
  lands here.** *Check:* Chrome — the arm is in the viewport on load, its
  joints swing; dropping `cube_binary.stl` adds a link; dropping the five
  files of `assets/fixtures/arm/` replaces the document with no warning;
  dropping `arm.riggen` alone reports the four missing meshes by name.
  *Done; all four checked in Chrome.* Two things the plan had not settled:
  a gesture that carries a **document** replaces the app's source, while a
  gesture of **meshes alone** adds to it (ADR-0017 §4) — without that rule
  a document dropped after an earlier drop quietly borrows the earlier
  meshes; and egui 0.36's `DroppedFile` reads asynchronously on the web
  (`bytes_async`, not a `bytes` field), so a gesture is read by one spawned
  future into an inbox the frame loop drains.
- [x] **Step 5 — Downloads out.** Save / Save As → a `.riggen` download;
  Export → the export directory as one stored (uncompressed) zip; Debug ›
  Save state → a `.json`. The "no filesystem in the browser" strings become
  what will actually happen. *Test (native):* the zip's central directory and
  entry bytes over a fixture; the wording is checked in the Chrome pass.
  *Done.* The `zip` crate is **not** gated to wasm as OPEN 1 sketched: the
  test that reads the archive back is native, so `stored_zip` is built for
  `cfg(any(target_arch = "wasm32", test))`. `riggen_core::to_json` lands
  here rather than in step 2 — it is a writer. Chrome: the export
  downloaded `arm.zip`, and unzipping it gives files byte-identical to
  `riggen --export all --out … assets/fixtures/arm/arm.riggen`, which is
  the plan's acceptance 3, met for all three formats and all four meshes.
- [x] **Step 6 — The decomposition freeze, consented to.** In the web build
  the properties panel says a V-HACD run will freeze the tab for a few
  seconds and asks once before starting it (`jobs` has no thread on wasm — 01
  §Jobs and threads). *Check:* Chrome, on `bracket.stl`. *Done:* the
  question is asked once per session, not once per link, and a document
  that wants a decomposition while the answer is outstanding is no longer
  reported as `decompositions_pending` — nothing is running. The screen is
  wasm-only, so `set_decomp_consent` lets the native snapshot suite render
  it: new golden `decomp_needs_consent` (ADR-0003).
- [x] **Step 7 — Size, and the deploy.** A wasm release profile (`opt-level`,
  `lto`), `wasm-opt` if it earns its minutes, and the gzipped `.wasm` size
  measured and written into the roadmap the way M4 recorded the wheel sizes.
  `.github/workflows/pages.yml` builds and deploys on push to `main`; the
  README gets the URL. *Observable:* the public URL serves the demo.
  *Done, with one measured refusal:* `wasm-opt` does **not** earn its
  minutes. `-O2`, `-Os` and `-Oz` each take ~1 MB off the raw `.wasm` and
  put ~0.12 MB back on the gzipped one, and gzipped is what a visitor
  downloads. The `[profile.web]` (`opt-level = "s"`, fat LTO) is the whole
  saving: 3.67 → 3.35 MB gzipped, 61 fps unchanged. Numbers are in
  docs/03-roadmap.md beside M4's wheel sizes. `pages.yml` needs one
  by-hand step before the URL is live — **GitHub › Settings › Pages ›
  Source: GitHub Actions** — and the first deploy is the human's push.

## Acceptance

The milestone's own check, run against the deployed page:

1. `web/build.sh` produces `web/dist/`, and the `wasm` CI job builds the same
   bundle on every push.
2. In Chrome (agent-driven, `claude-in-chrome`): the URL loads with a clean
   console, the sample arm is in the viewport, a joint slider swings it, and
   the gizmo places a joint.
3. Drop the five files of `assets/fixtures/arm/`, then Export → MJCF. The
   downloaded zip's `arm.xml` is **byte-identical** to
   `cargo run -p riggen-app -- --export mjcf --out <dir> assets/fixtures/arm/arm.riggen`.
4. `cargo test` is green, including the step-2 and step-3 in-memory
   round-trips, and no native behaviour changed.

## Docs to update on completion

- `docs/01-architecture.md` §Layer map + §Cargo workspace — the `web/`
  directory and the wasm-bindgen bundle in the tree.
- `docs/01-architecture.md` §File format — the `FileSource` seam,
  `load_mesh_bytes`, `export_files`, and what "relative mesh path" means when
  there is no directory (ADR-0017).
- `docs/01-architecture.md` §Jobs and threads — decomposition on wasm is
  consented to, not merely inline (step 6).
- `docs/01-architecture.md` §Testing — the `wasm` job builds the bundle;
  `pages.yml`; the Chrome pass is the by-hand half and the agent runs it.
- `docs/02-data-model.md` §URDF import / §MJCF import — both `load`s take a
  `FileSource`; the dropped-set name-matching rule and its warnings.
- `docs/adr/0017-*.md` — new, written in step 4.
- `README.md` — a "Try it in the browser" line with the URL, above §Install.
- `docs/03-roadmap.md` — the v0.2 "Web demo build" line becomes a done entry
  with the gzipped wasm size, the way M4 recorded the wheel sizes. Nothing
  conditional is left, so `/close-cycle` can run.
- `AGENTS.md` current state — the web build is a product now, not a build
  check.
- `docs/BACKLOG.md` — remove "Web demo build"; rewrite the "Convex
  decomposition freezes a wasm build" line as the web-worker line it really
  is; add a WebGL2-fallback line and a touch/mobile line if step 1 wants
  them.

## Open questions

All four were put to the human on 2026-09-01, before step 1, and all four
were answered with the agent's recommendation. Nothing here is open.

- **OPEN 1 — the zip. RESOLVED: the `zip` crate**, `default-features =
  false`, stored, no compression. A hand-rolled CRC-32 and central directory
  is a liability for no gain. Step 5 — where "wasm-only" turned out to be
  wrong: the acceptance test that reads the archive back is native.
- **OPEN 2 — decomposition on the web. RESOLVED: confirm, then freeze.** A
  demo that hides a v0.2 headline feature undersells it, and the freeze is
  seconds. Step 6.
- **OPEN 3 — WebGPU only. RESOLVED: yes.** A visitor without WebGPU gets a
  plain-English page; the WebGL2 fallback stays a backlog line. Step 1.
- **OPEN 4 — when the demo deploys. RESOLVED: every push to `main`.** The
  demo tracks the trunk, which is always green. Step 7.
