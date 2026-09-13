# Plan: package-map-ui

- Started: 2026-09-13
- Milestone: v0.6 — the import gap's last mile
- Idea: `docs/ideas/package-map-ui.md` (absorbed). Verbatim from the
  human: "next thing", then "use recommended for open questions and /plan".
  Decided: B, ask on a miss and import again with the map; the window
  closes on the first history entry; the CLI's `--package` is in this
  plan; no remembered map or ROS environment probe (a backlog line).

## Goal

A URDF whose `package://` paths the beside-the-file heuristic misses is
fixable in the window. After File › Import URDF… or a dropped `.urdf`,
a **Missing packages** window names each package that did not resolve and
how many meshes it cost, with **Choose folder…** on each row. Choosing a
folder imports the same file again through a `PackageMap` that holds every
folder chosen so far, replacing the untitled, unedited document. The
window closes when nothing is left unresolved, when the user dismisses it,
on the first history entry, or when another file opens. The headless path
matches: `riggen --export … --package NAME=DIR INPUT`, repeatable, is the
SDK's `packages=` on the command line. A file that resolves shows nothing
new.

## Non-goals

- **No table on every import** and no override of a package that resolved
  to the wrong copy (the idea's option A, not taken).
- **No remembered map and no `ROS_PACKAGE_PATH` / `AMENT_PREFIX_PATH`
  probe.** A backlog line.
- **No change to the resolver's order or to `ImportWarning`.**
  `PackageUnresolved` stays one per mesh, so the SDK's and the CLI's
  warning text is unchanged; the app groups them by package.
- **No relinking after an edit.** The document stores resolved paths, not
  package names, so once the history holds an entry the SDK and the CLI
  flag are the way out.
- **Not the web build.** A browser drop has no folders (ADR-0017 §3); the
  window is native-only, and the browser keeps today's status line.
- **Not MJCF**, which has no `package://`, and not the other v0.6 line
  (`validate`'s skipped numbers).

## Design deltas

- **`riggen-app::cli`** (ARCHITECTURE §Export, README §Command line).
  `ExportArgs` gains `packages: PackageMap`, filled by `--package NAME=DIR`,
  which may repeat. Like `--out`, it needs `--export`. A value without `=`,
  an empty name, or the same name twice is a usage error. `run` passes the
  map to `urdf_in::load`. An `.xml` or `.riggen` input ignores it. The usage
  line and `--help` gain the flag.
- **`riggen-app::file_io`**. An URDF import remembers
  `MissingPackages { urdf: PathBuf, map: PackageMap, missing: BTreeMap<String, usize> }`
  when its warnings hold a `PackageUnresolved`: package name to the count
  of those warnings. `open_urdf` imports with the stored map when it
  re-imports the same file. Public seams, which the window calls and tests
  drive:
  - `missing_packages() -> Option<&BTreeMap<String, usize>>`;
  - `set_package_dir(name, dir)`, which adds to the map and imports again;
  - `dismiss_missing_packages()`.

  The state is cleared by a re-import with no misses left, by the first
  history entry, by dismissal, and by any other open or new document.
- **`riggen-app`, the window.** A non-modal `egui::Window` ("Missing
  packages"): the user can still look at the broken model behind it,
  which a `Modal` would dim. Each row shows the package, "N meshes" and
  **Choose folder…** (`rfd::FileDialog::pick_folder`, then
  `set_package_dir`); a **Dismiss** button closes it. It is compiled out
  on `wasm32`.
- **`riggen-app::debug`**. `debug_state().ui.missing_packages`, a list of
  `{package, meshes}` omitted when empty so no existing golden changes.
- **Fixture** `assets/fixtures/vendor/`. A URDF under `urdf/` referencing
  two packages: one whose directory carries its package name, so the
  ancestor walk finds it, and one whose directory does not
  (`Finger-Repo/` holding `finger_description`'s meshes), missing two
  meshes. The meshes are small copies of an existing fixture STL.
- No ADR. The idea found that option B leaves ADR-0006 §3 (no per-file
  dialog) and ADR-0017 standing: the window appears only after a miss.

## Steps

Complexity: **[1]** routine — the design says what to write, the tests are
mechanical; **[2]** careful — a case to get right within a given design;
**[3]** unproven — behaviour that has to be established here.

- [x] **[1]** Step 1 — The fixture and the CLI's `--package`.
  - `assets/fixtures/vendor/` as in the design deltas, with a header
    comment saying which package the heuristic finds and which it misses.
  - `cli.rs` `parse` and `run`, with tests:
    - `--package` parses, repeats and needs `--export`; the three usage
      errors are refused with `USAGE`.
    - `run` on the vendor URDF without the flag: two `MeshNotFound`, and
      the export refused with `UnloadableMesh`.
    - With `--package finger_description=…/Finger-Repo`: the export writes
      both finger meshes.
  - README §Command line (usage block, options) and the ARCHITECTURE
    sentence quoting the export form change in this commit.
- [ ] **[2]** Step 2 — The app knows what is missing and imports again.
  - `MissingPackages`, the three seams and the clearing rules from the
    design deltas; `debug_state().ui.missing_packages`.
  - App tests in `tests/visual/main.rs` (`with_app`, no golden):
    - Importing the vendor fixture gives `missing_packages() ==
      {finger_description: 2}`, and two instances with no triangles.
    - `set_package_dir("finger_description", …)` leaves `None`, every
      instance with triangles, history depth 0, the status `imported …`
      with no warning, and the document still untitled in View.
    - A folder that does not hold the meshes keeps the package listed.
    - Any history entry clears the state, and so does opening
      `pendulum.riggen`.
    - `arm/arm.urdf`, which resolves, never sets it.
- [ ] **[2]** Step 3 — The window.
  - The "Missing packages" window over the viewport, native only, with
    its rows, **Choose folder…** and **Dismiss**; it is not drawn in Zen.
  - Snapshot `missing_packages`: the vendor fixture just imported, the
    finger links empty, the window naming `finger_description · 2 meshes`.
    Shown to the human.
  - A test that clicks **Dismiss** by its label: the window goes and
    `missing_packages()` is `None`.

## Acceptance

- `riggen --export mjcf --package finger_description=assets/fixtures/vendor/Finger-Repo --out DIR assets/fixtures/vendor/urdf/<file>.urdf`
  writes both finger meshes, and the same command without `--package` is
  refused naming them (step 1's tests are this check).
- In the app, the vendor fixture opens with the window. One
  `set_package_dir` leaves nothing missing, every link drawn, and the
  export dialog ready (step 2's tests).
- `cargo test --workspace` passes, including the `missing_packages`
  snapshot.

## Docs to update on completion

- `docs/DATA-MODEL.md` §URDF import: who passes a map — the app's
  Missing packages window, the CLI's `--package`, the SDK's `packages=`.
  Today it says the map exists and nothing about where one comes from.
- `docs/ARCHITECTURE.md`:
  - the paragraph after the export form (a `.urdf` opens as a new
    document): a miss opens the Missing packages window, what closes it,
    native only;
  - the snapshot list gains `missing_packages`.
- `README.md`: the **Imports** bullet ("`package://` paths resolved beside
  the file") names the folder the window asks for and `--package`.
- `docs/ROADMAP.md` v0.6: the line reworded to "a missed `package://` is
  fixable in the window" and marked *Landed*.
- `AGENTS.md` current state: the `PackageMap` UI moves from **Next** to
  landed.

## Open questions

None for the human. The idea's four decisions are taken (header).
Left to the agent, and recorded at the step where each is settled:
- where the window anchors;
- ~~whether the fixture's two meshes are one STL or two~~ — two
  (`finger_left.stl`, `finger_right.stl`, both copies of
  `cube_binary.stl`), so the refusal and the export each name two files.
  The fixture is `vendor/urdf/gripper.urdf`, with `palm.stl` under
  `vendor/gripper_description/` (step 1). The `run` checks go through
  the built binary in `tests/cli.rs`, because the `MeshNotFound`
  warnings are only on stderr.
