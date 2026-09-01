# ADR-0017: On the web, bytes come in through one `FileSource` seam, files go out as downloads, and a dropped set resolves its meshes by file name

- Status: Accepted
- Date: 2026-09-01

## Context

`docs/03-roadmap.md`'s last open v0.2 line is the web demo: riggen at a
public URL, with the sample arm already in it and your own meshes droppable
onto the page. The viewport, the document, the writers and the importers are
all already portable — `riggen-core` and `riggen-export` never touch egui or
wgpu, the `wasm` CI job has built `riggen-app` for `wasm32-unknown-unknown`
since M0 — so nothing about the *computation* is in question.

What is in question is IO, and it is in question because a browser has no
filesystem. Everything riggen does with files is path-shaped:
`riggen_core::load(path)` reads the document and then hashes each mesh
beside it; `MeshStore::load` opens each `MeshAsset::path`;
`urdf_in::load` resolves `package://arm/base.stl` by walking the file's
ancestors on disk; `export` writes a directory. In a browser none of that
exists. Files arrive as a flat set of `File` objects per drop gesture, read
asynchronously, with no directory and — deliberately, for privacy — no local
path: the browser gives us the *name* `base.stl` and nothing more. And
nothing can be written anywhere; the only way out is a download the user
accepts.

The tempting shape is a second, thinner web reader: parse the JSON, skip the
hashes, ignore `meshdir`, and accept that the browser build understands a
narrower subset of what the desktop understands. That is how the two halves
drift, and the drift is invisible until a user reports that their URDF opens
in the app and not on the page.

Three smaller questions came with it, and were put to the human before step 1
of `plans/web-demo`:

- **WebGPU only, or a WebGL2 fallback?** The viewport's picking is an
  `R32Uint` target read back with `copy_texture_to_buffer`, which wgpu's GL
  backend will not do.
- **A zip crate, or a hand-rolled stored-zip writer?** An export is a
  directory, and a browser download is one file.
- **What does a mesh path mean when there is no directory?**

## Decision

**1. One seam: `riggen_core::file::FileSource`.** A read-only trait —
`read`, plus `exists` and `hash` with defaults over it — with two
implementations: `Disk`, which is `std::fs` and is what every native path
takes, and `MemorySource`, bytes keyed by path. Every reader in the
workspace takes one: `load_from(text, base, source)` under `load`,
`MeshStore::load`, `urdf_in::load` / `from_urdf` / `resolve_mesh_path`,
`mjcf_in::load` / `from_mjcf`. `riggen_mesh::load_mesh_bytes(name, bytes)`
joins `load_mesh` as the extension dispatcher over the same `parse_stl` /
`parse_obj` the path version uses.

Not a virtual filesystem. A VFS would need directories, metadata, a
mount table and a notion of the current directory, all to serve readers that
only ever ask one question: *give me the bytes at this path*.

**2. The write side splits the same way.** `export_files(robot, options,
dir) -> Vec<(PathBuf, Vec<u8>)>` is the whole export directory as bytes;
`export()` is that list plus `create_dir_all` and the
`.tmp`-sibling-and-rename of ADR-0008, which is discipline that only means
something on a real filesystem. `dir` stays an argument because
`MeshPathStyle::Absolute` writes it into the model files, but nothing is
read from it and nothing is created in it, so a virtual root is a good
answer.

**3. A dropped set is the resolution scope, and it resolves by file name.**
`riggen_app::DroppedSet` is the files of one drop gesture keyed by their
names, and its `FileSource::read` looks only at `path.file_name()`. So
`meshes/base.stl`, `../base.stl` and `base.stl` all mean *the file called
`base.stl` that came with this drop*. Dropped files are given the synthetic
directory `/dropped/`, which exists nowhere, so every path in the document
is absolute exactly as it is on disk (docs/01-architecture.md §File format)
and no code has to learn a second kind of path.

A reference the set does not carry is **missing**, and the reader that asked
says so in the vocabulary it already had — `file::Warning::MeshUnreadable`,
`ImportWarning::MeshNotFound`, `ExportError::UnloadableMesh`. Not a new
error type, and not silence.

**4. A set carrying a document is that document's set.** `load_dropped`
opens the documents in the gesture and treats every other file as a
companion, where `load_files` opens each path in turn. Dropping
`arm.riggen` and its four STLs gives one document, not one document and
four stray links. The rule differs from the path route because the
situation does: on disk a mesh reference resolves whether or not the mesh
was dropped, and here it does not.

So a gesture carrying a document **replaces** the set — the new document's
meshes are the ones it arrived with, and one it did not bring is missing,
exactly as a moved file is missing on disk. A gesture of meshes alone
**adds** to the set: those meshes are joining the document already open,
not redefining what it is made of. Anything else makes the answer depend on
what was dropped ten minutes ago.

Which world the app is in is one field, `RiggenApp::files: Files` —
`Disk` on the desktop, `Dropped(DroppedSet)` in the browser — and every mesh
the app loads goes through it.

**5. A document opened from bytes is untitled.** There is no path to save it
back to, so Save behaves as Save As: a download.

**6. Out is a download.** Save and Save As hand the browser the `.riggen`
text; Export hands it the `export_files` list as one **stored,
uncompressed** zip; Debug › Save state hands it the JSON. The `zip` crate,
`default-features = false`, is a wasm-only dependency: a hand-rolled CRC-32
and central directory is a liability for no gain, and with no compression it
pulls no dependency tree.

**7. WebGPU only.** A browser without it gets a page in plain English
naming the browsers that have it and pointing at `pip install riggen`. A
WebGL2 fallback stays a backlog line.

## Consequences

- The browser and the desktop run the **same** readers and the same
  writers. A `.riggen` v1 file opens on the page through the same upgrade
  chain; `<compiler meshdir="arm">` resolves; `package://arm/base.stl`
  resolves; the content hash is checked and a changed mesh is reported.
  Tests hold this literally: `arm.riggen`, `arm.urdf` and
  `menagerie_style.xml` are opened out of an in-memory set rooted at a
  directory that does not exist — so any read that leaked to the filesystem
  would fail — and compared field for field against the on-disk load.
- Two files with the same name in one gesture collide, and the later one
  wins. A browser drop of two directories that each contain `base.stl` is
  therefore ambiguous, and the demo does not pretend otherwise. A directory
  drop with real relative paths is a backlog line, not this.
- `FileSource` is taken as `&dyn`, not `&impl`: `mjcf_in::Import` holds one
  for the whole conversion.
- Native behaviour is unchanged. `Disk` is the default everywhere the app
  is not a browser, and `load`, `save` and `export` keep their signatures
  and their messages.
- The wasm build gains an inbox: a browser reads a dropped file
  asynchronously, so `handle_file_drops` spawns one future per gesture and
  the bytes land a frame or two later. wasm is single-threaded, so an `Rc`
  and a `RefCell` are the whole synchronisation story.

## Alternatives considered

- **A virtual filesystem in `riggen-core`.** Directories, metadata, a mount
  table and a current directory, to answer a question that is always "the
  bytes at this path". Rejected: the seam is one method wide.
- **A second, browser-only reader.** Parse the JSON, skip the hashes, ignore
  `meshdir`. Rejected: it is exactly the drift the demo is meant to disprove
  — the page would understand a narrower riggen than the app, and nothing
  would tell us when the two diverged.
- **Resolving a dropped mesh by its full relative path.** Chromium's
  `webkitRelativePath` gives one for a *directory* picker, but not for a
  plain drop, and not in every browser. Rejected: the rule would work
  sometimes, which is worse than a rule that always works by name.
- **A `File System Access` handle for real save-in-place.** Chromium only,
  behind a permission prompt, and a second write path to keep working.
  Rejected for the demo; a download is understood everywhere.
- **A WebGL2 fallback.** Picking would need a different mechanism entirely
  — a CPU raycast, or colour-encoded readback — which is a second picking
  implementation to keep honest. Rejected; backlog.
