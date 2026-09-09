//! MJCF composition, resolved before the reader sees the tree (ADR-0026):
//! `<include>` spliced, `<frame>` folded into what it wraps. Runs before
//! `Compiler::read` and `Defaults::read`, because an included
//! `<compiler angle="radian"/>` changes how the main file's `euler` reads.
//!
//! The pass is written against MuJoCo's measured behaviour, not the
//! documentation's. Probed on MuJoCo 3.13.0 with scratch models and
//! `mj_saveLastXML` (2026-09-09, plans/composition step 1); every line
//! below is a case that was run, and a case that a later reader should
//! re-run before changing the rule.
//!
//! # `<include>`
//!
//! - **Directory.** `file` is joined to the **main** model's directory
//!   first; if that path does not exist, to the *including* file's
//!   directory. A miss names the second path in MuJoCo's error.
//! - **Root tag.** Any: `<mujoco>`, `<mujocoinclude>`, even `<fragment>`.
//!   The included root's attributes are ignored — `<mujoco model="sub">`
//!   does not rename the model.
//! - **Site.** Legal anywhere — top level, `<worldbody>`, `<body>`,
//!   `<default>`, `<asset>`, `<frame>` — and the root's children are
//!   spliced at the site, in place, in document order. Splicing does not
//!   unwrap: a `<worldbody>` fragment included *inside* `<worldbody>` is
//!   MuJoCo's schema error, not a merge.
//! - **Twice.** `File 'x.xml' already included`, a hard error. A cycle and
//!   a self-include are the same error. The key is the path **as
//!   written**, normalised (`a.xml` and `./a.xml` collide), not the file
//!   that was opened: `y.xml` found through the fallback and `sub/y.xml`
//!   are two keys for one file, and both load.
//! - **Several `<compiler>` blocks** merge per attribute, last in document
//!   order wins, and all of them are read **before any body** — an
//!   included `<compiler>` placed after `<worldbody>` still governs it.
//!   `Compiler::read` already does exactly this.
//! - **Several `<worldbody>` blocks**: their bodies are all root bodies.
//! - **Several `<default>` blocks** are one tree. A class name defined in
//!   two of them is `repeated default class name` (MuJoCo refuses; our
//!   `Defaults::absorb` merges — ADR-0026 §6). Two anonymous top-level
//!   `<default>`s merge per attribute.
//! - **A `<mesh file>` in an included file** resolves against main
//!   directory + `meshdir` + `file`; failing that, the including file's
//!   directory + `file` with **no** `meshdir` (ADR-0026 §6; the plan's
//!   open question 5).
//!
//! # `<frame>`
//!
//! - **Pose.** `child = frame ∘ child` for `<body>`, `<geom>` (`fromto`
//!   too), `<site>`, `<camera>`, `<light>` (`pos` and `dir`) and `<joint>`
//!   (`pos` and `axis`). Nested frames compose outer-first. The frame's
//!   orientation may be any of the five spellings and is read under the
//!   `<compiler>`'s `angle` / `eulerseq` exactly as a body's; a child's
//!   own orientation, in any spelling, composes rather than being
//!   replaced. A body under a frame has its pose composed and its
//!   contents left alone: a joint inside it keeps its body-local `axis`.
//! - **`childclass`.** The innermost enclosing `<frame childclass>` beats
//!   the enclosing body's `childclass` for every direct child that names
//!   no `class`; a wrapped `<body>` with no `childclass` of its own takes
//!   the frame's as its `childclass`, so its whole subtree inherits it. A
//!   body's own `childclass` wins. `class` on a frame acts as
//!   `childclass` — the frame schema accepts unknown attributes
//!   (`bogus="1"` loads), unlike `<body>`.
//! - **Left alone.** An `<inertial>` under a frame is accepted and *not*
//!   transformed. A `<freejoint>` under a frame has no pose to compose.
//! - **Gone.** `mj_saveLastXML` writes the flattened model: no
//!   `<include>`, no `<frame>`, one `<compiler>`, one `<worldbody>`.
//!
//! What stays refused, and why, is ADR-0026 §4: `<replicate>` (a subtree
//! copy with `Tᵏ` composition and a rename of everything that names it)
//! and `<attach>` (a submodel under a name prefix — two robots composed,
//! which is a document question, not a tree rewrite).
