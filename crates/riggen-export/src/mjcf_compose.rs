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
//!   directory + `file` with **no** `meshdir`. The pass carries that
//!   second try, because the reader has only the first
//!   ([`rebase_included_meshes`], ADR-0026 §6).
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

use std::collections::BTreeSet;
use std::path::{Component, Path, PathBuf};

use riggen_core::FileSource;
use riggen_core::glam::{DQuat, DVec3};

use crate::import::ImportError;
use crate::mjcf_in::{Compiler, Defaults, MAIN_CLASS};
use crate::xml::{AngleConvention, Node, ORIENTATION_ATTRS};

/// The composition-free tree the reader reads: `root` — the parsed main
/// file — with every `<include>` under it spliced and every `<frame>`
/// folded into what it wrapped. `path` is the main
/// file, whose directory is where an included file is looked for first;
/// `source` is where its bytes come from, the same one the meshes come
/// through (ADR-0017). A pure tree → tree rewrite: no document types, no
/// import state, and the main root's own attributes (`model`) survive.
pub(crate) fn compose(
    root: Node,
    path: &Path,
    source: &dyn FileSource,
) -> Result<Node, ImportError> {
    let main_dir = path.parent().unwrap_or(Path::new(".")).to_owned();
    let mut includes = Includes {
        main: path.to_owned(),
        main_dir: main_dir.clone(),
        source,
        seen: BTreeSet::new(),
    };
    let mut root = includes.splice(root, path)?;
    rebase_included_meshes(&mut root, &main_dir, source);
    fold_frames(&mut root, path)?;
    Ok(root)
}

/// The attribute [`Includes::splice`] leaves on a `<mesh file>` that came
/// out of an *included* file: the directory of the file that declared it,
/// which [`rebase_included_meshes`] needs and then removes. Angle brackets
/// are not legal in an XML attribute name, so no file can carry it.
const FROM: &str = "<from>";

/// The `<include>` half of the pass.
struct Includes<'a> {
    /// The main file itself, so a `<mesh>` out of any other one is marked.
    main: PathBuf,
    main_dir: PathBuf,
    source: &'a dyn FileSource,
    /// Every `<include file>` seen so far, keyed as written and normalised
    /// — not by the file it opened (the module comment's "Twice").
    seen: BTreeSet<PathBuf>,
}

impl Includes<'_> {
    /// `node` with every `<include>` under it, at any depth, replaced by
    /// the children of the file it names — themselves spliced — in place
    /// and in document order. `from` is the file `node` was read out of:
    /// the second directory an include is looked for in, and the name a
    /// miss is reported against.
    fn splice(&mut self, mut node: Node, from: &Path) -> Result<Node, ImportError> {
        let children = std::mem::take(&mut node.children);
        for child in children {
            if child.tag == "include" {
                let included = self.open(&child, from)?;
                node.children.extend(included.children);
            } else {
                let mut child = self.splice(child, from)?;
                // MuJoCo's second directory for this mesh, resolved once
                // the whole tree — and so the merged `meshdir` — is known.
                if child.tag == "mesh" && child.attrs.contains_key("file") && from != self.main {
                    let dir = from.parent().unwrap_or(Path::new("."));
                    child
                        .attrs
                        .insert(FROM.to_owned(), dir.display().to_string());
                }
                node.children.push(child);
            }
        }
        Ok(node)
    }

    /// The root of the file `include` names, spliced. Any root tag is
    /// accepted — `<mujoco>`, `<mujocoinclude>`, anything — and its
    /// attributes are ignored; only its children are spliced (the module
    /// comment's "Root tag").
    fn open(&mut self, include: &Node, from: &Path) -> Result<Node, ImportError> {
        let file = include.attr("file").ok_or_else(|| ImportError::Parse {
            path: from.to_owned(),
            message: "<include> names no file".to_owned(),
        })?;
        let key = normalized(Path::new(file));
        if !self.seen.insert(key.clone()) {
            return Err(ImportError::DuplicateInclude { file: key });
        }
        let from_dir = from.parent().unwrap_or(Path::new("."));
        let path = [self.main_dir.join(file), from_dir.join(file)]
            .iter()
            .map(|p| normalized(p))
            .find(|p| self.source.exists(p))
            .ok_or_else(|| ImportError::IncludeNotFound {
                file: key,
                from: from.to_owned(),
            })?;
        let parse = |m: String| ImportError::Parse {
            path: path.clone(),
            message: m,
        };
        let bytes = self.source.read(&path).map_err(|e| ImportError::Io {
            path: path.clone(),
            message: e.to_string(),
        })?;
        let text = String::from_utf8(bytes).map_err(|e| parse(e.to_string()))?;
        let root = crate::xml::parse(&text).map_err(|e| parse(e.to_string()))?;
        self.splice(root, &path)
    }
}

/// `path` with `.` dropped and `a/..` collapsed, lexically — so `a.xml`
/// and `./a.xml` are one include key, as they are one file to MuJoCo.
/// Relative stays relative: the key is the path as written, not where it
/// was found.
fn normalized(path: &Path) -> PathBuf {
    let mut out = PathBuf::new();
    for component in path.components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir
                if matches!(out.components().next_back(), Some(Component::Normal(_))) =>
            {
                out.pop();
            }
            other => out.push(other),
        }
    }
    out
}

/// MuJoCo's second try for a `<mesh file>` an *included* file declared:
/// the main directory + `meshdir` + `file` is the first, and when nothing
/// is there, the including file's own directory + `file`, with **no**
/// `meshdir` (the module comment's "A `<mesh file>` in an included file";
/// ADR-0026 §6). The reader has only the first, so the pass rewrites
/// `file` to the second when that is the one that exists — composition
/// still never reaches the reader. A `file` the first try finds, an
/// absolute one, and one neither try finds are left exactly as written,
/// the last so the reader's `MeshMissing` still names what the file said.
fn rebase_included_meshes(root: &mut Node, main_dir: &Path, source: &dyn FileSource) {
    // The merged `meshdir` of the *composed* tree, which is why this runs
    // after splicing. A `<compiler>` the reader will refuse outright reads
    // here as no `meshdir`; the reader reports it a moment later.
    let meshdir = Compiler::read(root).map(|c| c.meshdir).unwrap_or_default();
    rebase(root, &main_dir.join(meshdir), source);
}

/// [`rebase_included_meshes`] over one node and its subtree. `first_dir`
/// is the main directory with `meshdir` already joined onto it.
fn rebase(node: &mut Node, first_dir: &Path, source: &dyn FileSource) {
    for child in &mut node.children {
        rebase(child, first_dir, source);
    }
    let Some(from) = node.attrs.remove(FROM) else {
        return;
    };
    let file = Path::new(node.attrs.get("file").map_or("", String::as_str)).to_owned();
    if file.is_absolute() {
        return;
    }
    let at = |dir: &Path| {
        let path = normalized(&dir.join(&file));
        riggen_core::absolute(&path).unwrap_or(path)
    };
    if source.exists(&at(first_dir)) {
        return;
    }
    let fallback = at(Path::new(&from));
    if source.exists(&fallback) {
        // Absolute, because the reader joins a relative one onto the main
        // directory and `meshdir` all over again.
        node.attrs
            .insert("file".to_owned(), fallback.display().to_string());
    }
}

// ---------------------------------------------------------------------------
// `<frame>` (ADR-0026 §3)
// ---------------------------------------------------------------------------

/// The tags a frame's pose reaches. Everything else one may wrap — an
/// `<inertial>`, a `<freejoint>`, a `<plugin>` — is spliced through
/// exactly as written, which is what MuJoCo does with it (the module
/// comment's "Left alone").
const POSED: &[&str] = &["body", "geom", "site", "joint", "camera", "light"];

/// Of [`POSED`], those that take a `class`; a `<body>` takes a
/// `childclass` instead, and gets one in [`push_class`].
const CLASSED: &[&str] = &["geom", "site", "joint", "camera", "light"];

/// Attributes a `<default>` class would have given an element that the
/// frame's pose has to reach — the pose itself in any of its five
/// spellings, a joint's `axis`, a light's `dir`, a geom's `fromto`.
const REACHED: &[&str] = &[
    "pos",
    "quat",
    "euler",
    "axisangle",
    "xyaxes",
    "zaxis",
    "axis",
    "dir",
    "fromto",
];

/// Every `<frame>` in `root` folded into what it wraps, until none is
/// left. Reading the `<compiler>` and the `<default>` tree is what a frame
/// costs — the frame's own orientation is spelled under `angle` and
/// `eulerseq`, and half of Menagerie's joints take the `axis` a rotating
/// frame has to turn from a class — so a tree without one pays nothing and
/// keeps the reader's error for a bad `<compiler>`.
fn fold_frames(root: &mut Node, path: &Path) -> Result<(), ImportError> {
    if !has_frame(root) {
        return Ok(());
    }
    let parse = |m: String| ImportError::Parse {
        path: path.to_owned(),
        message: m,
    };
    let frames = Frames {
        conv: Compiler::read(root).map_err(parse)?.angle,
        defaults: Defaults::read(root).map_err(parse)?,
        path: path.to_owned(),
    };
    frames.walk(root, MAIN_CLASS)
}

fn has_frame(node: &Node) -> bool {
    node.children
        .iter()
        .any(|c| c.tag == "frame" || has_frame(c))
}

/// The `<frame>` half of the pass.
struct Frames {
    conv: AngleConvention,
    /// The classes, for [`Frames::materialize`] — an element's pose may be
    /// its class's, and the frame composes onto the pose it *has*.
    defaults: Defaults,
    /// The main file, which every error here is reported against: it is
    /// the arithmetic that failed, not a file that was opened.
    path: PathBuf,
}

impl Frames {
    fn err(&self, message: String) -> ImportError {
        ImportError::Parse {
            path: self.path.clone(),
            message,
        }
    }

    /// `node`'s children with every `<frame>` among them replaced by what
    /// it wrapped, then the same over each child's subtree. `inherited` is
    /// the `childclass` in force, which a frame's children need to find
    /// their own defaults before the frame's pose is composed onto them.
    fn walk(&self, node: &mut Node, inherited: &str) -> Result<(), ImportError> {
        let childclass = Defaults::childclass(node, inherited).to_owned();
        for child in std::mem::take(&mut node.children) {
            if child.tag == "frame" {
                node.children.extend(self.unwrap(child, None, &childclass)?);
            } else {
                node.children.push(child);
            }
        }
        for child in &mut node.children {
            self.walk(child, &childclass)?;
        }
        Ok(())
    }

    /// One `<frame>` gone: its children, in document order, each with
    /// `child = frame ∘ child` composed onto it and the frame's class
    /// pushed onto it. A nested frame is composed onto and then unwrapped
    /// in turn, so the outer transform reaches the inner frame's children
    /// — outer-first — and the outer class reaches through an inner frame
    /// that names none.
    fn unwrap(
        &self,
        frame: Node,
        outer_class: Option<&str>,
        inherited: &str,
    ) -> Result<Vec<Node>, ImportError> {
        let pos = frame
            .vec3("pos")
            .map_err(|e| self.err(e))?
            .unwrap_or(DVec3::ZERO);
        let rot = frame
            .orientation(self.conv)
            .map_err(|e| self.err(e))?
            .unwrap_or(DQuat::IDENTITY);
        // `class` on a frame means `childclass` (the module comment's
        // "childclass").
        let class = frame
            .attr("childclass")
            .or(frame.attr("class"))
            .or(outer_class)
            .map(str::to_owned);
        let mut out = Vec::new();
        for mut kid in frame.children {
            if kid.tag == "frame" {
                self.transform(&mut kid, pos, rot)?;
                out.extend(self.unwrap(kid, class.as_deref(), inherited)?);
                continue;
            }
            let in_force = kid
                .attr("class")
                .or(class.as_deref())
                .unwrap_or(inherited)
                .to_owned();
            self.materialize(&mut kid, &in_force);
            self.transform(&mut kid, pos, rot)?;
            if let Some(class) = &class {
                push_class(&mut kid, class);
            }
            out.push(kid);
        }
        Ok(out)
    }

    /// Whichever of [`REACHED`] this element's class would have given it
    /// written onto the element itself, so that the composition below acts
    /// on the pose the element really has and the class cannot smuggle an
    /// untransformed one in behind it. Everything else the class carries
    /// is left to the reader, which applies it again — over attributes the
    /// element now owns, so it changes nothing. An unknown class is left
    /// alone: the reader reports it a moment later.
    fn materialize(&self, node: &mut Node, class: &str) {
        if !POSED.contains(&node.tag.as_str()) {
            return;
        }
        // `apply` already drops the class's rotation when the element
        // spells its own, in whichever of the five spellings each used.
        let Ok(resolved) = self.defaults.apply(node, class) else {
            return;
        };
        for a in REACHED {
            if let (None, Some(v)) = (node.attrs.get(*a), resolved.attrs.get(*a)) {
                node.attrs.insert((*a).to_owned(), v.clone());
            }
        }
    }

    /// `child = frame ∘ child`, in place.
    fn transform(&self, node: &mut Node, fp: DVec3, fq: DQuat) -> Result<(), ImportError> {
        match node.tag.as_str() {
            "body" | "site" | "camera" | "frame" => {
                self.moved(node, "pos", fp, fq)?;
                self.turned(node, fq)?;
            }
            // `fromto` names two points in the parent's frame and replaces
            // the geom's pose; a geom may carry either.
            "geom" => {
                self.moved(node, "pos", fp, fq)?;
                self.turned(node, fq)?;
                if let Some(ends) = node.nums::<6>("fromto").map_err(|e| self.err(e))? {
                    let end = |i: usize| fp + fq * DVec3::new(ends[i], ends[i + 1], ends[i + 2]);
                    let (a, b) = (end(0), end(3));
                    let both = format!("{} {}", crate::xml::vec3(a), crate::xml::vec3(b));
                    node.attrs.insert("fromto".to_owned(), both);
                }
            }
            // A joint and a light spell a direction, not an orientation:
            // MJCF gives neither of them a `quat`. The defaults are MJCF's
            // own, and a rotating frame turns them like any other vector.
            "joint" => {
                self.moved(node, "pos", fp, fq)?;
                self.turned_vec(node, "axis", DVec3::Z, fq)?;
            }
            "light" => {
                self.moved(node, "pos", fp, fq)?;
                self.turned_vec(node, "dir", -DVec3::Z, fq)?;
            }
            _ => {}
        }
        Ok(())
    }

    /// A point: `fp + fq · p`, written only when it is not the origin the
    /// element already implied.
    fn moved(&self, node: &mut Node, name: &str, fp: DVec3, fq: DQuat) -> Result<(), ImportError> {
        let p = node.vec3(name).map_err(|e| self.err(e))?;
        let moved = fp + fq * p.unwrap_or(DVec3::ZERO);
        if p.is_some() || moved != DVec3::ZERO {
            node.attrs.insert(name.to_owned(), crate::xml::vec3(moved));
        }
        Ok(())
    }

    /// A rotation: `fq · q`, as a `quat`, replacing whichever of the five
    /// spellings the element used. A frame that only moves leaves the
    /// element's own spelling alone.
    fn turned(&self, node: &mut Node, fq: DQuat) -> Result<(), ImportError> {
        if fq == DQuat::IDENTITY {
            return Ok(());
        }
        let q = node
            .orientation(self.conv)
            .map_err(|e| self.err(e))?
            .unwrap_or(DQuat::IDENTITY);
        for a in ORIENTATION_ATTRS {
            node.attrs.remove(a);
        }
        node.attrs
            .insert("quat".to_owned(), crate::xml::quat(fq * q));
        Ok(())
    }

    /// A direction: `fq · v`, with MJCF's own default for `v` when the
    /// element leaves it out — an unwritten `axis` is `0 0 1` and a
    /// rotating frame turns it just the same.
    fn turned_vec(
        &self,
        node: &mut Node,
        name: &str,
        default: DVec3,
        fq: DQuat,
    ) -> Result<(), ImportError> {
        if fq == DQuat::IDENTITY {
            return Ok(());
        }
        let v = node.vec3(name).map_err(|e| self.err(e))?.unwrap_or(default);
        node.attrs.insert(name.to_owned(), crate::xml::vec3(fq * v));
        Ok(())
    }
}

/// The frame's class onto a child that names none: a `<body>` takes it as
/// its `childclass`, so its whole subtree inherits it, and everything else
/// that takes one as its `class`. A child's own wins (the module comment's
/// "childclass").
fn push_class(node: &mut Node, class: &str) {
    let attr = match node.tag.as_str() {
        "body" => "childclass",
        t if CLASSED.contains(&t) => "class",
        _ => return,
    };
    node.attrs
        .entry(attr.to_owned())
        .or_insert_with(|| class.to_owned());
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::parse;
    use riggen_core::MemorySource;

    const MAIN: &str = "/dropped/scene.xml";

    /// A file set under a root that does not exist, so a read that slipped
    /// through to the disk fails instead of quietly agreeing.
    fn set(files: &[(&str, &str)]) -> MemorySource {
        let mut memory = MemorySource::default();
        for (at, text) in files {
            memory.insert(Path::new("/dropped").join(at), text.as_bytes().to_vec());
        }
        memory
    }

    fn composed(files: &[(&str, &str)]) -> Result<Node, ImportError> {
        let memory = set(files);
        let text = String::from_utf8(memory.0[Path::new(MAIN)].clone()).unwrap();
        compose(parse(&text).unwrap(), Path::new(MAIN), &memory)
    }

    /// `compose` over `files` gives the tree `flat` parses to.
    fn assert_flattens_to(files: &[(&str, &str)], flat: &str) {
        assert_eq!(composed(files).unwrap(), parse(flat).unwrap());
    }

    /// A model split in two: the include is replaced by the included
    /// root's children, at its site, and the main root keeps its `model`.
    #[test]
    fn two_files_splice_at_the_site() {
        assert_flattens_to(
            &[
                (
                    "scene.xml",
                    r#"<mujoco model="m"><compiler angle="radian"/>
                         <include file="robot.xml"/>
                         <worldbody><light/></worldbody></mujoco>"#,
                ),
                (
                    "robot.xml",
                    r#"<mujoco model="sub"><asset><mesh name="a" file="a.stl"/></asset>
                         <worldbody><body name="b"/></worldbody></mujoco>"#,
                ),
            ],
            r#"<mujoco model="m"><compiler angle="radian"/>
                 <asset><mesh name="a" file="a.stl"/></asset>
                 <worldbody><body name="b"/></worldbody>
                 <worldbody><light/></worldbody></mujoco>"#,
        );
    }

    /// Three deep, through a subdirectory: the leaf's `file` is looked for
    /// in the main directory first, and the innermost file names it
    /// relative to *that*, the way Menagerie's `scene.xml` → `robot.xml`
    /// → `assets.xml` chain does.
    #[test]
    fn a_chain_resolves_every_level_against_the_main_directory_first() {
        assert_flattens_to(
            &[
                (
                    "scene.xml",
                    r#"<mujoco><include file="sub/robot.xml"/></mujoco>"#,
                ),
                (
                    "sub/robot.xml",
                    r#"<mujoco><include file="sub/leaf.xml"/><worldbody/></mujoco>"#,
                ),
                ("sub/leaf.xml", r#"<mujoco><asset/></mujoco>"#),
            ],
            r#"<mujoco><asset/><worldbody/></mujoco>"#,
        );
    }

    /// A file the main directory does not hold is looked for beside the
    /// including file — MuJoCo's second try — and the main directory wins
    /// when both have one.
    #[test]
    fn the_including_directory_is_the_fallback_not_the_first_try() {
        let scene = r#"<mujoco><include file="sub/robot.xml"/></mujoco>"#;
        let robot = r#"<mujoco><include file="leaf.xml"/></mujoco>"#;
        assert_flattens_to(
            &[
                ("scene.xml", scene),
                ("sub/robot.xml", robot),
                ("sub/leaf.xml", r#"<mujoco><asset name="beside"/></mujoco>"#),
            ],
            r#"<mujoco><asset name="beside"/></mujoco>"#,
        );
        assert_flattens_to(
            &[
                ("scene.xml", scene),
                ("sub/robot.xml", robot),
                ("leaf.xml", r#"<mujoco><asset name="main"/></mujoco>"#),
                ("sub/leaf.xml", r#"<mujoco><asset name="beside"/></mujoco>"#),
            ],
            r#"<mujoco><asset name="main"/></mujoco>"#,
        );
    }

    /// An include inside a `<body>` splices the fragment's children there
    /// — bodies, geoms, joints become the body's own — without unwrapping.
    #[test]
    fn an_include_inside_a_body_splices_into_the_body() {
        assert_flattens_to(
            &[
                (
                    "scene.xml",
                    r#"<mujoco><worldbody><body name="base">
                         <geom type="box" size="1 1 1"/>
                         <include file="arm.xml"/>
                       </body></worldbody></mujoco>"#,
                ),
                (
                    "arm.xml",
                    r#"<mujocoinclude>
                         <body name="upper"><joint name="j"/></body>
                         <site name="tip"/>
                       </mujocoinclude>"#,
                ),
            ],
            r#"<mujoco><worldbody><body name="base">
                 <geom type="box" size="1 1 1"/>
                 <body name="upper"><joint name="j"/></body>
                 <site name="tip"/>
               </body></worldbody></mujoco>"#,
        );
    }

    /// Any root tag: `<mujocoinclude>` and a made-up one both splice, and
    /// the included root's attributes go nowhere.
    #[test]
    fn the_included_root_tag_and_attributes_are_ignored() {
        assert_flattens_to(
            &[
                (
                    "scene.xml",
                    r#"<mujoco model="m"><include file="a.xml"/><include file="b.xml"/></mujoco>"#,
                ),
                (
                    "a.xml",
                    r#"<mujocoinclude model="ignored"><asset/></mujocoinclude>"#,
                ),
                ("b.xml", r#"<fragment bogus="1"><worldbody/></fragment>"#),
            ],
            r#"<mujoco model="m"><asset/><worldbody/></mujoco>"#,
        );
    }

    /// The same file twice is MuJoCo's hard error, keyed on the path as
    /// written: `./a.xml` is `a.xml`.
    #[test]
    fn the_same_file_twice_is_refused() {
        assert_eq!(
            composed(&[
                (
                    "scene.xml",
                    r#"<mujoco><include file="a.xml"/><worldbody><include file="./a.xml"/></worldbody></mujoco>"#,
                ),
                ("a.xml", r#"<mujoco><asset/></mujoco>"#),
            ])
            .unwrap_err(),
            ImportError::DuplicateInclude {
                file: PathBuf::from("a.xml")
            }
        );
    }

    /// … but two spellings that open one file are two keys, and both load
    /// (the module comment's "Twice": `y.xml` through the fallback and
    /// `sub/y.xml` are one file to the disk and two to the visited set).
    #[test]
    fn two_spellings_of_one_file_are_two_includes() {
        assert_flattens_to(
            &[
                (
                    "scene.xml",
                    r#"<mujoco><include file="sub/y.xml"/><include file="sub/x.xml"/></mujoco>"#,
                ),
                ("sub/x.xml", r#"<mujoco><include file="y.xml"/></mujoco>"#),
                ("sub/y.xml", r#"<mujoco><asset/></mujoco>"#),
            ],
            r#"<mujoco><asset/><asset/></mujoco>"#,
        );
    }

    /// A cycle and a self-include terminate on the visited set with the
    /// same error a plain duplicate gets.
    #[test]
    fn a_cycle_and_a_self_include_terminate() {
        assert_eq!(
            composed(&[
                ("scene.xml", r#"<mujoco><include file="a.xml"/></mujoco>"#),
                ("a.xml", r#"<mujoco><include file="b.xml"/></mujoco>"#),
                ("b.xml", r#"<mujoco><include file="a.xml"/></mujoco>"#),
            ])
            .unwrap_err(),
            ImportError::DuplicateInclude {
                file: PathBuf::from("a.xml")
            }
        );
        assert_eq!(
            composed(&[(
                "scene.xml",
                r#"<mujoco><include file="scene.xml"/></mujoco>"#
            )])
            .unwrap_err(),
            ImportError::DuplicateInclude {
                file: PathBuf::from("scene.xml")
            }
        );
    }

    /// A file neither directory holds names itself and the file that
    /// asked for it — the including file, not always the main one.
    #[test]
    fn a_missing_file_names_itself_and_who_asked() {
        assert_eq!(
            composed(&[
                (
                    "scene.xml",
                    r#"<mujoco><include file="sub/robot.xml"/></mujoco>"#
                ),
                (
                    "sub/robot.xml",
                    r#"<mujoco><include file="assets.xml"/></mujoco>"#
                ),
            ])
            .unwrap_err(),
            ImportError::IncludeNotFound {
                file: PathBuf::from("assets.xml"),
                from: PathBuf::from("/dropped/sub/robot.xml"),
            }
        );
        assert_eq!(
            composed(&[("scene.xml", r#"<mujoco><include/></mujoco>"#)]).unwrap_err(),
            ImportError::Parse {
                path: PathBuf::from(MAIN),
                message: "<include> names no file".to_owned(),
            }
        );
        assert!(matches!(
            composed(&[
                ("scene.xml", r#"<mujoco><include file="a.xml"/></mujoco>"#),
                ("a.xml", "<mujoco><asset></mujoco>"),
            ])
            .unwrap_err(),
            ImportError::Parse { path, .. } if path == Path::new("/dropped/a.xml")
        ));
    }

    /// OPEN 5, decided: a `<mesh file>` an included file declared is
    /// looked for where MuJoCo looks — main directory + `meshdir` first,
    /// then beside the file that declared it with no `meshdir` — and the
    /// pass rewrites `file` to the second when only the second is there.
    /// A mesh the first try finds, one neither finds, and one the *main*
    /// file declared are left exactly as written; the marker attribute
    /// never survives the pass.
    #[test]
    fn a_mesh_an_included_file_declared_falls_back_to_that_files_directory() {
        let mut memory = set(&[
            (
                "scene.xml",
                r#"<mujoco><compiler meshdir="parts"/>
                     <asset><mesh name="main" file="here.stl"/></asset>
                     <include file="sub/robot.xml"/></mujoco>"#,
            ),
            (
                "sub/robot.xml",
                r#"<mujoco><asset>
                     <mesh name="beside" file="only_there.stl"/>
                     <mesh name="found" file="here.stl"/>
                     <mesh name="gone" file="nowhere.stl"/>
                     <mesh name="inline" vertex="0 0 0"/>
                   </asset></mujoco>"#,
            ),
        ]);
        for at in [
            "parts/here.stl",
            "sub/only_there.stl",
            "parts/only_there.stl.no",
        ] {
            memory.insert(Path::new("/dropped").join(at), b"mesh".to_vec());
        }
        let text = String::from_utf8(memory.0[Path::new(MAIN)].clone()).unwrap();
        let root = compose(parse(&text).unwrap(), Path::new(MAIN), &memory).unwrap();
        let file = |name: &str| {
            root.kids("asset")
                .flat_map(|a| a.kids("mesh"))
                .find(|m| m.attr("name") == Some(name))
                .map(|m| {
                    assert!(m.attr(FROM).is_none(), "the marker is gone");
                    m.attr("file").map(str::to_owned)
                })
                .unwrap()
        };
        assert_eq!(
            file("beside").as_deref(),
            Some("/dropped/sub/only_there.stl")
        );
        assert_eq!(file("found").as_deref(), Some("here.stl"));
        assert_eq!(file("gone").as_deref(), Some("nowhere.stl"));
        assert_eq!(file("main").as_deref(), Some("here.stl"));
        assert_eq!(file("inline"), None, "an inline mesh has no file to rebase");
    }

    /// `compose` over one file gives the tree `flat` parses to: a model
    /// with a `<frame>` in it against the same model hand-flattened.
    fn assert_folds_to(model: &str, flat: &str) {
        assert_flattens_to(&[("scene.xml", model)], flat);
    }

    /// `child = frame ∘ child` for everything a frame poses, with the
    /// frame's rotation read under the file's `<compiler angle>` and the
    /// child's own — in any of the five spellings — composed onto rather
    /// than replaced. The frame itself is gone.
    #[test]
    fn a_frame_moves_and_turns_what_it_wraps() {
        assert_folds_to(
            r#"<mujoco><worldbody>
                 <frame pos="0 0 1" euler="0 0 90">
                   <body name="b" pos="1 0 0"/>
                   <geom name="g" pos="0 2 0" euler="0 0 90"/>
                   <site name="s" quat="0 1 0 0"/>
                   <camera name="c" pos="1 1 1"/>
                 </frame>
               </worldbody></mujoco>"#,
            r#"<mujoco><worldbody>
                 <body name="b" pos="0 1 1" quat="0.707106781187 0 0 0.707106781187"/>
                 <geom name="g" pos="-2 0 1" quat="0 0 0 1"/>
                 <site name="s" pos="0 0 1" quat="0 0.707106781187 0.707106781187 0"/>
                 <camera name="c" pos="-1 1 2" quat="0.707106781187 0 0 0.707106781187"/>
               </worldbody></mujoco>"#,
        );
    }

    /// A frame that only moves leaves every rotation exactly as spelled —
    /// nothing is rewritten that the composition does not touch.
    #[test]
    fn a_frame_that_only_moves_keeps_the_childs_spelling() {
        assert_folds_to(
            r#"<mujoco><worldbody><body name="p">
                 <frame pos="0 0 1">
                   <geom name="g" euler="0 0 45" size="1"/>
                   <joint name="j" axis="0 1 0"/>
                 </frame>
               </body></worldbody></mujoco>"#,
            r#"<mujoco><worldbody><body name="p">
                 <geom name="g" euler="0 0 45" size="1" pos="0 0 1"/>
                 <joint name="j" axis="0 1 0" pos="0 0 1"/>
               </body></worldbody></mujoco>"#,
        );
    }

    /// A rotating frame turns the directions too, not just the poses: a
    /// joint's `axis` and a light's `dir` — including the ones MJCF left
    /// implicit — and both ends of a geom's `fromto`.
    #[test]
    fn a_rotating_frame_turns_an_axis_a_dir_and_a_fromto() {
        assert_folds_to(
            r#"<mujoco><compiler angle="radian"/><worldbody><body name="p">
                 <frame euler="0 0 1.5707963267948966">
                   <joint name="j"/>
                   <joint name="k" axis="1 0 0" pos="1 0 0"/>
                   <light name="l"/>
                   <geom name="g" fromto="0 0 0 1 0 0" size="0.1"/>
                 </frame>
               </body></worldbody></mujoco>"#,
            r#"<mujoco><compiler angle="radian"/><worldbody><body name="p">
                 <joint name="j" axis="0 0 1"/>
                 <joint name="k" axis="0 1 0" pos="0 1 0"/>
                 <light name="l" dir="0 0 -1"/>
                 <geom name="g" fromto="0 0 0 0 1 0" size="0.1"
                       quat="0.707106781187 0 0 0.707106781187"/>
               </body></worldbody></mujoco>"#,
        );
    }

    /// Nested frames compose outer-first: the outer transform reaches the
    /// inner frame's children through the inner one.
    #[test]
    fn nested_frames_compose_outer_first() {
        assert_folds_to(
            r#"<mujoco><worldbody>
                 <frame euler="0 0 90">
                   <frame pos="1 0 0">
                     <body name="b" pos="1 0 0"/>
                   </frame>
                 </frame>
               </worldbody></mujoco>"#,
            r#"<mujoco><worldbody>
                 <body name="b" pos="0 2 0" quat="0.707106781187 0 0 0.707106781187"/>
               </worldbody></mujoco>"#,
        );
    }

    /// The frame's `childclass` — or its `class`, which MuJoCo reads the
    /// same way — lands on every child that names none, a wrapped `<body>`
    /// taking it as its `childclass` so its whole subtree inherits it. A
    /// child's own wins, and an inner frame that names no class passes the
    /// outer's through.
    #[test]
    fn a_frames_class_lands_on_the_children_that_name_none() {
        let defaults = r#"<default>
                            <default class="vis"><geom rgba="1 0 0 1"/></default>
                            <default class="col"><geom rgba="0 1 0 1"/></default>
                          </default>"#;
        assert_folds_to(
            &format!(
                r#"<mujoco>{defaults}<worldbody>
                     <frame class="vis">
                       <geom name="a" size="1"/>
                       <geom name="b" size="1" class="col"/>
                       <body name="w"/>
                       <body name="own" childclass="col"/>
                       <frame pos="0 0 1"><site name="s"/></frame>
                     </frame>
                   </worldbody></mujoco>"#
            ),
            &format!(
                r#"<mujoco>{defaults}<worldbody>
                     <geom name="a" size="1" class="vis"/>
                     <geom name="b" size="1" class="col"/>
                     <body name="w" childclass="vis"/>
                     <body name="own" childclass="col"/>
                     <site name="s" pos="0 0 1" class="vis"/>
                   </worldbody></mujoco>"#
            ),
        );
    }

    /// A pose a `<default>` would have given the child is composed onto
    /// too — the frame acts on the pose the element really has, and half
    /// of Menagerie's joints take their `axis` from a class. The class in
    /// force is the child's own, else the frame's, else the enclosing
    /// body's.
    #[test]
    fn a_pose_a_class_would_have_given_is_composed_too() {
        let defaults = r#"<default><default class="hinge">
                            <joint axis="0 1 0" pos="0 0 0.5"/>
                          </default></default>"#;
        assert_folds_to(
            &format!(
                r#"<mujoco>{defaults}<worldbody><body name="p" childclass="hinge">
                     <frame euler="0 0 90"><joint name="inherited"/></frame>
                     <frame euler="0 0 90" childclass="hinge"><joint name="framed"/></frame>
                     <frame euler="0 0 90"><joint name="named" class="hinge"/></frame>
                   </body></worldbody></mujoco>"#
            ),
            &format!(
                r#"<mujoco>{defaults}<worldbody><body name="p" childclass="hinge">
                     <joint name="inherited" axis="-1 0 0" pos="0 0 0.5"/>
                     <joint name="framed" axis="-1 0 0" pos="0 0 0.5" class="hinge"/>
                     <joint name="named" axis="-1 0 0" pos="0 0 0.5" class="hinge"/>
                   </body></worldbody></mujoco>"#
            ),
        );
    }

    /// What a frame does not touch: an `<inertial>` is left exactly as
    /// written — MuJoCo does not transform it either — and a `<freejoint>`
    /// has no pose to compose.
    #[test]
    fn an_inertial_and_a_freejoint_under_a_frame_are_left_alone() {
        assert_folds_to(
            r#"<mujoco><worldbody><body name="p">
                 <frame pos="0 0 1" euler="0 0 90">
                   <inertial pos="1 0 0" mass="2" diaginertia="1 1 1"/>
                   <freejoint name="f"/>
                   <body name="c"/>
                 </frame>
               </body></worldbody></mujoco>"#,
            r#"<mujoco><worldbody><body name="p">
                 <inertial pos="1 0 0" mass="2" diaginertia="1 1 1"/>
                 <freejoint name="f"/>
                 <body name="c" pos="0 0 1" quat="0.707106781187 0 0 0.707106781187"/>
               </body></worldbody></mujoco>"#,
        );
    }

    /// A frame in an included file is folded like any other: the two
    /// halves of the pass run in one order, includes first.
    #[test]
    fn a_frame_an_included_file_brought_is_folded_too() {
        assert_flattens_to(
            &[
                (
                    "scene.xml",
                    r#"<mujoco><worldbody><include file="arm.xml"/></worldbody></mujoco>"#,
                ),
                (
                    "arm.xml",
                    r#"<mujocoinclude><frame pos="0 0 1">
                         <body name="b" pos="1 0 0"/>
                       </frame></mujocoinclude>"#,
                ),
            ],
            r#"<mujoco><worldbody><body name="b" pos="1 0 1"/></worldbody></mujoco>"#,
        );
    }

    /// The arithmetic a frame does is reported against the main file, not
    /// swallowed: a rotation spelled twice on the frame, and a `pos` that
    /// is not three numbers.
    #[test]
    fn a_frame_that_does_not_parse_is_the_main_files_error() {
        for (model, says) in [
            (
                r#"<mujoco><worldbody><frame quat="1 0 0 0" euler="0 0 90">
                     <body name="b"/></frame></worldbody></mujoco>"#,
                "two spellings of one rotation",
            ),
            (
                r#"<mujoco><worldbody><frame pos="0 0">
                     <body name="b"/></frame></worldbody></mujoco>"#,
                "expected 3 numbers",
            ),
        ] {
            let err = composed(&[("scene.xml", model)]).unwrap_err();
            let ImportError::Parse { path, message } = &err else {
                panic!("{err} is not a parse error");
            };
            assert_eq!(path, Path::new(MAIN));
            assert!(message.contains(says), "{message:?}");
        }
    }

    #[test]
    fn keys_are_normalised_but_stay_relative() {
        assert_eq!(normalized(Path::new("./a.xml")), PathBuf::from("a.xml"));
        assert_eq!(
            normalized(Path::new("sub/../a.xml")),
            PathBuf::from("a.xml")
        );
        assert_eq!(normalized(Path::new("../a.xml")), PathBuf::from("../a.xml"));
        assert_eq!(normalized(Path::new("/x/./y/../z")), PathBuf::from("/x/z"));
    }
}
