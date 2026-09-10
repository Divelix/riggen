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

use std::collections::BTreeSet;
use std::path::{Component, Path, PathBuf};

use riggen_core::FileSource;

use crate::import::ImportError;
use crate::xml::Node;

/// The composition-free tree the reader reads: `root` — the parsed main
/// file — with every `<include>` under it spliced. `path` is the main
/// file, whose directory is where an included file is looked for first;
/// `source` is where its bytes come from, the same one the meshes come
/// through (ADR-0017). A pure tree → tree rewrite: no document types, no
/// import state, and the main root's own attributes (`model`) survive.
pub(crate) fn compose(
    root: Node,
    path: &Path,
    source: &dyn FileSource,
) -> Result<Node, ImportError> {
    let mut includes = Includes {
        main_dir: path.parent().unwrap_or(Path::new(".")).to_owned(),
        source,
        seen: BTreeSet::new(),
    };
    includes.splice(root, path)
}

/// The `<include>` half of the pass.
struct Includes<'a> {
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
                node.children.push(self.splice(child, from)?);
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
