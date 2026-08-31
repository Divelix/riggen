//! MJCF → `Robot` (docs/02-data-model.md §MJCF import, ADR-0015), over the
//! reading half of [`crate::xml`].
//!
//! MJCF is a MuJoCo *scene*, not a robot description, so the import reads
//! the subset the document has fields for, names everything else in an
//! [`crate::ImportWarning`], and refuses the handful of shapes the document
//! cannot represent at all (ADR-0015 §5). Two of those shapes have to be
//! settled before a single body is read, and they are what this half of the
//! module is: [`Compiler`] — are the angles degrees, where do the meshes
//! live — and [`Defaults`], the `<default>` class tree, which decides what
//! every unqualified attribute on every `<geom>` and `<joint>` means.
//!
//! The class tree is **resolved here and dropped**: the document holds
//! resolved numbers, exactly as `resolve` hands the writers resolved numbers
//! (ADR-0004 §1, ADR-0015 §3). Re-exporting an imported foreign file
//! therefore produces a flat, class-free MJCF of the same model.

use std::collections::BTreeMap;
use std::path::PathBuf;

use crate::xml::{AngleConvention, Node, ORIENTATION_ATTRS};

/// The class an element belongs to when neither it nor an enclosing
/// `childclass` names one. MuJoCo calls the top-level `<default>` this.
pub const MAIN_CLASS: &str = "main";

/// What `<compiler>` says about the rest of the file. Read before any body
/// is, because it changes what the numbers in it mean.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Compiler {
    /// `angle` and `eulerseq`, as [`Node::orientation`] wants them.
    pub angle: AngleConvention,
    /// `meshdir`, else `assetdir`, else empty — relative to the file's own
    /// directory, or absolute.
    pub meshdir: PathBuf,
    /// `autolimits`: whether a written `range` is by itself a limit.
    pub autolimits: bool,
}

impl Default for Compiler {
    /// MuJoCo's own defaults, which is what a file that omits the element
    /// means: **degrees**, an intrinsic `xyz` sequence, no `meshdir`, and
    /// `autolimits` on (its default since 2.2.2).
    fn default() -> Self {
        Self {
            angle: AngleConvention::default(),
            meshdir: PathBuf::new(),
            autolimits: true,
        }
    }
}

impl Compiler {
    pub fn read(root: &Node) -> Result<Self, String> {
        let mut out = Self::default();
        for c in root.kids("compiler") {
            match c.attr("angle") {
                None => {}
                Some("degree") => out.angle.degrees = true,
                Some("radian") => out.angle.degrees = false,
                Some(other) => {
                    return Err(format!(
                        "<compiler angle=\"{other}\">: expected degree or radian"
                    ));
                }
            }
            if let Some(seq) = c.attr("eulerseq") {
                let b = seq.as_bytes();
                if b.len() != 3 || !b.iter().all(|c| b"xyzXYZ".contains(c)) {
                    return Err(format!(
                        "<compiler eulerseq=\"{seq}\">: expected three of xyzXYZ"
                    ));
                }
                out.angle.eulerseq = [b[0], b[1], b[2]];
            }
            // `meshdir` is the narrower of the two, so it is applied second
            // however the attributes were spelled in the file.
            if let Some(dir) = c.attr("assetdir") {
                out.meshdir = PathBuf::from(dir);
            }
            if let Some(dir) = c.attr("meshdir") {
                out.meshdir = PathBuf::from(dir);
            }
            if let Some(v) = c.flag("autolimits")? {
                out.autolimits = v;
            }
        }
        Ok(out)
    }
}

/// A class's default attributes, per element tag.
type ByTag = BTreeMap<String, BTreeMap<String, String>>;

/// The `<default>` class tree, flattened: each class already carries its
/// ancestors' attributes, so [`Defaults::apply`] is one lookup and a merge
/// rather than a walk back up.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Defaults {
    classes: BTreeMap<String, ByTag>,
}

impl Defaults {
    /// Every `<default>` under the root element. A file with none still
    /// gets an (empty) `main`, so nothing has to special-case its absence.
    pub fn read(root: &Node) -> Result<Self, String> {
        let mut out = Self::default();
        out.classes.insert(MAIN_CLASS.to_owned(), ByTag::new());
        for d in root.kids("default") {
            out.absorb(d, &ByTag::new(), MAIN_CLASS)?;
        }
        Ok(out)
    }

    /// `node`'s effective attributes: its class's defaults with its own
    /// attributes over them. The class is the one it names, else the
    /// `childclass` in force, else `main`.
    ///
    /// The result carries **no children** — it is an attribute view, and
    /// the caller walks the original node for its children.
    pub fn apply(&self, node: &Node, childclass: &str) -> Result<Node, String> {
        let class = node.attr("class").unwrap_or(childclass);
        let by_tag = self.classes.get(class).ok_or_else(|| {
            format!(
                "<{} class=\"{class}\">: no <default> declares that class",
                node.tag
            )
        })?;
        let mut attrs = by_tag.get(&node.tag).cloned().unwrap_or_default();
        // An element that spells its own rotation replaces the class's,
        // whichever of the five spellings each of them used — inheriting a
        // `quat` beside an `euler` would be two rotations, not one.
        if ORIENTATION_ATTRS
            .iter()
            .any(|a| node.attrs.contains_key(*a))
        {
            for a in ORIENTATION_ATTRS {
                attrs.remove(a);
            }
        }
        attrs.extend(node.attrs.iter().map(|(k, v)| (k.clone(), v.clone())));
        Ok(Node {
            tag: node.tag.clone(),
            attrs,
            children: Vec::new(),
        })
    }

    /// The class this element's children belong to: its own `childclass`,
    /// else the one it inherited.
    pub fn childclass<'a>(node: &'a Node, inherited: &'a str) -> &'a str {
        node.attr("childclass").unwrap_or(inherited)
    }

    fn absorb(&mut self, node: &Node, inherited: &ByTag, parent_class: &str) -> Result<(), String> {
        let class = node.attr("class").unwrap_or(parent_class);
        if class.is_empty() {
            return Err("<default class=\"\">: a class needs a name".to_owned());
        }
        let class = class.to_owned();
        let mut merged = inherited.clone();
        for child in &node.children {
            if child.tag == "default" {
                continue;
            }
            let slot = merged.entry(child.tag.clone()).or_default();
            for (k, v) in &child.attrs {
                slot.insert(k.clone(), v.clone());
            }
        }
        // A class opened twice adds to itself rather than replacing itself.
        let slot = self.classes.entry(class.clone()).or_default();
        for (tag, attrs) in &merged {
            slot.entry(tag.clone()).or_default().extend(attrs.clone());
        }
        for nested in node.kids("default") {
            self.absorb(nested, &merged, &class)?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::parse;

    /// Nested classes, a `childclass` on the body, and a `<compiler>` that
    /// disagrees with every default.
    const FILE: &str = r#"<mujoco model="d">
      <compiler angle="degree" eulerseq="zyx" assetdir="stuff" meshdir="parts" autolimits="false"/>
      <default>
        <geom type="mesh" contype="1" rgba="1 0 0 1"/>
        <joint damping="0.5" armature="0.01"/>
        <site quat="0 1 0 0"/>
        <default class="visual">
          <geom contype="0" conaffinity="0" group="2"/>
        </default>
        <default class="collision">
          <geom group="3"/>
          <default class="collision_fine">
            <geom margin="0.001" group="4"/>
          </default>
        </default>
      </default>
      <worldbody>
        <body name="base" childclass="visual">
          <joint name="j" type="hinge"/>
          <geom mesh="a"/>
          <geom class="collision_fine" mesh="a_hull"/>
          <site name="tcp" euler="90 0 0"/>
        </body>
      </worldbody>
    </mujoco>"#;

    fn attrs(node: &Node) -> Vec<(&str, &str)> {
        node.attrs
            .iter()
            .map(|(k, v)| (k.as_str(), v.as_str()))
            .collect()
    }

    #[test]
    fn the_compiler_is_read_before_anything_it_changes() {
        let c = Compiler::read(&parse(FILE).unwrap()).unwrap();
        assert!(c.angle.degrees);
        assert_eq!(c.angle.eulerseq, *b"zyx");
        // `meshdir` is narrower than `assetdir` and wins whatever the
        // attribute order was.
        assert_eq!(c.meshdir, PathBuf::from("parts"));
        assert!(!c.autolimits);

        // A file with no <compiler> means MuJoCo's defaults, not ours.
        let bare = Compiler::read(&parse("<mujoco><worldbody/></mujoco>").unwrap()).unwrap();
        assert_eq!(bare, Compiler::default());
        assert!(bare.angle.degrees, "MJCF's default is degrees");
        assert!(bare.autolimits);

        // `assetdir` alone still points the meshes somewhere.
        let asset = Compiler::read(&parse(r#"<mujoco><compiler assetdir="a"/></mujoco>"#).unwrap())
            .unwrap();
        assert_eq!(asset.meshdir, PathBuf::from("a"));

        for (text, message) in [
            (r#"<compiler angle="deg"/>"#, "expected degree or radian"),
            (r#"<compiler eulerseq="xy"/>"#, "expected three of xyzXYZ"),
            (r#"<compiler eulerseq="xyw"/>"#, "expected three of xyzXYZ"),
            (r#"<compiler autolimits="yes"/>"#, "expected true or false"),
        ] {
            let root = parse(&format!("<mujoco>{text}</mujoco>")).unwrap();
            let err = Compiler::read(&root).unwrap_err();
            assert!(err.contains(message), "{text}: {err}");
        }
    }

    #[test]
    fn a_childclass_and_nested_defaults_resolve_the_way_mujoco_would() {
        let root = parse(FILE).unwrap();
        let defaults = Defaults::read(&root).unwrap();
        let body = root.child("worldbody").unwrap().child("body").unwrap();
        // The body itself is `main` — MJCF has no `<default><body/>` — and
        // its `childclass` is what its children inherit.
        let childclass = Defaults::childclass(body, MAIN_CLASS);
        assert_eq!(childclass, "visual");

        // The joint's class has no `<joint>` of its own, so it inherits
        // `main`'s through the class tree rather than losing it.
        let joint = defaults
            .apply(body.child("joint").unwrap(), childclass)
            .unwrap();
        assert_eq!(
            attrs(&joint),
            [
                ("armature", "0.01"),
                ("damping", "0.5"),
                ("name", "j"),
                ("type", "hinge"),
            ]
        );

        let geoms: Vec<Node> = body
            .kids("geom")
            .map(|g| defaults.apply(g, childclass).unwrap())
            .collect();
        // `visual` overrides main's `contype` and adds two attributes.
        assert_eq!(
            attrs(&geoms[0]),
            [
                ("conaffinity", "0"),
                ("contype", "0"),
                ("group", "2"),
                ("mesh", "a"),
                ("rgba", "1 0 0 1"),
                ("type", "mesh"),
            ]
        );
        // An explicit `class` beats the `childclass`, and the innermost
        // class beats the one it is nested in (`group` 4, not 3).
        assert_eq!(
            attrs(&geoms[1]),
            [
                ("class", "collision_fine"),
                ("contype", "1"),
                ("group", "4"),
                ("margin", "0.001"),
                ("mesh", "a_hull"),
                ("rgba", "1 0 0 1"),
                ("type", "mesh"),
            ]
        );

        // The site spells its own rotation, so the class's `quat` goes
        // rather than joining it: one element, one rotation.
        let site = defaults
            .apply(body.child("site").unwrap(), childclass)
            .unwrap();
        assert_eq!(attrs(&site), [("euler", "90 0 0"), ("name", "tcp")]);
        // …and a site that spells none inherits the class's.
        let bare = Node {
            tag: "site".to_owned(),
            ..Default::default()
        };
        assert_eq!(defaults.apply(&bare, MAIN_CLASS).unwrap().attrs.len(), 1);
    }

    #[test]
    fn our_own_writers_defaults_read_back_and_an_unknown_class_is_refused() {
        let root = parse(crate::mjcf::tests::GOLDEN).unwrap();
        let defaults = Defaults::read(&root).unwrap();
        let geom = Node {
            tag: "geom".to_owned(),
            attrs: BTreeMap::from([("class".to_owned(), "visual".to_owned())]),
            ..Default::default()
        };
        assert_eq!(
            attrs(&defaults.apply(&geom, MAIN_CLASS).unwrap()),
            [
                ("class", "visual"),
                ("conaffinity", "0"),
                ("contype", "0"),
                ("group", "2"),
                ("type", "mesh"),
            ]
        );
        let collision = Node {
            tag: "geom".to_owned(),
            attrs: BTreeMap::from([("class".to_owned(), "collision".to_owned())]),
            ..Default::default()
        };
        assert_eq!(
            attrs(&defaults.apply(&collision, MAIN_CLASS).unwrap()),
            [
                ("class", "collision"),
                ("group", "3"),
                ("rgba", crate::mjcf::COLLISION_RGBA),
            ]
        );

        let stray = Node {
            tag: "geom".to_owned(),
            attrs: BTreeMap::from([("class".to_owned(), "nope".to_owned())]),
            ..Default::default()
        };
        assert_eq!(
            defaults.apply(&stray, MAIN_CLASS).unwrap_err(),
            "<geom class=\"nope\">: no <default> declares that class"
        );
        // A file with no <default> at all still resolves against `main`.
        let none = Defaults::read(&parse("<mujoco/>").unwrap()).unwrap();
        assert!(none.apply(&stray, MAIN_CLASS).is_err());
        assert!(
            none.apply(
                &Node {
                    tag: "geom".to_owned(),
                    ..Default::default()
                },
                MAIN_CLASS
            )
            .unwrap()
            .attrs
            .is_empty()
        );
    }
}
