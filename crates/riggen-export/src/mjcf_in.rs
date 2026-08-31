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
use std::path::{Path, PathBuf};

use riggen_core::glam::{DMat3, DQuat, DVec3};
use riggen_core::{
    Dynamics, InertialSpec, Joint, JointId, JointKind, Limits, Link, LinkId, Pose, Robot, validate,
};

use crate::import::{ImportError, ImportWarning};
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

/// Every element the import reads. Anything else in the file is counted and
/// named once per tag in an [`ImportWarning::ElementDropped`] (ADR-0015 §1)
/// — one warning with a count, not one per `<geom rgba>` in a Menagerie
/// model.
const READ: &[&str] = &[
    "mujoco",
    "compiler",
    "default",
    "asset",
    "mesh",
    "worldbody",
    "body",
    "joint",
    "freejoint",
    "inertial",
    "geom",
    "site",
    "equality",
    "actuator",
    "position",
    "velocity",
    "motor",
    // Read only far enough to name the actuator we are dropping (step 6).
    "general",
    "muscle",
    "adhesion",
];

/// Elements that compose other files or re-shape the tree. Reading around
/// them would silently lose bodies, so the file is refused (ADR-0015 §5).
/// `<frame>` is here for that second reason: it is a transform wrapper, and
/// the bodies inside one are not children of any body.
const REFUSED: &[&str] = &["include", "replicate", "attach", "frame"];

/// The conversion itself, for a parsed file. `path` is the model file: its
/// directory is where a relative `meshdir` and the mesh files are looked
/// for, and its name is what a parse error is reported against.
pub fn from_mjcf(root: &Node, path: &Path) -> Result<(Robot, Vec<ImportWarning>), ImportError> {
    if root.tag != "mujoco" {
        return Err(ImportError::Parse {
            path: path.to_owned(),
            message: format!("<{}> is not a <mujoco> model", root.tag),
        });
    }
    refuse(root)?;
    let parse_err = |m: String| ImportError::Parse {
        path: path.to_owned(),
        message: m,
    };
    let mut im = Import {
        path: path.to_owned(),
        base_dir: path.parent().unwrap_or(Path::new(".")).to_owned(),
        compiler: Compiler::read(root).map_err(parse_err)?,
        defaults: Defaults::read(root).map_err(parse_err)?,
        robot: Robot::new(root.attr("model").unwrap_or("robot")),
        warnings: Vec::new(),
        dropped: BTreeMap::new(),
        joint_ids: BTreeMap::new(),
        unnamed: 0,
    };
    im.robot.links.clear();
    im.run(root)?;
    Ok((im.robot, im.warnings))
}

/// `<include>` and friends, anywhere in the file (ADR-0015 §5).
fn refuse(node: &Node) -> Result<(), ImportError> {
    for c in &node.children {
        if REFUSED.contains(&c.tag.as_str()) {
            return Err(ImportError::UnsupportedElement {
                element: format!("<{}>", c.tag),
            });
        }
        // Removed from MJCF years ago, but old files carry it, and it means
        // every pose in them is something else entirely.
        if c.tag == "compiler" && c.attr("coordinate") == Some("global") {
            return Err(ImportError::UnsupportedElement {
                element: "<compiler coordinate=\"global\">".to_owned(),
            });
        }
        refuse(c)?;
    }
    Ok(())
}

/// One import in progress.
struct Import {
    path: PathBuf,
    #[allow(dead_code, reason = "the meshes arrive in the next step")]
    base_dir: PathBuf,
    compiler: Compiler,
    defaults: Defaults,
    robot: Robot,
    warnings: Vec<ImportWarning>,
    /// Element name → how many were dropped, flushed into one warning each.
    dropped: BTreeMap<String, usize>,
    /// Joint name → id, for `<equality>` and `<actuator>`, which name
    /// joints that may be anywhere in the file.
    #[allow(dead_code, reason = "the blocks after </worldbody> arrive in step 6")]
    joint_ids: BTreeMap<String, JointId>,
    unnamed: usize,
}

impl Import {
    fn run(&mut self, root: &Node) -> Result<(), ImportError> {
        let world = root.child("worldbody").ok_or(ImportError::NoRoot)?;
        let bodies: Vec<&Node> = world.kids("body").collect();
        match bodies[..] {
            [] => return Err(ImportError::NoRoot),
            [only] => self.robot.root = self.body(only, None, MAIN_CLASS)?,
            _ => {
                return Err(ImportError::MultipleRoots(
                    bodies
                        .iter()
                        .map(|b| b.attr("name").unwrap_or("<unnamed>").to_owned())
                        .collect(),
                ));
            }
        }
        count_dropped(root, &mut self.dropped);
        for (element, count) in std::mem::take(&mut self.dropped) {
            self.warnings
                .push(ImportWarning::ElementDropped { element, count });
        }
        validate(&self.robot).map_err(ImportError::Invalid)
    }

    /// One `<body>` and everything under it. `parent` carries the parent
    /// link **and its joint anchor**, because every pose inside a body is
    /// written against the body frame, and the link frame may have moved
    /// off it (see [`Import::joint`]).
    fn body(
        &mut self,
        node: &Node,
        parent: Option<(LinkId, DVec3)>,
        inherited: &str,
    ) -> Result<LinkId, ImportError> {
        let childclass = Defaults::childclass(node, inherited).to_owned();
        let name = self.body_name(node);

        // The joints this body carries, and the three shapes ADR-0015 §5
        // refuses rather than half-importing.
        let mut joints: Vec<&Node> = Vec::new();
        for j in node
            .children
            .iter()
            .filter(|c| c.tag == "joint" || c.tag == "freejoint")
        {
            if j.tag == "freejoint" || j.attr("type") == Some("free") {
                if parent.is_some() {
                    // Welding a floating body to its parent would change the
                    // kinematics; on the root it only drops a boolean.
                    return Err(ImportError::UnsupportedJoint {
                        joint: joint_name(j, &name),
                        kind: "free".to_owned(),
                    });
                }
                self.warnings
                    .push(ImportWarning::FreeJointDropped { body: name.clone() });
                continue;
            }
            joints.push(j);
        }
        if joints.len() > 1 {
            return Err(ImportError::CompositeJoint {
                body: name.clone(),
                joints: joints.iter().map(|j| joint_name(j, &name)).collect(),
            });
        }
        let joint_node = match joints.first() {
            None => None,
            Some(j) => {
                if parent.is_none() {
                    return Err(ImportError::JointOnRoot {
                        body: name.clone(),
                        joint: joint_name(j, &name),
                    });
                }
                Some(self.resolved(j, &childclass)?)
            }
        };
        let anchor = match &joint_node {
            Some(j) => self.vec3(j, "pos")?.unwrap_or(DVec3::ZERO),
            None => DVec3::ZERO,
        };

        let mut link = Link::new(name.clone());
        link.inertial = self.inertial(node, &name, anchor)?;
        let id: LinkId = self.robot.next_id.alloc();
        self.robot.links.insert(id, link);

        match parent {
            None => {
                // The root body's own placement is where the robot sits in
                // the world, which is not a thing the document holds.
                if node.attrs.contains_key("pos")
                    || ORIENTATION_ATTRS
                        .iter()
                        .any(|a| node.attrs.contains_key(*a))
                {
                    self.drop("the root <body>'s own pos/quat");
                }
            }
            Some((p, parent_anchor)) => {
                let jname = joints
                    .first()
                    .map_or_else(|| format!("{name}_joint"), |j| joint_name(j, &name));
                let joint = self.joint(
                    joint_node.as_ref(),
                    node,
                    jname,
                    p,
                    id,
                    parent_anchor,
                    anchor,
                )?;
                let jid: JointId = self.robot.next_id.alloc();
                self.joint_ids.insert(joint.name.clone(), jid);
                self.robot.joints.insert(jid, joint);
            }
        }

        for child in node.kids("body") {
            self.body(child, Some((id, anchor)), &childclass)?;
        }
        Ok(id)
    }

    /// The edge from `parent` to `child`.
    ///
    /// MJCF anchors a joint at `pos` in the **body** frame; the document's
    /// joint frame *is* the child link frame (02 §Conventions). So the link
    /// frame is the body frame moved to the anchor, `origin` carries that
    /// move, and everything else the body holds is re-expressed against it
    /// by subtracting `anchor`. A file that leaves `pos` out — ours always
    /// does — moves nothing.
    #[allow(clippy::too_many_arguments)]
    fn joint(
        &mut self,
        jn: Option<&Node>,
        body: &Node,
        name: String,
        parent: LinkId,
        child: LinkId,
        parent_anchor: DVec3,
        anchor: DVec3,
    ) -> Result<Joint, ImportError> {
        let body_pose = self.pose(body)?;
        let origin = Pose::new(
            body_pose.t - parent_anchor + body_pose.r * anchor,
            body_pose.r,
        );
        let Some(jn) = jn else {
            // MJCF writes no element for a fixed edge, so its name is
            // invented — the `<link>_joint` our own documents tend to carry.
            return Ok(Joint {
                origin,
                ..Joint::fixed(name, parent, child)
            });
        };

        let conv = self.compiler.angle;
        let range = self.nums::<2>(jn, "range")?;
        let limited = match jn.flag("limited").map_err(|m| self.parse_err(m))? {
            Some(v) => v,
            // `autolimits` is what turns a written range into a limit, and
            // MuJoCo reads `0 0` as no range at all.
            None => self.compiler.autolimits && range.is_some_and(|[a, b]| a != 0.0 || b != 0.0),
        };
        let range = range.filter(|_| limited);
        let span = |lower: f64, upper: f64| Limits {
            lower,
            upper,
            // MJCF keeps these on the `<actuator>`, not the joint (ADR-0014).
            effort: 0.0,
            velocity: 0.0,
        };
        let (kind, limits) = match jn.attr("type").unwrap_or("hinge") {
            "hinge" => match range {
                Some([a, b]) => (
                    JointKind::Revolute,
                    Some(span(conv.radians(a), conv.radians(b))),
                ),
                None => (JointKind::Continuous, None),
            },
            "slide" => match range {
                Some([a, b]) => (JointKind::Prismatic, Some(span(a, b))),
                None => {
                    self.warnings.push(ImportWarning::LimitsInvented {
                        joint: name.clone(),
                        lower: -1.0,
                        upper: 1.0,
                    });
                    (JointKind::Prismatic, Some(span(-1.0, 1.0)))
                }
            },
            other => {
                return Err(ImportError::UnsupportedJoint {
                    joint: name,
                    kind: other.to_owned(),
                });
            }
        };
        // `ref` moves the joint's zero, which the document has no field for
        // and which would quietly shift the whole subtree.
        if jn.attrs.contains_key("ref") {
            self.drop("<joint ref>");
        }
        Ok(Joint {
            name,
            kind,
            parent,
            child,
            origin,
            axis: self.vec3(jn, "axis")?.unwrap_or(DVec3::Z),
            limits,
            dynamics: Dynamics {
                damping: self.num(jn, "damping")?.unwrap_or(0.0),
                friction: self.num(jn, "frictionloss")?.unwrap_or(0.0),
                armature: self.num(jn, "armature")?.unwrap_or(0.0),
            },
            mimic: None,
            actuator: None,
        })
    }

    /// `<inertial>`, in link axes about the CoM. No element — or one with
    /// no mass or no tensor — leaves `Computed` and says so: MuJoCo's
    /// `inertiafromgeom` fallback density is a number the user never chose
    /// (ADR-0015 §7).
    fn inertial(
        &mut self,
        body: &Node,
        link: &str,
        anchor: DVec3,
    ) -> Result<InertialSpec, ImportError> {
        let computed = InertialSpec::Computed {
            density_override: None,
        };
        fn unweighed(im: &mut Import, body: &Node, link: &str) {
            if body.kids("geom").next().is_some() {
                im.warnings.push(ImportWarning::NoInertial {
                    link: link.to_owned(),
                });
                if body
                    .kids("geom")
                    .any(|g| g.attrs.contains_key("mass") || g.attrs.contains_key("density"))
                {
                    im.warnings.push(ImportWarning::MassFromGeomIgnored {
                        link: link.to_owned(),
                    });
                }
            }
        }
        let Some(i) = body.child("inertial") else {
            unweighed(self, body, link);
            return Ok(computed);
        };
        let mass = self.num(i, "mass")?.unwrap_or(0.0);
        let inertia = if let Some([xx, yy, zz, xy, xz, yz]) = self.nums::<6>(i, "fullinertia")? {
            // MuJoCo's order, already in body axes about the CoM: what our
            // own writer emits, read straight back.
            DMat3::from_cols(
                DVec3::new(xx, xy, xz),
                DVec3::new(xy, yy, yz),
                DVec3::new(xz, yz, zz),
            )
        } else if let Some(d) = self.nums::<3>(i, "diaginertia")? {
            // Principal moments, with the element's own rotation naming the
            // axes they are about.
            let r = DMat3::from_quat(
                i.orientation(self.compiler.angle)
                    .map_err(|m| self.parse_err(m))?
                    .unwrap_or(DQuat::IDENTITY),
            );
            r * DMat3::from_diagonal(DVec3::from_array(d)) * r.transpose()
        } else {
            unweighed(self, body, link);
            return Ok(computed);
        };
        if mass <= 0.0 {
            unweighed(self, body, link);
            return Ok(computed);
        }
        Ok(InertialSpec::Override {
            mass,
            com: self.vec3(i, "pos")?.unwrap_or(DVec3::ZERO) - anchor,
            inertia,
        })
    }

    fn pose(&self, node: &Node) -> Result<Pose, ImportError> {
        Ok(Pose::new(
            self.vec3(node, "pos")?.unwrap_or(DVec3::ZERO),
            node.orientation(self.compiler.angle)
                .map_err(|m| self.parse_err(m))?
                .unwrap_or(DQuat::IDENTITY),
        ))
    }

    fn resolved(&self, node: &Node, childclass: &str) -> Result<Node, ImportError> {
        self.defaults
            .apply(node, childclass)
            .map_err(|m| self.parse_err(m))
    }

    fn num(&self, node: &Node, name: &str) -> Result<Option<f64>, ImportError> {
        node.num(name).map_err(|m| self.parse_err(m))
    }

    fn nums<const N: usize>(
        &self,
        node: &Node,
        name: &str,
    ) -> Result<Option<[f64; N]>, ImportError> {
        node.nums::<N>(name).map_err(|m| self.parse_err(m))
    }

    fn vec3(&self, node: &Node, name: &str) -> Result<Option<DVec3>, ImportError> {
        node.vec3(name).map_err(|m| self.parse_err(m))
    }

    fn parse_err(&self, message: String) -> ImportError {
        ImportError::Parse {
            path: self.path.clone(),
            message,
        }
    }

    fn drop(&mut self, element: &str) {
        *self.dropped.entry(element.to_owned()).or_default() += 1;
    }

    fn body_name(&mut self, node: &Node) -> String {
        match node.attr("name") {
            Some(n) => n.to_owned(),
            None => {
                self.unnamed += 1;
                format!("body{}", self.unnamed)
            }
        }
    }
}

/// The name a `<joint>` carries, or the one it is given: MJCF lets an
/// element go unnamed, the document does not.
fn joint_name(j: &Node, body: &str) -> String {
    j.attr("name")
        .map(str::to_owned)
        .unwrap_or_else(|| format!("{body}_joint"))
}

/// Counts every element the import does not read, by tag. A dropped
/// element's children are part of it and are not counted again.
fn count_dropped(node: &Node, out: &mut BTreeMap<String, usize>) {
    for c in &node.children {
        if READ.contains(&c.tag.as_str()) {
            count_dropped(c, out);
        } else {
            *out.entry(format!("<{}>", c.tag)).or_default() += 1;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::xml::parse;
    use riggen_core::{JointState, fk};

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

    fn load(text: &str) -> Result<(Robot, Vec<ImportWarning>), ImportError> {
        from_mjcf(&parse(text).unwrap(), Path::new("/nowhere/m.xml"))
    }

    /// A model with `body` as the whole of its `<worldbody>`.
    fn model(body: &str) -> String {
        format!(
            r#"<mujoco model="m"><compiler angle="radian"/><worldbody>{body}</worldbody></mujoco>"#
        )
    }

    #[test]
    fn every_joint_kind_comes_back_as_the_same_tree_and_the_same_fk() {
        let b = crate::test_util::every_joint_kind();
        let xml = crate::mjcf::write(&b.resolve().unwrap(), &crate::ExportOptions::default());
        let (robot, warnings) = load(&xml).unwrap();
        assert_eq!(
            warnings,
            vec![],
            "our own MJCF holds nothing we cannot read"
        );
        assert_eq!(robot.name, "test");

        let link = |n: &str| robot.links.values().find(|l| l.name == n).unwrap();
        let joint = |n: &str| robot.joints.values().find(|j| j.name == n).unwrap();
        assert_eq!(robot.links.len(), 5);
        assert_eq!(robot.links[&robot.root].name, "base_link");
        for name in ["base_link", "upper", "slider", "wheel", "tip"] {
            assert_eq!(link(name).name, name);
        }
        // The fixed edge has no `<joint>` element at all, so its name is
        // the invented `<body>_joint` — which is the one the document had.
        assert_eq!(robot.joints.len(), 4);
        for (name, kind) in [
            ("upper_joint", JointKind::Revolute),
            ("slider_joint", JointKind::Prismatic),
            ("wheel_joint", JointKind::Continuous),
            ("tip_joint", JointKind::Fixed),
        ] {
            let j = joint(name);
            assert_eq!(j.kind, kind, "{name}");
            assert_eq!(j.origin, Pose::from_translation(DVec3::Z * 0.1), "{name}");
        }
        assert_eq!(joint("upper_joint").axis, DVec3::Y);
        assert_eq!(joint("upper_joint").dynamics.damping, 0.1);
        assert_eq!(joint("wheel_joint").limits, None, "Continuous keeps none");
        for name in ["upper_joint", "slider_joint"] {
            let l = joint(name).limits.unwrap();
            assert_eq!((l.lower, l.upper), (-1.0, 1.0), "{name}");
            // `effort` and `velocity` live on the `<actuator>`, not the
            // joint (ADR-0004 §4 as amended): a joint that has one gets
            // them back in step 6, one that has none — the slider — cannot.
            assert_eq!((l.effort, l.velocity), (0.0, 0.0), "{name}");
        }
        // `<inertial pos mass fullinertia>` read straight back.
        assert_eq!(
            link("upper").inertial,
            InertialSpec::Override {
                mass: 2.7,
                com: DVec3::ZERO,
                inertia: DMat3::from_diagonal(DVec3::splat(0.0045)),
            }
        );
        // An empty static body was written with no `<inertial>` and comes
        // back `Computed`, with nothing to warn about (ADR-0015 §7).
        assert!(matches!(
            link("tip").inertial,
            InertialSpec::Computed { .. }
        ));

        // The oracle (ADR-0004): the same world pose per link at five
        // configurations. The couplings are cleared on both sides —
        // `<equality>` is step 6 — so this compares the tree, not ADR-0013.
        let mut original = b.robot.clone();
        for j in original.joints.values_mut() {
            j.mimic = None;
        }
        let state = |r: &Robot, q: f64| {
            let mut s = JointState::new();
            for (&id, j) in &r.joints {
                if j.kind.is_movable() {
                    s.set(id, q);
                }
            }
            s
        };
        for q in [0.0, 0.3, -0.7, 1.0, -1.0] {
            let want = fk(&original, &state(&original, q));
            let got = fk(&robot, &state(&robot, q));
            for (id, pose) in &want {
                let name = &original.links[id].name;
                let mine = robot.links.iter().find(|(_, l)| &l.name == name).unwrap().0;
                let p = got[mine];
                assert!(
                    (pose.t - p.t).length() < 1e-12,
                    "{name} at q={q}: {pose:?} {p:?}"
                );
                assert!(pose.r.dot(p.r).abs() > 1.0 - 1e-12, "{name} at q={q}");
            }
        }
    }

    #[test]
    fn a_joint_anchor_moves_the_link_frame_rather_than_being_ignored() {
        // MJCF turns `arm` about the axis through its `pos`; the document
        // turns it about the child link frame. So the link frame moves
        // there, and everything inside the body is re-expressed against it.
        let (robot, warnings) = load(&model(
            r#"<body name="base">
                 <body name="arm" pos="0 0 1">
                   <joint name="j" type="hinge" axis="0 1 0" pos="0 0 0.5" range="-3 3"/>
                   <inertial pos="0 0 0.5" mass="1" diaginertia="1 1 1"/>
                   <body name="tip" pos="0 0 1"/>
                 </body>
               </body>"#,
        ))
        .unwrap();
        assert_eq!(warnings, vec![]);
        let joint = |n: &str| robot.joints.values().find(|j| j.name == n).unwrap();
        let link = |n: &str| robot.links.values().find(|l| l.name == n).unwrap();
        assert_eq!(joint("j").origin.t, DVec3::new(0.0, 0.0, 1.5));
        assert_eq!(joint("tip_joint").origin.t, DVec3::new(0.0, 0.0, 0.5));
        assert_eq!(
            link("arm").inertial,
            InertialSpec::Override {
                mass: 1.0,
                com: DVec3::ZERO,
                inertia: DMat3::IDENTITY,
            }
        );
        // A quarter turn about +Y takes the tip, which MuJoCo would swing
        // about the world point (0, 0, 1.5), to (0.5, 0, 1.5).
        let mut q = JointState::new();
        let jid = *robot.joints.iter().find(|(_, j)| j.name == "j").unwrap().0;
        q.set(jid, std::f64::consts::FRAC_PI_2);
        let poses = fk(&robot, &q);
        let tip = *robot.links.iter().find(|(_, l)| l.name == "tip").unwrap().0;
        assert!(
            (poses[&tip].t - DVec3::new(0.5, 0.0, 1.5)).length() < 1e-12,
            "{:?}",
            poses[&tip].t
        );
    }

    #[test]
    fn the_shapes_the_document_cannot_hold_are_refused_by_name() {
        assert_eq!(
            load(&model(
                r#"<body name="a"><body name="w"><joint name="w0"/><joint name="w1"/></body></body>"#
            ))
            .unwrap_err(),
            ImportError::CompositeJoint {
                body: "w".to_owned(),
                joints: vec!["w0".to_owned(), "w1".to_owned()],
            }
        );
        assert_eq!(
            load(&model(r#"<body name="a"><joint name="j"/></body>"#)).unwrap_err(),
            ImportError::JointOnRoot {
                body: "a".to_owned(),
                joint: "j".to_owned(),
            }
        );
        // A free joint below the root would weld a floating body to its
        // parent; on the root it only costs an export option.
        assert_eq!(
            load(&model(
                r#"<body name="a"><body name="b"><freejoint name="f"/></body></body>"#
            ))
            .unwrap_err(),
            ImportError::UnsupportedJoint {
                joint: "f".to_owned(),
                kind: "free".to_owned(),
            }
        );
        assert_eq!(
            load(&model(
                r#"<body name="a"><body name="b"><joint name="j" type="ball"/></body></body>"#
            ))
            .unwrap_err(),
            ImportError::UnsupportedJoint {
                joint: "j".to_owned(),
                kind: "ball".to_owned(),
            }
        );
        assert_eq!(
            load(&model(r#"<body name="a"/><body name="b"/>"#)).unwrap_err(),
            ImportError::MultipleRoots(vec!["a".to_owned(), "b".to_owned()])
        );
        assert_eq!(load(&model("")).unwrap_err(), ImportError::NoRoot);
        assert_eq!(load("<mujoco/>").unwrap_err(), ImportError::NoRoot);
        for (text, element) in [
            (r#"<mujoco><include file="x.xml"/></mujoco>"#, "<include>"),
            (
                r#"<mujoco><worldbody><frame pos="0 0 1"><body name="a"/></frame></worldbody></mujoco>"#,
                "<frame>",
            ),
            (
                r#"<mujoco><compiler coordinate="global"/></mujoco>"#,
                "<compiler coordinate=\"global\">",
            ),
        ] {
            assert_eq!(
                load(text).unwrap_err(),
                ImportError::UnsupportedElement {
                    element: element.to_owned()
                }
            );
        }
        assert!(matches!(
            load("<robot name=\"a\"/>").unwrap_err(),
            ImportError::Parse { .. }
        ));
    }

    #[test]
    fn what_the_document_cannot_hold_is_counted_and_named() {
        let (robot, warnings) = load(
            r#"<mujoco model="m">
                 <compiler angle="radian"/>
                 <worldbody>
                   <body name="a">
                     <freejoint/>
                     <geom type="box" size="1 1 1" mass="3"/>
                     <light/>
                     <camera name="eye"/>
                     <body name="b">
                       <joint name="j" ref="0.2" range="-1 1"/>
                       <body name="c"><joint name="s" type="slide"/></body>
                     </body>
                   </body>
                 </worldbody>
                 <sensor><jointpos joint="j"/></sensor>
               </mujoco>"#,
        )
        .unwrap();
        assert_eq!(robot.links.len(), 3);
        assert_eq!(
            warnings,
            vec![
                // Dropped while the tree is walked, in the order it is walked.
                ImportWarning::FreeJointDropped {
                    body: "a".to_owned()
                },
                ImportWarning::NoInertial {
                    link: "a".to_owned()
                },
                ImportWarning::MassFromGeomIgnored {
                    link: "a".to_owned()
                },
                ImportWarning::LimitsInvented {
                    joint: "s".to_owned(),
                    lower: -1.0,
                    upper: 1.0,
                },
                // …then one line per element name, however many there were.
                ImportWarning::ElementDropped {
                    element: "<camera>".to_owned(),
                    count: 1
                },
                ImportWarning::ElementDropped {
                    element: "<joint ref>".to_owned(),
                    count: 1
                },
                ImportWarning::ElementDropped {
                    element: "<light>".to_owned(),
                    count: 1
                },
                // The `<jointpos>` inside it is part of it, not a second line.
                ImportWarning::ElementDropped {
                    element: "<sensor>".to_owned(),
                    count: 1
                },
            ]
        );
        // Every warning says what and why, for the status bar.
        assert_eq!(
            warnings[3].to_string(),
            "joint \"s\" has no range and the document has no unlimited prismatic; -1..1 used"
        );
        assert_eq!(
            warnings[7].to_string(),
            "<sensor> × 1: nothing in the document holds it; not read"
        );
    }

    #[test]
    fn degrees_are_the_default_and_the_compiler_is_believed() {
        let range = |compiler: &str| {
            let (robot, _) = load(&format!(
                r#"<mujoco model="m">{compiler}<worldbody><body name="a">
                     <body name="b"><joint name="j" range="-90 90"/></body>
                   </body></worldbody></mujoco>"#
            ))
            .unwrap();
            robot
                .joints
                .values()
                .find(|j| j.name == "j")
                .unwrap()
                .limits
                .unwrap()
        };
        let half = std::f64::consts::FRAC_PI_2;
        assert_eq!(range("").lower, -half, "MJCF's default is degrees");
        assert_eq!(range(r#"<compiler angle="degree"/>"#).lower, -half);
        assert_eq!(range(r#"<compiler angle="radian"/>"#).lower, -90.0);
        // With `autolimits` off a range is not a limit, so the hinge is
        // `Continuous` and carries none.
        let (robot, _) = load(
            r#"<mujoco model="m"><compiler autolimits="false"/><worldbody><body name="a">
                 <body name="b"><joint name="j" range="-90 90"/></body>
               </body></worldbody></mujoco>"#,
        )
        .unwrap();
        let j = robot.joints.values().find(|j| j.name == "j").unwrap();
        assert_eq!(j.kind, JointKind::Continuous);
        assert_eq!(j.limits, None);
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
