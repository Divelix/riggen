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

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

use riggen_core::glam::{DMat3, DQuat, DVec3};
use riggen_core::{
    ActuatorSpec, CollisionPolicy, Dynamics, FileSource, Frame, FrameId, Geom, GeomId,
    InertialSpec, Joint, JointId, JointKind, Limits, Link, LinkId, MeshAsset, MeshId, Mimic, Pose,
    Primitive, Robot, ValidationError, validate,
};

use crate::import::{ImportError, ImportWarning, mimic_refusals};
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

/// Reads `path` through `source` and builds the document; mesh files are
/// resolved against the file's directory and its `<compiler meshdir>`, and
/// hashed through the same `source` — the filesystem natively
/// ([`riggen_core::Disk`]), the drop gesture's files in a browser
/// (ADR-0017).
pub fn load(
    path: &Path,
    source: &dyn FileSource,
) -> Result<(Robot, Vec<ImportWarning>), ImportError> {
    let io = |e: std::io::Error| ImportError::Io {
        path: path.to_owned(),
        message: e.to_string(),
    };
    let parse = |m: String| ImportError::Parse {
        path: path.to_owned(),
        message: m,
    };
    let bytes = source.read(path).map_err(io)?;
    let text = String::from_utf8(bytes).map_err(|e| parse(e.to_string()))?;
    let abs = riggen_core::absolute(path).map_err(io)?;
    let root = crate::xml::parse(&text).map_err(|e| parse(e.to_string()))?;
    from_mjcf(&root, &abs, source)
}

/// The conversion itself, for a parsed file. `path` is the model file: its
/// directory is where a relative `meshdir` and the mesh files are looked
/// for, and its name is what a parse error is reported against.
pub fn from_mjcf(
    root: &Node,
    path: &Path,
    source: &dyn FileSource,
) -> Result<(Robot, Vec<ImportWarning>), ImportError> {
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
        source,
        base_dir: path.parent().unwrap_or(Path::new(".")).to_owned(),
        compiler: Compiler::read(root).map_err(parse_err)?,
        defaults: Defaults::read(root).map_err(parse_err)?,
        robot: Robot::new(root.attr("model").unwrap_or("robot")),
        warnings: Vec::new(),
        dropped: BTreeMap::new(),
        joint_ids: BTreeMap::new(),
        assets: BTreeMap::new(),
        registered: BTreeMap::new(),
        sites: Vec::new(),
        moved_zero: BTreeSet::new(),
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
struct Import<'a> {
    path: PathBuf,
    /// Where mesh bytes come from, for the hash of every registered asset.
    source: &'a dyn FileSource,
    base_dir: PathBuf,
    compiler: Compiler,
    defaults: Defaults,
    robot: Robot,
    warnings: Vec<ImportWarning>,
    /// Element name → how many were dropped, flushed into one warning each.
    dropped: BTreeMap<String, usize>,
    /// Joint name → id, for `<equality>` and `<actuator>`, which name
    /// joints that may be anywhere in the file.
    joint_ids: BTreeMap<String, JointId>,
    /// Joints whose `<joint ref>` moved their zero. A coupling over one of
    /// them is not `q(follower) = m·q(leader) + o` in the document's terms,
    /// so it cannot be kept.
    moved_zero: BTreeSet<String>,
    /// `<asset><mesh>` by name: the file it points at, and its scale.
    assets: BTreeMap<String, (Option<String>, [f64; 3])>,
    /// (resolved path, scale) → the asset already registered for it, so a
    /// mesh used by both a visual and a collision is one `MeshAsset`.
    registered: BTreeMap<(PathBuf, u64), MeshId>,
    /// `<site>`s, held until the whole tree is read: a frame shares the
    /// links' namespace (ADR-0012), and a link further down the file may
    /// be the one that takes the name.
    sites: Vec<(LinkId, String, Pose)>,
    unnamed: usize,
}

impl Import<'_> {
    fn run(&mut self, root: &Node) -> Result<(), ImportError> {
        self.read_assets(root)?;
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
        self.place_frames();
        self.read_equalities(root)?;
        self.read_actuators(root)?;
        self.drop_what_validate_refuses();
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
        let (visuals, collision) = self.geoms(node, &name, &childclass, anchor)?;
        link.visuals = visuals;
        link.collision = collision;
        let id: LinkId = self.robot.next_id.alloc();
        self.robot.links.insert(id, link);

        // A `<site>` is a `Frame` (ADR-0012's promised symmetry), but not
        // until every link has claimed its name.
        for site in node.kids("site") {
            let site = self.resolved(site, &childclass)?;
            let pose = self.pose(&site)?;
            self.sites.push((
                id,
                site.attr("name").unwrap_or_default().to_owned(),
                Pose::new(pose.t - anchor, pose.r),
            ));
        }

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
            self.moved_zero.insert(name.clone());
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

    /// `<asset><mesh>`, by the name the geoms will use. MuJoCo names an
    /// unnamed mesh after its file's stem, and so do we.
    fn read_assets(&mut self, root: &Node) -> Result<(), ImportError> {
        for asset in root.kids("asset") {
            for m in asset.kids("mesh") {
                let m = self.resolved(m, MAIN_CLASS)?;
                let file = m.attr("file").map(str::to_owned);
                let name = match (m.attr("name"), &file) {
                    (Some(n), _) => n.to_owned(),
                    (None, Some(f)) => Path::new(f)
                        .file_stem()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into_owned(),
                    (None, None) => continue,
                };
                let scale = self.nums::<3>(&m, "scale")?.unwrap_or([1.0; 3]);
                self.assets.insert(name, (file, scale));
            }
        }
        Ok(())
    }

    /// The link's visual geoms and its collision policy.
    ///
    /// Which side a geom falls on is ADR-0015 §6: our own two class names
    /// first, then MuJoCo's own idiom for a decorative geom
    /// (`contype`/`conaffinity` both zero), then everything is a visual —
    /// never a silent loss of geometry.
    fn geoms(
        &mut self,
        body: &Node,
        link: &str,
        childclass: &str,
        anchor: DVec3,
    ) -> Result<(Vec<Geom>, CollisionPolicy), ImportError> {
        let resolved: Vec<Node> = body
            .kids("geom")
            .map(|g| self.resolved(g, childclass))
            .collect::<Result<_, _>>()?;
        let class_of = |g: &Node| g.attr("class").unwrap_or(childclass).to_owned();
        // Which *rule* applies is decided once for the link, not per geom:
        // in a file that uses `contype`, a geom that omits it is a
        // colliding one at MuJoCo's own default, not an undecided one.
        let by_class = resolved
            .iter()
            .any(|g| matches!(class_of(g).as_str(), "visual" | "collision"));
        let by_contype = resolved
            .iter()
            .any(|g| g.attrs.contains_key("contype") || g.attrs.contains_key("conaffinity"));
        let sides: Vec<bool> = resolved
            .iter()
            .map(|g| match class_of(g).as_str() {
                "visual" => Ok(true),
                "collision" => Ok(false),
                _ if by_contype => Ok(self.num(g, "contype")?.unwrap_or(1.0) == 0.0
                    && self.num(g, "conaffinity")?.unwrap_or(1.0) == 0.0),
                _ => Ok(true),
            })
            .collect::<Result<_, ImportError>>()?;
        // Whether this link's file said which of its geoms collide at all.
        let distinguished = by_class || by_contype;

        let mut visuals: Vec<Geom> = Vec::new();
        let mut visual_keys: Vec<(MeshId, Pose)> = Vec::new();
        let mut meshes: Vec<(Geom, (MeshId, Pose))> = Vec::new();
        let mut primitives: Vec<(Primitive, &'static str)> = Vec::new();
        for (g, visual) in resolved.iter().zip(sides) {
            let p = self.pose(g)?;
            let pose = Pose::new(p.t - anchor, p.r);
            // MJCF's own default, which a `<default>` class usually replaces.
            let kind = g.attr("type").unwrap_or("sphere").to_owned();
            if kind == "mesh" {
                let Some(name) = g.attr("mesh").map(str::to_owned) else {
                    self.drop_geom(link, "a <geom type=\"mesh\"> naming no mesh");
                    continue;
                };
                let Some(mesh) = self.mesh_id(&name, link)? else {
                    continue;
                };
                let color = self.nums::<4>(g, "rgba")?.map(|c| c.map(|v| v as f32));
                let id: GeomId = self.robot.next_id.alloc();
                let geom = Geom {
                    id,
                    mesh,
                    pose,
                    color: color.filter(|_| visual),
                };
                if visual {
                    visual_keys.push((mesh, pose));
                    visuals.push(geom);
                } else {
                    meshes.push((geom, (mesh, pose)));
                }
                continue;
            }
            if g.attrs.contains_key("mesh") {
                // MuJoCo would size the primitive from the mesh; fitting a
                // shape to geometry is the app's own tool, not an import.
                self.drop_geom(link, &format!("{} {kind} fitted to a mesh", article(&kind)));
                continue;
            }
            let Some((primitive, kind)) = self.primitive(g, &kind, pose, anchor)? else {
                self.drop_geom(link, &format!("{} {kind} geom", article(&kind)));
                continue;
            };
            if visual {
                self.warnings.push(ImportWarning::PrimitiveVisualDropped {
                    link: link.to_owned(),
                    kind,
                });
            } else {
                primitives.push((primitive, kind));
            }
        }

        let policy = if !meshes.is_empty() {
            for (_, kind) in &primitives {
                self.warnings.push(ImportWarning::MixedCollisionDropped {
                    link: link.to_owned(),
                    kind,
                });
            }
            let keys: Vec<(MeshId, Pose)> = meshes.iter().map(|(_, k)| *k).collect();
            if keys.len() == visual_keys.len() && keys.iter().all(|k| visual_keys.contains(k)) {
                CollisionPolicy::SameAsVisual
            } else {
                CollisionPolicy::Meshes(meshes.into_iter().map(|(g, _)| g).collect())
            }
        } else if !primitives.is_empty() {
            CollisionPolicy::Primitives(primitives.into_iter().map(|(p, _)| p).collect())
        } else if visuals.is_empty() || distinguished {
            // The file named which geoms collide, and this link has none.
            CollisionPolicy::None
        } else {
            // ADR-0015 §6's last step, for a file that never distinguished.
            CollisionPolicy::SameAsVisual
        };
        Ok((visuals, policy))
    }

    /// `size` undone from MJCF's half-extents, and `fromto` — which names
    /// the two ends of a cylinder or capsule and replaces its pose.
    fn primitive(
        &self,
        g: &Node,
        kind: &str,
        pose: Pose,
        anchor: DVec3,
    ) -> Result<Option<(Primitive, &'static str)>, ImportError> {
        // MuJoCo pads `size` to three numbers, so a sphere may carry three.
        let size = self.numbers(g, "size")?.unwrap_or_default();
        let s = |i: usize| size.get(i).copied().unwrap_or(0.0);
        let (pose, length) = match self.nums::<6>(g, "fromto")? {
            Some([x1, y1, z1, x2, y2, z2]) => {
                let (a, b) = (DVec3::new(x1, y1, z1), DVec3::new(x2, y2, z2));
                let d = b - a;
                let r = if d.length_squared() > 0.0 {
                    DQuat::from_rotation_arc(DVec3::Z, d.normalize())
                } else {
                    DQuat::IDENTITY
                };
                (Pose::new((a + b) * 0.5 - anchor, r), d.length())
            }
            None => (pose, s(1) * 2.0),
        };
        Ok(Some(match kind {
            "box" => (
                Primitive::Box {
                    pose,
                    size: DVec3::new(s(0), s(1), s(2)) * 2.0,
                },
                "box",
            ),
            "sphere" => (Primitive::Sphere { pose, radius: s(0) }, "sphere"),
            "cylinder" => (
                Primitive::Cylinder {
                    pose,
                    radius: s(0),
                    length,
                },
                "cylinder",
            ),
            "capsule" => (
                Primitive::Capsule {
                    pose,
                    radius: s(0),
                    length,
                },
                "capsule",
            ),
            _ => return Ok(None),
        }))
    }

    /// The asset a `<geom mesh>` names, registered once per file and scale.
    fn mesh_id(&mut self, name: &str, link: &str) -> Result<Option<MeshId>, ImportError> {
        let Some((file, scale)) = self.assets.get(name).cloned() else {
            self.drop_geom(link, &format!("a mesh \"{name}\" no <asset> declares"));
            return Ok(None);
        };
        let Some(file) = file else {
            self.drop_geom(link, &format!("the inline <mesh \"{name}\">"));
            return Ok(None);
        };
        if file.to_ascii_lowercase().ends_with(".msh") {
            self.drop_geom(link, &format!("\"{file}\", a MuJoCo binary mesh"));
            return Ok(None);
        }
        let [x, y, z] = scale;
        let largest = x.max(y).max(z);
        let used = if largest.is_finite() && largest > 0.0 {
            largest
        } else {
            1.0
        };
        let p = Path::new(&file);
        let path = if p.is_absolute() {
            p.to_owned()
        } else {
            self.base_dir.join(&self.compiler.meshdir).join(p)
        };
        // The one way a path enters the document: absolute and lexically
        // normalised, so `meshdir="."` is not part of it forever
        // (docs/01-architecture.md §File format).
        let path = riggen_core::absolute(&path).unwrap_or(path);
        let key = (path.clone(), used.to_bits());
        if let Some(&id) = self.registered.get(&key) {
            return Ok(Some(id));
        }
        // `used != x` also catches a scale that is zero, negative or not a
        // number, where 1 is the only thing left to use.
        if used != x || (x - y).abs() > 1e-12 || (x - z).abs() > 1e-12 {
            self.warnings.push(ImportWarning::NonUniformScale {
                link: link.to_owned(),
                file: file.clone(),
                used,
            });
        }
        let content_hash = match self.source.hash(&path) {
            Ok(h) => h,
            Err(_) => {
                self.warnings.push(ImportWarning::MeshNotFound {
                    link: link.to_owned(),
                    file,
                    tried: path.clone(),
                });
                0
            }
        };
        let id = self.robot.add_asset(MeshAsset {
            path,
            content_hash,
            scale: used,
            fix_up: None,
        });
        self.registered.insert(key, id);
        Ok(Some(id))
    }

    /// Every `<site>` that can become a `Frame`, now that the links have
    /// claimed their names. Frames and links are one namespace (ADR-0012),
    /// and so is the `<frame>_fixed` joint the URDF writer will need.
    fn place_frames(&mut self) {
        let mut taken: BTreeSet<String> =
            self.robot.links.values().map(|l| l.name.clone()).collect();
        let joints: BTreeSet<String> = self.robot.joints.values().map(|j| j.name.clone()).collect();
        for (parent, name, pose) in std::mem::take(&mut self.sites) {
            let reason = if name.is_empty() {
                Some("it has no name".to_owned())
            } else if !taken.insert(name.clone()) {
                Some("a link or frame of that name is already in the file".to_owned())
            } else if joints.contains(&format!("{name}_fixed")) {
                Some(format!("a joint is already called \"{name}_fixed\""))
            } else {
                None
            };
            match reason {
                Some(reason) => self.warnings.push(ImportWarning::FrameDropped {
                    site: if name.is_empty() {
                        "<unnamed>".to_owned()
                    } else {
                        name
                    },
                    reason,
                }),
                None => {
                    let id: FrameId = self.robot.next_id.alloc();
                    self.robot.frames.insert(id, Frame { name, parent, pose });
                }
            }
        }
    }

    fn drop_geom(&mut self, link: &str, kind: &str) {
        self.warnings.push(ImportWarning::GeomDropped {
            link: link.to_owned(),
            kind: kind.to_owned(),
        });
    }

    /// `<equality><joint polycoef>` → `Joint::mimic` (ADR-0013).
    ///
    /// `polycoef` is `y − y0 = a0 + a1(x − x0) + …` over the two joints'
    /// deviations from `qpos0`, so it is our `q(follower) = a1·q(leader) +
    /// a0` exactly when the last three terms are zero and neither joint
    /// moved its zero with a `ref`. Anything else is dropped with the
    /// reason, the way the URDF import phrases a `<mimic>` it cannot keep.
    fn read_equalities(&mut self, root: &Node) -> Result<(), ImportError> {
        for block in root.kids("equality") {
            for e in block.kids("joint") {
                let e = self.resolved(e, MAIN_CLASS)?;
                let follower = e.attr("joint1").unwrap_or_default().to_owned();
                let leader = e.attr("joint2").unwrap_or_default().to_owned();
                let [a0, a1, a2, a3, a4] = self
                    .nums::<5>(&e, "polycoef")?
                    .unwrap_or([0.0, 1.0, 0.0, 0.0, 0.0]);
                let drop = |im: &mut Self, reason: &str| {
                    im.warnings.push(ImportWarning::MimicDropped {
                        joint: follower.clone(),
                        mimics: leader.clone(),
                        reason: reason.to_owned(),
                    });
                };
                let active = e.flag("active").map_err(|m| self.parse_err(m))?;
                if active == Some(false) {
                    drop(self, "the constraint is not active");
                    continue;
                }
                if leader.is_empty() {
                    drop(self, "it holds one joint to a constant, not to another");
                    continue;
                }
                if a2 != 0.0 || a3 != 0.0 || a4 != 0.0 {
                    drop(self, "its polycoef is not linear");
                    continue;
                }
                if self.moved_zero.contains(&follower) || self.moved_zero.contains(&leader) {
                    drop(self, "a <joint ref> moved one of the two zeros");
                    continue;
                }
                let (Some(&id), true) = (
                    self.joint_ids.get(&follower),
                    self.joint_ids.contains_key(&leader),
                ) else {
                    drop(self, "no joint of that name is in the file");
                    continue;
                };
                let joint = self.joint_ids[&leader];
                self.robot.joints.get_mut(&id).expect("just walked").mimic = Some(Mimic {
                    joint,
                    multiplier: a1,
                    offset: a0,
                });
            }
        }
        Ok(())
    }

    /// `<position>` / `<velocity>` / `<motor>` on a joint → `ActuatorSpec`
    /// (ADR-0014). Its `forcerange` and `ctrlrange` are also where
    /// `Limits::effort` and `Limits::velocity` come back from: MJCF keeps
    /// them on the actuator, not on the joint.
    fn read_actuators(&mut self, root: &Node) -> Result<(), ImportError> {
        for block in root.kids("actuator") {
            for a in &block.children {
                let a = self.resolved(a, MAIN_CLASS)?;
                let name = a
                    .attr("name")
                    .unwrap_or_else(|| a.attr("joint").unwrap_or("<unnamed>"))
                    .to_owned();
                let drop = |im: &mut Self, reason: String| {
                    im.warnings.push(ImportWarning::ActuatorDropped {
                        actuator: name.clone(),
                        reason,
                    });
                };
                // Anything not driving a joint is out of the document's
                // three presets by definition (ADR-0015 §1).
                let Some(joint) = a.attr("joint").map(str::to_owned) else {
                    let target = ["tendon", "site", "body", "cranksite", "slidersite"]
                        .into_iter()
                        .find(|t| a.attrs.contains_key(*t))
                        .unwrap_or("nothing");
                    drop(self, format!("it drives a {target}, not a joint"));
                    continue;
                };
                let spec = match a.tag.as_str() {
                    "position" => ActuatorSpec::Position {
                        kp: self.num(&a, "kp")?.unwrap_or(1.0),
                        kv: self.num(&a, "kv")?.unwrap_or(0.0),
                    },
                    "velocity" => ActuatorSpec::Velocity {
                        kv: self.num(&a, "kv")?.unwrap_or(1.0),
                    },
                    // `gear` is six numbers; for a joint only the first
                    // scales it.
                    "motor" => ActuatorSpec::Motor {
                        gear: self
                            .numbers(&a, "gear")?
                            .and_then(|g| g.first().copied())
                            .unwrap_or(1.0),
                    },
                    other => {
                        drop(self, format!("<{other}> is not one of the three presets"));
                        continue;
                    }
                };
                let Some(&id) = self.joint_ids.get(&joint) else {
                    drop(self, format!("no joint \"{joint}\" is in the file"));
                    continue;
                };
                let force = self.nums::<2>(&a, "forcerange")?;
                let ctrl = self.nums::<2>(&a, "ctrlrange")?;
                let j = self.robot.joints.get_mut(&id).expect("just walked");
                j.actuator = Some(spec);
                if let Some(limits) = &mut j.limits {
                    // Both are written as ±v and read back as the upper
                    // half; a zero one was never filled in (ADR-0014).
                    if let Some([_, upper]) = force {
                        limits.effort = upper;
                    }
                    if let (ActuatorSpec::Velocity { .. }, Some([_, upper])) = (spec, ctrl) {
                        limits.velocity = upper;
                    }
                }
            }
        }
        Ok(())
    }

    /// A coupling or an actuator `validate` refuses is dropped with its
    /// reason rather than failing the whole import — the rule the URDF
    /// import already follows, so a file still opens and the user is told.
    fn drop_what_validate_refuses(&mut self) {
        // Couplings first: an actuator on a mimic follower is refused
        // because the `<equality>` already drives it (ADR-0014), and a
        // coupling that is itself dropped never drove anything.
        for (follower, reason) in mimic_refusals(&self.robot) {
            let leader = self
                .robot
                .joints
                .get_mut(&follower)
                .and_then(|j| j.mimic.take())
                .map(|m| m.joint);
            let mimics = leader
                .and_then(|l| self.robot.joints.get(&l))
                .map(|j| j.name.clone())
                .unwrap_or_default();
            self.warnings.push(ImportWarning::MimicDropped {
                joint: self.robot.joints[&follower].name.clone(),
                mimics,
                reason,
            });
        }
        for (joint, reason) in actuator_refusals(&self.robot) {
            self.robot
                .joints
                .get_mut(&joint)
                .expect("named by validate")
                .actuator = None;
            self.warnings.push(ImportWarning::ActuatorDropped {
                actuator: self.robot.joints[&joint].name.clone(),
                reason,
            });
        }
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

    fn numbers(&self, node: &Node, name: &str) -> Result<Option<Vec<f64>>, ImportError> {
        node.numbers(name).map_err(|m| self.parse_err(m))
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

/// What `validate` refuses about an actuator, per joint (ADR-0014). As
/// with the couplings, `validate` owns the rules and this only phrases its
/// verdict; every other error it reports still fails the import.
fn actuator_refusals(robot: &Robot) -> Vec<(JointId, String)> {
    riggen_core::validation_errors(robot)
        .into_iter()
        .filter_map(|e| match e {
            ValidationError::ActuatorOnFixedJoint(j) => {
                Some((j, "a fixed joint has no <joint> for it to drive".to_owned()))
            }
            ValidationError::ActuatorOnMimicFollower { joint, .. } => Some((
                joint,
                "the joint is already driven by an <equality>".to_owned(),
            )),
            ValidationError::InvalidActuatorGain { joint, what } => {
                Some((joint, format!("its {what}")))
            }
            _ => None,
        })
        .collect()
}

/// "a" or "an", so a warning about an ellipsoid reads like a sentence.
fn article(word: &str) -> &'static str {
    if word.starts_with(['a', 'e', 'i', 'o', 'u']) {
        "an"
    } else {
        "a"
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
    use crate::resolve::{ResolvedGeom, ResolvedRobot};
    use crate::xml::parse;
    use crate::{ComputeNow, ExportOptions, Format, MeshStore, export, resolve};
    use riggen_core::{Disk, JointState, fk};

    /// Writes `robot`'s MJCF and its meshes into a scratch directory and
    /// reads the `.xml` back — the acceptance route, in one function.
    fn round_trip(robot: &Robot, store: &MeshStore, tag: &str) -> (Robot, Vec<ImportWarning>) {
        let dir = std::env::temp_dir().join(format!("riggen-mjcf-in-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let files = export(&resolved(robot, store), &options(), &dir).unwrap();
        let text = std::fs::read_to_string(&files[0]).unwrap();
        from_mjcf(&parse(&text).unwrap(), &files[0], &Disk).unwrap()
    }

    fn options() -> ExportOptions {
        ExportOptions {
            format: Format::MJCF,
            ..Default::default()
        }
    }

    fn resolved(robot: &Robot, store: &MeshStore) -> ResolvedRobot {
        resolve(robot, store, &ComputeNow, &options()).unwrap()
    }

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
        from_mjcf(&parse(text).unwrap(), Path::new("/nowhere/m.xml"), &Disk)
    }

    /// A model with `body` as the whole of its `<worldbody>`.
    fn model(body: &str) -> String {
        // MuJoCo refuses a `class` no `<default>` declares, and so do we,
        // so the wrapper declares the two the tests reach for.
        let cube = crate::test_util::fixtures().join("cube_binary.stl");
        format!(
            r#"<mujoco model="m"><compiler angle="radian"/>
                 <default><default class="visual"/><default class="collision"/></default>
                 <asset><mesh name="m" file="{}"/></asset>
                 <worldbody>{body}</worldbody></mujoco>"#,
            cube.display()
        )
    }

    #[test]
    fn every_joint_kind_comes_back_as_the_same_tree_and_the_same_fk() {
        let b = crate::test_util::every_joint_kind();
        let (robot, warnings) = round_trip(&b.robot, &b.store, "every-joint-kind");
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
            assert_eq!(
                joint(name).limits.map(|l| (l.lower, l.upper)),
                Some((-1.0, 1.0)),
                "{name}"
            );
        }
        // The coupling and the two actuators (ADR-0013, ADR-0014).
        let upper = *robot
            .joints
            .iter()
            .find(|(_, j)| j.name == "upper_joint")
            .unwrap()
            .0;
        assert_eq!(
            joint("slider_joint").mimic,
            Some(Mimic {
                joint: upper,
                multiplier: -0.5,
                offset: 0.1
            })
        );
        assert_eq!(joint("upper_joint").mimic, None);
        assert_eq!(
            joint("upper_joint").actuator,
            Some(ActuatorSpec::Position { kp: 100.0, kv: 5.0 })
        );
        assert_eq!(
            joint("wheel_joint").actuator,
            Some(ActuatorSpec::Velocity { kv: 2.0 })
        );
        assert_eq!(
            joint("slider_joint").actuator,
            None,
            "a mimic follower carries none, and the writer wrote none"
        );
        // `effort` and `velocity` live on the `<actuator>`, not the joint
        // (ADR-0004 §4 as amended by ADR-0014), so they come back only
        // where an actuator carried them: `forcerange` is the hinge's
        // effort, its `ctrlrange` is the *position* range and says nothing
        // about velocity, and the slider — which has no actuator at all —
        // keeps neither.
        assert_eq!(joint("upper_joint").limits.unwrap().effort, 1.0);
        assert_eq!(joint("upper_joint").limits.unwrap().velocity, 0.0);
        let slider = joint("slider_joint").limits.unwrap();
        assert_eq!((slider.effort, slider.velocity), (0.0, 0.0));
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
        // configurations, couplings and all — `fk` resolves the mimic
        // through the one implementation of ADR-0013's rule.
        let original = b.robot.clone();
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

    /// The foreign corpus (ADR-0015): a file nobody wrote with our writer,
    /// carrying every shape the import has to cope with. What it loses is
    /// pinned warning by warning, because "nothing is dropped silently" is
    /// only true if somebody checks.
    #[test]
    fn the_menagerie_style_corpus_imports_with_the_warnings_it_should() {
        let path = crate::test_util::fixtures().join("menagerie_style.xml");
        let (robot, warnings) = super::load(&path, &Disk).unwrap();
        let link = |n: &str| robot.links.values().find(|l| l.name == n).unwrap();
        let joint = |n: &str| robot.joints.values().find(|j| j.name == n).unwrap();

        assert_eq!(robot.name, "menagerie_style");
        assert_eq!(robot.links.len(), 5);
        assert_eq!(robot.links[&robot.root].name, "base_link");
        // `<compiler angle="degree">` is MJCF's default and the opposite of
        // ours: ±180° is ±π, and the class two levels up is where the range
        // came from at all.
        let pan = joint("shoulder_pan").limits.unwrap();
        assert!((pan.lower + std::f64::consts::PI).abs() < 1e-12);
        assert!((pan.upper - std::f64::consts::PI).abs() < 1e-12);
        // …and `damping` / `armature` / `frictionloss` from the root class.
        assert_eq!(
            joint("shoulder_pan").dynamics,
            Dynamics {
                damping: 0.1,
                friction: 0.02,
                armature: 0.01
            }
        );
        assert_eq!(joint("wrist_slide").kind, JointKind::Prismatic);
        assert_eq!(
            joint("wrist_slide").limits.map(|l| (l.lower, l.upper)),
            Some((0.0, 0.05))
        );

        // Each body spells its rotation a different way, and all four
        // reach the document as one `DQuat`.
        let rot = |n: &str| joint(n).origin.r;
        // `dot`, not `angle_between`: `acos` near 1 loses eight digits.
        let same = |a: DQuat, b: DQuat| a.dot(b).abs() > 1.0 - 1e-12;
        let quarter = std::f64::consts::FRAC_PI_4;
        assert!(
            same(rot("shoulder_pan"), DQuat::from_rotation_z(quarter)),
            "euler=\"0 0 45\""
        );
        assert!(
            same(
                rot("shoulder_lift"),
                DQuat::from_rotation_y(30f64.to_radians())
            ),
            "axisangle=\"0 1 0 30\""
        );
        assert!(
            same(
                rot("wrist_slide"),
                DQuat::from_mat3(&DMat3::from_cols(DVec3::X, DVec3::Z, -DVec3::Y))
            ),
            "xyaxes=\"1 0 0 0 0 1\""
        );
        assert!(
            same(
                rot("tool_joint"),
                DQuat::from_rotation_arc(DVec3::Z, -DVec3::Y)
            ),
            "zaxis=\"0 -1 0\""
        );
        // The `<site quat>` written by our own writer is the fifth
        // spelling, and `xml::tests` holds it to the same rotation.
        // The `<site euler>` too, on a frame that survived.
        let mut frames: Vec<&str> = robot.frames.values().map(|f| f.name.as_str()).collect();
        frames.sort_unstable();
        assert_eq!(frames, ["mount", "tcp"]);

        // `class="arm_visual"` is not our class name, so the split fell
        // through to `contype`/`conaffinity` (ADR-0015 §6, rule 2).
        assert_eq!(link("base_link").visuals.len(), 1);
        assert!(
            matches!(link("base_link").collision, CollisionPolicy::Meshes(ref g) if g.len() == 1),
            "the collision mesh has its own `quat`, so it is not the visual"
        );
        assert_eq!(
            link("shoulder").collision,
            CollisionPolicy::None,
            "the file said which geoms collide and this link has none"
        );
        // `fromto` names the capsule's two ends.
        let CollisionPolicy::Primitives(p) = &link("upper").collision else {
            panic!("{:?}", link("upper").collision)
        };
        assert_eq!(
            p[..],
            [Primitive::Capsule {
                pose: Pose::from_translation(DVec3::Z * 0.05),
                radius: 0.02,
                length: 0.1
            }]
        );
        // The mesh scale is non-uniform, and the meshes are beside the arm.
        for asset in robot.assets.values() {
            assert!(asset.path.exists(), "{}", asset.path.display());
        }
        assert_eq!(
            joint("shoulder_pan").actuator,
            Some(ActuatorSpec::Position {
                kp: 120.0,
                kv: 12.0
            })
        );
        assert_eq!(joint("shoulder_pan").limits.unwrap().effort, 30.0);
        assert_eq!(
            joint("shoulder_lift").mimic.map(|m| m.multiplier),
            Some(0.25)
        );

        assert_eq!(
            warnings,
            vec![
                // The shoulder's mesh is scaled 1:2:1, and the document
                // holds one number.
                ImportWarning::NonUniformScale {
                    link: "shoulder".to_owned(),
                    file: "shoulder.stl".to_owned(),
                    used: 0.002
                },
                // Then the `<geom>`s of one body, in document order.
                ImportWarning::PrimitiveVisualDropped {
                    link: "wrist".to_owned(),
                    kind: "box"
                },
                ImportWarning::GeomDropped {
                    link: "wrist".to_owned(),
                    kind: "an ellipsoid geom".to_owned()
                },
                // Then the two actuators outside the three presets.
                ImportWarning::ActuatorDropped {
                    actuator: "lift".to_owned(),
                    reason: "<general> is not one of the three presets".to_owned()
                },
                ImportWarning::ActuatorDropped {
                    actuator: "grip".to_owned(),
                    reason: "it drives a tendon, not a joint".to_owned()
                },
                // Then one line per element name, whatever the count.
                ImportWarning::ElementDropped {
                    element: "<camera>".to_owned(),
                    count: 1
                },
                ImportWarning::ElementDropped {
                    element: "<contact>".to_owned(),
                    count: 1
                },
                ImportWarning::ElementDropped {
                    element: "<keyframe>".to_owned(),
                    count: 1
                },
                ImportWarning::ElementDropped {
                    element: "<light>".to_owned(),
                    count: 1
                },
                ImportWarning::ElementDropped {
                    element: "<material>".to_owned(),
                    count: 1
                },
                ImportWarning::ElementDropped {
                    element: "<option>".to_owned(),
                    count: 1
                },
                ImportWarning::ElementDropped {
                    element: "<sensor>".to_owned(),
                    count: 1
                },
                ImportWarning::ElementDropped {
                    element: "<tendon>".to_owned(),
                    count: 1
                },
                ImportWarning::ElementDropped {
                    element: "<texture>".to_owned(),
                    count: 1
                },
            ]
        );
    }

    #[test]
    fn a_coupling_and_an_actuator_the_document_cannot_hold_are_named() {
        let (robot, warnings) = load(
            r#"<mujoco model="m"><compiler angle="radian"/><worldbody>
                 <body name="a">
                   <body name="b"><joint name="j" range="-1 1"/>
                     <body name="c"><joint name="k" range="-1 1"/>
                       <body name="d"><joint name="l" ref="0.2" range="-1 1"/></body>
                     </body>
                   </body>
                 </body>
               </worldbody>
               <equality>
                 <joint joint1="k" joint2="j" polycoef="0 2 0 0 0"/>
                 <joint joint1="k" joint2="j" polycoef="0 1 0.5 0 0"/>
                 <joint joint1="l" joint2="j"/>
                 <joint joint1="k" joint2="j" active="false"/>
                 <joint joint1="k" joint2="nope"/>
                 <joint joint1="k" polycoef="0 1 0 0 0"/>
                 <weld body1="a" body2="b"/>
               </equality>
               <actuator>
                 <motor name="drive" joint="j" gear="50 0 0 0 0 0" forcerange="-7 7"/>
                 <velocity name="rate" joint="k" kv="3" ctrlrange="-4 4"/>
                 <general name="fancy" joint="j" dyntype="filter"/>
                 <position name="tendon_servo" tendon="t" kp="9"/>
                 <motor name="ghost" joint="missing"/>
               </actuator>
               </mujoco>"#,
        )
        .unwrap();
        let joint = |n: &str| robot.joints.values().find(|j| j.name == n).unwrap();
        // The last coupling written for a follower is the one that stands,
        // and only the linear, active, `ref`-free one survives at all.
        assert_eq!(joint("k").mimic.map(|m| (m.multiplier, m.offset)), None);
        assert_eq!(
            joint("j").actuator,
            Some(ActuatorSpec::Motor { gear: 50.0 })
        );
        assert_eq!(joint("j").limits.unwrap().effort, 7.0);
        assert_eq!(
            joint("k").actuator,
            Some(ActuatorSpec::Velocity { kv: 3.0 })
        );
        // A velocity servo *is* commanded in the joint's own rate, so its
        // `ctrlrange` is where `Limits::velocity` comes back from.
        assert_eq!(joint("k").limits.unwrap().velocity, 4.0);

        let said: Vec<String> = warnings.iter().map(ToString::to_string).collect();
        for line in [
            "joint \"k\": <mimic joint=\"j\"> dropped, its polycoef is not linear",
            "joint \"l\": <mimic joint=\"j\"> dropped, a <joint ref> moved one of the two zeros",
            "joint \"k\": <mimic joint=\"j\"> dropped, the constraint is not active",
            "joint \"k\": <mimic joint=\"nope\"> dropped, no joint of that name is in the file",
            "joint \"k\": <mimic joint=\"\"> dropped, it holds one joint to a constant, not to another",
            "actuator \"fancy\" dropped, <general> is not one of the three presets",
            "actuator \"tendon_servo\" dropped, it drives a tendon, not a joint",
            "actuator \"ghost\" dropped, no joint \"missing\" is in the file",
            "<weld> × 1: nothing in the document holds it; not read",
            "<joint ref> × 1: nothing in the document holds it; not read",
        ] {
            assert!(
                said.contains(&line.to_owned()),
                "missing {line:?}\n{said:#?}"
            );
        }
        // The one coupling that was fine is the one `validate` then refused
        // — `k` would reach ±2, outside its own ±1 — and it says so.
        assert!(
            said.iter()
                .any(|s| s.contains("it would reach -2..2, outside its own limits")),
            "{said:#?}"
        );
    }

    #[test]
    fn an_actuator_validate_refuses_is_dropped_rather_than_failing_the_file() {
        let (robot, warnings) = load(
            r#"<mujoco model="m"><compiler angle="radian"/><worldbody>
                 <body name="a">
                   <body name="b"><joint name="j" range="-1 1"/>
                     <body name="c"><joint name="k" range="-1 1"/>
                       <body name="d"/>
                     </body>
                   </body>
                 </body>
               </worldbody>
               <equality><joint joint1="k" joint2="j" polycoef="0 1 0 0 0"/></equality>
               <actuator>
                 <position name="follower" joint="k" kp="10"/>
                 <position name="welded" joint="d_joint" kp="10"/>
                 <position name="sour" joint="j" kp="-1"/>
               </actuator>
               </mujoco>"#,
        )
        .unwrap();
        // The file opens; every actuator the document cannot hold is gone
        // with its reason, and the coupling that motivated the first one
        // stays.
        for j in robot.joints.values() {
            assert_eq!(j.actuator, None, "{}", j.name);
        }
        assert!(robot.joints.values().any(|j| j.mimic.is_some()));
        let said: Vec<String> = warnings.iter().map(ToString::to_string).collect();
        for line in [
            "actuator \"k\" dropped, the joint is already driven by an <equality>",
            "actuator \"d_joint\" dropped, a fixed joint has no <joint> for it to drive",
        ] {
            assert!(
                said.contains(&line.to_owned()),
                "missing {line:?}\n{said:#?}"
            );
        }
        assert!(
            said.iter()
                .any(|s| s.starts_with("actuator \"j\" dropped, its kp")),
            "{said:#?}"
        );
    }

    #[test]
    fn the_arms_own_export_comes_back_with_the_same_geometry() {
        let (arm, _) = riggen_core::load(&crate::test_util::fixtures().join("arm/arm.riggen"))
            .expect("the sample document");
        let (store, errors) = MeshStore::load(&arm, &Disk);
        assert!(errors.is_empty(), "{errors:?}");
        let (back, warnings) = round_trip(&arm, &store, "arm");
        assert_eq!(warnings, vec![], "our own MJCF holds nothing unreadable");

        // Every frame is back as a frame, which is the symmetry ADR-0012
        // promised and the URDF import deliberately does not have.
        let names = |r: &Robot| {
            let mut n: Vec<String> = r.frames.values().map(|f| f.name.clone()).collect();
            n.sort();
            n
        };
        assert!(!arm.frames.is_empty(), "the fixture has frames to lose");
        assert_eq!(names(&back), names(&arm));

        // The real question is not what the document looks like — a hull
        // policy comes back as the meshes it produced — but whether the
        // *geometry* is the same. So: resolve both and compare.
        let (back_store, errors) = MeshStore::load(&back, &Disk);
        assert!(errors.is_empty(), "{errors:?}");
        same_geometry(&resolved(&arm, &store), &resolved(&back, &back_store));
    }

    #[test]
    fn a_decomposition_comes_back_as_the_collision_meshes_it_produced() {
        let (bracket, _) = riggen_core::load(&crate::test_util::fixtures().join("bracket.riggen"))
            .expect("the sample document");
        let (store, errors) = MeshStore::load(&bracket, &Disk);
        assert!(errors.is_empty(), "{errors:?}");
        let pieces = resolved(&bracket, &store)
            .links
            .iter()
            .map(|l| l.collisions.len())
            .max()
            .unwrap_or(0);
        assert!(pieces > 1, "the fixture decomposes into several pieces");

        let (back, warnings) = round_trip(&bracket, &store, "bracket");
        assert_eq!(warnings, vec![]);
        // The parameters are gone — they were parameters, never geometry
        // (ADR-0011) — and what is left is the N meshes they produced.
        let link = back.links.values().find(|l| !l.visuals.is_empty()).unwrap();
        match &link.collision {
            CollisionPolicy::Meshes(geoms) => assert_eq!(geoms.len(), pieces),
            other => panic!("{other:?}"),
        }
        let (back_store, errors) = MeshStore::load(&back, &Disk);
        assert!(errors.is_empty(), "{errors:?}");
        same_geometry(&resolved(&bracket, &store), &resolved(&back, &back_store));
    }

    #[track_caller]
    fn same_geometry(want: &ResolvedRobot, got: &ResolvedRobot) {
        assert_eq!(want.links.len(), got.links.len());
        for (a, b) in want.links.iter().zip(&got.links) {
            assert_eq!(a.name, b.name);
            for (side, x, y) in [
                ("visual", &a.visuals, &b.visuals),
                ("collision", &a.collisions, &b.collisions),
            ] {
                assert_eq!(x.len(), y.len(), "{} {side}", a.name);
                for (g, h) in x.iter().zip(y) {
                    match (g, h) {
                        (
                            ResolvedGeom::Mesh {
                                name: n, pose: p, ..
                            },
                            ResolvedGeom::Mesh {
                                name: m, pose: q, ..
                            },
                        ) => {
                            assert_eq!(n, m, "{} {side}", a.name);
                            assert_close(*p, *q, &format!("{} {side} {n}", a.name));
                        }
                        (ResolvedGeom::Primitive(p), ResolvedGeom::Primitive(q)) => {
                            assert_eq!(format!("{p:?}"), format!("{q:?}"), "{}", a.name)
                        }
                        _ => panic!("{} {side}: {g:?} became {h:?}", a.name),
                    }
                }
            }
            assert_eq!(a.sites.len(), b.sites.len(), "{} sites", a.name);
            for (s, t) in a.sites.iter().zip(&b.sites) {
                assert_eq!(s.name, t.name);
                assert_close(s.pose, t.pose, &s.name);
            }
        }
    }

    #[track_caller]
    fn assert_close(a: Pose, b: Pose, what: &str) {
        // The quaternion made a round trip through twelve decimals, so this
        // is a tolerance and not an equality.
        assert!((a.t - b.t).length() < 1e-11, "{what}: {a:?} vs {b:?}");
        assert!(a.r.dot(b.r).abs() > 1.0 - 1e-11, "{what}: {a:?} vs {b:?}");
    }

    #[test]
    fn the_visual_collision_split_falls_back_the_way_adr_0015_says() {
        // 1. Our own class names decide when they are there.
        let (robot, _) = load(&model(
            r#"<body name="a">
                 <geom class="visual" type="mesh" mesh="m"/>
                 <geom class="collision" type="mesh" mesh="m" pos="0 0 1"/>
               </body>"#,
        ))
        .unwrap();
        let link = robot.links.values().next().unwrap();
        assert_eq!(link.visuals.len(), 1);
        assert!(matches!(link.collision, CollisionPolicy::Meshes(ref g) if g.len() == 1));

        // 2. Failing those, MuJoCo's own idiom for a decorative geom.
        let (robot, _) = load(&model(
            r#"<body name="a">
                 <geom type="mesh" mesh="m" contype="0" conaffinity="0"/>
                 <geom type="box" size="1 1 1"/>
               </body>"#,
        ))
        .unwrap();
        let link = robot.links.values().next().unwrap();
        assert_eq!(link.visuals.len(), 1);
        assert!(matches!(
            link.collision,
            CollisionPolicy::Primitives(ref p) if matches!(p[..], [Primitive::Box { .. }])
        ));

        // 3. Failing both, every geom is a visual and the link collides
        // with itself — never a silent loss of geometry.
        let (robot, _) = load(&model(
            r#"<body name="a"><geom type="mesh" mesh="m"/></body>"#,
        ))
        .unwrap();
        let link = robot.links.values().next().unwrap();
        assert_eq!(link.visuals.len(), 1);
        assert_eq!(link.collision, CollisionPolicy::SameAsVisual);

        // …and a link whose file *did* distinguish, with nothing on the
        // collision side, collides with nothing.
        let (robot, _) = load(&model(
            r#"<body name="a"><geom class="visual" type="mesh" mesh="m"/></body>"#,
        ))
        .unwrap();
        assert_eq!(
            robot.links.values().next().unwrap().collision,
            CollisionPolicy::None
        );
    }

    #[test]
    fn primitive_sizes_are_undone_and_the_rest_is_named() {
        let (robot, warnings) = load(&model(
            r#"<body name="a">
                 <geom class="collision" type="box" size="0.05 0.1 0.15" pos="0 0 1"/>
                 <geom class="collision" type="cylinder" size="0.02 0.25"/>
                 <geom class="collision" type="sphere" size="0.03 0 0"/>
                 <geom class="collision" type="capsule" size="0.04" fromto="0 0 0 0 0 0.6"/>
                 <geom class="collision" type="ellipsoid" size="1 2 3"/>
                 <geom class="visual" type="box" size="1 1 1"/>
               </body>"#,
        ))
        .unwrap();
        let CollisionPolicy::Primitives(p) = &robot.links.values().next().unwrap().collision else {
            panic!("{:?}", robot.links.values().next().unwrap().collision);
        };
        // MJCF `size` is half of what the document holds — the classic
        // mistake, pinned here in the other direction too.
        assert_eq!(
            p[0],
            Primitive::Box {
                pose: Pose::from_translation(DVec3::Z),
                size: DVec3::new(0.1, 0.2, 0.3)
            }
        );
        assert_eq!(
            p[1],
            Primitive::Cylinder {
                pose: Pose::IDENTITY,
                radius: 0.02,
                length: 0.5
            }
        );
        assert_eq!(
            p[2],
            Primitive::Sphere {
                pose: Pose::IDENTITY,
                radius: 0.03
            }
        );
        // `fromto` names the two ends and replaces the pose with the
        // midpoint and the rotation onto that direction.
        assert_eq!(
            p[3],
            Primitive::Capsule {
                pose: Pose::from_translation(DVec3::Z * 0.3),
                radius: 0.04,
                length: 0.6
            }
        );
        assert_eq!(p.len(), 4);
        assert_eq!(
            warnings,
            vec![
                ImportWarning::NoInertial {
                    link: "a".to_owned()
                },
                ImportWarning::GeomDropped {
                    link: "a".to_owned(),
                    kind: "an ellipsoid geom".to_owned()
                },
                ImportWarning::PrimitiveVisualDropped {
                    link: "a".to_owned(),
                    kind: "box"
                },
            ]
        );
    }

    #[test]
    fn a_mesh_asset_is_found_through_meshdir_and_named_when_it_is_not() {
        let dir = riggen_core::absolute(&crate::test_util::fixtures().join("arm")).unwrap();
        let text = r#"<mujoco model="m">
                 <compiler angle="radian" meshdir="."/>
                 <default><default class="collision"/></default>
                 <asset>
                   <mesh name="base" file="base.stl" scale="0.001 0.001 0.001"/>
                   <mesh file="upper.stl"/>
                   <mesh name="lumpy" file="base.stl" scale="1 2 3"/>
                   <mesh name="binary" file="thing.msh"/>
                   <mesh name="gone" file="nowhere.stl"/>
                 </asset>
                 <worldbody><body name="a">
                   <geom class="collision" type="mesh" mesh="base"/>
                   <geom class="collision" type="mesh" mesh="upper"/>
                   <geom class="collision" type="mesh" mesh="lumpy"/>
                   <geom class="collision" type="mesh" mesh="binary"/>
                   <geom class="collision" type="mesh" mesh="gone"/>
                   <geom class="collision" type="mesh" mesh="never_declared"/>
                 </body></worldbody>
               </mujoco>"#;
        let (robot, warnings) =
            from_mjcf(&parse(text).unwrap(), &dir.join("m.xml"), &Disk).unwrap();
        // An unnamed `<mesh>` is known by its file's stem, as in MuJoCo.
        let CollisionPolicy::Meshes(geoms) = &robot.links.values().next().unwrap().collision else {
            panic!()
        };
        assert_eq!(geoms.len(), 4, "the .msh and the undeclared one are gone");
        let asset = |g: &Geom| &robot.assets[&g.mesh];
        assert_eq!(asset(&geoms[0]).path, dir.join("base.stl"));
        assert_eq!(asset(&geoms[0]).scale, 0.001);
        assert_ne!(asset(&geoms[0]).content_hash, 0, "the file was read");
        assert_eq!(asset(&geoms[1]).path, dir.join("upper.stl"));
        assert_eq!(asset(&geoms[2]).scale, 3.0, "the largest component");
        assert_eq!(asset(&geoms[3]).content_hash, 0, "nothing to hash");
        assert_eq!(
            warnings,
            vec![
                ImportWarning::NoInertial {
                    link: "a".to_owned()
                },
                ImportWarning::NonUniformScale {
                    link: "a".to_owned(),
                    file: "base.stl".to_owned(),
                    used: 3.0
                },
                ImportWarning::GeomDropped {
                    link: "a".to_owned(),
                    kind: "\"thing.msh\", a MuJoCo binary mesh".to_owned()
                },
                ImportWarning::MeshNotFound {
                    link: "a".to_owned(),
                    file: "nowhere.stl".to_owned(),
                    tried: dir.join("nowhere.stl")
                },
                ImportWarning::GeomDropped {
                    link: "a".to_owned(),
                    kind: "a mesh \"never_declared\" no <asset> declares".to_owned()
                },
            ]
        );
    }

    #[test]
    fn a_site_becomes_a_frame_unless_the_name_is_taken() {
        let (robot, warnings) = load(&model(
            r#"<body name="a">
                 <site name="tcp" pos="0 0 0.05"/>
                 <site pos="1 0 0"/>
                 <site name="b"/>
                 <site name="tcp"/>
                 <body name="b"><joint name="j" range="-1 1"/></body>
               </body>"#,
        ))
        .unwrap();
        assert_eq!(robot.frames.len(), 1);
        let frame = robot.frames.values().next().unwrap();
        assert_eq!(frame.name, "tcp");
        assert_eq!(frame.pose, Pose::from_translation(DVec3::Z * 0.05));
        assert_eq!(frame.parent, robot.root);
        assert_eq!(
            warnings,
            vec![
                ImportWarning::FrameDropped {
                    site: "<unnamed>".to_owned(),
                    reason: "it has no name".to_owned()
                },
                // A link further down the file took the name, which is why
                // the sites wait for the whole tree (ADR-0012).
                ImportWarning::FrameDropped {
                    site: "b".to_owned(),
                    reason: "a link or frame of that name is already in the file".to_owned()
                },
                ImportWarning::FrameDropped {
                    site: "tcp".to_owned(),
                    reason: "a link or frame of that name is already in the file".to_owned()
                },
            ]
        );
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
                // No class and no `contype`, so it is a visual — and
                // visuals are meshes in the document (ADR-0015 §6).
                ImportWarning::PrimitiveVisualDropped {
                    link: "a".to_owned(),
                    kind: "box"
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
            warnings[4].to_string(),
            "joint \"s\" has no range and the document has no unlimited prismatic; -1..1 used"
        );
        assert_eq!(
            warnings[8].to_string(),
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
