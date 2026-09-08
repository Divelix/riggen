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
    Actuator, ActuatorId, ActuatorRanges, ActuatorSpec, ActuatorTarget, BiasType, CollisionPolicy,
    DynType, Dynamics, FileSource, Frame, FrameId, GainType, General, Geom, GeomId, InertialSpec,
    Joint, JointId, JointKind, Limits, Link, LinkId, MeshAsset, MeshId, Mimic, Pose, Primitive,
    Robot, Tendon, TendonId, TendonJoint, ValidationError, content_hash, validate,
};
use riggen_mesh::TriMesh;

use crate::import::{ImportError, ImportWarning, mimic_refusals};
use crate::xml::{AngleConvention, Node, ORIENTATION_ATTRS};

/// The class an element belongs to when neither it nor an enclosing
/// `childclass` names one. MuJoCo calls the top-level `<default>` this.
pub const MAIN_CLASS: &str = "main";

/// [`load`] / [`from_mjcf`]'s result: the document, its warnings, and one
/// `(filename, bytes)` pair per inline `<mesh vertex face>` the file
/// declared, for the caller to place beside the source (docs/02-data-model.md
/// §Geometry).
pub type MjcfImport = (Robot, Vec<ImportWarning>, Vec<(String, Vec<u8>)>);

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
    // The tendon block and its two children: a `<fixed>` is read, a
    // `<spatial>` only far enough to name the tendon being dropped
    // (ADR-0025 §4). `<equality><tendon>` is a different element of the
    // same name and is counted — see [`read_here`].
    "tendon",
    "fixed",
    "spatial",
    "actuator",
    "position",
    "velocity",
    "motor",
    "general",
    // MuJoCo's other actuator shortcuts: read only far enough to name the
    // actuator being dropped (ADR-0024), once, as an actuator.
    "muscle",
    "adhesion",
    "intvelocity",
    "damper",
    "cylinder",
];

/// The attributes any `<actuator>` element may carry that the document
/// reads whatever the tag: its name and class, the joint or tendon it
/// drives, and the four range fields (ADR-0024 §3, ADR-0025 §4). `group`
/// is decorative and silent.
const ACTUATOR_COMMON: &[&str] = &[
    "name",
    "class",
    "group",
    "joint",
    "tendon",
    "ctrllimited",
    "forcelimited",
    "ctrlrange",
    "forcerange",
];

/// Per tag: the attributes its preset expresses, and the ones that make
/// the element a `General` instead because the preset cannot hold them
/// but MuJoCo's own model can (ADR-0024 §2). `<general>` expresses its
/// whole vocabulary and desugars nothing.
const PRESET_ATTRS: &[(&str, &[&str], &[&str])] = &[
    ("position", &["kp", "kv"], &["gear", "timeconst"]),
    ("velocity", &["kv"], &["gear"]),
    ("motor", &["gear"], &[]),
    (
        "general",
        &[
            "dyntype", "gaintype", "biastype", "dynprm", "gainprm", "biasprm", "gear",
        ],
        &[],
    ),
];

/// What an actuator drives when it is neither a joint nor a tendon, as
/// the drop message names it. `jointinparent` is a joint, but a different
/// transmission.
const ACTUATOR_TARGETS: &[(&str, &str)] = &[
    ("site", "a site"),
    ("body", "a body"),
    ("cranksite", "a cranksite"),
    ("slidersite", "a slidersite"),
    ("jointinparent", "a joint through jointinparent"),
];

/// What a `<tendon><fixed>` carries that the document keeps (ADR-0025
/// §4): its name and class, the range it is bounded by, and its passive
/// dynamics.
const TENDON_ATTRS: &[&str] = &[
    "name",
    "class",
    "limited",
    "range",
    "stiffness",
    "damping",
    "frictionloss",
];

/// Decorative on a tendon the way `rgba` is on a geom: read by nothing,
/// counted by nothing. Everything else a `<fixed>` carries —
/// `springlength`, the four `sol*` pairs, `margin`, `armature`, `user` —
/// is counted once per attribute name (ADR-0025 §4, on ADR-0024 §4's
/// pattern).
const TENDON_SILENT: &[&str] = &["group", "rgba", "width"];

/// Elements that compose other files or re-shape the tree. Reading around
/// them would silently lose bodies, so the file is refused (ADR-0015 §5).
/// `<frame>` is here for that second reason: it is a transform wrapper, and
/// the bodies inside one are not children of any body.
const REFUSED: &[&str] = &["include", "replicate", "attach", "frame"];

/// Reads `path` through `source` and builds the document; mesh files are
/// resolved against the file's directory and its `<compiler meshdir>`, and
/// hashed through the same `source` — the filesystem natively
/// ([`riggen_core::Disk`]), the drop gesture's files in a browser
/// (ADR-0017). The third element is one `(name, bytes)` pair per inline
/// `<mesh vertex face>` the file declared — the caller writes each beside
/// `path` (or, with nowhere to write, adds it under that name to its own
/// drop set), which is where every `MeshAsset` this import produced for
/// one already points (docs/02-data-model.md §Geometry).
pub fn load(path: &Path, source: &dyn FileSource) -> Result<MjcfImport, ImportError> {
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
/// for, and its name is what a parse error is reported against. See
/// [`load`] for the third element of the result.
pub fn from_mjcf(
    root: &Node,
    path: &Path,
    source: &dyn FileSource,
) -> Result<MjcfImport, ImportError> {
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
        tendon_ids: BTreeMap::new(),
        assets: BTreeMap::new(),
        registered: BTreeMap::new(),
        sites: Vec::new(),
        unnamed: 0,
        inline_hash: BTreeMap::new(),
        inline_meshes: Vec::new(),
        unnamed_meshes: 0,
    };
    im.robot.links.clear();
    im.run(root)?;
    Ok((im.robot, im.warnings, im.inline_meshes))
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
    /// Joint name → id, for `<equality>`, `<tendon>` and `<actuator>`,
    /// which name joints that may be anywhere in the file.
    joint_ids: BTreeMap<String, JointId>,
    /// Tendon name → id, for the `<actuator>`s that drive one. A tendon
    /// that was dropped is absent, so an actuator on it is dropped too.
    tendon_ids: BTreeMap<String, TendonId>,
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
    /// Asset name → content hash, for an inline `<mesh vertex face>`: its
    /// bytes are already known (`inline_meshes`, below) rather than
    /// waiting on disk for [`FileSource::hash`] to find.
    inline_hash: BTreeMap<String, u64>,
    /// `(synthesized filename, STL bytes)` for every inline mesh, handed
    /// back to the caller to place beside the source file (step 2's
    /// decision, docs/02-data-model.md §Geometry).
    inline_meshes: Vec<(String, Vec<u8>)>,
    /// Counts unnamed inline meshes, for `inline_N`.
    unnamed_meshes: usize,
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
        self.read_tendons(root)?;
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
        // `ref` is the `qpos` at which the body sits at its authored pose
        // (ADR-0025 §3): an angle on a hinge, a length on a slide, held as
        // `Joint::qpos_ref`. `range` is in `qpos` terms too, and
        // `Limits` is in the document's `q` — the deviation from that
        // pose — so it is shifted by `ref` on the way in; the writer shifts
        // it back. An invented ±1 m is already a deviation and stays.
        let ref_ = self.num(jn, "ref")?.unwrap_or(0.0);
        let span = |lower: f64, upper: f64, qpos_ref: f64| Limits {
            lower: lower - qpos_ref,
            upper: upper - qpos_ref,
            // MJCF keeps these on the `<actuator>`, not the joint (ADR-0014).
            effort: 0.0,
            velocity: 0.0,
        };
        let (kind, limits, qpos_ref) = match jn.attr("type").unwrap_or("hinge") {
            "hinge" => {
                let qpos_ref = conv.radians(ref_);
                match range {
                    Some([a, b]) => (
                        JointKind::Revolute,
                        Some(span(conv.radians(a), conv.radians(b), qpos_ref)),
                        qpos_ref,
                    ),
                    None => (JointKind::Continuous, None, qpos_ref),
                }
            }
            "slide" => match range {
                Some([a, b]) => (JointKind::Prismatic, Some(span(a, b, ref_)), ref_),
                None => {
                    self.warnings.push(ImportWarning::LimitsInvented {
                        joint: name.clone(),
                        lower: -1.0,
                        upper: 1.0,
                    });
                    (JointKind::Prismatic, Some(span(-1.0, 1.0, 0.0)), ref_)
                }
            },
            other => {
                return Err(ImportError::UnsupportedJoint {
                    joint: name,
                    kind: other.to_owned(),
                });
            }
        };
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
            qpos_ref,
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
    /// unnamed mesh after its file's stem, and so do we; an inline
    /// `<mesh vertex face>` has no file to name it after, so an unnamed one
    /// becomes `inline_N`.
    fn read_assets(&mut self, root: &Node) -> Result<(), ImportError> {
        for asset in root.kids("asset") {
            for m in asset.kids("mesh") {
                let m = self.resolved(m, MAIN_CLASS)?;
                let file = m.attr("file").map(str::to_owned);
                let scale = self.nums::<3>(&m, "scale")?.unwrap_or([1.0; 3]);
                if file.is_none() && m.attr("vertex").is_some() {
                    self.read_inline_mesh(&m, scale)?;
                    continue;
                }
                let name = match (m.attr("name"), &file) {
                    (Some(n), _) => n.to_owned(),
                    (None, Some(f)) => Path::new(f)
                        .file_stem()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into_owned(),
                    (None, None) => continue,
                };
                self.assets.insert(name, (file, scale));
            }
        }
        Ok(())
    }

    /// `<mesh vertex face normal>` with no `file`: MJCF's own syntax for an
    /// inline mesh. Parsed into a real `TriMesh`, written out as `.stl`
    /// bytes and registered like any other asset, `path` pointing beside
    /// the source file — the caller places the bytes there once import
    /// finishes (step 2's decision, docs/02-data-model.md §Geometry). A
    /// `vertex` with no `face` asks MuJoCo for its convex hull, which is
    /// out of scope (docs/plans/mjcf-mesh-geometry.md non-goals) and stays
    /// refused, exactly as every file-less mesh was before this.
    fn read_inline_mesh(&mut self, m: &Node, scale: [f64; 3]) -> Result<(), ImportError> {
        let key = m.attr("name").map(str::to_owned).unwrap_or_else(|| {
            self.unnamed_meshes += 1;
            format!("inline_{}", self.unnamed_meshes)
        });
        let Some(mesh) = self.parse_inline_mesh(m)? else {
            self.assets.insert(key, (None, scale));
            return Ok(());
        };
        let filename = self.synthesize_filename(&key);
        let bytes = riggen_mesh::write_binary(&mesh);
        self.inline_hash.insert(key.clone(), content_hash(&bytes));
        let path = self.base_dir.join(&filename);
        self.inline_meshes.push((filename, bytes));
        self.assets
            .insert(key, (Some(path.display().to_string()), scale));
        Ok(())
    }

    /// `vertex`/`face`/optional `normal` into a `TriMesh` — the same
    /// whitespace-separated-number shape `xml.rs` already parses for poses
    /// and scales. `None` when `face` is absent (the implicit-convex-hull
    /// form). Normals are kept, welded, when there is one per vertex —
    /// `obj.rs`'s own rule — and recomputed flat otherwise.
    fn parse_inline_mesh(&self, m: &Node) -> Result<Option<TriMesh>, ImportError> {
        let parse_err = |message: String| ImportError::Parse {
            path: self.path.clone(),
            message,
        };
        let Some(face) = self.numbers(m, "face")? else {
            return Ok(None);
        };
        let vertex = self.numbers(m, "vertex")?.unwrap_or_default();
        if vertex.is_empty() || !vertex.len().is_multiple_of(3) {
            return Err(parse_err(format!(
                "<mesh vertex>: {} numbers is not a positive multiple of three",
                vertex.len()
            )));
        }
        if face.is_empty() || !face.len().is_multiple_of(3) {
            return Err(parse_err(format!(
                "<mesh face>: {} numbers is not a positive multiple of three",
                face.len()
            )));
        }
        let positions: Vec<DVec3> = vertex
            .as_chunks::<3>()
            .0
            .iter()
            .map(|&[x, y, z]| DVec3::new(x, y, z))
            .collect();
        let mut indices = Vec::with_capacity(face.len());
        for &v in &face {
            if v < 0.0 || v.fract() != 0.0 || v as usize >= positions.len() {
                return Err(parse_err(format!(
                    "<mesh face>: index {v} out of range for {} vertices",
                    positions.len()
                )));
            }
            indices.push(v as u32);
        }
        let mut mesh = TriMesh {
            positions,
            normals: Vec::new(),
            indices,
        };
        match self.numbers(m, "normal")? {
            Some(normal) if normal.len() == mesh.positions.len() * 3 => {
                mesh.normals = normal
                    .as_chunks::<3>()
                    .0
                    .iter()
                    .map(|&[x, y, z]| DVec3::new(x, y, z).normalize_or_zero())
                    .collect();
                mesh.validate().map_err(|e| parse_err(e.to_string()))?;
            }
            Some(normal) => {
                return Err(parse_err(format!(
                    "<mesh normal>: {} numbers for {} vertices",
                    normal.len(),
                    mesh.positions.len()
                )));
            }
            None => {
                mesh.validate().map_err(|e| parse_err(e.to_string()))?;
                mesh.flat_normals();
            }
        }
        Ok(Some(mesh))
    }

    /// A `.stl` filename for `stem` that nothing at `self.base_dir`
    /// already answers to — checked through `self.source`, so a browser
    /// drop and a real directory are asked the same way (ADR-0017) — and
    /// that no inline mesh earlier in this same file has already claimed.
    fn synthesize_filename(&self, stem: &str) -> String {
        let mut filename = format!("{stem}.stl");
        let mut suffix = 1;
        while self.source.exists(&self.base_dir.join(&filename))
            || self.inline_meshes.iter().any(|(n, _)| *n == filename)
        {
            suffix += 1;
            filename = format!("{stem}_{suffix}.stl");
        }
        filename
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
        // An inline mesh's bytes are already known — `self.source` has
        // nothing to hash until the caller writes them out, and asking it
        // to would turn every inline mesh into a spurious `MeshNotFound`.
        let content_hash = if let Some(&hash) = self.inline_hash.get(name) {
            hash
        } else {
            match self.source.hash(&path) {
                Ok(h) => h,
                Err(_) => {
                    self.warnings.push(ImportWarning::MeshNotFound {
                        link: link.to_owned(),
                        file,
                        tried: path.clone(),
                    });
                    0
                }
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
    /// deviations from `qpos0`, and `qpos0` is each joint's `ref` — which is
    /// exactly what the document's `q` is a deviation from (ADR-0025 §3) —
    /// so it is our `q(follower) = a1·q(leader) + a0` exactly when the last
    /// three terms are zero, whatever `ref` either joint carries. Anything
    /// else is dropped with the reason, the way the URDF import phrases a
    /// `<mimic>` it cannot keep.
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

    /// `<tendon><fixed>` → one entry of `Robot::tendons` each (ADR-0025
    /// §4): a named linear combination of joint values, its `range` and
    /// its passive dynamics. A fixed tendon's length is `Σ coef · qpos` —
    /// **absolute**, not a deviation from `qpos0` the way an
    /// `<equality>`'s `polycoef` is — so nothing here is shifted by a
    /// `Joint::qpos_ref`, on the way in or on the way out.
    ///
    /// A `<fixed>`'s defaults come from `<default><tendon>`: MuJoCo files
    /// the class under the block's name, not the element's, so the class
    /// tree is resolved through a view of the element wearing that tag.
    ///
    /// A `<spatial>` routes over sites, wrapping geoms and pulleys, none
    /// of which the document has, so it is dropped under its name. So is a
    /// `<fixed>` whose joints the document cannot hold — one not in the
    /// file, one that is fixed, one twice, a zero coefficient, none at all
    /// — and the tendon goes **whole**: half a linear combination is a
    /// different tendon, not a lesser one. An actuator on a dropped
    /// tendon is then dropped in its turn, since the name is gone.
    fn read_tendons(&mut self, root: &Node) -> Result<(), ImportError> {
        for block in root.kids("tendon") {
            for t in &block.children {
                let name = match t.attr("name") {
                    Some(n) => n.to_owned(),
                    // MJCF lets a tendon go unnamed; the document does not,
                    // and nothing can drive one that has no name anyway.
                    None => format!("tendon{}", self.robot.tendons.len() + 1),
                };
                let drop = |im: &mut Self, reason: String| {
                    im.warnings.push(ImportWarning::TendonDropped {
                        tendon: name.clone(),
                        reason,
                    });
                };
                if t.tag != "fixed" {
                    let reason = format!(
                        "<{}> routes over sites and wrapping geoms; the document holds a fixed tendon",
                        t.tag
                    );
                    drop(self, reason);
                    continue;
                }
                if self.tendon_ids.contains_key(&name) {
                    drop(
                        self,
                        "a tendon of that name is already in the file".to_owned(),
                    );
                    continue;
                }
                let as_tendon = Node {
                    tag: "tendon".to_owned(),
                    attrs: t.attrs.clone(),
                    children: Vec::new(),
                };
                let f = self.resolved(&as_tendon, MAIN_CLASS)?;
                for attr in f.attrs.keys() {
                    let attr = attr.as_str();
                    if !TENDON_ATTRS.contains(&attr) && !TENDON_SILENT.contains(&attr) {
                        self.drop(&format!("<fixed {attr}>"));
                    }
                }
                // MuJoCo reads `0 0` as no range at all, on a tendon as on
                // a joint; any other range that bounds nothing is a tendon
                // `validate` would refuse, so it is dropped by name here
                // rather than failing the file.
                let range = self
                    .nums::<2>(&f, "range")?
                    .filter(|[lo, hi]| *lo != 0.0 || *hi != 0.0);
                if let Some([lo, hi]) = range
                    && lo >= hi
                {
                    drop(self, format!("its range {lo} {hi} bounds nothing"));
                    continue;
                }
                let limited = self.limited_flag(&f, "limited", range)?;
                let mut joints: Vec<TendonJoint> = Vec::new();
                let mut refused = None;
                for jn in t.kids("joint") {
                    let jn = self.resolved(jn, MAIN_CLASS)?;
                    let jname = jn.attr("joint").unwrap_or_default().to_owned();
                    let coef = self.num(&jn, "coef")?.unwrap_or(1.0);
                    let Some(&id) = self.joint_ids.get(&jname) else {
                        refused = Some(format!("no joint \"{jname}\" is in the file"));
                        break;
                    };
                    if !self.robot.joints[&id].kind.is_movable() {
                        refused = Some(format!(
                            "joint \"{jname}\" is fixed and has no value to combine"
                        ));
                        break;
                    }
                    if joints.iter().any(|e| e.joint == id) {
                        refused = Some(format!("joint \"{jname}\" is on it twice"));
                        break;
                    }
                    if coef == 0.0 {
                        refused = Some(format!("joint \"{jname}\" is weighed by zero"));
                        break;
                    }
                    joints.push(TendonJoint { joint: id, coef });
                }
                if let Some(reason) = refused {
                    drop(self, reason);
                    continue;
                }
                if joints.is_empty() {
                    drop(self, "it runs over no joints".to_owned());
                    continue;
                }
                let id: TendonId = self.robot.next_id.alloc();
                self.tendon_ids.insert(name.clone(), id);
                let tendon = Tendon {
                    name,
                    joints,
                    range,
                    limited,
                    stiffness: self.num(&f, "stiffness")?.unwrap_or(0.0),
                    damping: self.num(&f, "damping")?.unwrap_or(0.0),
                    frictionloss: self.num(&f, "frictionloss")?.unwrap_or(0.0),
                };
                self.robot.tendons.insert(id, tendon);
            }
        }
        Ok(())
    }

    /// `<position>` / `<velocity>` / `<motor>` / `<general>` driving a
    /// joint **or a fixed tendon** → one entry of `Robot::actuators` each,
    /// under the name the file gave it (ADR-0014, ADR-0023, ADR-0024,
    /// ADR-0025 §4). A second element on an already-driven target is a
    /// second entry: MuJoCo sums their controls, so keeping only one would
    /// throw away what the file said.
    ///
    /// An element is read as a preset **iff every attribute it carries is
    /// one the preset can express** (ADR-0024 §2); a `<position gear>` or a
    /// `<position timeconst>` is what MuJoCo makes of it, a `General`. The
    /// attributes the document has no room for — `actdim`, `actearly`,
    /// `actrange`, `lengthrange`, `cranklength`, and a preset's
    /// `dampratio` / `inheritrange`, which MuJoCo computes from the model —
    /// are counted once per attribute name and the element is read without
    /// them; so is anything else MuJoCo would refuse. `<muscle>` and
    /// `<adhesion>` stay dropped (ADR-0024: a muscle needs a `lengthrange`
    /// riggen does not compute; an adhesion drives a body), as does any
    /// element driving neither a joint nor a tendon.
    ///
    /// `forcerange` and `ctrlrange` are kept on the actuator as the file
    /// said them, with their `ctrllimited` / `forcelimited` (ADR-0024). A
    /// written flag is recorded as written; an absent one stays `auto` —
    /// which the writer's own `autolimits="true"` reproduces — **unless**
    /// the file named no range at all (or `autolimits` was off), when
    /// MuJoCo makes the actuator unlimited and the flag is recorded as
    /// `false`, so the writer does not derive from the joint the range the
    /// file left open. They are also where `Limits::effort` and
    /// `Limits::velocity` come back from — MJCF keeps them on the actuator,
    /// not on the joint — and the joint has one of each, so the **first**
    /// actuator to drive it fills them and a later one leaves them alone —
    /// and an actuator on a *tendon* fills neither, there being no joint
    /// behind it (ADR-0025 §4).
    fn read_actuators(&mut self, root: &Node) -> Result<(), ImportError> {
        for block in root.kids("actuator") {
            for a in &block.children {
                let a = self.resolved(a, MAIN_CLASS)?;
                let name = a
                    .attr("name")
                    .or_else(|| a.attr("joint"))
                    .or_else(|| a.attr("tendon"))
                    .unwrap_or("<unnamed>")
                    .to_owned();
                let drop = |im: &mut Self, reason: String| {
                    im.warnings.push(ImportWarning::ActuatorDropped {
                        actuator: name.clone(),
                        reason,
                    });
                };
                // Anything driving neither a joint nor a fixed tendon is
                // outside the document's two targets (ADR-0023, ADR-0024,
                // ADR-0025 §4): a site, a body, a crank, or a joint
                // through `jointinparent`, which is a different
                // transmission.
                let joint = a.attr("joint").map(str::to_owned);
                let tendon = a.attr("tendon").map(str::to_owned);
                if joint.is_none() && tendon.is_none() {
                    let target = ACTUATOR_TARGETS
                        .iter()
                        .find(|(attr, _)| a.attrs.contains_key(*attr))
                        .map_or("nothing", |(_, what)| what);
                    drop(self, format!("it drives {target}, not a joint or a tendon"));
                    continue;
                }
                let tag = a.tag.as_str();
                let Some((_, gains, desugared)) = PRESET_ATTRS.iter().find(|(t, ..)| *t == tag)
                else {
                    let reason = if tag == "muscle" {
                        "a <muscle> needs a lengthrange riggen does not compute (ADR-0024)"
                            .to_owned()
                    } else {
                        format!("<{tag}> is not one of the three presets or <general>")
                    };
                    drop(self, reason);
                    continue;
                };
                // What the element carries beyond its tag's own vocabulary
                // is counted, not silently ignored (02 §Nothing is dropped
                // silently); what it carries beyond its *preset's* is why
                // it is read as a `General` instead.
                let mut beyond_preset = false;
                for attr in a.attrs.keys() {
                    let known =
                        ACTUATOR_COMMON.contains(&attr.as_str()) || gains.contains(&attr.as_str());
                    if known {
                        continue;
                    }
                    if desugared.contains(&attr.as_str()) {
                        beyond_preset = true;
                    } else {
                        self.drop(&format!("<{tag} {attr}>"));
                    }
                }
                let kp = self.num(&a, "kp")?.unwrap_or(1.0);
                let kv = self.num(&a, "kv")?;
                let gear = self
                    .numbers(&a, "gear")?
                    .and_then(|g| g.first().copied())
                    .unwrap_or(1.0);
                let spec = match (tag, beyond_preset) {
                    ("position", false) => ActuatorSpec::Position {
                        kp,
                        kv: kv.unwrap_or(0.0),
                    },
                    ("velocity", false) => ActuatorSpec::Velocity {
                        kv: kv.unwrap_or(1.0),
                    },
                    ("motor", _) => ActuatorSpec::Motor { gear },
                    // MuJoCo's own reading of the two servos: a fixed
                    // gain with an affine bias, and `timeconst` an exact
                    // first-order filter on the activation.
                    ("position", true) => {
                        let timeconst = self.num(&a, "timeconst")?;
                        ActuatorSpec::General(General {
                            dyntype: timeconst.map_or(DynType::None, |_| DynType::FilterExact),
                            gaintype: GainType::Fixed,
                            biastype: BiasType::Affine,
                            dynprm: timeconst.into_iter().collect(),
                            gainprm: vec![kp],
                            biasprm: vec![0.0, -kp, -kv.unwrap_or(0.0)],
                            gear,
                        })
                    }
                    ("velocity", true) => {
                        let kv = kv.unwrap_or(1.0);
                        ActuatorSpec::General(General {
                            dyntype: DynType::None,
                            gaintype: GainType::Fixed,
                            biastype: BiasType::Affine,
                            dynprm: Vec::new(),
                            gainprm: vec![kv],
                            biasprm: vec![0.0, 0.0, -kv],
                            gear,
                        })
                    }
                    _ => ActuatorSpec::General(General {
                        dyntype: self
                            .actuator_type(
                                &a,
                                "dyntype",
                                DynType::from_mjcf,
                                DynType::ALL.iter().map(|t| t.mjcf_name()),
                            )?
                            .unwrap_or(DynType::None),
                        gaintype: self
                            .actuator_type(
                                &a,
                                "gaintype",
                                GainType::from_mjcf,
                                GainType::ALL.iter().map(|t| t.mjcf_name()),
                            )?
                            .unwrap_or(GainType::Fixed),
                        biastype: self
                            .actuator_type(
                                &a,
                                "biastype",
                                BiasType::from_mjcf,
                                BiasType::ALL.iter().map(|t| t.mjcf_name()),
                            )?
                            .unwrap_or(BiasType::None),
                        dynprm: self.numbers(&a, "dynprm")?.unwrap_or_default(),
                        gainprm: self.numbers(&a, "gainprm")?.unwrap_or_default(),
                        biasprm: self.numbers(&a, "biasprm")?.unwrap_or_default(),
                        gear,
                    }),
                };
                // A `<fixed>` MuJoCo would not load — or one riggen
                // dropped by name — leaves no tendon to drive.
                let target = match (&joint, &tendon) {
                    (Some(j), _) => match self.joint_ids.get(j) {
                        Some(&id) => ActuatorTarget::Joint(id),
                        None => {
                            drop(self, format!("no joint \"{j}\" is in the file"));
                            continue;
                        }
                    },
                    (None, Some(t)) => match self.tendon_ids.get(t) {
                        Some(&id) => ActuatorTarget::Tendon(id),
                        None => {
                            drop(self, format!("no tendon \"{t}\" is in the file"));
                            continue;
                        }
                    },
                    (None, None) => unreachable!("both absent is dropped above"),
                };
                let force = self.nums::<2>(&a, "forcerange")?;
                let ctrl = self.nums::<2>(&a, "ctrlrange")?;
                let ranges = ActuatorRanges {
                    ctrl,
                    force,
                    ctrl_limited: self.limited_flag(&a, "ctrllimited", ctrl)?,
                    force_limited: self.limited_flag(&a, "forcelimited", force)?,
                };
                // One entry per element, keyed in its own namespace
                // (ADR-0023) — a second one on this target is a second
                // entry, not a replacement.
                let first = target
                    .joint()
                    .is_some_and(|id| self.robot.actuators_on(id).next().is_none());
                let velocity_servo = matches!(spec, ActuatorSpec::Velocity { .. });
                let aid = self.robot.next_id.alloc();
                self.robot.actuators.insert(
                    aid,
                    Actuator {
                        name,
                        target,
                        spec,
                        ranges,
                    },
                );
                // `Limits::effort` / `::velocity` are a joint's, and a
                // tendon has none behind it (ADR-0025 §4): a tendon
                // actuator fills nothing in.
                let Some(id) = target.joint() else { continue };
                let j = self.robot.joints.get_mut(&id).expect("just walked");
                if first && let Some(limits) = &mut j.limits {
                    // Both are written as ±v and read back as the upper
                    // half; a zero one was never filled in (ADR-0014).
                    if let Some([_, upper]) = force {
                        limits.effort = upper;
                    }
                    if let (true, Some([_, upper])) = (velocity_servo, ctrl) {
                        limits.velocity = upper;
                    }
                }
            }
        }
        Ok(())
    }

    /// One of `<general>`'s three type attributes, by its MJCF spelling;
    /// a value MuJoCo would not take is a parse error naming the choices.
    fn actuator_type<T>(
        &self,
        node: &Node,
        name: &str,
        from_mjcf: fn(&str) -> Option<T>,
        choices: impl Iterator<Item = &'static str>,
    ) -> Result<Option<T>, ImportError> {
        let Some(value) = node.attr(name) else {
            return Ok(None);
        };
        from_mjcf(value).map(Some).ok_or_else(|| {
            self.parse_err(format!(
                "<{}> {name}=\"{value}\": expected one of {}",
                node.tag,
                choices.collect::<Vec<_>>().join(", ")
            ))
        })
    }

    /// An actuator's `ctrllimited` / `forcelimited` as the document keeps
    /// it (ADR-0024): the written `true` / `false`; `None` for `auto`
    /// beside a range under `autolimits`, which the writer reproduces; and
    /// `Some(false)` where MuJoCo would compute `false` and the writer,
    /// left to itself, would derive a range instead — no range written,
    /// or `autolimits` off.
    fn limited_flag(
        &self,
        node: &Node,
        name: &str,
        range: Option<[f64; 2]>,
    ) -> Result<Option<bool>, ImportError> {
        let written = match node.attr(name) {
            Some("auto") => None,
            _ => node.flag(name).map_err(|m| self.parse_err(m))?,
        };
        Ok(written.or_else(|| (!(self.compiler.autolimits && range.is_some())).then_some(false)))
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
        for (id, reason) in actuator_refusals(&self.robot) {
            let actuator = self
                .robot
                .actuators
                .remove(&id)
                .expect("named by validate")
                .name;
            self.warnings
                .push(ImportWarning::ActuatorDropped { actuator, reason });
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

/// What `validate` refuses about an actuator, per actuator (ADR-0014,
/// re-keyed by ADR-0023). As with the couplings, `validate` owns the rules
/// and this only phrases its verdict; every other error it reports still
/// fails the import.
fn actuator_refusals(robot: &Robot) -> Vec<(ActuatorId, String)> {
    riggen_core::validation_errors(robot)
        .into_iter()
        .filter_map(|e| match e {
            ValidationError::ActuatorOnFixedJoint { actuator, .. } => Some((
                actuator,
                "a fixed joint has no <joint> for it to drive".to_owned(),
            )),
            ValidationError::ActuatorOnMimicFollower { actuator, .. } => Some((
                actuator,
                "the joint is already driven by an <equality>".to_owned(),
            )),
            ValidationError::InvalidActuatorGain { actuator, what } => {
                Some((actuator, format!("its {what}")))
            }
            ValidationError::ActuatorPrmTooLong {
                actuator,
                what,
                len,
            } => Some((
                actuator,
                format!(
                    "its {what} has {len} entries; MuJoCo holds at most {}",
                    General::MAX_PRM
                ),
            )),
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
/// element's children are part of it and are not counted again — and
/// neither are a read-whole element's, since its reader already named
/// what it could not keep.
fn count_dropped(node: &Node, out: &mut BTreeMap<String, usize>) {
    for c in &node.children {
        if !read_here(&node.tag, &c.tag) {
            *out.entry(format!("<{}>", c.tag)).or_default() += 1;
        } else if !READ_WHOLE.contains(&c.tag.as_str()) {
            count_dropped(c, out);
        }
    }
}

/// Whether `tag` under `parent` is an element the import reads. Almost
/// every tag answers for itself, but MJCF spells two different elements
/// `<tendon>`: the block of tendons at the root, which is read, and
/// `<equality><tendon>`, which holds two tendons' lengths together and the
/// document has no field for (ADR-0025 §4).
fn read_here(parent: &str, tag: &str) -> bool {
    if tag == "tendon" {
        return parent != "equality";
    }
    READ.contains(&tag)
}

/// Elements [`Import`] reads whole: whatever is inside one belongs to it,
/// and its reader has already warned about what it dropped, so counting
/// the children again would say the same loss twice. A `<fixed>`'s
/// `<joint>`s are the tendon; a `<spatial>`'s route is named with the
/// tendon that carried it.
const READ_WHOLE: &[&str] = &["fixed", "spatial"];

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resolve::{ResolvedGeom, ResolvedRobot};
    use crate::xml::parse;
    use crate::{ComputeNow, ExportOptions, Format, MeshStore, export, resolve};
    use riggen_core::{Disk, JointState, fk};

    /// The preset driving the joint called `name`, if one does — the
    /// readout `Joint::actuator` used to be (ADR-0023).
    fn actuator_of(robot: &Robot, name: &str) -> Option<ActuatorSpec> {
        let (&jid, _) = robot.joints.iter().find(|(_, j)| j.name == name)?;
        robot.actuators_on(jid).next().map(|(_, a)| a.spec.clone())
    }

    /// Writes `robot`'s MJCF and its meshes into a scratch directory and
    /// reads the `.xml` back — the acceptance route, in one function.
    fn round_trip(robot: &Robot, store: &MeshStore, tag: &str) -> (Robot, Vec<ImportWarning>) {
        let dir = std::env::temp_dir().join(format!("riggen-mjcf-in-{tag}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        let files = export(&resolved(robot, store), &options(), &dir).unwrap();
        let text = std::fs::read_to_string(&files[0]).unwrap();
        let (robot, warnings, inline) =
            from_mjcf(&parse(&text).unwrap(), &files[0], &Disk).unwrap();
        assert_eq!(
            inline,
            Vec::new(),
            "our own writer never emits an inline mesh"
        );
        (robot, warnings)
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
            .map(|(robot, warnings, _inline)| (robot, warnings))
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
        assert_eq!(robot.links.len(), 6);
        assert_eq!(robot.links[&robot.root].name, "base_link");
        for name in ["base_link", "upper", "slider", "wheel", "tip", "finger"] {
            assert_eq!(link(name).name, name);
        }
        // The fixed edge has no `<joint>` element at all, so its name is
        // the invented `<body>_joint` — which is the one the document had.
        assert_eq!(robot.joints.len(), 5);
        for (name, kind) in [
            ("upper_joint", JointKind::Revolute),
            ("slider_joint", JointKind::Prismatic),
            ("wheel_joint", JointKind::Continuous),
            ("tip_joint", JointKind::Fixed),
            ("finger_joint", JointKind::Revolute),
        ] {
            let j = joint(name);
            assert_eq!(j.kind, kind, "{name}");
            assert_eq!(j.origin, Pose::from_translation(DVec3::Z * 0.1), "{name}");
        }
        assert_eq!(joint("upper_joint").axis, DVec3::Y);
        assert_eq!(joint("upper_joint").dynamics.damping, 0.1);
        assert_eq!(joint("wheel_joint").limits, None, "Continuous keeps none");
        // The finger's `ref` went out as `ref="0.2" range="-0.8 1.2"` and
        // comes back as the `qpos_ref` and the ±1 it left with (ADR-0025).
        assert_eq!(joint("finger_joint").qpos_ref, 0.2);
        for name in ["upper_joint", "slider_joint"] {
            assert_eq!(joint(name).qpos_ref, 0.0, "{name}");
        }
        for name in ["upper_joint", "slider_joint", "finger_joint"] {
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
        // …and the chain past it: the finger names the slider, which is
        // itself a follower, and the reader keeps both (ADR-0025 §1/§2).
        let slider = *robot
            .joints
            .iter()
            .find(|(_, j)| j.name == "slider_joint")
            .unwrap()
            .0;
        assert_eq!(
            joint("finger_joint").mimic,
            Some(Mimic {
                joint: slider,
                multiplier: 0.5,
                offset: 0.0
            })
        );
        assert_eq!(
            actuator_of(&robot, "upper_joint"),
            Some(ActuatorSpec::Position { kp: 100.0, kv: 5.0 })
        );
        assert_eq!(
            actuator_of(&robot, "wheel_joint"),
            Some(ActuatorSpec::Velocity { kv: 2.0 })
        );
        assert_eq!(
            actuator_of(&robot, "slider_joint"),
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
        let (robot, warnings, inline) = super::load(&path, &Disk).unwrap();
        // `pad`, the corpus's one inline mesh: its bytes are synthesized,
        // not found on disk, so they are checked on their own rather than
        // through the asset-path loop below (docs/02-data-model.md
        // §Geometry).
        assert_eq!(inline.len(), 1);
        assert_eq!(inline[0].0, "pad.stl");
        let pad_mesh = riggen_mesh::parse_stl(&inline[0].1, Path::new("pad.stl")).unwrap();
        assert_eq!(pad_mesh.triangle_count(), 4);

        let link = |n: &str| robot.links.values().find(|l| l.name == n).unwrap();
        let joint = |n: &str| robot.joints.values().find(|j| j.name == n).unwrap();

        assert_eq!(robot.name, "menagerie_style");
        assert_eq!(robot.links.len(), 6);
        assert_eq!(robot.links[&robot.root].name, "base_link");
        // `<compiler angle="degree">` is MJCF's default and the opposite of
        // ours: ±180° is ±π, and the class two levels up is where the range
        // came from at all. The pan's `ref="10"` is 10° too (ADR-0025 §3),
        // and the range — in `qpos` terms in the file — comes in shifted
        // by it, so `Limits` bounds the deviation from the authored pose.
        let ten = 10f64.to_radians();
        assert!((joint("shoulder_pan").qpos_ref - ten).abs() < 1e-12);
        let pan = joint("shoulder_pan").limits.unwrap();
        assert!((pan.lower + std::f64::consts::PI + ten).abs() < 1e-12);
        assert!((pan.upper - std::f64::consts::PI + ten).abs() < 1e-12);
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
        // The mesh scale is non-uniform, and the meshes are beside the arm
        // — `pad`, the one inline mesh, is not written to disk by `load`
        // itself (that is its caller's job, step 2's decision), so it is
        // skipped here and checked above instead.
        for asset in robot.assets.values() {
            if asset.path.file_name() == Some(std::ffi::OsStr::new("pad.stl")) {
                continue;
            }
            assert!(asset.path.exists(), "{}", asset.path.display());
        }
        // Both `<actuator>`s on the pan joint survive, each under the name
        // the file gave it, and neither is the joint's (ADR-0023). Before
        // the table, the second silently overwrote the first and both were
        // renamed to "shoulder_pan".
        let pan = *robot
            .joints
            .iter()
            .find(|(_, j)| j.name == "shoulder_pan")
            .unwrap()
            .0;
        assert_eq!(
            robot
                .actuators_on(pan)
                .map(|(_, a)| (a.name.as_str(), a.spec.clone()))
                .collect::<Vec<_>>(),
            [
                (
                    "pan",
                    ActuatorSpec::Position {
                        kp: 120.0,
                        kv: 12.0
                    }
                ),
                ("pan_damp", ActuatorSpec::Velocity { kv: 3.0 }),
            ]
        );
        // The joint has one `effort`, so the **first** actuator's
        // `forcerange` fills it and the second one leaves it alone — it
        // carries no `forcerange` and no `ctrlrange` of its own.
        assert_eq!(joint("shoulder_pan").limits.unwrap().effort, 30.0);
        assert_eq!(
            joint("shoulder_pan").limits.unwrap().velocity,
            0.0,
            "unfilled: the file's `<joint>` has no velocity and the second \
             actuator carries no ctrlrange"
        );
        // And each actuator keeps what it said itself (ADR-0024): `pan`'s
        // `forcerange` under an `auto` flag, and — the loss step 2 of the
        // plan measured — **no** `ctrlrange`, recorded as `false` so the
        // writer does not clamp it to the joint's ±π. `pan_damp` wrote an
        // explicit `ctrllimited="false"` beside a range, kept as written,
        // and no `forcerange`.
        assert_eq!(
            robot
                .actuators_on(pan)
                .map(|(_, a)| a.ranges)
                .collect::<Vec<_>>(),
            [
                ActuatorRanges {
                    ctrl: None,
                    force: Some([-30.0, 30.0]),
                    ctrl_limited: Some(false),
                    force_limited: None,
                },
                ActuatorRanges {
                    ctrl: Some([-2.0, 2.0]),
                    force: None,
                    ctrl_limited: Some(false),
                    force_limited: Some(false),
                },
            ]
        );
        // The `<general>` comes in (ADR-0024), its gains resolved through
        // the class tree: `gainprm` / `biasprm` from `arm`, two levels up,
        // `gear` from `arm_drive`, one up, and its own `dyntype` /
        // `dynprm` on the element. `gaintype` was inherited too.
        let slide = *robot
            .joints
            .iter()
            .find(|(_, j)| j.name == "wrist_slide")
            .unwrap()
            .0;
        let (_, lift) = robot
            .actuators_on(slide)
            .next()
            .expect("lift drives the slide");
        assert_eq!(lift.name, "lift");
        assert_eq!(
            lift.spec,
            ActuatorSpec::General(General {
                dyntype: DynType::Filter,
                gaintype: GainType::Fixed,
                biastype: BiasType::Affine,
                dynprm: vec![0.02],
                gainprm: vec![150.0],
                biasprm: vec![0.0, -150.0, -3.0],
                gear: 2.0,
            })
        );
        assert_eq!(
            lift.ranges,
            ActuatorRanges {
                ctrl_limited: Some(false),
                force_limited: Some(false),
                ..ActuatorRanges::default()
            },
            "nothing said: unlimited, and no preset to derive from anyway"
        );
        assert_eq!(
            joint("shoulder_lift").mimic.map(|m| m.multiplier),
            Some(0.25)
        );
        // …and the chain past it (ADR-0025 §1): the finger follows the
        // lift, which follows the pan. Nothing is dropped and nothing is
        // flattened — `resolve_q` composes it, and the reach check does
        // too (0.125 of the pan's ±π, inside the finger's ±30°).
        let lift = *robot
            .joints
            .iter()
            .find(|(_, j)| j.name == "shoulder_lift")
            .unwrap()
            .0;
        assert_eq!(
            joint("finger_flex").mimic.map(|m| (m.joint, m.multiplier)),
            Some((lift, 0.5))
        );

        // The `<tendon><fixed>` is a document tendon now (ADR-0025 §4):
        // its two joints in the order the file wrote them, its `range` in
        // MuJoCo's own absolute terms — **not** shifted by any `qpos_ref`,
        // since a fixed tendon's length is `Σ coef · qpos` — its passive
        // dynamics, and `limited` left `auto`, which the writer reproduces
        // under its own `autolimits="true"`.
        let finger = *robot
            .joints
            .iter()
            .find(|(_, j)| j.name == "finger_flex")
            .unwrap()
            .0;
        let (tid, tendon) = robot.tendons.iter().next().expect("grip_tendon");
        assert_eq!(tendon.name, "grip_tendon");
        assert_eq!(
            tendon
                .joints
                .iter()
                .map(|j| (j.joint, j.coef))
                .collect::<Vec<_>>(),
            [(slide, 1.0), (finger, -0.02)]
        );
        assert_eq!(tendon.range, Some([-0.06, 0.06]));
        assert_eq!(
            tendon.limited, None,
            "auto, beside a range, under autolimits"
        );
        assert_eq!(
            (tendon.stiffness, tendon.damping, tendon.frictionloss),
            (20.0, 0.5, 0.0)
        );
        // …and the `<motor>` on it comes back as the second target
        // `ActuatorTarget` gained (ADR-0023 as amended): the last name in
        // the round trip's dropped list.
        let (_, grip) = robot
            .actuators
            .iter()
            .find(|(_, a)| a.name == "grip")
            .expect("the tendon motor is read now");
        assert_eq!(grip.target, ActuatorTarget::Tendon(*tid));
        assert_eq!(grip.spec, ActuatorSpec::Motor { gear: 80.0 });
        assert_eq!(
            grip.ranges,
            ActuatorRanges {
                ctrl_limited: Some(false),
                force_limited: Some(false),
                ..ActuatorRanges::default()
            },
            "no range said: unlimited, and no joint behind it to derive one from"
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
                // No coupling, no tendon and no actuator of this file
                // is dropped any more (ADR-0025): the chain, the `ref`,
                // the `<fixed>` and the `<motor>` on it all come in.
                // Then one line per element name, whatever the count —
                // including the one tendon attribute the document counts
                // rather than keeps (ADR-0025 §4).
                ImportWarning::ElementDropped {
                    element: "<camera>".to_owned(),
                    count: 1
                },
                ImportWarning::ElementDropped {
                    element: "<contact>".to_owned(),
                    count: 1
                },
                ImportWarning::ElementDropped {
                    element: "<fixed springlength>".to_owned(),
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
                    element: "<texture>".to_owned(),
                    count: 1
                },
            ]
        );
    }

    /// `<joint ref>` is `Joint::qpos_ref` (ADR-0025 §3): angle-converted on
    /// a hinge, verbatim on a slide, and `range` — which MuJoCo keeps in
    /// `qpos` terms, unshifted by the compiler — comes in as the deviation
    /// it bounds. A `polycoef` over such a joint is read as the mimic it
    /// is, since MuJoCo's deviations are from `qpos0 = ref`.
    #[test]
    fn a_joint_ref_is_read_as_qpos_ref_and_the_range_is_shifted_by_it() {
        let (robot, warnings) = load(
            r#"<mujoco model="m"><compiler angle="degree"/><worldbody>
                 <body name="a">
                   <body name="b"><joint name="h" ref="10" range="-30 60"/>
                     <body name="c"><joint name="s" type="slide" ref="0.2" range="-0.5 1"/>
                       <body name="d"><joint name="free" ref="45"/></body>
                     </body>
                   </body>
                 </body>
               </worldbody>
               <equality>
                 <joint joint1="s" joint2="h" polycoef="0.1 0.5 0 0 0"/>
               </equality>
               </mujoco>"#,
        )
        .unwrap();
        let joint = |n: &str| robot.joints.values().find(|j| j.name == n).unwrap();
        let deg = f64::to_radians;
        let h = joint("h");
        assert!((h.qpos_ref - deg(10.0)).abs() < 1e-12);
        let l = h.limits.unwrap();
        assert!((l.lower - deg(-40.0)).abs() < 1e-12, "{}", l.lower);
        assert!((l.upper - deg(50.0)).abs() < 1e-12, "{}", l.upper);
        let s = joint("s");
        assert_eq!(s.qpos_ref, 0.2, "a length: no conversion");
        assert_eq!(s.limits.map(|l| (l.lower, l.upper)), Some((-0.7, 0.8)));
        // No range: `Continuous`, and the ref is still its own.
        let free = joint("free");
        assert_eq!((free.kind, free.limits), (JointKind::Continuous, None));
        assert!((free.qpos_ref - deg(45.0)).abs() < 1e-12);
        let hid = *robot.joints.iter().find(|(_, j)| j.name == "h").unwrap().0;
        assert_eq!(
            s.mimic,
            Some(Mimic {
                joint: hid,
                multiplier: 0.5,
                offset: 0.1
            }),
            "the coupling over two ref joints is kept as written"
        );
        assert_eq!(warnings, vec![], "nothing about `ref`");
    }

    #[test]
    fn a_coupling_and_an_actuator_the_document_cannot_hold_are_named() {
        let (robot, warnings) = load(
            r#"<mujoco model="m"><compiler angle="radian"/><worldbody>
                 <body name="a">
                   <body name="b"><joint name="j" range="-1 1"/>
                     <body name="c"><joint name="k" range="-1 1"/>
                       <body name="d"><joint name="l" ref="0.2" range="-1.5 1.5"/></body>
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
        // and only a linear, active one survives at all — `ref` on one of
        // the two joints is no reason any more (ADR-0025 §3): `l`'s zero is
        // 0.2, its range comes in shifted to the deviation it bounds, and
        // the `polycoef` MuJoCo reads from `qpos0` is the mimic as written.
        assert_eq!(joint("k").mimic.map(|m| (m.multiplier, m.offset)), None);
        let jid = *robot.joints.iter().find(|(_, j)| j.name == "j").unwrap().0;
        assert_eq!(
            joint("l").mimic,
            Some(Mimic {
                joint: jid,
                multiplier: 1.0,
                offset: 0.0
            })
        );
        assert_eq!(joint("l").qpos_ref, 0.2);
        assert_eq!(
            joint("l").limits.map(|l| (l.lower, l.upper)),
            Some((-1.7, 1.3))
        );
        assert_eq!(
            actuator_of(&robot, "j"),
            Some(ActuatorSpec::Motor { gear: 50.0 })
        );
        // The `<general>` beside it is the joint's second actuator now
        // (ADR-0024), not a warning.
        assert_eq!(
            robot
                .actuators_on(jid)
                .map(|(_, a)| (a.name.as_str(), a.spec.kind_name()))
                .collect::<Vec<_>>(),
            [("drive", "motor"), ("fancy", "general")]
        );
        assert!(matches!(
            robot.actuators_on(jid).nth(1).unwrap().1.spec,
            ActuatorSpec::General(General {
                dyntype: DynType::Filter,
                ..
            })
        ));
        assert_eq!(joint("j").limits.unwrap().effort, 7.0);
        assert_eq!(
            actuator_of(&robot, "k"),
            Some(ActuatorSpec::Velocity { kv: 3.0 })
        );
        // A velocity servo *is* commanded in the joint's own rate, so its
        // `ctrlrange` is where `Limits::velocity` comes back from.
        assert_eq!(joint("k").limits.unwrap().velocity, 4.0);

        let said: Vec<String> = warnings.iter().map(ToString::to_string).collect();
        for line in [
            "joint \"k\": <mimic joint=\"j\"> dropped, its polycoef is not linear",
            "joint \"k\": <mimic joint=\"j\"> dropped, the constraint is not active",
            "joint \"k\": <mimic joint=\"nope\"> dropped, no joint of that name is in the file",
            "joint \"k\": <mimic joint=\"\"> dropped, it holds one joint to a constant, not to another",
            "actuator \"tendon_servo\" dropped, no tendon \"t\" is in the file",
            "actuator \"ghost\" dropped, no joint \"missing\" is in the file",
            "<weld> × 1: nothing in the document holds it; not read",
        ] {
            assert!(
                said.contains(&line.to_owned()),
                "missing {line:?}\n{said:#?}"
            );
        }
        // The linear coupling on `k` is the one `validate` then refused —
        // `k` would reach ±2, outside its own ±1 — and it says so.
        assert!(
            said.iter()
                .any(|s| s.contains("it would reach -2..2, outside its own limits")),
            "{said:#?}"
        );
        assert!(
            !said.iter().any(|s| s.contains("<joint ref>")),
            "a ref is read, not dropped (ADR-0025 §3)\n{said:#?}"
        );
    }

    /// A `<tendon><fixed>` is a document tendon (ADR-0025 §4): its joints
    /// in the file's order with their coefficients, its `range` in
    /// MuJoCo's own absolute terms, its passive dynamics — through
    /// `<default><tendon>`, which is the class a `<fixed>`'s defaults are
    /// filed under — and an actuator of any of the four kinds may drive
    /// it. Out again it is the same block, and nothing was derived from a
    /// joint for a target that has none.
    #[test]
    fn a_fixed_tendon_and_the_actuators_on_it_come_back_as_the_file_said_them() {
        let (robot, warnings) = load(
            r#"<mujoco model="m"><compiler angle="radian" autolimits="true"/>
                 <default><tendon stiffness="7" damping="0.25"/></default>
                 <worldbody>
                   <body name="a">
                     <body name="b"><joint name="j" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                       <body name="c"><joint name="k" type="slide" range="0 0.5"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>
                     </body>
                   </body>
                 </worldbody>
                 <tendon>
                   <fixed name="grip" range="-2 2" frictionloss="0.1">
                     <joint joint="j" coef="1"/>
                     <joint joint="k" coef="-0.5"/>
                   </fixed>
                 </tendon>
                 <actuator>
                   <motor name="pull" tendon="grip" gear="80"/>
                   <position name="hold" tendon="grip" kp="9"/>
                   <velocity name="reel" tendon="grip" kv="3" ctrlrange="-4 4"/>
                   <general name="soft" tendon="grip" dyntype="filter" dynprm="0.02"/>
                 </actuator>
               </mujoco>"#,
        )
        .unwrap();
        assert_eq!(warnings, vec![]);
        let joint = |n: &str| *robot.joints.iter().find(|(_, j)| j.name == n).unwrap().0;
        let (&tid, tendon) = robot.tendons.iter().next().expect("one tendon");
        assert_eq!(robot.tendons.len(), 1);
        assert_eq!(
            (
                tendon.name.as_str(),
                tendon.range,
                tendon.limited,
                tendon.stiffness,
                tendon.damping,
                tendon.frictionloss
            ),
            // `stiffness` and `damping` come from the class, `frictionloss`
            // from the element, and `limited` stays `auto` beside a range
            // under `autolimits` — the writer says the same thing.
            ("grip", Some([-2.0, 2.0]), None, 7.0, 0.25, 0.1)
        );
        assert_eq!(
            tendon
                .joints
                .iter()
                .map(|e| (e.joint, e.coef))
                .collect::<Vec<_>>(),
            [(joint("j"), 1.0), (joint("k"), -0.5)]
        );
        assert!(
            robot
                .actuators
                .values()
                .all(|a| a.target == ActuatorTarget::Tendon(tid)),
            "{:?}",
            robot.actuators
        );
        // A tendon has no `Limits` behind it, so the first actuator on it
        // fills nothing in — `j`'s effort and velocity are still unfilled.
        let limits = robot.joints[&joint("j")].limits.unwrap();
        assert_eq!((limits.effort, limits.velocity), (0.0, 0.0));

        let (store, errors) = MeshStore::load(&robot, &Disk);
        assert!(errors.is_empty(), "{errors:?}");
        let xml = crate::mjcf::write(&resolved(&robot, &store), &options());
        let block = |tag: &str| -> Vec<String> {
            xml.lines()
                .map(str::trim)
                .skip_while(|l| *l != format!("<{tag}>"))
                .skip(1)
                .take_while(|l| *l != format!("</{tag}>"))
                .map(str::to_owned)
                .collect()
        };
        assert_eq!(
            block("tendon"),
            [
                r#"<fixed name="grip" range="-2 2" stiffness="7" damping="0.25" frictionloss="0.1">"#,
                r#"<joint joint="j" coef="1"/>"#,
                r#"<joint joint="k" coef="-0.5"/>"#,
                r#"</fixed>"#,
            ],
            "{xml}"
        );
        assert_eq!(
            block("actuator"),
            [
                // What the file said is said back — `reel`'s `ctrlrange`
                // — and nothing else: a tendon has no `Limits` behind it,
                // so not even the motor's normalised `-1 1` is derived
                // where the file left the range open (ADR-0025 §4). Each
                // such actuator says `false` rather than leaving MuJoCo to
                // guess, exactly as a joint's would.
                r#"<motor name="pull" tendon="grip" gear="80" ctrllimited="false" forcelimited="false"/>"#,
                r#"<position name="hold" tendon="grip" kp="9" kv="0" ctrllimited="false" forcelimited="false"/>"#,
                r#"<velocity name="reel" tendon="grip" kv="3" ctrlrange="-4 4" forcelimited="false"/>"#,
                r#"<general name="soft" tendon="grip" dyntype="filter" gaintype="fixed" biastype="none" dynprm="0.02" gear="1" ctrllimited="false" forcelimited="false"/>"#,
            ],
            "{xml}"
        );
    }

    /// Everything about a `<tendon>` the document cannot hold is named,
    /// and named as a **tendon** (ADR-0025 §4): a `<spatial>`, and a
    /// `<fixed>` whose joints it has no room for — which goes whole, not
    /// half-read. `<equality><tendon>` is a different element of the same
    /// name and is counted like any other unread one, and an actuator on a
    /// tendon that went is dropped in its turn.
    #[test]
    fn a_tendon_the_document_cannot_hold_is_dropped_whole_and_by_name() {
        let (robot, warnings) = load(
            r#"<mujoco model="m"><compiler angle="radian" autolimits="true"/>
                 <worldbody>
                   <body name="a">
                     <body name="b"><joint name="j" range="-1 1"/>
                       <body name="c"><joint name="k" range="-1 1"/>
                         <body name="welded"/>
                       </body>
                     </body>
                   </body>
                 </worldbody>
                 <tendon>
                   <spatial name="routed"><site site="s1"/><site site="s2"/></spatial>
                   <fixed name="ghost"><joint joint="nope" coef="1"/></fixed>
                   <fixed name="frozen"><joint joint="welded_joint" coef="1"/></fixed>
                   <fixed name="twice"><joint joint="j" coef="1"/><joint joint="j" coef="2"/></fixed>
                   <fixed name="idle"><joint joint="j" coef="0"/></fixed>
                   <fixed name="bare"/>
                   <fixed name="backwards" range="3 1"><joint joint="j" coef="1"/></fixed>
                   <fixed name="keeper"><joint joint="k" coef="1"/></fixed>
                   <fixed name="keeper"><joint joint="j" coef="1"/></fixed>
                 </tendon>
                 <equality><tendon tendon1="keeper" tendon2="keeper"/></equality>
                 <actuator><motor name="orphan" tendon="ghost"/></actuator>
               </mujoco>"#,
        )
        .unwrap();
        assert_eq!(
            robot
                .tendons
                .values()
                .map(|t| t.name.as_str())
                .collect::<Vec<_>>(),
            ["keeper"],
            "the first of the two names stands, and nothing else survived"
        );
        assert!(robot.actuators.is_empty(), "{:?}", robot.actuators);
        let said: Vec<String> = warnings.iter().map(ToString::to_string).collect();
        for line in [
            "tendon \"routed\" dropped, <spatial> routes over sites and wrapping geoms; \
             the document holds a fixed tendon",
            "tendon \"ghost\" dropped, no joint \"nope\" is in the file",
            "tendon \"frozen\" dropped, joint \"welded_joint\" is fixed and has no value to combine",
            "tendon \"twice\" dropped, joint \"j\" is on it twice",
            "tendon \"idle\" dropped, joint \"j\" is weighed by zero",
            "tendon \"bare\" dropped, it runs over no joints",
            "tendon \"backwards\" dropped, its range 3 1 bounds nothing",
            "tendon \"keeper\" dropped, a tendon of that name is already in the file",
            "actuator \"orphan\" dropped, no tendon \"ghost\" is in the file",
            "<tendon> × 1: nothing in the document holds it; not read",
        ] {
            assert!(
                said.contains(&line.to_owned()),
                "missing {line:?}\n{said:#?}"
            );
        }
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
        assert!(robot.actuators.is_empty(), "{:?}", robot.actuators);
        assert!(robot.joints.values().any(|j| j.mimic.is_some()));
        // Each warning names the **actuator**, in its own namespace, not
        // the joint it was driving (ADR-0023).
        let said: Vec<String> = warnings.iter().map(ToString::to_string).collect();
        for line in [
            "actuator \"follower\" dropped, the joint is already driven by an <equality>",
            "actuator \"welded\" dropped, a fixed joint has no <joint> for it to drive",
        ] {
            assert!(
                said.contains(&line.to_owned()),
                "missing {line:?}\n{said:#?}"
            );
        }
        assert!(
            said.iter()
                .any(|s| s.starts_with("actuator \"sour\" dropped, its kp")),
            "{said:#?}"
        );
    }

    /// The preset rule (ADR-0024 §2): an element is a preset iff every
    /// attribute it carries is one the preset can express, else it is a
    /// `General` — MuJoCo's own reading of it. What no variant holds is
    /// counted once per attribute name (§4); what MuJoCo would refuse is a
    /// parse error or, past `validate`, a named drop. `<muscle>` and
    /// `<adhesion>` stay out, and `jointinparent` is named as the target
    /// it is.
    #[test]
    fn a_preset_is_read_iff_every_attribute_fits_it_else_a_general() {
        let (robot, warnings) = load(
            r#"<mujoco model="m"><compiler angle="radian"/><worldbody>
                 <body name="a">
                   <body name="b"><joint name="j" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>
                 </body>
               </worldbody>
               <actuator>
                 <position name="plain" joint="j" kp="10" kv="1" dampratio="1"/>
                 <position name="geared" joint="j" kp="10" gear="2"/>
                 <position name="filtered" joint="j" kp="10" kv="1" timeconst="0.05" inheritrange="1"/>
                 <velocity name="rate" joint="j" kv="3" gear="4"/>
                 <motor name="drive" joint="j" gear="5"/>
                 <general name="bare" joint="j"/>
                 <general name="full" joint="j" dyntype="integrator" gaintype="user" biastype="muscle"
                          dynprm="1 2" gainprm="3" biasprm="4 5 6" gear="7 0 0 0 0 0"
                          actdim="2" actearly="true" actrange="0 1" lengthrange="0 1" cranklength="0.1"/>
                 <general name="long" joint="j" gainprm="1 2 3 4 5 6 7 8 9 10 11"/>
                 <muscle name="sinew" joint="j"/>
                 <general name="parented" jointinparent="j"/>
                 <adhesion name="sticky" body="b"/>
                 <intvelocity name="ramp" joint="j" kp="1"/>
               </actuator>
               </mujoco>"#,
        )
        .unwrap();
        let by_name = |n: &str| {
            robot
                .actuators
                .values()
                .find(|a| a.name == n)
                .map(|a| a.spec.clone())
        };
        // Every attribute fits: a preset, its unread `dampratio` counted.
        assert_eq!(
            by_name("plain"),
            Some(ActuatorSpec::Position { kp: 10.0, kv: 1.0 })
        );
        // A `gear` a position servo cannot hold: what MuJoCo makes of it.
        assert_eq!(
            by_name("geared"),
            Some(ActuatorSpec::General(General {
                gaintype: GainType::Fixed,
                biastype: BiasType::Affine,
                gainprm: vec![10.0],
                biasprm: vec![0.0, -10.0, 0.0],
                gear: 2.0,
                ..General::default()
            }))
        );
        assert_eq!(
            by_name("filtered"),
            Some(ActuatorSpec::General(General {
                dyntype: DynType::FilterExact,
                gaintype: GainType::Fixed,
                biastype: BiasType::Affine,
                dynprm: vec![0.05],
                gainprm: vec![10.0],
                biasprm: vec![0.0, -10.0, -1.0],
                gear: 1.0,
            }))
        );
        assert_eq!(
            by_name("rate"),
            Some(ActuatorSpec::General(General {
                gaintype: GainType::Fixed,
                biastype: BiasType::Affine,
                gainprm: vec![3.0],
                biasprm: vec![0.0, 0.0, -3.0],
                gear: 4.0,
                ..General::default()
            }))
        );
        // A motor expresses `gear`: still a motor.
        assert_eq!(by_name("drive"), Some(ActuatorSpec::Motor { gear: 5.0 }));
        assert_eq!(
            by_name("bare"),
            Some(ActuatorSpec::General(General::default()))
        );
        assert_eq!(
            by_name("full"),
            Some(ActuatorSpec::General(General {
                dyntype: DynType::Integrator,
                gaintype: GainType::User,
                biastype: BiasType::Muscle,
                dynprm: vec![1.0, 2.0],
                gainprm: vec![3.0],
                biasprm: vec![4.0, 5.0, 6.0],
                gear: 7.0,
            }))
        );
        for gone in ["long", "sinew", "parented", "sticky", "ramp"] {
            assert_eq!(by_name(gone), None, "{gone}");
        }

        let said: Vec<String> = warnings.iter().map(ToString::to_string).collect();
        for line in [
            "actuator \"long\" dropped, its gainprm has 11 entries; MuJoCo holds at most 10",
            "actuator \"sinew\" dropped, a <muscle> needs a lengthrange riggen does not compute (ADR-0024)",
            "actuator \"parented\" dropped, it drives a joint through jointinparent, not a joint or a tendon",
            "actuator \"sticky\" dropped, it drives a body, not a joint or a tendon",
            "actuator \"ramp\" dropped, <intvelocity> is not one of the three presets or <general>",
            "<general actdim> × 1: nothing in the document holds it; not read",
            "<general actearly> × 1: nothing in the document holds it; not read",
            "<general actrange> × 1: nothing in the document holds it; not read",
            "<general cranklength> × 1: nothing in the document holds it; not read",
            "<general lengthrange> × 1: nothing in the document holds it; not read",
            "<position dampratio> × 1: nothing in the document holds it; not read",
            "<position inheritrange> × 1: nothing in the document holds it; not read",
        ] {
            assert!(
                said.contains(&line.to_owned()),
                "missing {line:?}\n{said:#?}"
            );
        }
        assert_eq!(said.len(), 12, "{said:#?}");

        // A type MuJoCo would not take is a parse error naming the choices.
        let err = load(
            r#"<mujoco model="m"><compiler angle="radian"/><worldbody>
                 <body name="a"><body name="b"><joint name="j"/></body></body>
               </worldbody>
               <actuator><general name="odd" joint="j" gaintype="spring"/></actuator>
               </mujoco>"#,
        )
        .unwrap_err()
        .to_string();
        assert!(
            err.contains(
                r#"<general> gaintype="spring": expected one of fixed, affine, muscle, user"#
            ),
            "{err}"
        );
    }

    /// The ranges come back as the file said them and go out the same way
    /// (ADR-0024): written flags explicit, `auto` beside a range left to
    /// `autolimits`, and an actuator that named no range recorded as
    /// unlimited rather than handed the joint's numbers — with the joint's
    /// `effort` / `velocity` still back-filled from the first actuator,
    /// which is where URDF reads them.
    #[test]
    fn actuator_ranges_survive_import_and_export_as_the_file_said_them() {
        let (robot, warnings) = load(
            r#"<mujoco model="m"><compiler angle="radian"/><worldbody>
                 <body name="a">
                   <body name="b"><joint name="j" range="-1 1"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                     <body name="c"><joint name="k" range="-2 2"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                       <body name="d"><joint name="l" range="-3 3"/><inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/></body>
                     </body>
                   </body>
                 </body>
               </worldbody>
               <actuator>
                 <position name="bare" joint="j" kp="10"/>
                 <position name="narrow" joint="j" kp="10" ctrlrange="-0.5 0.5" forcerange="-7 7"/>
                 <velocity name="rate" joint="k" kv="3" ctrllimited="true" ctrlrange="-4 4" forcelimited="false" forcerange="-9 9"/>
                 <motor name="drive" joint="l" gear="2" ctrllimited="auto" ctrlrange="0 0" forcerange="-5 5"/>
               </actuator>
               </mujoco>"#,
        )
        .unwrap();
        assert_eq!(warnings, vec![]);
        let ranges: Vec<(String, ActuatorRanges)> = robot
            .actuators
            .values()
            .map(|a| (a.name.clone(), a.ranges))
            .collect();
        let none = ActuatorRanges::default();
        assert_eq!(
            ranges,
            [
                // Nothing written: unlimited both ways, and said so, or
                // the writer would clamp `ctrl` to the joint's -1 1.
                (
                    "bare".to_owned(),
                    ActuatorRanges {
                        ctrl_limited: Some(false),
                        force_limited: Some(false),
                        ..none
                    }
                ),
                // Ranges under `auto`: the writer's `autolimits` says the
                // same thing, so the flags stay `auto`.
                (
                    "narrow".to_owned(),
                    ActuatorRanges {
                        ctrl: Some([-0.5, 0.5]),
                        force: Some([-7.0, 7.0]),
                        ..none
                    }
                ),
                // Written flags are kept as written.
                (
                    "rate".to_owned(),
                    ActuatorRanges {
                        ctrl: Some([-4.0, 4.0]),
                        force: Some([-9.0, 9.0]),
                        ctrl_limited: Some(true),
                        force_limited: Some(false),
                    }
                ),
                // `auto` spelled out is `auto`; a `0 0` range is kept as
                // the numbers it is — MuJoCo, not the import, decides it
                // does not limit.
                (
                    "drive".to_owned(),
                    ActuatorRanges {
                        ctrl: Some([0.0, 0.0]),
                        force: Some([-5.0, 5.0]),
                        ..none
                    }
                ),
            ]
        );
        // The joint's own numbers come from the **first** actuator on it:
        // `bare` said nothing, so `j` keeps an unfilled effort, `k` reads
        // both of `rate`'s, and `l` reads `drive`'s force.
        let joint = |n: &str| robot.joints.values().find(|j| j.name == n).unwrap();
        let limits = |n: &str| {
            let l = joint(n).limits.unwrap();
            (l.effort, l.velocity)
        };
        assert_eq!(limits("j"), (0.0, 0.0));
        assert_eq!(limits("k"), (9.0, 4.0));
        assert_eq!(limits("l"), (5.0, 0.0));

        // Out again: byte for byte what was said, in the writer's order,
        // and the joint's numbers used only where nothing was.
        let (store, errors) = MeshStore::load(&robot, &Disk);
        assert!(errors.is_empty(), "{errors:?}");
        let xml = crate::mjcf::write(&resolved(&robot, &store), &options());
        let block: Vec<&str> = xml
            .lines()
            .map(str::trim)
            .skip_while(|l| *l != "<actuator>")
            .skip(1)
            .take_while(|l| *l != "</actuator>")
            .collect();
        assert_eq!(
            block,
            [
                r#"<position name="bare" joint="j" kp="10" kv="0" ctrllimited="false" forcelimited="false"/>"#,
                r#"<position name="narrow" joint="j" kp="10" kv="0" ctrlrange="-0.5 0.5" forcerange="-7 7"/>"#,
                r#"<velocity name="rate" joint="k" kv="3" ctrllimited="true" ctrlrange="-4 4" forcelimited="false" forcerange="-9 9"/>"#,
                r#"<motor name="drive" joint="l" gear="2" ctrlrange="0 0" forcerange="-5 5"/>"#,
            ],
            "{xml}"
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
        let (robot, warnings, inline) =
            from_mjcf(&parse(text).unwrap(), &dir.join("m.xml"), &Disk).unwrap();
        assert_eq!(inline, Vec::new(), "no inline mesh in this file");
        // An unnamed `<mesh>` is known by its file's stem, as in MuJoCo.
        let CollisionPolicy::Meshes(geoms) = &robot.links.values().next().unwrap().collision else {
            panic!()
        };
        assert_eq!(geoms.len(), 5, "only the undeclared one is gone");
        let asset = |g: &Geom| &robot.assets[&g.mesh];
        assert_eq!(asset(&geoms[0]).path, dir.join("base.stl"));
        assert_eq!(asset(&geoms[0]).scale, 0.001);
        assert_ne!(asset(&geoms[0]).content_hash, 0, "the file was read");
        assert_eq!(asset(&geoms[1]).path, dir.join("upper.stl"));
        assert_eq!(asset(&geoms[2]).scale, 3.0, "the largest component");
        assert_eq!(asset(&geoms[3]).path, dir.join("thing.msh"));
        assert_ne!(
            asset(&geoms[3]).content_hash,
            0,
            "the .msh file was read too"
        );
        assert_eq!(asset(&geoms[4]).content_hash, 0, "nothing to hash");
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
    fn a_msh_mesh_geom_reads_with_no_warning() {
        let msh = crate::test_util::fixtures().join("cube.msh");
        let text = format!(
            r#"<mujoco model="m"><compiler angle="radian"/>
                 <asset><mesh name="m" file="{}"/></asset>
                 <worldbody><body name="a">
                   <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                   <geom type="mesh" mesh="m"/>
                 </body></worldbody></mujoco>"#,
            msh.display()
        );
        let (robot, warnings) = load(&text).unwrap();
        assert_eq!(warnings, vec![], "a .msh mesh is read like any other");
        let link = robot.links.values().next().unwrap();
        assert_eq!(link.visuals.len(), 1);
        assert_ne!(robot.assets[&link.visuals[0].mesh].content_hash, 0);
    }

    #[test]
    fn an_inline_mesh_geom_reads_with_no_warning_and_synthesizes_a_file() {
        let text = r#"<mujoco model="m"><compiler angle="radian"/>
                 <asset>
                   <mesh name="widget" vertex="0 0 0  1 0 0  0 1 0  0 0 1"
                         face="0 1 2  0 1 3  0 2 3  1 2 3"/>
                 </asset>
                 <worldbody><body name="a">
                   <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                   <geom type="mesh" mesh="widget"/>
                 </body></worldbody></mujoco>"#;
        let (robot, warnings, inline) =
            from_mjcf(&parse(text).unwrap(), Path::new("/nowhere/m.xml"), &Disk).unwrap();
        assert_eq!(warnings, vec![], "an inline mesh is read like any other");

        // The synthesized file — step 2's decision, docs/02-data-model.md
        // §Geometry — named after the `<mesh name>`, beside the source.
        assert_eq!(inline.len(), 1);
        let (name, bytes) = &inline[0];
        assert_eq!(name, "widget.stl");
        let parsed = riggen_mesh::parse_stl(bytes, Path::new(name)).unwrap();
        assert_eq!(parsed.triangle_count(), 4);

        let link = robot.links.values().next().unwrap();
        assert_eq!(link.visuals.len(), 1);
        let asset = &robot.assets[&link.visuals[0].mesh];
        assert_eq!(asset.path, Path::new("/nowhere/widget.stl"));
        assert_eq!(asset.content_hash, riggen_core::content_hash(bytes));
    }

    #[test]
    fn an_inline_mesh_with_no_face_is_still_refused() {
        // MJCF's implicit-convex-hull form (`vertex` with no `face`) is out
        // of scope (docs/plans/mjcf-mesh-geometry.md non-goals) and stays
        // refused, the way every file-less mesh was before this plan.
        let (robot, warnings) = load(
            r#"<mujoco model="m"><compiler angle="radian"/>
                 <asset><mesh name="hull" vertex="0 0 0 1 0 0 0 1 0"/></asset>
                 <worldbody><body name="a">
                   <inertial pos="0 0 0" mass="1" diaginertia="1 1 1"/>
                   <geom type="mesh" mesh="hull"/>
                 </body></worldbody></mujoco>"#,
        )
        .unwrap();
        let link = robot.links.values().next().unwrap();
        assert_eq!(link.visuals.len(), 0);
        assert_eq!(
            warnings,
            vec![ImportWarning::GeomDropped {
                link: "a".to_owned(),
                kind: "the inline <mesh \"hull\">".to_owned()
            }]
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

    /// The composite refusal is the one a real Menagerie file hits
    /// (`hello_robot_stretch`'s rubber tips), and it is a dead end unless
    /// it says what the shape means and how to get past it (ADR-0022 §2).
    #[test]
    fn the_composite_refusal_names_the_body_and_the_way_out() {
        let message = load(&model(
            r#"<body name="a"><body name="w"><joint name="w0"/><joint name="w1"/></body></body>"#,
        ))
        .unwrap_err()
        .to_string();
        for part in ["\"w\"", "w0", "w1", "2 joints", "nested bodies"] {
            assert!(message.contains(part), "{part:?} missing from {message:?}");
        }
        // It reaches the user as the status bar's one line (the app's
        // `finish_import` hands the `Display` string on as a `String`).
        assert!(!message.contains('\n'), "{message:?}");
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
            warnings[7].to_string(),
            "<sensor> × 1: nothing in the document holds it; not read"
        );
        // …and `<joint ref>` is not among them: it is `Joint::qpos_ref`
        // now, with the range shifted into the document's own terms
        // (ADR-0025 §3).
        let j = robot.joints.values().find(|j| j.name == "j").unwrap();
        assert_eq!(j.qpos_ref, 0.2);
        assert_eq!(j.limits.map(|l| (l.lower, l.upper)), Some((-1.2, 0.8)));
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
