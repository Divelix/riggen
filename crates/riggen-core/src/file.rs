//! The `.riggen` file: `{ "schema_version": 3, "robot": Robot }` as JSON
//! (docs/01-architecture.md §File format, docs/02-data-model.md §Schema).
//!
//! Mesh paths are **absolute in memory and relative to the file on disk**
//! (forward slashes): [`save`] rebases them on the way out, [`load`]
//! resolves them on the way in, so no other code ever sees a relative path.
//! Each asset carries an FNV-1a 64 hash of its mesh file; [`load`]
//! recomputes it and reports a difference as a [`Warning`], never an error —
//! the document still opens and the mesh still loads.
//!
//! A schema bump comes with an `upgrade_vN_to_vN+1` step and a corpus file
//! under `assets/fixtures/` that must open forever; `pendulum.riggen` is
//! the first, and it stays at schema 1 so the upgrade chain has something
//! old to read. [`load`] accepts every version from
//! [`OLDEST_SCHEMA_VERSION`] up and walks the chain to [`SCHEMA_VERSION`];
//! [`save`] always writes the newest.

use std::fmt;
use std::io;
use std::path::{Component, Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::ids::MeshId;
use crate::robot::Robot;
use crate::validate::{ValidationError, validate};

/// The version this build writes and the newest it reads. 3 since
/// `Joint::actuator` (ADR-0014).
pub const SCHEMA_VERSION: u32 = 3;

/// The oldest version [`load`] still accepts, upgrading it on the way in.
pub const OLDEST_SCHEMA_VERSION: u32 = 1;

/// The file envelope. `deny_unknown_fields` here too: a stray top-level key
/// is as much a typo as one inside `robot`.
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct File {
    schema_version: u32,
    robot: Robot,
}

/// Read first, tolerant of everything else, so an unsupported version is
/// reported as such rather than as an unknown field.
#[derive(Deserialize)]
struct Header {
    schema_version: u32,
}

#[derive(Debug)]
pub enum FileError {
    Io {
        path: PathBuf,
        source: io::Error,
    },
    /// Malformed JSON or a schema mismatch; serde's message names the
    /// offending field and its line/column.
    Json {
        path: PathBuf,
        source: serde_json::Error,
    },
    UnsupportedVersion {
        path: PathBuf,
        found: u32,
    },
    /// The document violates an invariant — on load, a hand-edited file; on
    /// save, a bug, since the command layer never produces one.
    Invalid {
        path: PathBuf,
        source: ValidationError,
    },
}

impl fmt::Display for FileError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Io { path, source } => write!(f, "{}: {source}", path.display()),
            Self::Json { path, source } => write!(f, "{}: {source}", path.display()),
            Self::UnsupportedVersion { path, found } => write!(
                f,
                "{}: schema version {found} is not one of the {OLDEST_SCHEMA_VERSION}–{SCHEMA_VERSION} this build reads",
                path.display()
            ),
            Self::Invalid { path, source } => write!(f, "{}: {source}", path.display()),
        }
    }
}

impl FileError {
    /// The same error against a different path — [`load`] uses it to quote
    /// the path it was given rather than its absolute form.
    fn at(self, path: &Path) -> Self {
        let path = path.to_owned();
        match self {
            Self::Io { source, .. } => Self::Io { path, source },
            Self::Json { source, .. } => Self::Json { path, source },
            Self::UnsupportedVersion { found, .. } => Self::UnsupportedVersion { path, found },
            Self::Invalid { source, .. } => Self::Invalid { path, source },
        }
    }
}

impl std::error::Error for FileError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Io { source, .. } => Some(source),
            Self::Json { source, .. } => Some(source),
            Self::UnsupportedVersion { .. } => None,
            Self::Invalid { source, .. } => Some(source),
        }
    }
}

/// Something worth telling the user about a file that did open.
#[derive(Debug, Clone, PartialEq)]
pub enum Warning {
    /// The mesh file's bytes are not the ones the document was saved with.
    HashMismatch {
        mesh: MeshId,
        path: PathBuf,
        expected: u64,
        found: u64,
    },
    /// The mesh file could not be read (moved, deleted, unreadable).
    MeshUnreadable {
        mesh: MeshId,
        path: PathBuf,
        reason: String,
    },
}

impl fmt::Display for Warning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::HashMismatch { mesh, path, .. } => write!(
                f,
                "mesh {mesh} ({}) has changed since the document was saved",
                path.display()
            ),
            Self::MeshUnreadable { mesh, path, reason } => {
                write!(
                    f,
                    "mesh {mesh} ({}) cannot be read: {reason}",
                    path.display()
                )
            }
        }
    }
}

/// The absolute, lexically normalized form of `path` — what the document
/// holds for every mesh path in memory. `std::path::absolute` alone keeps
/// `a/../b`, and two spellings of one file would compare unequal.
pub fn absolute(path: &Path) -> io::Result<PathBuf> {
    std::path::absolute(path).map(|p| normalized(&p))
}

/// FNV-1a 64 — small, dependency-free, and a change anywhere in a mesh file
/// flips it; not a security property.
pub fn content_hash(bytes: &[u8]) -> u64 {
    const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;
    bytes
        .iter()
        .fold(OFFSET, |h, &b| (h ^ u64::from(b)).wrapping_mul(PRIME))
}

/// [`content_hash`] of a file's bytes; what `MeshAsset::content_hash` holds.
pub fn hash_file(path: &Path) -> io::Result<u64> {
    Disk.hash(path)
}

/// Where the bytes behind a path come from (ADR-0017).
///
/// On the desktop that is the filesystem, [`Disk`]. In a browser there is
/// no filesystem to reach for: the bytes arrive with the drop gesture and
/// the paths in the document are resolved against *that* set instead. Every
/// reader in the workspace — the `.riggen` loader here, `MeshStore`, the
/// URDF and MJCF imports — reads through this one trait, so both worlds run
/// the same reader rather than a second, thinner web version of it
/// (docs/01-architecture.md §File format).
pub trait FileSource {
    fn read(&self, path: &Path) -> io::Result<Vec<u8>>;

    /// Whether `path` is readable. Only a probe — the URDF import walks
    /// candidate directories looking for a `package://` mesh — so a source
    /// with a cheaper answer than a full read should say so.
    fn exists(&self, path: &Path) -> bool {
        self.read(path).is_ok()
    }

    /// [`content_hash`] of what [`FileSource::read`] returns.
    fn hash(&self, path: &Path) -> io::Result<u64> {
        self.read(path).map(|bytes| content_hash(&bytes))
    }
}

/// The filesystem: what every native path takes.
#[derive(Debug, Clone, Copy, Default)]
pub struct Disk;

impl FileSource for Disk {
    fn read(&self, path: &Path) -> io::Result<Vec<u8>> {
        std::fs::read(path)
    }

    fn exists(&self, path: &Path) -> bool {
        path.is_file()
    }
}

/// Bytes held in memory, keyed by the exact path asked for. The browser's
/// source is built on this shape (a drop gesture's files), and it is what
/// the tests use to prove a reader never touches the disk.
#[derive(Debug, Clone, Default)]
pub struct MemorySource(pub std::collections::BTreeMap<PathBuf, Vec<u8>>);

impl MemorySource {
    pub fn insert(&mut self, path: impl Into<PathBuf>, bytes: Vec<u8>) {
        self.0.insert(path.into(), bytes);
    }
}

impl FileSource for MemorySource {
    fn read(&self, path: &Path) -> io::Result<Vec<u8>> {
        self.0.get(path).cloned().ok_or_else(|| {
            io::Error::new(
                io::ErrorKind::NotFound,
                format!("{} is not in this file set", path.display()),
            )
        })
    }

    fn exists(&self, path: &Path) -> bool {
        self.0.contains_key(path)
    }
}

/// Writes `robot` to `path`, mesh paths rebased relative to it, assets no
/// geom references dropped. The write goes through a sibling temp file and
/// a rename, so a crash mid-write leaves the old file intact.
pub fn save(robot: &Robot, path: &Path) -> Result<(), FileError> {
    let io = |source| FileError::Io {
        path: path.to_owned(),
        source,
    };
    validate(robot).map_err(|source| FileError::Invalid {
        path: path.to_owned(),
        source,
    })?;
    let path_abs = absolute(path).map_err(io)?;
    let dir = path_abs.parent().unwrap_or(Path::new("/"));

    let mut on_disk = robot.clone();
    let referenced = robot.referenced_assets();
    on_disk.assets.retain(|id, _| referenced.contains(id));
    for asset in on_disk.assets.values_mut() {
        let abs = absolute(&asset.path).map_err(io)?;
        asset.path = relative_to(dir, &abs);
    }

    let file = File {
        schema_version: SCHEMA_VERSION,
        robot: on_disk,
    };
    let mut json = serde_json::to_string_pretty(&file).map_err(|source| FileError::Json {
        path: path.to_owned(),
        source,
    })?;
    json.push('\n');

    let tmp = path_abs.with_extension("riggen.tmp");
    std::fs::write(&tmp, json).map_err(io)?;
    std::fs::rename(&tmp, &path_abs).map_err(io)
}

/// Reads `path` from the filesystem and hands it to [`load_from`].
pub fn load(path: &Path) -> Result<(Robot, Vec<Warning>), FileError> {
    let io = |source| FileError::Io {
        path: path.to_owned(),
        source,
    };
    let text = std::fs::read_to_string(path).map_err(io)?;
    let path_abs = absolute(path).map_err(io)?;
    // Errors name the path the caller gave, not the absolutised one: that
    // is the spelling the user typed and the one every message has quoted
    // since M1.
    load_from(&text, &path_abs, &Disk).map_err(|e| e.at(path))
}

/// Parses a `.riggen` document, resolves its mesh paths against `base`'s
/// directory, validates, and checks every mesh against its recorded hash
/// through `source`. Warnings are in `MeshId` order.
///
/// `base` is where the document *is* — its directory is what relative mesh
/// paths are relative to, and its name is what an error quotes. It must be
/// absolute and needs no filesystem behind it: in a browser it is the
/// dropped file under a synthetic root (ADR-0017).
pub fn load_from(
    text: &str,
    base: &Path,
    source: &dyn FileSource,
) -> Result<(Robot, Vec<Warning>), FileError> {
    let json = |source| FileError::Json {
        path: base.to_owned(),
        source,
    };
    let header: Header = serde_json::from_str(text).map_err(json)?;
    if !(OLDEST_SCHEMA_VERSION..=SCHEMA_VERSION).contains(&header.schema_version) {
        return Err(FileError::UnsupportedVersion {
            path: base.to_owned(),
            found: header.schema_version,
        });
    }
    // Every version so far parses into today's `Robot` — the fields added
    // since carry `#[serde(default)]` — so the chain runs on the parsed
    // document rather than on the JSON.
    let file: File = serde_json::from_str(text).map_err(json)?;
    let mut robot = file.robot;
    for from in header.schema_version..SCHEMA_VERSION {
        match from {
            1 => upgrade_v1_to_v2(&mut robot),
            2 => upgrade_v2_to_v3(&mut robot),
            _ => unreachable!("no upgrade step from schema {from}"),
        }
    }

    let dir = base.parent().unwrap_or(Path::new("/"));
    for asset in robot.assets.values_mut() {
        asset.path = resolve_against(dir, &asset.path);
    }
    validate(&robot).map_err(|source| FileError::Invalid {
        path: base.to_owned(),
        source,
    })?;

    let mut warnings = Vec::new();
    for (&mesh, asset) in &robot.assets {
        match source.hash(&asset.path) {
            Ok(found) if found == asset.content_hash => {}
            Ok(found) => warnings.push(Warning::HashMismatch {
                mesh,
                path: asset.path.clone(),
                expected: asset.content_hash,
                found,
            }),
            Err(e) => warnings.push(Warning::MeshUnreadable {
                mesh,
                path: asset.path.clone(),
                reason: e.to_string(),
            }),
        }
    }
    Ok((robot, warnings))
}

/// v1 → v2: `Joint::mimic` (ADR-0013). A v1 file simply has no `mimic`
/// key and serde's default fills in `None`, which is what a v1 document
/// meant, so the step is a no-op on the parsed document. It exists as the
/// first link of the chain [`load`] walks; the next bump joins it here.
fn upgrade_v1_to_v2(_robot: &mut Robot) {}

/// v2 → v3: `Joint::actuator` (ADR-0014), the same shape of bump and the
/// same empty step — a v2 file has no `actuator` key and `None` is what it
/// meant: nothing drove its joints.
fn upgrade_v2_to_v3(_robot: &mut Robot) {}

/// `target` expressed relative to `dir`, with `..` where needed and forward
/// slashes. Both must be absolute. A target on another Windows drive has no
/// relative form and stays absolute.
fn relative_to(dir: &Path, target: &Path) -> PathBuf {
    let (dir, target) = (normalized(dir), normalized(target));
    let dir: Vec<Component> = dir.components().collect();
    let target: Vec<Component> = target.components().collect();
    if dir.first() != target.first() {
        return target.iter().collect(); // different prefix / root
    }
    let common = dir.iter().zip(&target).take_while(|(a, b)| a == b).count();
    let mut parts: Vec<String> = std::iter::repeat_n("..".to_owned(), dir.len() - common).collect();
    parts.extend(
        target[common..]
            .iter()
            .map(|c| c.as_os_str().to_string_lossy().into_owned()),
    );
    PathBuf::from(parts.join("/"))
}

/// `rel` joined onto `dir` and lexically normalized (`.` and `..` folded);
/// an already absolute `rel` is only normalized.
fn resolve_against(dir: &Path, rel: &Path) -> PathBuf {
    if rel.is_absolute() {
        normalized(rel)
    } else {
        normalized(&dir.join(rel))
    }
}

/// Folds `.` and `..` without touching the filesystem (symlinks stay as
/// written, which is what a user who arranged them wants).
fn normalized(path: &Path) -> PathBuf {
    let mut out = PathBuf::new();
    for component in path.components() {
        match component {
            Component::CurDir => {}
            Component::ParentDir => {
                let popped = matches!(out.components().next_back(), Some(Component::Normal(_)));
                if popped {
                    out.pop();
                } else if !matches!(
                    out.components().next_back(),
                    Some(Component::RootDir | Component::Prefix(_))
                ) {
                    out.push("..");
                }
            }
            other => out.push(other),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::command::Command;
    use crate::pose::Pose;
    use crate::robot::{
        ActuatorSpec, CollisionPolicy, Geom, Joint, JointKind, Limits, Link, MeshAsset,
    };
    use riggen_mesh::glam::DVec3;
    use std::f64::consts::FRAC_PI_2;

    fn fixtures() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/fixtures")
    }

    /// A fresh, empty directory under the OS temp dir, unique per test.
    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("riggen-core-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    fn write_mesh(path: &Path, bytes: &[u8]) -> MeshAsset {
        std::fs::create_dir_all(path.parent().unwrap()).unwrap();
        std::fs::write(path, bytes).unwrap();
        MeshAsset {
            path: path.to_owned(),
            content_hash: content_hash(bytes),
            scale: 1.0,
            fix_up: None,
        }
    }

    fn geom(robot: &mut Robot, mesh: MeshId) -> Geom {
        Geom {
            id: robot.next_id.alloc(),
            mesh,
            pose: Pose::IDENTITY,
            color: None,
        }
    }

    /// base (mesh in the file's dir) ─ arm (mesh in a sibling dir), plus an
    /// asset nothing references.
    fn pendulum_in(dir: &Path) -> (Robot, PathBuf) {
        let mut robot = Robot::new("pendulum");
        let root = robot.root;
        let base_mesh = robot.add_asset(write_mesh(&dir.join("robot/base.stl"), b"base"));
        let arm_mesh = robot.add_asset(write_mesh(&dir.join("meshes/arm.stl"), b"arm"));
        let _unused = robot.add_asset(write_mesh(&dir.join("meshes/unused.stl"), b"?"));
        let g = geom(&mut robot, base_mesh);
        Command::AddGeom(root, g).apply(&mut robot).unwrap();
        let mut arm = Link::new("arm");
        arm.visuals.push(geom(&mut robot, arm_mesh));
        Command::AddLink {
            link: Box::new(arm),
            parent: root,
            joint: Joint {
                kind: JointKind::Revolute,
                axis: DVec3::Y,
                origin: Pose::from_translation(DVec3::new(0.0, 0.0, 0.5)),
                limits: Some(Limits {
                    lower: -FRAC_PI_2,
                    upper: FRAC_PI_2,
                    effort: 10.0,
                    velocity: 3.0,
                }),
                ..Joint::fixed("hinge", root, root)
            },
        }
        .apply(&mut robot)
        .unwrap();
        (robot, dir.join("robot/pendulum.riggen"))
    }

    #[test]
    fn fnv1a_matches_the_reference_vectors() {
        assert_eq!(content_hash(b""), 0xcbf2_9ce4_8422_2325);
        assert_eq!(content_hash(b"a"), 0xaf63_dc4c_8601_ec8c);
        assert_eq!(content_hash(b"foobar"), 0x8594_4171_f739_67e8);
        assert_eq!(
            hash_file(&fixtures().join("cube_binary.stl")).unwrap(),
            16366162079779491545
        );
    }

    #[test]
    fn round_trip_is_equal_and_prunes_unreferenced_assets() {
        let dir = scratch("round_trip");
        let (mut robot, file) = pendulum_in(&dir);
        assert_eq!(robot.assets.len(), 3);
        save(&robot, &file).unwrap();
        let (back, warnings) = load(&file).unwrap();
        assert_eq!(warnings, vec![]);
        // The unreferenced asset is gone from the file, nothing else changed.
        let referenced = robot.referenced_assets();
        robot.assets.retain(|id, _| referenced.contains(id));
        assert_eq!(back, robot);
        assert!(
            back.assets.values().all(|a| a.path.is_absolute()),
            "{:?}",
            back.assets
        );
        assert!(!file.with_extension("riggen.tmp").exists());
    }

    #[test]
    fn written_json_holds_relative_forward_slash_paths_and_the_version() {
        let dir = scratch("relative");
        let (robot, file) = pendulum_in(&dir);
        save(&robot, &file).unwrap();
        let text = std::fs::read_to_string(&file).unwrap();
        assert!(
            text.starts_with("{\n  \"schema_version\": 3,\n  \"robot\": {"),
            "{text}"
        );
        assert!(text.contains("\"path\": \"base.stl\""), "{text}");
        assert!(text.contains("\"path\": \"../meshes/arm.stl\""), "{text}");
        assert!(!text.contains("unused.stl"), "{text}");
        assert!(text.ends_with("}\n"));
    }

    #[test]
    fn hash_mismatch_and_missing_mesh_are_warnings_not_errors() {
        let dir = scratch("warnings");
        let (robot, file) = pendulum_in(&dir);
        save(&robot, &file).unwrap();
        std::fs::write(dir.join("meshes/arm.stl"), b"arm v2").unwrap();
        std::fs::remove_file(dir.join("robot/base.stl")).unwrap();
        let (back, warnings) = load(&file).unwrap();
        assert_eq!(back.links.len(), 2);
        let mut ids = back.assets.keys();
        let (base_mesh, arm_mesh) = (*ids.next().unwrap(), *ids.next().unwrap());
        assert_eq!(warnings.len(), 2);
        assert!(
            matches!(&warnings[0], Warning::MeshUnreadable { mesh, .. } if *mesh == base_mesh),
            "{warnings:?}"
        );
        assert_eq!(
            warnings[1],
            Warning::HashMismatch {
                mesh: arm_mesh,
                path: dir.join("meshes/arm.stl"),
                expected: content_hash(b"arm"),
                found: content_hash(b"arm v2"),
            }
        );
        assert!(warnings[1].to_string().contains("has changed since"));
    }

    #[test]
    fn unknown_field_is_an_error_naming_it() {
        let dir = scratch("unknown_field");
        let (robot, file) = pendulum_in(&dir);
        save(&robot, &file).unwrap();
        let text = std::fs::read_to_string(&file).unwrap();
        for (from, to) in [
            ("\"materials\"", "\"materialz\""),
            ("\"robot\"", "\"robbot\""),
            ("\"velocity\"", "\"velocty\""),
        ] {
            std::fs::write(&file, text.replacen(from, to, 1)).unwrap();
            let err = load(&file).unwrap_err();
            let msg = err.to_string();
            assert!(matches!(err, FileError::Json { .. }), "{msg}");
            assert!(msg.contains(to.trim_matches('"')), "{msg}");
        }
    }

    #[test]
    fn newer_version_and_invalid_document_are_errors() {
        let dir = scratch("version");
        let (robot, file) = pendulum_in(&dir);
        save(&robot, &file).unwrap();
        let text = std::fs::read_to_string(&file).unwrap();
        for bogus in [0, SCHEMA_VERSION + 1] {
            std::fs::write(
                &file,
                text.replacen(
                    "\"schema_version\": 3",
                    &format!("\"schema_version\": {bogus}"),
                    1,
                ),
            )
            .unwrap();
            let err = load(&file).unwrap_err();
            assert!(
                matches!(err, FileError::UnsupportedVersion { found, .. } if found == bogus),
                "{err:?}"
            );
            assert!(err.to_string().contains("1–3"), "{err}");
        }
        std::fs::write(&file, &text).unwrap();
        // A hand-edited file that breaks an invariant.
        std::fs::write(
            &file,
            text.replacen("\"upper\": 1.5707963267948966", "\"upper\": -3.0", 1),
        )
        .unwrap();
        assert!(matches!(
            load(&file),
            Err(FileError::Invalid {
                source: ValidationError::LimitsUnordered { .. },
                ..
            })
        ));
        // And saving one is refused before anything is written.
        let mut broken = robot.clone();
        let root = broken.root;
        broken.links.get_mut(&root).unwrap().name = "1".into();
        let target = dir.join("never.riggen");
        assert!(matches!(
            save(&broken, &target),
            Err(FileError::Invalid { .. })
        ));
        assert!(!target.exists());
        assert!(matches!(
            load(&dir.join("nope.riggen")),
            Err(FileError::Io { .. })
        ));
    }

    /// `assets/fixtures/arm/arm.riggen`, the M3 sample robot (written by
    /// `write_arm_sample` in the app's visual tests): four parts on three
    /// revolute joints, every mesh hashed, and a re-save reproduces the
    /// committed bytes.
    #[test]
    fn corpus_sample_arm_opens() {
        let file = fixtures().join("arm/arm.riggen");
        let (robot, warnings) = load(&file).unwrap();
        assert_eq!(warnings, vec![], "fixture meshes must match their hashes");
        assert_eq!(robot.name, "arm");
        assert_eq!(robot.links.len(), 5, "root plus four parts");
        assert_eq!(robot.joints.len(), 4);
        // Two named frames, saved and read back with the rest (ADR-0012);
        // `frames` is a v1 field that finally holds something, so the
        // schema does not move.
        assert_eq!(SCHEMA_VERSION, 3, "the actuator is schema 3 (ADR-0014)");
        assert_eq!(robot.frames.len(), 2);
        let frame = |n: &str| robot.frames.values().find(|f| f.name == n).unwrap();
        assert_eq!(frame("tcp").pose.t, DVec3::new(0.0, 0.0, 0.08));
        assert_eq!(
            robot.links[&frame("tcp").parent].name,
            "fore",
            "the TCP is on the last link"
        );
        assert_eq!(robot.links[&frame("camera_mount").parent].name, "base");
        let revolute = robot
            .joints
            .values()
            .filter(|j| j.kind == JointKind::Revolute)
            .count();
        assert_eq!(revolute, 3);
        assert!(
            robot
                .joints
                .values()
                .all(|j| !j.kind.requires_limits() || j.limits.is_some())
        );
        assert!(
            robot
                .links
                .values()
                .all(|l| l.name == "base_link" || l.material.is_some())
        );
        for asset in robot.assets.values() {
            assert_eq!(asset.scale, 0.001, "the STLs are in millimetres");
            assert!(asset.path.exists(), "{}", asset.path.display());
        }
        // The forearm's tip is where the design says: 0.235 m up at rest,
        // and the shoulder swings it about Z.
        let world = crate::fk(&robot, &crate::JointState::default());
        let fore = *robot
            .links
            .iter()
            .find(|(_, l)| l.name == "fore")
            .unwrap()
            .0;
        assert!((world[&fore].t - DVec3::new(0.0, 0.0, 0.195)).length() < 1e-12);
        // …and the TCP rides it, 80 mm further out — but no longer
        // straight up: `fore_joint` follows `upper_joint` at
        // `-0.5 q + 0.1` (ADR-0013), so the forearm sits at 0.1 rad about
        // Y even in the rest configuration, and the sample's whole point
        // is that the coupling is visible in the numbers.
        let fore_joint = *robot
            .joints
            .iter()
            .find(|(_, j)| j.name == "fore_joint")
            .unwrap()
            .0;
        let mimic = robot.joints[&fore_joint].mimic.expect("the coupling");
        assert_eq!(robot.joints[&mimic.joint].name, "upper_joint");
        assert_eq!((mimic.multiplier, mimic.offset), (-0.5, 0.1));
        // The two joints nothing else drives carry an actuator each
        // (ADR-0014); the forearm follows, so it carries none.
        let actuator = |name: &str| {
            robot
                .joints
                .values()
                .find(|j| j.name == name)
                .unwrap()
                .actuator
        };
        assert_eq!(
            actuator("shoulder_joint"),
            Some(ActuatorSpec::Position {
                kp: 100.0,
                kv: 10.0
            })
        );
        assert_eq!(
            actuator("upper_joint"),
            Some(ActuatorSpec::Velocity { kv: 8.0 })
        );
        assert_eq!(actuator("fore_joint"), None, "a follower is already driven");
        assert_eq!(
            crate::resolve_q(&robot, &crate::JointState::default()).get(fore_joint),
            0.1
        );
        let tcp = *robot
            .frames
            .iter()
            .find(|(_, f)| f.name == "tcp")
            .unwrap()
            .0;
        let world_frames = crate::frames(&robot, &crate::JointState::default());
        let swung = DVec3::new(0.08 * 0.1_f64.sin(), 0.0, 0.195 + 0.08 * 0.1_f64.cos());
        assert!((world_frames[&tcp].t - swung).length() < 1e-12);

        let dir = scratch("corpus-arm");
        for mesh in ["base.stl", "shoulder.stl", "upper.stl", "fore.stl"] {
            std::fs::copy(fixtures().join("arm").join(mesh), dir.join(mesh)).unwrap();
        }
        let mut relocated = robot.clone();
        for asset in relocated.assets.values_mut() {
            asset.path = dir.join(asset.path.file_name().unwrap());
        }
        let again = dir.join("arm.riggen");
        save(&relocated, &again).unwrap();
        assert_eq!(
            std::fs::read_to_string(&again).unwrap(),
            std::fs::read_to_string(&file).unwrap(),
            "re-saving the sample must reproduce the committed bytes"
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn corpus_pendulum_opens() {
        let file = fixtures().join("pendulum.riggen");
        let (robot, warnings) = load(&file).unwrap();
        assert_eq!(warnings, vec![], "fixture meshes must match their hashes");
        assert_eq!(robot.name, "pendulum");
        assert_eq!(robot.links.len(), 2);
        assert_eq!(robot.joints.len(), 1);
        let hinge = robot.joints.values().next().unwrap();
        assert_eq!(hinge.name, "hinge");
        assert_eq!(hinge.kind, JointKind::Revolute);
        assert!(hinge.limits.is_some());
        let names: Vec<&str> = robot.links.values().map(|l| l.name.as_str()).collect();
        assert_eq!(names, ["base_link", "arm"]);
        for asset in robot.assets.values() {
            assert!(asset.path.is_absolute());
            assert!(asset.path.exists(), "{}", asset.path.display());
        }
        // It is the **v1** corpus and stays one forever: the upgrade chain
        // needs a real old document to read (§Schema, ADR-0013). So this
        // one cannot also be the byte-for-byte fixture — `bracket.riggen`
        // and `arm/arm.riggen` are, at v3.
        let text = std::fs::read_to_string(&file).unwrap();
        assert!(text.contains("\"schema_version\": 1"), "{text}");
        assert!(!text.contains("mimic"), "a v1 file has no mimic key");
        assert!(!text.contains("actuator"), "nor an actuator key");
        assert!(
            robot.joints.values().all(|j| j.mimic.is_none()),
            "upgrade_v1_to_v2 fills mimic in as None"
        );
        assert!(
            robot.joints.values().all(|j| j.actuator.is_none()),
            "and upgrade_v2_to_v3 fills actuator in as None"
        );

        // Re-saving it writes v3, and that round-trips to the same document.
        let dir = scratch("corpus");
        let again = dir.join("pendulum.riggen");
        // Relative paths only survive a same-directory save; copy the meshes.
        for mesh in ["cube_binary.stl", "cube_ascii.stl"] {
            std::fs::copy(fixtures().join(mesh), dir.join(mesh)).unwrap();
        }
        let mut relocated = robot.clone();
        for asset in relocated.assets.values_mut() {
            asset.path = dir.join(asset.path.file_name().unwrap());
        }
        save(&relocated, &again).unwrap();
        let upgraded = std::fs::read_to_string(&again).unwrap();
        assert!(upgraded.contains("\"schema_version\": 3"), "{upgraded}");
        assert!(upgraded.contains("\"mimic\": null"), "{upgraded}");
        assert!(upgraded.contains("\"actuator\": null"), "{upgraded}");
        assert_eq!(load(&again).unwrap().0, relocated);
    }

    /// A v2 document — one written before `Joint::actuator` existed
    /// (ADR-0014) — opens with `None` on every joint and re-saves as v3.
    /// Built by stripping the key back out of the committed v3 fixture, so
    /// it is a whole real document rather than a fragment (§Schema).
    #[test]
    fn a_v2_file_opens_as_v3_with_no_actuators() {
        let dir = scratch("v2");
        std::fs::copy(fixtures().join("bracket.stl"), dir.join("bracket.stl")).unwrap();
        let text = std::fs::read_to_string(fixtures().join("bracket.riggen")).unwrap();
        let mut doc: serde_json::Value = serde_json::from_str(&text).unwrap();
        doc["schema_version"] = 2.into();
        for joint in doc["robot"]["joints"].as_object_mut().unwrap().values_mut() {
            assert!(
                joint.as_object_mut().unwrap().remove("actuator").is_some(),
                "the v3 fixture has the key this strips back out"
            );
        }
        let old = serde_json::to_string_pretty(&doc).unwrap();
        assert!(!old.contains("actuator"), "{old}");
        let file = dir.join("bracket.riggen");
        std::fs::write(&file, &old).unwrap();

        let (robot, warnings) = load(&file).unwrap();
        assert_eq!(warnings, vec![]);
        assert!(robot.joints.values().all(|j| j.actuator.is_none()));
        save(&robot, &file).unwrap();
        let upgraded = std::fs::read_to_string(&file).unwrap();
        assert!(upgraded.contains("\"schema_version\": 3"), "{upgraded}");
        assert!(upgraded.contains("\"actuator\": null"), "{upgraded}");
        assert_eq!(load(&file).unwrap().0, robot);
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// A `.riggen` written before `ConvexDecomposition` grew `resolution`
    /// and `concavity` still opens, with the algorithm's defaults filled in
    /// — the `#[serde(default)]` promise that keeps this a v1 file and not
    /// a schema bump (§Schema, ADR-0011). Built from the committed corpus
    /// file so it is a whole real document, not a fragment.
    #[test]
    fn a_v1_file_with_only_max_hulls_reads_with_the_defaults() {
        let dir = scratch("decomp-v1");
        for mesh in ["cube_binary.stl", "cube_ascii.stl"] {
            std::fs::copy(fixtures().join(mesh), dir.join(mesh)).unwrap();
        }
        let text = std::fs::read_to_string(fixtures().join("pendulum.riggen")).unwrap();
        let old = text.replacen(
            "\"collision\": \"SameAsVisual\"",
            "\"collision\": { \"ConvexDecomposition\": { \"max_hulls\": 4 } }",
            1,
        );
        assert_ne!(old, text, "the corpus file must have a collision field");
        let file = dir.join("pendulum.riggen");
        std::fs::write(&file, &old).unwrap();

        let (robot, warnings) = load(&file).unwrap();
        assert_eq!(warnings, vec![]);
        let defaults = riggen_mesh::DecompParams::default();
        let base = &robot.links[&robot.root].collision;
        assert_eq!(
            *base,
            CollisionPolicy::ConvexDecomposition {
                max_hulls: 4,
                resolution: defaults.resolution,
                concavity: defaults.concavity,
            },
            "the two new fields come from riggen_mesh::DecompParams"
        );

        // And the filled-in document round-trips through save/load.
        let again = dir.join("again.riggen");
        save(&robot, &again).unwrap();
        assert_eq!(load(&again).unwrap().0, robot);
    }

    /// Every collision policy survives the document's JSON, the widened
    /// variant included.
    #[test]
    fn collision_policies_round_trip_through_json() {
        let defaults = riggen_mesh::DecompParams::default();
        for policy in [
            CollisionPolicy::None,
            CollisionPolicy::SameAsVisual,
            CollisionPolicy::ConvexHull,
            CollisionPolicy::ConvexDecomposition {
                max_hulls: defaults.max_hulls,
                resolution: defaults.resolution,
                concavity: defaults.concavity,
            },
            CollisionPolicy::ConvexDecomposition {
                max_hulls: 3,
                resolution: 96,
                concavity: 0.004,
            },
        ] {
            let json = serde_json::to_string(&policy).unwrap();
            assert_eq!(
                serde_json::from_str::<CollisionPolicy>(&json).unwrap(),
                policy,
                "{json}"
            );
        }
        assert_eq!(
            serde_json::to_string(&CollisionPolicy::ConvexDecomposition {
                max_hulls: 3,
                resolution: 96,
                concavity: 0.004,
            })
            .unwrap(),
            r#"{"ConvexDecomposition":{"max_hulls":3,"resolution":96,"concavity":0.004}}"#
        );
    }

    /// `assets/fixtures/bracket.riggen`, the decomposition acceptance's
    /// model (plans/convex-decomposition step 8): the U-channel of
    /// `bracket.stl` on a revolute joint, its collision a
    /// `ConvexDecomposition`. The `mujoco` CI job exports it and checks
    /// that the decomposed body carries several collision geoms and that
    /// `mj_forward` agrees with our FK.
    fn bracket_sample() -> Robot {
        use crate::robot::{Geom, InertialSpec, Joint, JointKind, Limits, Link, MeshAsset};

        // Normalised the way `load` returns it, so the two compare equal.
        let path = absolute(&fixtures().join("bracket.stl")).unwrap();
        let mut robot = Robot::new("bracket");
        let base = robot.root;
        robot.links.get_mut(&base).unwrap().material = Some("aluminium".into());
        let mesh = robot.add_asset(MeshAsset {
            content_hash: crate::hash_file(&path).unwrap(),
            path,
            scale: 0.001, // the fixture is in millimetres, like the arm's parts
            fix_up: None,
        });
        let mut link = Link::new("bracket");
        link.material = Some("PLA".into());
        link.inertial = InertialSpec::Computed {
            density_override: None,
        };
        // The pieces are derived at export from these three numbers; the
        // document never stores them (ADR-0011).
        link.collision = CollisionPolicy::ConvexDecomposition {
            max_hulls: 4,
            resolution: 48,
            concavity: 0.01,
        };
        link.visuals.push(Geom {
            id: robot.next_id.alloc(),
            mesh,
            pose: Pose::IDENTITY,
            color: None,
        });
        let joint = Joint {
            kind: JointKind::Revolute,
            axis: DVec3::Y,
            origin: Pose::from_translation(DVec3::new(0.0, 0.0, 0.1)),
            limits: Some(Limits {
                lower: -FRAC_PI_2,
                upper: FRAC_PI_2,
                effort: 5.0,
                velocity: 3.0,
            }),
            // The third actuator preset (ADR-0014), so the MuJoCo
            // acceptance sees a `<motor>` too: the arm carries the two
            // servos, and this hinge is the only other joint the CI job
            // exports.
            actuator: Some(crate::robot::ActuatorSpec::Motor { gear: 50.0 }),
            ..Joint::fixed("hinge", base, base)
        };
        Command::AddLink {
            link: Box::new(link),
            parent: base,
            joint,
        }
        .apply(&mut robot)
        .unwrap();
        robot
    }

    /// Regenerates `assets/fixtures/bracket.riggen`. Ignored like the other
    /// fixture generators; the test below keeps the committed bytes in step
    /// with it. `cargo test -p riggen-core write_bracket_sample --
    /// --ignored`.
    #[test]
    #[ignore = "writes the committed fixture; run on purpose"]
    fn write_bracket_sample() {
        save(&bracket_sample(), &fixtures().join("bracket.riggen")).unwrap();
    }

    #[test]
    fn corpus_bracket_opens_and_matches_its_generator() {
        let file = fixtures().join("bracket.riggen");
        let (robot, warnings) = load(&file).unwrap();
        assert_eq!(warnings, vec![], "the fixture mesh must match its hash");
        assert_eq!(robot, bracket_sample());
        let bracket = robot.links.values().find(|l| l.name == "bracket").unwrap();
        assert_eq!(
            bracket.collision,
            CollisionPolicy::ConvexDecomposition {
                max_hulls: 4,
                resolution: 48,
                concavity: 0.01,
            }
        );
        // The hinge is the `<motor>` the MuJoCo acceptance checks
        // (ADR-0014); the arm's two joints carry the other two presets.
        assert_eq!(
            robot.joints.values().next().unwrap().actuator,
            Some(ActuatorSpec::Motor { gear: 50.0 })
        );

        // Saving it again reproduces the committed bytes.
        let dir = scratch("bracket");
        std::fs::copy(fixtures().join("bracket.stl"), dir.join("bracket.stl")).unwrap();
        let again = dir.join("bracket.riggen");
        let mut relocated = robot.clone();
        for asset in relocated.assets.values_mut() {
            asset.path = dir.join(asset.path.file_name().unwrap());
        }
        save(&relocated, &again).unwrap();
        assert_eq!(
            std::fs::read_to_string(&again).unwrap(),
            std::fs::read_to_string(&file).unwrap()
        );
    }

    #[test]
    fn relative_and_resolve_are_inverses() {
        let dir = Path::new("/home/u/proj/robot");
        for (target, want) in [
            ("/home/u/proj/robot/a.stl", "a.stl"),
            ("/home/u/proj/robot/sub/a.stl", "sub/a.stl"),
            ("/home/u/proj/meshes/a.stl", "../meshes/a.stl"),
            ("/tmp/a.stl", "../../../../tmp/a.stl"),
        ] {
            let rel = relative_to(dir, Path::new(target));
            assert_eq!(rel, Path::new(want), "{target}");
            assert_eq!(resolve_against(dir, &rel), Path::new(target), "{target}");
        }
        assert_eq!(
            resolve_against(dir, Path::new("/abs/x.stl")),
            Path::new("/abs/x.stl")
        );
        assert_eq!(normalized(Path::new("/a/./b/../c//d")), Path::new("/a/c/d"));
        assert_eq!(normalized(Path::new("/../a")), Path::new("/a"));
        assert_eq!(normalized(Path::new("../a")), Path::new("../a"));
        // `absolute` folds `..` so two spellings of one file compare equal.
        let here = std::env::current_dir().unwrap();
        assert_eq!(
            absolute(Path::new("sub/../x.stl")).unwrap(),
            normalized(&here).join("x.stl")
        );
    }

    /// The whole point of [`FileSource`] (ADR-0017): the same document
    /// opened from a set of bytes that has no directory behind it, and the
    /// result identical to the on-disk load once the meshes' directory is
    /// accounted for. The synthetic root does not exist, so any read that
    /// slipped through to the filesystem would fail and show up as a
    /// `MeshUnreadable` warning.
    #[test]
    fn load_from_memory_matches_load_from_disk() {
        let dir = fixtures().join("arm");
        let root = Path::new("/dropped");
        assert!(!root.exists(), "the synthetic root must not exist");

        let mut memory = MemorySource::default();
        for name in [
            "arm.riggen",
            "base.stl",
            "shoulder.stl",
            "upper.stl",
            "fore.stl",
        ] {
            memory.insert(root.join(name), std::fs::read(dir.join(name)).unwrap());
        }
        let text = String::from_utf8(memory.read(&root.join("arm.riggen")).unwrap()).unwrap();
        let (from_memory, memory_warnings) =
            load_from(&text, &root.join("arm.riggen"), &memory).unwrap();
        assert_eq!(memory_warnings, Vec::new());

        let (mut from_disk, disk_warnings) = load(&dir.join("arm.riggen")).unwrap();
        assert_eq!(disk_warnings, Vec::new());
        // The one thing that legitimately differs: where the meshes are.
        for asset in from_disk.assets.values_mut() {
            asset.path = root.join(asset.path.file_name().unwrap());
        }
        assert_eq!(
            serde_json::to_value(&from_memory).unwrap(),
            serde_json::to_value(&from_disk).unwrap()
        );
    }

    /// A mesh the set does not carry is a warning, not an error: the
    /// document still opens, exactly as a moved file on disk does.
    #[test]
    fn load_from_warns_for_every_mesh_the_set_is_missing() {
        let dir = fixtures().join("arm");
        let root = Path::new("/dropped");
        let mut memory = MemorySource::default();
        memory.insert(
            root.join("arm.riggen"),
            std::fs::read(dir.join("arm.riggen")).unwrap(),
        );
        let text = String::from_utf8(memory.read(&root.join("arm.riggen")).unwrap()).unwrap();
        let (_, warnings) = load_from(&text, &root.join("arm.riggen"), &memory).unwrap();
        assert_eq!(warnings.len(), 4, "{warnings:?}");
        assert!(
            warnings
                .iter()
                .all(|w| matches!(w, Warning::MeshUnreadable { .. }))
        );
    }
}
