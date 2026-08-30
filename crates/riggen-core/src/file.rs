//! The `.riggen` file: `{ "schema_version": 1, "robot": Robot }` as JSON
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
//! the first.

use std::fmt;
use std::io;
use std::path::{Component, Path, PathBuf};

use serde::{Deserialize, Serialize};

use crate::ids::MeshId;
use crate::robot::Robot;
use crate::validate::{ValidationError, validate};

/// The version this build writes and the newest it reads.
pub const SCHEMA_VERSION: u32 = 1;

/// The file envelope. `deny_unknown_fields` here too: a stray top-level key
/// is as much a typo as one inside `robot`.
#[derive(Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
struct FileV1 {
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
                "{}: schema version {found} is newer than the {SCHEMA_VERSION} this build reads",
                path.display()
            ),
            Self::Invalid { path, source } => write!(f, "{}: {source}", path.display()),
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
    std::fs::read(path).map(|bytes| content_hash(&bytes))
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

    let file = FileV1 {
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

/// Reads `path`, resolves mesh paths against its directory, validates, and
/// checks every mesh file against its recorded hash. Warnings are in
/// `MeshId` order.
pub fn load(path: &Path) -> Result<(Robot, Vec<Warning>), FileError> {
    let text = std::fs::read_to_string(path).map_err(|source| FileError::Io {
        path: path.to_owned(),
        source,
    })?;
    let json = |source| FileError::Json {
        path: path.to_owned(),
        source,
    };
    let header: Header = serde_json::from_str(&text).map_err(json)?;
    if header.schema_version != SCHEMA_VERSION {
        return Err(FileError::UnsupportedVersion {
            path: path.to_owned(),
            found: header.schema_version,
        });
    }
    let file: FileV1 = serde_json::from_str(&text).map_err(json)?;
    let mut robot = file.robot;

    let path_abs = absolute(path).map_err(|source| FileError::Io {
        path: path.to_owned(),
        source,
    })?;
    let dir = path_abs.parent().unwrap_or(Path::new("/"));
    for asset in robot.assets.values_mut() {
        asset.path = resolve_against(dir, &asset.path);
    }
    validate(&robot).map_err(|source| FileError::Invalid {
        path: path.to_owned(),
        source,
    })?;

    let mut warnings = Vec::new();
    for (&mesh, asset) in &robot.assets {
        match hash_file(&asset.path) {
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
    use crate::robot::{CollisionPolicy, Geom, Joint, JointKind, Limits, Link, MeshAsset};
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
            text.starts_with("{\n  \"schema_version\": 1,\n  \"robot\": {"),
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
        std::fs::write(
            &file,
            text.replacen("\"schema_version\": 1", "\"schema_version\": 2", 1),
        )
        .unwrap();
        assert!(matches!(
            load(&file),
            Err(FileError::UnsupportedVersion { found: 2, .. })
        ));
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
        // Saving it again reproduces the committed bytes: the fixture is
        // what `save` writes, so a format drift shows up here.
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
        assert_eq!(
            std::fs::read_to_string(&again).unwrap(),
            std::fs::read_to_string(&file).unwrap()
        );
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
}
