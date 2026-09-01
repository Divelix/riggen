//! Writing the export directory (ADR-0008, ADR-0016): any of `<name>.xml`,
//! `<name>.urdf` and `<name>.sdf` beside one `meshes/` folder of binary STL
//! in meters, every file through a `.tmp` sibling and a rename so a crash
//! mid-write never leaves a half file behind — the same discipline as
//! `file::save`.

use std::fmt;
use std::io;
use std::path::{Path, PathBuf};

use crate::resolve::{ExportOptions, ResolvedRobot};
use crate::{mjcf, sdf, urdf};

/// A file that could not be written.
#[derive(Debug)]
pub struct ExportIoError {
    pub path: PathBuf,
    pub source: io::Error,
}

impl fmt::Display for ExportIoError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.path.display(), self.source)
    }
}

impl std::error::Error for ExportIoError {}

/// The export directory as bytes, without touching a filesystem: `(path,
/// contents)` for every file, model files first, then meshes in stem
/// order. [`export`] is this plus the writing, and the browser's Export is
/// this plus a zip (ADR-0017).
///
/// `dir` is where the files *would* go. Nothing is read from it or created
/// in it — it only prefixes the paths and settles what
/// [`MeshPathStyle::Absolute`](crate::MeshPathStyle) writes into the model
/// files, so a virtual root is a perfectly good answer.
pub fn export_files(
    robot: &ResolvedRobot,
    options: &ExportOptions,
    dir: &Path,
) -> Vec<(PathBuf, Vec<u8>)> {
    let mut files: Vec<(PathBuf, Vec<u8>)> = Vec::new();
    if options.format.writes_mjcf() {
        let path = dir.join(format!("{}.xml", robot.name));
        files.push((path, mjcf::write(robot, options).into_bytes()));
    }
    if options.format.writes_urdf() {
        let path = dir.join(format!("{}.urdf", robot.name));
        files.push((path, urdf::write(robot, options, dir).into_bytes()));
    }
    if options.format.writes_sdf() {
        let path = dir.join(format!("{}.sdf", robot.name));
        files.push((path, sdf::write(robot, options, dir).into_bytes()));
    }
    let meshes = dir.join("meshes");
    for (stem, mesh) in &robot.meshes {
        files.push((
            meshes.join(format!("{stem}.stl")),
            riggen_mesh::write_binary(mesh),
        ));
    }
    files
}

/// Writes `robot` into `dir` (created if missing) and returns every path
/// written, model files first, then meshes in stem order.
///
/// The list comes from [`export_files`]; what is added here is the part
/// that only means something on a real filesystem — creating `meshes/`,
/// and the `.tmp`-sibling-and-rename that keeps a crash mid-write from
/// leaving half a file behind.
pub fn export(
    robot: &ResolvedRobot,
    options: &ExportOptions,
    dir: &Path,
) -> Result<Vec<PathBuf>, ExportIoError> {
    let io = |path: &Path| {
        let path = path.to_owned();
        move |source| ExportIoError { path, source }
    };
    let meshes = dir.join("meshes");
    std::fs::create_dir_all(&meshes).map_err(io(&meshes))?;

    let files = export_files(robot, options, dir);
    let mut written = Vec::with_capacity(files.len());
    for (path, bytes) in files {
        write_atomically(&path, &bytes).map_err(io(&path))?;
        written.push(path);
    }
    Ok(written)
}

fn write_atomically(path: &Path, bytes: &[u8]) -> io::Result<()> {
    let tmp = path.with_extension(format!(
        "{}.tmp",
        path.extension().and_then(|e| e.to_str()).unwrap_or("")
    ));
    std::fs::write(&tmp, bytes)?;
    std::fs::rename(&tmp, path)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::resolve::Format;
    use crate::test_util::Builder;
    use riggen_core::glam::{DQuat, DVec3};
    use riggen_core::{JointKind, MeshAsset};
    use riggen_mesh::TriMesh;

    fn scratch(name: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!("riggen-export-{name}-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&dir);
        dir
    }

    #[test]
    fn writes_model_and_meshes_and_no_tmp_files_remain() {
        let mut b = Builder::new();
        let cube = b.mesh("cube", TriMesh::cube(0.05));
        let root = b.robot.root;
        b.link("arm", root, JointKind::Revolute, Some(cube));
        let resolved = b.resolve().unwrap();
        let dir = scratch("basic");
        let options = ExportOptions {
            format: Format::MJCF,
            ..Default::default()
        };
        let written = export(&resolved, &options, &dir).unwrap();
        assert_eq!(
            written,
            vec![dir.join("test.xml"), dir.join("meshes/cube.stl")]
        );
        for p in &written {
            assert!(p.is_file(), "{}", p.display());
        }
        let leftovers: Vec<_> = std::fs::read_dir(&dir)
            .unwrap()
            .chain(std::fs::read_dir(dir.join("meshes")).unwrap())
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .filter(|n| n.ends_with(".tmp"))
            .collect();
        assert!(leftovers.is_empty(), "{leftovers:?}");
        let xml = std::fs::read_to_string(dir.join("test.xml")).unwrap();
        assert!(xml.starts_with("<?xml"));
        assert!(!dir.join("test.urdf").exists(), "MJCF only was asked for");

        // The default is every writer (ADR-0016): three model files, one
        // `meshes/`, and each file is the format its extension claims.
        let all = export(&resolved, &ExportOptions::default(), &dir).unwrap();
        assert_eq!(
            all[..3],
            [
                dir.join("test.xml"),
                dir.join("test.urdf"),
                dir.join("test.sdf")
            ]
        );
        let urdf = std::fs::read_to_string(dir.join("test.urdf")).unwrap();
        assert!(urdf.contains("<robot name=\"test\">"), "{urdf}");
        let sdf = std::fs::read_to_string(dir.join("test.sdf")).unwrap();
        assert!(sdf.contains("<sdf version=\"1.11\">"), "{sdf}");
        assert!(sdf.contains("<model name=\"test\">"), "{sdf}");
        std::fs::remove_dir_all(&dir).unwrap();
    }

    #[test]
    fn written_stl_is_in_meters_with_fix_up_baked() {
        // A millimetre, Y-up cube: scale 0.001 and a fix-up that turns Y
        // into Z. The written file's AABB is the document-space one.
        let mut b = Builder::new();
        let raw = TriMesh {
            positions: TriMesh::cube(50.0)
                .positions
                .iter()
                .map(|p| *p + DVec3::new(0.0, 100.0, 0.0))
                .collect(),
            ..TriMesh::cube(50.0)
        };
        let asset = MeshAsset {
            path: PathBuf::from("/nowhere/mm.stl"),
            content_hash: 0,
            scale: 0.001,
            fix_up: Some(DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2)),
        };
        let id = b.robot.add_asset(asset.clone());
        b.store
            .insert(id, crate::mesh_store::to_document_units(raw, &asset));
        let root = b.robot.root;
        b.link("arm", root, JointKind::Fixed, Some(id));
        let resolved = b.resolve().unwrap();
        let dir = scratch("units");
        export(&resolved, &ExportOptions::default(), &dir).unwrap();
        let back = riggen_mesh::load_stl(&dir.join("meshes/mm.stl")).unwrap();
        let aabb = back.aabb().unwrap();
        // 100 mm along +Y becomes 0.1 m along +Z; half-extent 0.05 m.
        assert!(
            (aabb.min - DVec3::new(-0.05, -0.05, 0.05)).length() < 1e-6,
            "{aabb:?}"
        );
        assert!(
            (aabb.max - DVec3::new(0.05, 0.05, 0.15)).length() < 1e-6,
            "{aabb:?}"
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// The seam of step 3 (ADR-0017): whatever `export` puts on disk,
    /// `export_files` already had in hand — the same names in the same
    /// order and byte-for-byte the same contents. The sample arm, all
    /// three formats, and every mesh under `meshes/`.
    #[test]
    fn export_files_and_export_agree_on_the_arm() {
        let (robot, warnings) =
            riggen_core::load(&crate::test_util::fixtures().join("arm/arm.riggen")).unwrap();
        assert_eq!(warnings, Vec::new());
        let (store, errors) = crate::MeshStore::load(&robot, &riggen_core::Disk);
        assert_eq!(errors, Vec::new());
        let options = ExportOptions::default();
        let resolved = crate::resolve(&robot, &store, &crate::ComputeNow, &options).unwrap();

        let dir = scratch("agree");
        let written = export(&resolved, &options, &dir).unwrap();
        let files = export_files(&resolved, &options, &dir);

        assert_eq!(files.len(), 3 + resolved.meshes.len());
        assert_eq!(
            written,
            files.iter().map(|(p, _)| p.clone()).collect::<Vec<_>>()
        );
        for (path, bytes) in &files {
            assert_eq!(&std::fs::read(path).unwrap(), bytes, "{}", path.display());
        }
        // `.tmp`-and-rename is the only thing the writer adds, and it
        // leaves nothing of its own behind.
        assert!(
            !files
                .iter()
                .any(|(p, _)| p.to_string_lossy().contains(".tmp")),
            "{files:?}"
        );
        std::fs::remove_dir_all(&dir).unwrap();
    }

    /// A directory that does not exist is still a perfectly good prefix:
    /// nothing is read from `dir` and nothing is created in it, which is
    /// what lets the browser hand it a virtual root.
    #[test]
    fn export_files_touches_no_filesystem() {
        let mut b = Builder::new();
        let cube = b.mesh("cube", TriMesh::cube(0.05));
        let root = b.robot.root;
        b.link("arm", root, JointKind::Revolute, Some(cube));
        let resolved = b.resolve().unwrap();

        let dir = Path::new("/nowhere/export");
        assert!(!dir.exists());
        let files = export_files(&resolved, &ExportOptions::default(), dir);
        assert_eq!(
            files.iter().map(|(p, _)| p.clone()).collect::<Vec<_>>(),
            vec![
                dir.join("test.xml"),
                dir.join("test.urdf"),
                dir.join("test.sdf"),
                dir.join("meshes/cube.stl"),
            ]
        );
        assert!(!dir.exists(), "export_files created something");
    }
}
