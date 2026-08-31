//! Writing the export directory (ADR-0008): `<name>.xml` and/or
//! `<name>.urdf` beside a `meshes/` folder of binary STL in meters, every
//! file through a `.tmp` sibling and a rename so a crash mid-write never
//! leaves a half file behind — the same discipline as `file::save`.

use std::fmt;
use std::io;
use std::path::{Path, PathBuf};

use crate::resolve::{ExportOptions, ResolvedRobot};
use crate::{mjcf, urdf};

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

/// Writes `robot` into `dir` (created if missing) and returns every path
/// written, model files first, then meshes in stem order.
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

    let mut written = Vec::new();
    if options.format.writes_mjcf() {
        let path = dir.join(format!("{}.xml", robot.name));
        write_atomically(&path, mjcf::write(robot, options).as_bytes()).map_err(io(&path))?;
        written.push(path);
    }
    if options.format.writes_urdf() {
        let path = dir.join(format!("{}.urdf", robot.name));
        let text = urdf::write(robot, options, dir);
        write_atomically(&path, text.as_bytes()).map_err(io(&path))?;
        written.push(path);
    }
    for (stem, mesh) in &robot.meshes {
        let path = meshes.join(format!("{stem}.stl"));
        write_atomically(&path, &riggen_mesh::write_binary(mesh)).map_err(io(&path))?;
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

        let both = export(&resolved, &ExportOptions::default(), &dir).unwrap();
        assert_eq!(both[..2], [dir.join("test.xml"), dir.join("test.urdf")]);
        let urdf = std::fs::read_to_string(dir.join("test.urdf")).unwrap();
        assert!(urdf.contains("<robot name=\"test\">"), "{urdf}");
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
}
