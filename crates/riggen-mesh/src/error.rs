use std::fmt;

/// Everything that can go wrong between a path and a valid [`crate::TriMesh`].
/// Plain enum with a `Display`, no `thiserror`: the app shows these in the
/// status bar and nothing matches on them yet.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MeshError {
    /// `indices.len()` is not a multiple of three.
    IndexCount { len: usize },
    /// An index names a vertex the mesh does not have.
    IndexOutOfRange { index: u32, vertex_count: usize },
    /// `normals` is neither empty nor one per vertex.
    NormalCount {
        normal_count: usize,
        vertex_count: usize,
    },
}

impl fmt::Display for MeshError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::IndexCount { len } => {
                write!(f, "index count {len} is not a multiple of three")
            }
            Self::IndexOutOfRange {
                index,
                vertex_count,
            } => write!(f, "index {index} out of range for {vertex_count} vertices"),
            Self::NormalCount {
                normal_count,
                vertex_count,
            } => write!(
                f,
                "{normal_count} normals for {vertex_count} vertices (expected none or one per vertex)"
            ),
        }
    }
}

impl std::error::Error for MeshError {}
