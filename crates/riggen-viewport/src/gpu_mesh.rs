//! A [`TriMesh`] on the GPU: the shaded vertex/index buffers, the pick
//! vertices tagged with [`crate::pick_id`]s, and the fixed axes triad.
//! `f64` → `f32` happens here and nowhere else (docs/02-data-model.md).

use egui_wgpu::wgpu;
use egui_wgpu::wgpu::util::DeviceExt;
use riggen_mesh::TriMesh;

use crate::scene::InstancePayload;

#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct Vertex {
    pub position: [f32; 3],
    pub normal: [f32; 3],
}

impl Vertex {
    const ATTRS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];

    pub fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRS,
        }
    }
}

/// Vertex layout for the ID-buffer pick pass: position plus the packed
/// [`crate::pick_id`] of the triangle it belongs to.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct PickVertex {
    pub position: [f32; 3],
    pub pick_id: u32,
}

impl PickVertex {
    const ATTRS: [wgpu::VertexAttribute; 2] = wgpu::vertex_attr_array![0 => Float32x3, 1 => Uint32];

    pub fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<PickVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRS,
        }
    }
}

/// A [`TriMesh`] uploaded to GPU buffers for one instance.
///
/// The shaded pass draws `index_buffer` over `vertex_buffer`. The pick pass
/// draws `pick_vertex_buffer` *non-indexed*, three vertices per triangle:
/// a pick id is per triangle, and a vertex shared by two triangles (a
/// welded OBJ) cannot carry both, so pick vertices are unwelded at upload
/// regardless of how the mesh came in.
pub struct GpuMesh {
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub index_count: u32,
    pub pick_vertex_buffer: wgpu::Buffer,
    pub triangle_count: u32,
}

impl InstancePayload for GpuMesh {
    type Context = wgpu::Device;

    fn upload(device: &wgpu::Device, slot: u32, mesh: &TriMesh) -> Self {
        GpuMesh::upload(device, slot, mesh)
    }
}

impl GpuMesh {
    /// Uploads `mesh` with its pick vertices stamped for `slot`.
    ///
    /// The loaders already refuse meshes over [`riggen_mesh::MAX_TRIANGLES`]
    /// with an error that names the file; this is the last line, for a mesh
    /// built in memory.
    pub fn upload(device: &wgpu::Device, slot: u32, mesh: &TriMesh) -> Self {
        assert!(
            mesh.triangle_count() <= riggen_mesh::MAX_TRIANGLES,
            "{} triangles exceed the pick-id cap of {}",
            mesh.triangle_count(),
            riggen_mesh::MAX_TRIANGLES
        );
        debug_assert!(mesh.validate().is_ok(), "uploading an invalid mesh");

        let vertices = shaded_vertices(mesh);
        let pick_vertices = pick_vertices(slot, mesh);

        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("riggen-viewport mesh vertices"),
            contents: non_empty(&vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("riggen-viewport mesh indices"),
            contents: non_empty(&mesh.indices),
            usage: wgpu::BufferUsages::INDEX,
        });
        let pick_vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("riggen-viewport mesh pick vertices"),
            contents: non_empty(&pick_vertices),
            usage: wgpu::BufferUsages::VERTEX,
        });

        Self {
            vertex_buffer,
            index_buffer,
            index_count: mesh.indices.len() as u32,
            pick_vertex_buffer,
            triangle_count: mesh.triangle_count() as u32,
        }
    }
}

/// The shaded vertices, narrowed to `f32`. A mesh without normals gets flat
/// ones so it still lights; loaders always supply them, so this is for
/// meshes built in memory.
fn shaded_vertices(mesh: &TriMesh) -> Vec<Vertex> {
    let flat;
    let mesh = if mesh.normals.is_empty() && !mesh.positions.is_empty() {
        let mut m = mesh.clone();
        m.flat_normals();
        flat = m;
        &flat
    } else {
        mesh
    };
    mesh.positions
        .iter()
        .zip(&mesh.normals)
        .map(|(p, n)| Vertex {
            position: p.as_vec3().to_array(),
            normal: n.as_vec3().to_array(),
        })
        .collect()
}

/// Three [`PickVertex`]es per triangle, in index order, each carrying the
/// id of the triangle it is a corner of.
fn pick_vertices(slot: u32, mesh: &TriMesh) -> Vec<PickVertex> {
    (0..mesh.triangle_count())
        .flat_map(|i| {
            let pick_id = crate::pick_id::encode(slot, i as u32);
            mesh.triangle(i).map(|p| PickVertex {
                position: p.as_vec3().to_array(),
                pick_id,
            })
        })
        .collect()
}

/// wgpu rejects zero-size buffers; an empty mesh never has a nonempty range
/// to draw from it anyway, but the buffer still has to exist. The filler is
/// one zeroed element, which for a [`PickVertex`] is exactly the `0`
/// "nothing hit" sentinel the pick target clears to.
fn non_empty<T: bytemuck::Pod>(items: &[T]) -> &[u8] {
    /// Enough zero bytes for any vertex or index type in this module.
    const ZEROS: [u8; 32] = [0; 32];
    if items.is_empty() {
        let size = std::mem::size_of::<T>();
        debug_assert!(size <= ZEROS.len(), "widen ZEROS for this vertex type");
        &ZEROS[..size.min(ZEROS.len())]
    } else {
        bytemuck::cast_slice(items)
    }
}

/// Vertex layout for the corner axes-triad gizmo: position plus a flat
/// per-axis color, no lighting.
#[repr(C)]
#[derive(Clone, Copy, Debug, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct ColorVertex {
    pub position: [f32; 3],
    pub color: [f32; 3],
}

impl ColorVertex {
    const ATTRS: [wgpu::VertexAttribute; 2] =
        wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];

    pub fn layout() -> wgpu::VertexBufferLayout<'static> {
        wgpu::VertexBufferLayout {
            array_stride: std::mem::size_of::<ColorVertex>() as wgpu::BufferAddress,
            step_mode: wgpu::VertexStepMode::Vertex,
            attributes: &Self::ATTRS,
        }
    }
}

/// Fixed unit-length X/Y/Z line-list, colored red/green/blue, drawn in a
/// small screen-space corner with a rotation-only view — the axes triad
/// orients with the camera but never pans or scales with the model.
pub struct AxesTriadMesh {
    pub vertex_buffer: wgpu::Buffer,
    pub vertex_count: u32,
}

impl AxesTriadMesh {
    pub fn new(device: &wgpu::Device) -> Self {
        let origin = [0.0, 0.0, 0.0];
        let verts = [
            ColorVertex {
                position: origin,
                color: [0.9, 0.25, 0.25],
            },
            ColorVertex {
                position: [1.0, 0.0, 0.0],
                color: [0.9, 0.25, 0.25],
            },
            ColorVertex {
                position: origin,
                color: [0.35, 0.85, 0.35],
            },
            ColorVertex {
                position: [0.0, 1.0, 0.0],
                color: [0.35, 0.85, 0.35],
            },
            ColorVertex {
                position: origin,
                color: [0.3, 0.55, 0.95],
            },
            ColorVertex {
                position: [0.0, 0.0, 1.0],
                color: [0.3, 0.55, 0.95],
            },
        ];
        let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("riggen-viewport axes triad"),
            contents: bytemuck::cast_slice(&verts),
            usage: wgpu::BufferUsages::VERTEX,
        });
        Self {
            vertex_buffer,
            vertex_count: verts.len() as u32,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use riggen_mesh::glam::DVec3;

    #[test]
    fn pick_vertices_are_one_per_corner_and_tagged_per_triangle() {
        // A welded square: 4 positions, 2 triangles sharing an edge.
        let mesh = TriMesh {
            positions: vec![DVec3::ZERO, DVec3::X, DVec3::new(1.0, 1.0, 0.0), DVec3::Y],
            normals: vec![],
            indices: vec![0, 1, 2, 0, 2, 3],
        };
        let picks = pick_vertices(5, &mesh);
        assert_eq!(picks.len(), 6, "unwelded: three per triangle");
        for (i, v) in picks.iter().enumerate() {
            let tri = (i / 3) as u32;
            assert_eq!(crate::pick_id::decode(v.pick_id), Some((5, tri)));
            let expected = mesh.positions[mesh.indices[i] as usize].as_vec3();
            assert_eq!(v.position, expected.to_array());
        }
    }

    #[test]
    fn shaded_vertices_narrow_and_fill_in_normals() {
        let cube = TriMesh::cube(0.5);
        let shaded = shaded_vertices(&cube);
        assert_eq!(shaded.len(), 36);
        assert_eq!(shaded[0].normal, [1.0, 0.0, 0.0]);

        let mut without = cube.clone();
        without.normals.clear();
        let filled = shaded_vertices(&without);
        assert_eq!(filled, shaded, "flat normals match the cube's own");
    }

    #[test]
    fn empty_buffers_get_one_zeroed_element() {
        let none: [PickVertex; 0] = [];
        let bytes = non_empty(&none);
        assert_eq!(bytes.len(), std::mem::size_of::<PickVertex>());
        assert!(bytes.iter().all(|&b| b == 0), "the zero pick id is a miss");
        let some = [7u32, 8];
        assert_eq!(non_empty(&some), bytemuck::cast_slice::<u32, u8>(&some));
    }
}
