use egui_wgpu::wgpu;

use crate::gpu_mesh::AxesTriadMesh;

pub const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// Size, in physical pixels, of the square the axes-triad gizmo is drawn
/// into, clamped to a viewport-relative cap so it never dwarfs a tiny
/// viewport panel.
pub const AXES_GIZMO_SIZE: f32 = 90.0;
pub const AXES_GIZMO_MARGIN: f32 = 10.0;

/// Bytes of one instance's model matrix (`mat4x4<f32>`), before alignment.
/// Per-instance uniform: the model matrix (64 bytes) followed by the
/// instance colour (`vec4<f32>`, 16 bytes). The pick / highlight shaders
/// declare only the matrix, which is valid against the larger binding.
pub const MODEL_UNIFORM_SIZE: u64 =
    (std::mem::size_of::<[[f32; 4]; 4]>() + std::mem::size_of::<[f32; 4]>()) as u64;

/// Packed camera uniforms passed to vertex and fragment shaders (the
/// background gradient reads the basis vectors).
#[repr(C)]
#[derive(Debug, Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct CameraUniforms {
    pub view_proj: [[f32; 4]; 4],
    pub inv_view_proj: [[f32; 4]; 4],
    pub eye: [f32; 4],
    pub up: [f32; 4],
    pub right: [f32; 4],
    pub forward: [f32; 4],
    /// `(is_ortho, aspect, is_dark_mode, 0.0)`.
    pub params: [f32; 4],
}

/// Persistent GPU resources: pipelines, the shared camera uniform, and the
/// orientation gradient background. Recreated only in `Viewport::new`.
pub struct GpuState {
    pub device: wgpu::Device,
    pub format: wgpu::TextureFormat,
    pub scene_pipeline: wgpu::RenderPipeline,
    /// The scene shader again, alpha-blended and depth-tested without a
    /// depth write: the pass every [`crate::RenderGroup::Translucent`]
    /// instance draws in, after the opaque ones.
    pub translucent_pipeline: wgpu::RenderPipeline,
    pub background_pipeline: wgpu::RenderPipeline,
    pub pick_pipeline: wgpu::RenderPipeline,
    pub hover_pipeline: wgpu::RenderPipeline,
    pub select_pipeline: wgpu::RenderPipeline,
    pub axes_pipeline: wgpu::RenderPipeline,
    pub blit_pipeline: wgpu::RenderPipeline,
    pub uniform_buffer: wgpu::Buffer,
    pub uniform_bind_group: wgpu::BindGroup,
    pub axes_uniform_buffer: wgpu::Buffer,
    pub axes_uniform_bind_group: wgpu::BindGroup,
    pub blit_bind_group_layout: wgpu::BindGroupLayout,
    pub sampler: wgpu::Sampler,
    pub axes_mesh: AxesTriadMesh,
    /// Per-instance model matrices, one per *visible* instance at
    /// [`ModelUniforms::stride`] apart, bound through a single
    /// dynamic-offset bind group. Grown, never re-created per frame.
    pub models: ModelUniforms,
}

/// The dynamic-offset uniform every per-instance draw indexes into. One
/// buffer, one bind group, one `set_bind_group(1, .., &[offset])` per
/// instance — which is what makes "N instances on screen" cost N draw calls
/// instead of N mesh merges.
pub struct ModelUniforms {
    pub layout: wgpu::BindGroupLayout,
    pub buffer: wgpu::Buffer,
    pub bind_group: wgpu::BindGroup,
    /// `MODEL_UNIFORM_SIZE` rounded up to the device's
    /// `min_uniform_buffer_offset_alignment`.
    pub stride: u64,
    /// How many instances the current buffer has room for.
    pub capacity: usize,
}

impl ModelUniforms {
    pub fn new(device: &wgpu::Device, capacity: usize) -> Self {
        let alignment = device.limits().min_uniform_buffer_offset_alignment as u64;
        let stride = MODEL_UNIFORM_SIZE.div_ceil(alignment.max(1)) * alignment.max(1);
        let layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("riggen-viewport model layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                // The vertex stage reads the matrix, the fragment stage the colour.
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: true,
                    min_binding_size: wgpu::BufferSize::new(MODEL_UNIFORM_SIZE),
                },
                count: None,
            }],
        });
        let (buffer, bind_group) = Self::allocate(device, &layout, stride, capacity);
        Self {
            layout,
            buffer,
            bind_group,
            stride,
            capacity,
        }
    }

    /// Makes room for `count` instances, re-creating the buffer only when
    /// the scene has actually outgrown it.
    pub fn reserve(&mut self, device: &wgpu::Device, count: usize) {
        if count <= self.capacity {
            return;
        }
        let capacity = count.next_power_of_two();
        let (buffer, bind_group) = Self::allocate(device, &self.layout, self.stride, capacity);
        self.buffer = buffer;
        self.bind_group = bind_group;
        self.capacity = capacity;
    }

    pub fn offset(&self, index: usize) -> u32 {
        (index as u64 * self.stride) as u32
    }

    fn allocate(
        device: &wgpu::Device,
        layout: &wgpu::BindGroupLayout,
        stride: u64,
        capacity: usize,
    ) -> (wgpu::Buffer, wgpu::BindGroup) {
        let buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("riggen-viewport model uniforms"),
            size: stride * capacity.max(1) as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("riggen-viewport model bind group"),
            layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: &buffer,
                    offset: 0,
                    size: wgpu::BufferSize::new(MODEL_UNIFORM_SIZE),
                }),
            }],
        });
        (buffer, bind_group)
    }
}

/// The offscreen color+depth pair the 3D scene renders into before being
/// blitted into egui's own render pass (depth testing needs a real depth
/// attachment, which egui's pass does not provide), plus the ID-buffer
/// pick target — resized together to match the allocated viewport rect.
pub struct OffscreenTarget {
    pub size: (u32, u32),
    pub color_view: wgpu::TextureView,
    pub depth_view: wgpu::TextureView,
    pub blit_bind_group: wgpu::BindGroup,
    pub pick_color_texture: wgpu::Texture,
    pub pick_color_view: wgpu::TextureView,
    pub pick_depth_view: wgpu::TextureView,
}

/// GPU buffer handles for one instance's [`crate::GpuMesh`], grouped so the
/// paint callback doesn't thread a positional tuple through
/// `prepare()`/`paint()`, plus the dynamic offset of that instance's model
/// matrix in [`ModelUniforms`].
pub struct InstanceBuffers {
    pub model_offset: u32,
    pub group: crate::RenderGroup,
    pub vertex_buffer: wgpu::Buffer,
    pub index_buffer: wgpu::Buffer,
    pub index_count: u32,
    /// Non-indexed, three vertices per triangle (`crate::GpuMesh`).
    pub pick_vertex_buffer: wgpu::Buffer,
    pub triangle_count: u32,
}
