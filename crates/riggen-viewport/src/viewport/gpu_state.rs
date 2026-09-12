use egui_wgpu::wgpu;

use crate::gpu_mesh::AxesTriadMesh;

pub const DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;

/// Sample count the offscreen scene pass runs at when the adapter can do it.
/// 4 is the only count WebGPU guarantees beyond 1, and the only one worth
/// having: the step from 1 to 4 is what turns a staircase edge into a line.
pub const PREFERRED_SAMPLES: u32 = 4;

/// How many samples the scene pass may use, given what the adapter says about
/// the two formats it writes.
///
/// Both have to agree — a pass cannot mix sample counts across its colour and
/// depth attachments — and the colour format additionally has to be
/// *resolvable*, since the blit reads a single-sampled resolve target rather
/// than the multisampled texture itself. Anything short of that is 1, which
/// is exactly the pipeline shape this viewport had before multisampling: no
/// resolve texture, no resolve pass, the blit sampling the colour attachment
/// directly.
pub fn choose_sample_count(
    color: wgpu::TextureFormatFeatureFlags,
    depth: wgpu::TextureFormatFeatureFlags,
) -> u32 {
    let supported = color.sample_count_supported(PREFERRED_SAMPLES)
        && color.contains(wgpu::TextureFormatFeatureFlags::MULTISAMPLE_RESOLVE)
        && depth.sample_count_supported(PREFERRED_SAMPLES);
    if supported { PREFERRED_SAMPLES } else { 1 }
}

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
    /// What every pipeline that draws into the scene pass was built at, and
    /// what [`OffscreenTarget`]'s colour and depth attachments are allocated
    /// at. Chosen once by [`choose_sample_count`]; `pick` and `blit` are
    /// always 1.
    pub sample_count: u32,
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
    /// Copies the multisampled depth attachment into a single-sampled one the
    /// overlay readback can be copied from. `None` at sample count 1, where
    /// the attachment already is that texture.
    pub depth_resolve: Option<DepthResolvePipeline>,
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

/// The depth-resolve pipeline and the layout its per-size bind group is
/// built against: a fullscreen triangle that reads the multisampled depth
/// attachment and writes `@builtin(frag_depth)` into a single-sampled one.
///
/// WebGPU has no depth equivalent of a colour `resolve_target`, and
/// `copy_texture_to_buffer` refuses a multisampled source outright — so the
/// overlay's depth readback (ADR-0020) needs this pass to have anything to
/// copy from at all.
pub struct DepthResolvePipeline {
    pub layout: wgpu::BindGroupLayout,
    pub pipeline: wgpu::RenderPipeline,
}

/// The offscreen color+depth pair the 3D scene renders into before being
/// blitted into egui's own render pass (depth testing needs a real depth
/// attachment, which egui's pass does not provide), plus the ID-buffer
/// pick target — resized together to match the allocated viewport rect.
///
/// Both scene attachments carry [`GpuState::sample_count`] samples, so each
/// has a single-sampled counterpart downstream of it: the colour one is the
/// pass's own `resolve_target` and is what the blit samples, the depth one is
/// written by [`DepthResolvePipeline`] and is what the readback copies.
pub struct OffscreenTarget {
    pub size: (u32, u32),
    /// The scene pass's colour attachment.
    pub color_view: wgpu::TextureView,
    /// The colour attachment's `resolve_target`, and the texture the blit
    /// samples. `None` at sample count 1, where the blit samples
    /// `color_view`'s own texture — a multisampled texture cannot be
    /// `textureSample`d the way `blit.wgsl` does.
    pub color_resolve_view: Option<wgpu::TextureView>,
    /// Kept as well as its view: the overlay copies it back to classify
    /// glyphs against the depth the scene pass wrote (`viewport::depth`).
    /// Single-sampled always — the resolve target when multisampling, the
    /// attachment itself otherwise.
    pub depth_texture: wgpu::Texture,
    /// The scene pass's depth attachment.
    pub depth_view: wgpu::TextureView,
    /// `depth_texture` as a render target, plus the bind group holding the
    /// multisampled attachment. `None` at sample count 1.
    pub depth_resolve: Option<DepthResolveTarget>,
    pub blit_bind_group: wgpu::BindGroup,
    pub pick_color_texture: wgpu::Texture,
    pub pick_color_view: wgpu::TextureView,
    pub pick_depth_view: wgpu::TextureView,
}

/// This size's half of the depth resolve: where it reads and where it writes.
pub struct DepthResolveTarget {
    /// The multisampled depth attachment, as `texture_depth_multisampled_2d`.
    pub bind_group: wgpu::BindGroup,
    /// [`OffscreenTarget::depth_texture`] as a depth-stencil attachment.
    pub target_view: wgpu::TextureView,
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
    /// Drawn as usual, left out of the **pick** pass: the cursor looks
    /// through it (`Viewport::set_pick_excluded`, ADR-0019 §5).
    pub pick_hidden: bool,
}

#[cfg(test)]
mod tests {
    use super::*;
    use wgpu::TextureFormatFeatureFlags as Flags;

    /// What the WebGPU spec guarantees for a renderable colour format and for
    /// `Depth32Float` — the case every adapter riggen runs on should hit.
    #[test]
    fn an_adapter_that_can_multisample_both_formats_gets_four_samples() {
        let color = Flags::MULTISAMPLE_X4 | Flags::MULTISAMPLE_RESOLVE | Flags::FILTERABLE;
        let depth = Flags::MULTISAMPLE_X4;
        assert_eq!(choose_sample_count(color, depth), 4);
    }

    #[test]
    fn anything_either_format_cannot_do_falls_back_to_one() {
        let color = Flags::MULTISAMPLE_X4 | Flags::MULTISAMPLE_RESOLVE;
        let depth = Flags::MULTISAMPLE_X4;
        // Colour that multisamples but cannot be resolved is no use: the blit
        // samples the resolve target.
        assert_eq!(choose_sample_count(Flags::MULTISAMPLE_X4, depth), 1);
        // Depth that cannot follow the colour count is no use either: one
        // pass, one sample count.
        assert_eq!(choose_sample_count(color, Flags::empty()), 1);
        assert_eq!(choose_sample_count(Flags::empty(), Flags::empty()), 1);
        // 2x and 8x are not a fallback ladder — 4 or nothing.
        assert_eq!(
            choose_sample_count(Flags::MULTISAMPLE_X2 | Flags::MULTISAMPLE_RESOLVE, depth),
            1
        );
    }
}
