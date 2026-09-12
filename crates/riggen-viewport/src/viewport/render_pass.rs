use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use egui_wgpu::wgpu;

use super::depth::row_stride;
use super::gpu_state::{CameraUniforms, InstanceBuffers};
use super::picking::{PICK_ROW_STRIDE, PickRegion};
use crate::RenderGroup;

/// A pick request being rendered and copied out this frame; `result` is the
/// same `Arc` the owning `PendingPick` polls.
pub struct PickPassData {
    pub region: PickRegion,
    pub readback_buffer: wgpu::Buffer,
    pub result: Arc<Mutex<Option<Vec<u32>>>>,
}

/// A depth readback being recorded this frame.
///
/// Unlike the pick, the copy goes on **egui's** encoder, right after the
/// scene pass — it has to see the depth this frame wrote, and our own
/// encoder is submitted before egui's. That means `map_async` cannot be
/// called here (a submit that touches a mapping buffer is invalid, see
/// [`ViewportCallback::pick_pass`]); `recorded` tells the next `ui()` that
/// the copy is on its way and it is safe to map.
pub struct DepthPassData {
    pub readback_buffer: wgpu::Buffer,
    pub size: (u32, u32),
    pub recorded: Arc<AtomicBool>,
}

/// One frame of the viewport, handed to egui as a paint callback.
/// `prepare` renders the scene into the offscreen target on egui's
/// encoder and, when asked, the ID buffer on its own; `paint` blits the
/// colour into egui's own pass.
pub struct ViewportCallback {
    pub camera_uniforms: CameraUniforms,
    pub axes_view_proj: [[f32; 4]; 4],
    /// Bottom-left gizmo rect in physical pixels: (x, y, width, height).
    pub axes_viewport: (f32, f32, f32, f32),
    pub uniform_buffer: wgpu::Buffer,
    pub uniform_bind_group: wgpu::BindGroup,
    pub axes_uniform_buffer: wgpu::Buffer,
    pub axes_uniform_bind_group: wgpu::BindGroup,
    pub scene_pipeline: wgpu::RenderPipeline,
    pub translucent_pipeline: wgpu::RenderPipeline,
    pub background_pipeline: wgpu::RenderPipeline,
    pub grid_pipeline: wgpu::RenderPipeline,
    /// Whether to draw the ground this frame (`Viewport::set_ground_visible`).
    pub draw_ground: bool,
    pub hover_pipeline: wgpu::RenderPipeline,
    pub select_pipeline: wgpu::RenderPipeline,
    pub axes_pipeline: wgpu::RenderPipeline,
    pub pick_pipeline: wgpu::RenderPipeline,
    pub blit_pipeline: wgpu::RenderPipeline,
    pub axes_vertex_buffer: wgpu::Buffer,
    pub axes_vertex_count: u32,
    /// Every visible instance, in scene order; every `model_offset`,
    /// `hover` and `select` is stated against this order.
    pub instances: Vec<InstanceBuffers>,
    pub model_bind_group: wgpu::BindGroup,
    /// This frame's model matrices, already packed at the uniform stride.
    pub model_data: Vec<u8>,
    pub model_buffer: wgpu::Buffer,
    /// Index into `instances` of the instance to tint as hovered.
    pub hover: Option<usize>,
    /// Index into `instances` of the instance to tint as selected.
    pub select: Option<usize>,
    pub color_view: wgpu::TextureView,
    /// The colour attachment's `resolve_target`, when the scene pass is
    /// multisampled (`GpuState::sample_count`).
    pub color_resolve_view: Option<wgpu::TextureView>,
    pub depth_view: wgpu::TextureView,
    /// The depth-resolve pipeline and this size's bind group + target, when
    /// the scene pass is multisampled. Run only on a frame that reads depth
    /// back — nothing else in the frame looks at the resolved texture.
    pub depth_resolve: Option<(wgpu::RenderPipeline, wgpu::BindGroup, wgpu::TextureView)>,
    pub blit_bind_group: wgpu::BindGroup,
    pub pick_color_view: wgpu::TextureView,
    pub pick_color_texture: wgpu::Texture,
    pub pick_depth_view: wgpu::TextureView,
    pub pick: Option<PickPassData>,
    pub depth: Option<DepthPassData>,
    /// The offscreen depth target, as a copy source.
    pub depth_texture: wgpu::Texture,
    /// Bumped once per rendered frame, so `Viewport::ui` can tell a logic
    /// pass from one that reached the GPU: a harness that never renders
    /// must not queue readbacks nobody will ever answer.
    pub rendered_frames: Arc<std::sync::atomic::AtomicU64>,
}

impl ViewportCallback {
    fn scene_pass(&self, encoder: &mut wgpu::CommandEncoder) {
        let is_dark = self.camera_uniforms.params[2] > 0.5;
        let clear_color = if is_dark {
            wgpu::Color {
                r: 0.09,
                g: 0.10,
                b: 0.12,
                a: 1.0,
            }
        } else {
            wgpu::Color {
                r: 0.88,
                g: 0.90,
                b: 0.92,
                a: 1.0,
            }
        };

        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("riggen-viewport scene pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &self.color_view,
                depth_slice: None,
                resolve_target: self.color_resolve_view.as_ref(),
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(clear_color),
                    // Nothing reads the multisampled texture once the
                    // resolve has run — only the resolved one is blitted —
                    // so its samples need not survive the pass.
                    store: if self.color_resolve_view.is_some() {
                        wgpu::StoreOp::Discard
                    } else {
                        wgpu::StoreOp::Store
                    },
                },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: &self.depth_view,
                depth_ops: Some(wgpu::Operations {
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });

        pass.set_pipeline(&self.background_pipeline);
        pass.set_bind_group(0, &self.uniform_bind_group, &[]);
        pass.draw(0..3, 0..1);

        // One draw per instance, each with its own model matrix at its own
        // dynamic offset — no CPU-side merge. Opaque first, then every
        // translucent instance over the finished depth buffer.
        for (pipeline, group) in [
            (&self.scene_pipeline, RenderGroup::Opaque),
            (&self.translucent_pipeline, RenderGroup::Translucent),
        ] {
            pass.set_pipeline(pipeline);
            for instance in self.instances.iter().filter(|i| i.group == group) {
                pass.set_bind_group(0, &self.uniform_bind_group, &[]);
                pass.set_bind_group(1, &self.model_bind_group, &[instance.model_offset]);
                pass.set_vertex_buffer(0, instance.vertex_buffer.slice(..));
                pass.set_index_buffer(instance.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..instance.index_count, 0, 0..1);
            }
        }

        // The ground, over the finished opaque depth buffer: depth-tested,
        // so a part in front of it hides it and it hides the background
        // behind it, but writing no depth of its own. Drawn unconditionally —
        // it is furniture, like the background and the axes triad, and zen
        // hides chrome, not the scene (ADR-0021, amended).
        if self.draw_ground {
            pass.set_pipeline(&self.grid_pipeline);
            pass.set_bind_group(0, &self.uniform_bind_group, &[]);
            pass.draw(0..3, 0..1);
        }

        // Whole-instance restyles for hover and selection, drawn over the
        // shaded geometry (selection last, so it reads on top of a hover of
        // the same instance).
        for (pipeline, index) in [
            (&self.hover_pipeline, self.hover),
            (&self.select_pipeline, self.select),
        ] {
            let Some(instance) = index.and_then(|i| self.instances.get(i)) else {
                continue;
            };
            pass.set_pipeline(pipeline);
            pass.set_bind_group(0, &self.uniform_bind_group, &[]);
            pass.set_bind_group(1, &self.model_bind_group, &[instance.model_offset]);
            pass.set_vertex_buffer(0, instance.vertex_buffer.slice(..));
            pass.set_index_buffer(instance.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..instance.index_count, 0, 0..1);
        }

        // Axes-triad gizmo: fixed screen-space corner, own rotation-only
        // camera, drawn last so it's never occluded by scene geometry.
        let (vx, vy, vw, vh) = self.axes_viewport;
        pass.set_viewport(vx, vy, vw, vh, 0.0, 1.0);
        pass.set_pipeline(&self.axes_pipeline);
        pass.set_bind_group(0, &self.axes_uniform_bind_group, &[]);
        pass.set_vertex_buffer(0, self.axes_vertex_buffer.slice(..));
        pass.draw(0..self.axes_vertex_count, 0..1);
    }

    /// Rasterizes every visible instance's pick ids, copies the region
    /// around the cursor into `pick.readback_buffer` and registers the
    /// `map_async` that fills `pick.result`.
    fn pick_pass(&self, device: &wgpu::Device, queue: &wgpu::Queue, pick: &PickPassData) {
        // Recorded and submitted through our own encoder, separate from
        // egui's: `map_async` below is only valid to call once the copy
        // that fills this buffer has actually been submitted — calling it
        // while the copy still sits unsubmitted in egui's encoder trips
        // wgpu's "buffer still mapped" validation on the *next*
        // `queue.submit` (egui's own, right after `prepare` returns).
        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("riggen-viewport pick encoder"),
        });
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("riggen-viewport pick pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &self.pick_color_view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        // 0 is the "nothing hit" sentinel `crate::pick_id`
                        // reserves.
                        load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.pick_depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&self.pick_pipeline);
            // Translucent instances are see-through for the cursor too,
            // and so is whatever a gizmo drag is carrying: it follows the
            // cursor, so leaving it in would mean the drag could only ever
            // snap to itself (ADR-0019 §5).
            for instance in self
                .instances
                .iter()
                .filter(|i| i.group == RenderGroup::Opaque && !i.pick_hidden)
            {
                pass.set_bind_group(0, &self.uniform_bind_group, &[]);
                pass.set_bind_group(1, &self.model_bind_group, &[instance.model_offset]);
                pass.set_vertex_buffer(0, instance.pick_vertex_buffer.slice(..));
                pass.draw(0..instance.triangle_count * 3, 0..1);
            }
        }

        let region = pick.region;
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &self.pick_color_texture,
                mip_level: 0,
                origin: wgpu::Origin3d {
                    x: region.origin.0,
                    y: region.origin.1,
                    z: 0,
                },
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &pick.readback_buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(PICK_ROW_STRIDE as u32),
                    rows_per_image: Some(region.height),
                },
            },
            wgpu::Extent3d {
                width: region.width,
                height: region.height,
                depth_or_array_layers: 1,
            },
        );
        queue.submit(std::iter::once(encoder.finish()));

        // Registered now, resolved whenever wgpu next processes it
        // (`Viewport::ui`'s `device.poll(PollType::Poll)`) — never awaited
        // here.
        let result = pick.result.clone();
        let buffer = pick.readback_buffer.clone();
        let row_stride = PICK_ROW_STRIDE as usize;
        buffer
            .clone()
            .slice(..)
            .map_async(wgpu::MapMode::Read, move |res| {
                if res.is_err() {
                    return;
                }
                if let Ok(data) = buffer.slice(..).get_mapped_range() {
                    let mut ids = Vec::with_capacity((region.width * region.height) as usize);
                    for row in 0..region.height as usize {
                        for col in 0..region.width as usize {
                            let offset = row * row_stride + col * 4;
                            ids.push(u32::from_le_bytes([
                                data[offset],
                                data[offset + 1],
                                data[offset + 2],
                                data[offset + 3],
                            ]));
                        }
                    }
                    drop(data);
                    buffer.unmap();
                    *result.lock().unwrap() = Some(ids);
                }
            });
    }
}

impl ViewportCallback {
    /// Writes sample 0 of the multisampled depth attachment into the
    /// single-sampled texture `depth_copy` reads, when the scene pass is
    /// multisampled. A no-op otherwise — at sample count 1 the attachment
    /// *is* that texture.
    ///
    /// Recorded on egui's encoder between the scene pass and the copy, for
    /// the same reason the copy is: it has to see the depth this frame wrote.
    fn depth_resolve_pass(&self, encoder: &mut wgpu::CommandEncoder) {
        let Some((pipeline, bind_group, target_view)) = self.depth_resolve.as_ref() else {
            return;
        };
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("riggen-viewport depth resolve pass"),
            color_attachments: &[],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: target_view,
                depth_ops: Some(wgpu::Operations {
                    // Every pixel is written, so there is nothing to load.
                    load: wgpu::LoadOp::Clear(1.0),
                    store: wgpu::StoreOp::Store,
                }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_pipeline(pipeline);
        pass.set_bind_group(0, bind_group, &[]);
        pass.draw(0..3, 0..1);
    }

    /// Copies the whole depth attachment into `depth.readback_buffer`.
    fn depth_copy(&self, encoder: &mut wgpu::CommandEncoder, depth: &DepthPassData) {
        let (width, height) = depth.size;
        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &self.depth_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::DepthOnly,
            },
            wgpu::TexelCopyBufferInfo {
                buffer: &depth.readback_buffer,
                layout: wgpu::TexelCopyBufferLayout {
                    offset: 0,
                    bytes_per_row: Some(row_stride(width)),
                    rows_per_image: Some(height),
                },
            },
            wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
        );
        depth.recorded.store(true, Ordering::Relaxed);
    }
}

impl egui_wgpu::CallbackTrait for ViewportCallback {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _screen_descriptor: &egui_wgpu::ScreenDescriptor,
        egui_encoder: &mut wgpu::CommandEncoder,
        _resources: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        queue.write_buffer(
            &self.uniform_buffer,
            0,
            bytemuck::cast_slice(&[self.camera_uniforms]),
        );
        queue.write_buffer(
            &self.axes_uniform_buffer,
            0,
            bytemuck::cast_slice(&[self.axes_view_proj]),
        );
        if !self.model_data.is_empty() {
            queue.write_buffer(&self.model_buffer, 0, &self.model_data);
        }

        self.scene_pass(egui_encoder);

        // After the scene pass, on the same encoder, so the copy sees the
        // depth this frame wrote (`viewport::depth`).
        if let Some(depth) = self.depth.as_ref() {
            self.depth_resolve_pass(egui_encoder);
            self.depth_copy(egui_encoder, depth);
        }

        self.rendered_frames.fetch_add(1, Ordering::Relaxed);

        // Nothing to rasterize ids from with an empty scene, and the axes
        // gizmo is not pickable.
        if let Some(pick) = self.pick.as_ref().filter(|_| !self.instances.is_empty()) {
            self.pick_pass(device, queue, pick);
        }

        Vec::new()
    }

    fn paint(
        &self,
        _info: egui::PaintCallbackInfo,
        render_pass: &mut wgpu::RenderPass<'static>,
        _resources: &egui_wgpu::CallbackResources,
    ) {
        render_pass.set_pipeline(&self.blit_pipeline);
        render_pass.set_bind_group(0, &self.blit_bind_group, &[]);
        render_pass.draw(0..3, 0..1);
    }
}
