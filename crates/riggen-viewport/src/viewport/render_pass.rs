use std::sync::{Arc, Mutex};

use egui_wgpu::wgpu;

use super::gpu_state::{CameraUniforms, InstanceBuffers};
use super::picking::{PICK_ROW_STRIDE, PickRegion};

/// A pick request being rendered and copied out this frame; `result` is the
/// same `Arc` the owning `PendingPick` polls.
pub struct PickPassData {
    pub region: PickRegion,
    pub readback_buffer: wgpu::Buffer,
    pub result: Arc<Mutex<Option<Vec<u32>>>>,
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
    pub background_pipeline: wgpu::RenderPipeline,
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
    pub depth_view: wgpu::TextureView,
    pub blit_bind_group: wgpu::BindGroup,
    pub pick_color_view: wgpu::TextureView,
    pub pick_color_texture: wgpu::Texture,
    pub pick_depth_view: wgpu::TextureView,
    pub pick: Option<PickPassData>,
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
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(clear_color),
                    store: wgpu::StoreOp::Store,
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
        // dynamic offset — no CPU-side merge.
        pass.set_pipeline(&self.scene_pipeline);
        for instance in &self.instances {
            pass.set_bind_group(0, &self.uniform_bind_group, &[]);
            pass.set_bind_group(1, &self.model_bind_group, &[instance.model_offset]);
            pass.set_vertex_buffer(0, instance.vertex_buffer.slice(..));
            pass.set_index_buffer(instance.index_buffer.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..instance.index_count, 0, 0..1);
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
            for instance in &self.instances {
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
