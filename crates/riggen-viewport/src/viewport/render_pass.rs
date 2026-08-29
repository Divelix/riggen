use egui_wgpu::wgpu;

use super::gpu_state::{CameraUniforms, InstanceBuffers};

/// One frame of the viewport, handed to egui as a paint callback.
/// `prepare` renders the scene into the offscreen target on egui's
/// encoder; `paint` blits it into egui's own pass.
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
    pub axes_pipeline: wgpu::RenderPipeline,
    pub blit_pipeline: wgpu::RenderPipeline,
    pub axes_vertex_buffer: wgpu::Buffer,
    pub axes_vertex_count: u32,
    /// Every visible instance, in scene order; every `model_offset` is
    /// stated against this order.
    pub instances: Vec<InstanceBuffers>,
    pub model_bind_group: wgpu::BindGroup,
    /// This frame's model matrices, already packed at the uniform stride.
    pub model_data: Vec<u8>,
    pub model_buffer: wgpu::Buffer,
    pub color_view: wgpu::TextureView,
    pub depth_view: wgpu::TextureView,
    pub blit_bind_group: wgpu::BindGroup,
}

impl egui_wgpu::CallbackTrait for ViewportCallback {
    fn prepare(
        &self,
        _device: &wgpu::Device,
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

        let mut pass = egui_encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
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

        // Axes-triad gizmo: fixed screen-space corner, own rotation-only
        // camera, drawn last so it's never occluded by scene geometry.
        let (vx, vy, vw, vh) = self.axes_viewport;
        pass.set_viewport(vx, vy, vw, vh, 0.0, 1.0);
        pass.set_pipeline(&self.axes_pipeline);
        pass.set_bind_group(0, &self.axes_uniform_bind_group, &[]);
        pass.set_vertex_buffer(0, self.axes_vertex_buffer.slice(..));
        pass.draw(0..self.axes_vertex_count, 0..1);

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
