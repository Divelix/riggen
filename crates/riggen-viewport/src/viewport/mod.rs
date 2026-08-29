//! The embeddable viewport: allocates the egui rect, handles orbit/pan/zoom
//! input, renders the scene through an `egui_wgpu` paint callback.

pub mod gpu_state;
pub mod picking;
pub mod pipelines;
pub mod render_pass;

use std::sync::{Arc, Mutex};
use web_time::Instant;

use egui_wgpu::wgpu;
use riggen_mesh::glam::{DMat4, DVec3, Mat4};
use riggen_mesh::{Aabb, Ray, TriMesh};

use crate::PickHit;
use crate::camera::{OrbitCamera, Projection, StandardView};
use crate::gpu_mesh::{AxesTriadMesh, GpuMesh, PickVertex, Vertex};
use crate::overlay::{Overlay, OverlayItem};
use crate::scene::{InstanceId, Scene, SceneFull};

use gpu_state::{
    AXES_GIZMO_MARGIN, AXES_GIZMO_SIZE, CameraUniforms, DEPTH_FORMAT, GpuState, InstanceBuffers,
    ModelUniforms, OffscreenTarget,
};
use picking::{
    PICK_FORMAT, PICK_READBACK_SIZE, PendingPick, PickDecision, PickInputs, PickKind, PickRegion,
    decide_pick, resolve_pick_region,
};
use pipelines::{
    build_axes_pipeline, build_background_pipeline, build_blit_pipeline, build_highlight_pipeline,
    build_render_pipeline,
};
use render_pass::{PickPassData, ViewportCallback};

/// Instances the per-instance model uniform has room for before it has to
/// grow. A robot with more links than this is normal; re-allocating once at
/// each power of two is not a cost worth tuning.
const INITIAL_INSTANCE_CAPACITY: usize = 16;

/// This frame's vertical wheel input in points, taken straight from the raw
/// events instead of `egui::InputState::smooth_scroll_delta`.
///
/// egui low-pass-filters a discrete wheel notch across ~0.1 s (see
/// `WheelState::after_events`) — right for scrolling a document, wrong for a
/// viewport, where the wheel is direct manipulation and the filter reads as
/// the camera coasting to a stop after the wheel already did. Summing the
/// events applies the whole notch on the frame it lands, and reusing egui's
/// own unit conversion keeps a notch worth the same amount of zoom.
///
/// Events carrying the zoom or horizontal-scroll modifier are skipped:
/// ctrl+wheel is egui's UI-scale gesture and shift+wheel its horizontal one,
/// and `smooth_scroll_delta.y` was empty for both, so neither ever reached
/// the camera.
fn raw_wheel_delta_y(input: &egui::InputState, options: &egui::InputOptions) -> f32 {
    let ignored = options.zoom_modifier | options.horizontal_scroll_modifier;
    input
        .raw
        .events
        .iter()
        .filter_map(|event| match event {
            egui::Event::MouseWheel {
                unit,
                delta,
                modifiers,
                ..
            } if !modifiers.matches_any(ignored) => Some(match unit {
                egui::MouseWheelUnit::Point => delta.y,
                egui::MouseWheelUnit::Line => options.line_scroll_speed * delta.y,
                egui::MouseWheelUnit::Page => input.viewport_rect().height() * delta.y,
            }),
            _ => None,
        })
        .sum()
}

/// One instance as `debug_state()` reports it. An accessor type rather than
/// a serialised one, so `serde` stays out of this crate
/// (docs/01-architecture.md §Crates).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InstanceState {
    pub id: InstanceId,
    pub visible: bool,
    pub triangle_count: u32,
    /// Model-space bounds, before `model`.
    pub bounds: Option<Aabb>,
    pub model: DMat4,
    /// Linear RGBA tint.
    pub color: [f32; 4],
}

/// Embeddable 3D viewport: owns the renderer, camera, scene and
/// hover/selection state. `ui()` allocates the egui rect, handles
/// orbit/pan/zoom input, drives ID-buffer picking and enqueues a paint
/// callback.
pub struct Viewport {
    gpu: GpuState,
    offscreen: Option<OffscreenTarget>,
    /// One entry per instance, each with its own buffers and model
    /// transform — the viewport draws them, it never merges them.
    scene: Scene<GpuMesh>,
    pub camera: OrbitCamera,
    hovered: Option<PickHit>,
    selected: Option<PickHit>,
    /// World-space primitives drawn over the scene after the paint
    /// callback (`overlay.rs`). Rebuilt by the app every frame.
    overlay: Overlay,
    /// While `true` a click does not start a *select* pick, but hovering
    /// still resolves. The placement tools set it: they need the hovered
    /// triangle to snap against, and their click means "place here", not
    /// "select the part under the cursor".
    select_suppressed: bool,
    /// While `true` the viewport ignores the pointer entirely: no camera
    /// input, no picking. The app sets it while the gizmo owns the cursor
    /// (ADR-0007) — the gizmo's own widget is registered after the viewport
    /// and so wins the *click*, but the viewport would otherwise still see
    /// a hover and keep re-picking under it.
    input_suppressed: bool,
    pending_pick: Option<PendingPick>,
    last_pick: Option<PickInputs>,
    /// The rect allocated by the most recent [`Viewport::ui`] call, in egui
    /// logical points.
    last_rect: Option<egui::Rect>,
}

impl Viewport {
    pub fn new(device: &wgpu::Device, target_format: wgpu::TextureFormat) -> Self {
        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("riggen-viewport uniforms"),
            size: std::mem::size_of::<CameraUniforms>() as wgpu::BufferAddress,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        let uniform_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("riggen-viewport uniform layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });

        let uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("riggen-viewport uniform bind group"),
            layout: &uniform_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: uniform_buffer.as_entire_binding(),
            }],
        });

        // Group 1 of every per-instance pipeline, so its layout has to
        // exist before they are built.
        let models = ModelUniforms::new(device, INITIAL_INSTANCE_CAPACITY);

        let background_pipeline = build_background_pipeline(
            device,
            "riggen-viewport background pipeline",
            &[&uniform_bind_group_layout],
            include_str!("../shaders/background.wgsl"),
            target_format,
        );
        let scene_pipeline = build_render_pipeline(
            device,
            "riggen-viewport scene pipeline",
            &[&uniform_bind_group_layout, &models.layout],
            include_str!("../shaders/scene.wgsl"),
            &[Some(Vertex::layout())],
            wgpu::PrimitiveTopology::TriangleList,
            target_format,
            wgpu::CompareFunction::Less,
            true,
        );
        let pick_pipeline = build_render_pipeline(
            device,
            "riggen-viewport pick pipeline",
            &[&uniform_bind_group_layout, &models.layout],
            include_str!("../shaders/pick.wgsl"),
            &[Some(PickVertex::layout())],
            wgpu::PrimitiveTopology::TriangleList,
            PICK_FORMAT,
            wgpu::CompareFunction::Less,
            true,
        );
        let hover_pipeline = build_highlight_pipeline(
            device,
            "riggen-viewport hover pipeline",
            &[&uniform_bind_group_layout, &models.layout],
            include_str!("../shaders/hover.wgsl"),
            target_format,
        );
        let select_pipeline = build_highlight_pipeline(
            device,
            "riggen-viewport select pipeline",
            &[&uniform_bind_group_layout, &models.layout],
            include_str!("../shaders/select.wgsl"),
            target_format,
        );

        let axes_uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("riggen-viewport axes uniforms"),
            size: std::mem::size_of::<[[f32; 4]; 4]>() as wgpu::BufferAddress,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let axes_uniform_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("riggen-viewport axes uniform bind group"),
            layout: &uniform_bind_group_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: axes_uniform_buffer.as_entire_binding(),
            }],
        });
        let axes_pipeline = build_axes_pipeline(
            device,
            "riggen-viewport axes pipeline",
            &[&uniform_bind_group_layout],
            target_format,
        );
        let axes_mesh = AxesTriadMesh::new(device);

        let (blit_bind_group_layout, blit_pipeline) = build_blit_pipeline(device, target_format);

        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("riggen-viewport blit sampler"),
            ..Default::default()
        });

        Self {
            gpu: GpuState {
                device: device.clone(),
                format: target_format,
                scene_pipeline,
                background_pipeline,
                pick_pipeline,
                hover_pipeline,
                select_pipeline,
                axes_pipeline,
                blit_pipeline,
                uniform_buffer,
                uniform_bind_group,
                axes_uniform_buffer,
                axes_uniform_bind_group,
                blit_bind_group_layout,
                sampler,
                axes_mesh,
                models,
            },
            offscreen: None,
            scene: Scene::default(),
            camera: OrbitCamera::default(),
            hovered: None,
            selected: None,
            overlay: Overlay::default(),
            select_suppressed: false,
            input_suppressed: false,
            pending_pick: None,
            last_pick: None,
            last_rect: None,
        }
    }

    /// Uploads one instance's geometry, leaving every other instance's
    /// buffers exactly where they were. Both an instance's first appearance
    /// and a replacement mesh for an existing one. Hover and selection are
    /// left alone: an `InstanceId` outlives its mesh, though a hit's
    /// triangle index may no longer mean the same thing.
    pub fn set_instance(&mut self, id: InstanceId, mesh: &TriMesh) -> Result<(), SceneFull> {
        let result = self.scene.set_instance(&self.gpu.device, id, mesh);
        // New geometry under an unmoved cursor changes what the cursor is
        // over; the memo must not answer for it.
        self.last_pick = None;
        result
    }

    /// Drops an instance and its buffers, and any hover/selection on it.
    pub fn remove_instance(&mut self, id: InstanceId) -> bool {
        let removed = self.scene.remove(id);
        if removed {
            self.forget_missing_picks();
        }
        removed
    }

    /// Shows or hides an instance. Uploads nothing. A hidden instance stops
    /// being hovered or selected.
    pub fn set_instance_visible(&mut self, id: InstanceId, visible: bool) -> bool {
        let changed = self.scene.set_visible(id, visible);
        if changed {
            self.forget_missing_picks();
        }
        changed
    }

    /// Places an instance. Also uploads nothing — the transform is a
    /// uniform, which is what makes a joint preview a matrix write.
    pub fn set_instance_model(&mut self, id: InstanceId, model: DMat4) -> bool {
        self.scene.set_model(id, model)
    }

    /// Tints an instance (the link's material colour). A uniform write,
    /// no upload.
    pub fn set_instance_color(&mut self, id: InstanceId, color: [f32; 4]) -> bool {
        self.scene.set_color(id, color)
    }

    pub fn has_instance(&self, id: InstanceId) -> bool {
        self.scene.contains(id)
    }

    pub fn instance_count(&self) -> usize {
        self.scene.len()
    }

    /// Every instance in the scene, visible or not, in draw order — what
    /// `debug_state()` reports (ADR-0003).
    pub fn instance_states(&self) -> impl Iterator<Item = InstanceState> + '_ {
        self.scene.entries().map(|entry| InstanceState {
            id: entry.key,
            visible: entry.visible,
            triangle_count: entry.mesh.triangle_count,
            bounds: entry.bounds,
            model: entry.model,
            color: entry.color,
        })
    }

    pub fn clear_scene(&mut self) {
        self.scene.clear();
        self.reset_picks();
    }

    /// Forgets hover, selection, and any pick in flight.
    pub fn reset_picks(&mut self) {
        self.hovered = None;
        self.selected = None;
        self.pending_pick = None;
        self.last_pick = None;
    }

    /// Drops a hover/selection whose instance just stopped being drawn.
    fn forget_missing_picks(&mut self) {
        let gone = |hit: &Option<PickHit>| {
            hit.is_some_and(|h| self.scene.visible_instance(h.instance).is_none())
        };
        if gone(&self.hovered) {
            self.hovered = None;
        }
        if gone(&self.selected) {
            self.selected = None;
        }
        self.last_pick = None;
    }

    /// The triangle under the cursor, if any — resolved one or more
    /// rendered frames after the cursor moved there.
    pub fn hovered(&self) -> Option<PickHit> {
        self.hovered
    }

    /// The selected triangle, if any: the last click's hit.
    pub fn selected(&self) -> Option<PickHit> {
        self.selected
    }

    pub fn clear_selection(&mut self) {
        self.selected = None;
    }

    /// Selects an instance from outside the viewport (the tree panel), or
    /// clears the selection with `None`. Selection is per instance, so the
    /// hit's triangle is `0`; an id that is not in the scene selects
    /// nothing rather than a ghost.
    pub fn set_selected(&mut self, id: Option<InstanceId>) {
        self.selected = id
            .filter(|id| self.scene.contains(*id))
            .map(|instance| PickHit {
                instance,
                triangle: 0,
            });
    }

    /// Bounding sphere `(center, radius)` of every visible instance.
    pub fn scene_bounds(&self) -> Option<(DVec3, f64)> {
        self.scene.bounds()
    }

    /// Frames every visible instance **without** animating there — one
    /// frame, reproducible, which is what a snapshot needs. The animated
    /// form is [`Self::animate_frame_scene`].
    pub fn frame_scene(&mut self) {
        let (center, radius) = self.scene_bounds().unwrap_or((DVec3::ZERO, 1.0));
        self.camera.frame_bounds(center.as_vec3(), radius as f32);
    }

    /// Animates the camera to frame every visible instance (Home, and
    /// zoom-to-fit after a load). No-op on an empty scene.
    pub fn animate_frame_scene(&mut self) {
        if let Some((center, radius)) = self.scene_bounds() {
            self.camera
                .animate_frame_bounds(center.as_vec3(), radius as f32);
        }
    }

    /// Replaces what is drawn over the scene next frame.
    pub fn set_overlay(&mut self, overlay: Overlay) {
        self.overlay = overlay;
    }

    pub fn overlay(&self) -> &Overlay {
        &self.overlay
    }

    /// The world-space ray through `pos` (egui logical points), from the
    /// near plane into the scene.
    ///
    /// `f64`, and from the inverse view-projection rather than from the
    /// camera basis, so it is right in both projections: under an
    /// orthographic camera the ray's origin moves with the cursor and its
    /// direction does not, which a basis-and-fov construction gets wrong.
    /// `dir` is unit length.
    pub fn cursor_ray(&self, pos: egui::Pos2) -> Option<Ray> {
        let rect = self.last_rect?;
        if rect.width() <= 0.0 || rect.height() <= 0.0 {
            return None;
        }
        let aspect = rect.width() / rect.height();
        let inverse = self.camera.view_proj(aspect).as_dmat4().inverse();
        let ndc_x = ((pos.x - rect.min.x) / rect.width()) as f64 * 2.0 - 1.0;
        let ndc_y = 1.0 - ((pos.y - rect.min.y) / rect.height()) as f64 * 2.0;
        // wgpu clip depth is [0, 1]: 0 is the near plane (ADR-0001).
        let unproject = |depth: f64| {
            let p = inverse * riggen_mesh::glam::DVec4::new(ndc_x, ndc_y, depth, 1.0);
            (p.w != 0.0).then(|| p.truncate() / p.w)
        };
        let near = unproject(0.0)?;
        let far = unproject(1.0)?;
        let dir = (far - near).normalize_or_zero();
        (dir != DVec3::ZERO).then_some(Ray { origin: near, dir })
    }

    /// Whether the pointer is ignored this frame (see `input_suppressed`).
    pub fn set_input_suppressed(&mut self, suppressed: bool) {
        self.input_suppressed = suppressed;
    }

    /// Whether a click may change the selection (see `select_suppressed`).
    pub fn set_select_suppressed(&mut self, suppressed: bool) {
        self.select_suppressed = suppressed;
    }

    /// Where `world` lands on screen, in egui logical points, or `None`
    /// when it is behind the camera or outside the depth range.
    ///
    /// The one projection everything drawn *over* the viewport goes through
    /// — glyphs, snap markers, a test aiming a click at a part — so an
    /// overlay can never disagree with the wgpu pass about where a point is:
    /// both start from `camera.view_proj`.
    pub fn project(&self, world: DVec3) -> Option<egui::Pos2> {
        let rect = self.last_rect?;
        let aspect = rect.width().max(1.0) / rect.height().max(1.0);
        let clip = self.camera.view_proj(aspect) * world.as_vec3().extend(1.0);
        if clip.w <= 0.0 {
            return None;
        }
        let ndc = clip.truncate() / clip.w;
        if !(-1.0..=1.0).contains(&ndc.z) {
            return None;
        }
        Some(egui::pos2(
            rect.min.x + (ndc.x * 0.5 + 0.5) * rect.width(),
            rect.min.y + (0.5 - ndc.y * 0.5) * rect.height(),
        ))
    }

    /// The rect the last [`Self::ui`] call allocated, in logical points.
    /// `None` before the first frame.
    pub fn viewport_rect(&self) -> Option<egui::Rect> {
        self.last_rect
    }

    /// Whether the next frame will look the same as this one absent input:
    /// nothing animating, no pick readback in flight. The snapshot harness
    /// pumps frames until this holds.
    pub fn is_settled(&self) -> bool {
        !self.camera.is_animating() && self.pending_pick.is_none()
    }

    /// Takes a resolved readback, if the one in flight has landed, and
    /// applies it to hover or selection. A slot nobody holds any more (the
    /// instance was removed while the pick was in flight) reads as a miss.
    fn resolve_pending_pick(&mut self) {
        let Some(pending) = self.pending_pick.take() else {
            return;
        };
        let resolved = pending.result.lock().unwrap().take();
        match resolved {
            Some(ids) => {
                let hit =
                    resolve_pick_region(&ids, &pending.region).and_then(|(slot, triangle)| {
                        let instance = self.scene.instance_at_slot(slot)?;
                        Some(PickHit { instance, triangle })
                    });
                match pending.kind {
                    PickKind::Hover => self.hovered = hit,
                    PickKind::Select => self.selected = hit,
                }
            }
            None => self.pending_pick = Some(pending),
        }
    }

    fn ensure_offscreen(&mut self, size: (u32, u32)) {
        if self.offscreen.as_ref().map(|o| o.size) == Some(size) {
            return;
        }
        let (width, height) = size;
        let extent = wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        };

        let color_texture = self.gpu.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("riggen-viewport color"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: self.gpu.format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let color_view = color_texture.create_view(&wgpu::TextureViewDescriptor::default());

        let depth_texture = self.gpu.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("riggen-viewport depth"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        let blit_bind_group = self
            .gpu
            .device
            .create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("riggen-viewport blit bind group"),
                layout: &self.gpu.blit_bind_group_layout,
                entries: &[
                    wgpu::BindGroupEntry {
                        binding: 0,
                        resource: wgpu::BindingResource::TextureView(&color_view),
                    },
                    wgpu::BindGroupEntry {
                        binding: 1,
                        resource: wgpu::BindingResource::Sampler(&self.gpu.sampler),
                    },
                ],
            });

        let pick_color_texture = self.gpu.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("riggen-viewport pick ids"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: PICK_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let pick_color_view =
            pick_color_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let pick_depth_texture = self.gpu.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("riggen-viewport pick depth"),
            size: extent,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            view_formats: &[],
        });
        let pick_depth_view =
            pick_depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        self.offscreen = Some(OffscreenTarget {
            size,
            color_view,
            depth_view,
            blit_bind_group,
            pick_color_texture,
            pick_color_view,
            pick_depth_view,
        });
    }

    /// Camera input while the pointer is over the viewport. Returns whether
    /// the camera changed, so the caller can request a repaint.
    fn handle_input(&mut self, ui: &egui::Ui, response: &egui::Response, rect: egui::Rect) -> bool {
        let mut changed = false;
        let aspect = rect.width().max(1.0) / rect.height().max(1.0);

        if response.dragged_by(egui::PointerButton::Middle) {
            let delta = response.drag_delta();
            if ui.input(|i| i.modifiers.shift) {
                self.camera.pan(delta.x, delta.y);
            } else {
                self.camera.orbit(-delta.x * 0.01, delta.y * 0.01);
            }
            changed = true;
        }

        // Unsmoothed, and only while the pointer is over the viewport — like
        // every other viewport shortcut (see `raw_wheel_delta_y`).
        let scroll = if response.hovered() {
            let options = ui.ctx().options(|o| o.input_options);
            ui.input(|i| raw_wheel_delta_y(i, &options))
        } else {
            0.0
        };
        if scroll != 0.0 {
            // Cursor position in NDC (x right, y up, `[-1, 1]`), falling
            // back to dead-center (target-anchored zoom) when the pointer
            // position isn't known this frame.
            let cursor = response.hover_pos().unwrap_or(rect.center());
            let ndc = (
                (cursor.x - rect.center().x) / (rect.width().max(1.0) * 0.5),
                -(cursor.y - rect.center().y) / (rect.height().max(1.0) * 0.5),
            );
            self.camera.zoom_to_cursor(scroll, ndc, aspect);
            changed = true;
        }

        // Standard views, persp/ortho toggle and zoom-to-fit are viewport
        // shortcuts, not global ones — only live while the pointer is over
        // it.
        if response.hovered() {
            let mut fit = false;
            ui.input(|i| {
                let view_key = |key: egui::Key, plain: StandardView, ctrl: StandardView| {
                    i.key_pressed(key)
                        .then_some(if i.modifiers.ctrl { ctrl } else { plain })
                };
                let views = [
                    view_key(egui::Key::Num1, StandardView::Front, StandardView::Back),
                    view_key(egui::Key::Num3, StandardView::Right, StandardView::Left),
                    view_key(egui::Key::Num7, StandardView::Top, StandardView::Bottom),
                    view_key(egui::Key::Num0, StandardView::Iso, StandardView::Iso),
                ];
                for view in views.into_iter().flatten() {
                    self.camera.set_standard_view(view);
                    changed = true;
                }
                if i.key_pressed(egui::Key::Num5) || i.key_pressed(egui::Key::P) {
                    self.camera.toggle_projection();
                    changed = true;
                }
                fit = i.key_pressed(egui::Key::Home);
            });
            if fit {
                self.animate_frame_scene();
                changed = true;
            }
        }
        changed
    }

    /// Projects and strokes every overlay item. Items whose points are all
    /// off screen (behind the camera, outside the depth range) are dropped;
    /// a polyline is split so a partly visible one still draws its visible
    /// run rather than vanishing.
    fn paint_overlay(&self, ui: &egui::Ui, rect: egui::Rect) {
        if self.overlay.is_empty() {
            return;
        }
        let painter = ui.painter().with_clip_rect(rect);
        let stroke_path = |points: &[DVec3], color: egui::Color32, width: f32| {
            let mut run: Vec<egui::Pos2> = Vec::with_capacity(points.len());
            for p in points {
                match self.project(*p) {
                    Some(screen) => run.push(screen),
                    None => {
                        if run.len() > 1 {
                            painter.add(egui::Shape::line(
                                std::mem::take(&mut run),
                                egui::Stroke::new(width, color),
                            ));
                        } else {
                            run.clear();
                        }
                    }
                }
            }
            if run.len() > 1 {
                painter.add(egui::Shape::line(run, egui::Stroke::new(width, color)));
            }
        };

        for item in &self.overlay.items {
            match item {
                OverlayItem::Segment {
                    from,
                    to,
                    color,
                    width,
                } => stroke_path(&[*from, *to], *color, *width),
                OverlayItem::Polyline {
                    points,
                    color,
                    width,
                } => stroke_path(points, *color, *width),
                OverlayItem::Arc {
                    center,
                    axis,
                    start,
                    radius,
                    sweep,
                    color,
                    width,
                } => stroke_path(
                    &OverlayItem::arc_points(*center, *axis, *start, *radius, *sweep),
                    *color,
                    *width,
                ),
                OverlayItem::Point { at, radius, color } => {
                    if let Some(screen) = self.project(*at) {
                        painter.circle_filled(screen, *radius, *color);
                    }
                }
                OverlayItem::Label {
                    at,
                    text,
                    color,
                    offset,
                } => {
                    if let Some(screen) = self.project(*at) {
                        painter.text(
                            screen + *offset,
                            egui::Align2::LEFT_BOTTOM,
                            text,
                            egui::FontId::proportional(12.0),
                            *color,
                        );
                    }
                }
            }
        }
    }

    /// Allocates the viewport rect, handles camera input and picking, and
    /// enqueues the paint callback. Call once per frame inside the central
    /// panel.
    pub fn ui(&mut self, ui: &mut egui::Ui) -> egui::Response {
        if self.camera.step_animation(Instant::now()) {
            ui.ctx().request_repaint();
        }

        // Non-blocking: lets a previously submitted pick readback's
        // `map_async` callback fire. The readback must never stall a frame.
        let _ = self.gpu.device.poll(wgpu::PollType::Poll);
        self.resolve_pending_pick();

        let (rect, response) =
            ui.allocate_exact_size(ui.available_size(), egui::Sense::click_and_drag());
        self.last_rect = Some(rect);
        let aspect = rect.width().max(1.0) / rect.height().max(1.0);

        if !self.input_suppressed && self.handle_input(ui, &response, rect) {
            ui.ctx().request_repaint();
        }

        let pixels_per_point = ui.ctx().pixels_per_point();
        let size = (
            ((rect.width() * pixels_per_point).round() as u32).max(1),
            ((rect.height() * pixels_per_point).round() as u32).max(1),
        );
        self.ensure_offscreen(size);
        let Some(offscreen) = &self.offscreen else {
            return response;
        };

        // Camera input above is already applied, so this is the matrix the
        // pick pass will rasterize with — which makes it the right thing to
        // compare a memoised hover pick against.
        let view_proj_matrix = self.camera.view_proj(aspect);
        let inv_view_proj_matrix = if view_proj_matrix.determinant() != 0.0 {
            view_proj_matrix.inverse()
        } else {
            Mat4::IDENTITY
        };
        let eye = self.camera.eye();
        let (forward, right, up) = self.camera.basis();
        let is_ortho = if self.camera.projection == Projection::Orthographic {
            1.0
        } else {
            0.0
        };
        let is_dark_mode = if ui.visuals().dark_mode { 1.0 } else { 0.0 };
        let camera_uniforms = CameraUniforms {
            view_proj: view_proj_matrix.to_cols_array_2d(),
            inv_view_proj: inv_view_proj_matrix.to_cols_array_2d(),
            eye: [eye.x, eye.y, eye.z, 1.0],
            up: [up.x, up.y, up.z, 0.0],
            right: [right.x, right.y, right.z, 0.0],
            forward: [forward.x, forward.y, forward.z, 0.0],
            params: [is_ortho, aspect, is_dark_mode, 0.0],
        };

        let to_pixel = |pos: egui::Pos2| -> (u32, u32) {
            let local = (pos - rect.min) * pixels_per_point;
            (
                (local.x.round() as i64).clamp(0, size.0 as i64 - 1) as u32,
                (local.y.round() as i64).clamp(0, size.1 as i64 - 1) as u32,
            )
        };
        let view_proj = view_proj_matrix.to_cols_array_2d();
        let decision = decide_pick(
            self.pending_pick.is_some() || self.scene.is_empty(),
            (!self.input_suppressed && !self.select_suppressed)
                .then(|| response.clicked().then(|| response.interact_pointer_pos()))
                .flatten()
                .flatten()
                .map(to_pixel),
            (!self.input_suppressed)
                .then(|| response.hover_pos())
                .flatten()
                .map(to_pixel),
            self.last_pick,
            view_proj,
        );
        let mut pick_pass: Option<PickPassData> = None;
        match decision {
            PickDecision::Nothing => {}
            PickDecision::ClearHover => {
                self.hovered = None;
                self.last_pick = None;
            }
            PickDecision::Issue { kind, pixel } => {
                let region = PickRegion::around(pixel, size);
                let result = Arc::new(Mutex::new(None));
                let readback_buffer = self.gpu.device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("riggen-viewport pick readback"),
                    size: PICK_READBACK_SIZE,
                    usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                    mapped_at_creation: false,
                });
                self.pending_pick = Some(PendingPick {
                    kind,
                    region,
                    result: result.clone(),
                });
                self.last_pick = Some(PickInputs { pixel, view_proj });
                pick_pass = Some(PickPassData {
                    region,
                    readback_buffer,
                    result,
                });
            }
        }
        if self.pending_pick.is_some() {
            // Guarantee at least one more frame runs to consume the async
            // result — egui otherwise only repaints on new input, and a
            // mouse that stops moving would leave the pick unresolved.
            ui.ctx().request_repaint();
        }

        let hover = self
            .hovered
            .and_then(|h| self.scene.visible_instance(h.instance))
            .map(|(i, _)| i);
        let select = self
            .selected
            .and_then(|h| self.scene.visible_instance(h.instance))
            .map(|(i, _)| i);

        // One model matrix and colour per visible instance, packed at the
        // uniform stride and indexed by visible order.
        let visible_count = self.scene.visible().count();
        self.gpu.models.reserve(&self.gpu.device, visible_count);
        let stride = self.gpu.models.stride as usize;
        let mut model_data = vec![0u8; stride * visible_count];
        let mut instances = Vec::with_capacity(visible_count);
        for (i, entry) in self.scene.visible().enumerate() {
            // Model space is `f64`; the GPU layout is `f32`, narrowed here
            // like every other value the viewport uploads.
            let m = entry.model.as_mat4().to_cols_array_2d();
            let matrix_bytes = std::mem::size_of::<[[f32; 4]; 4]>();
            let at = i * stride;
            model_data[at..at + matrix_bytes].copy_from_slice(bytemuck::cast_slice(&[m]));
            model_data[at + matrix_bytes..at + matrix_bytes + std::mem::size_of::<[f32; 4]>()]
                .copy_from_slice(bytemuck::cast_slice(&entry.color));
            instances.push(InstanceBuffers {
                model_offset: self.gpu.models.offset(i),
                vertex_buffer: entry.mesh.vertex_buffer.clone(),
                index_buffer: entry.mesh.index_buffer.clone(),
                index_count: entry.mesh.index_count,
                pick_vertex_buffer: entry.mesh.pick_vertex_buffer.clone(),
                triangle_count: entry.mesh.triangle_count,
            });
        }

        // Bottom-left corner square, clamped so it never outgrows a tiny
        // viewport panel.
        let gizmo_size = AXES_GIZMO_SIZE
            .min(size.0 as f32 * 0.5)
            .min(size.1 as f32 * 0.5);
        let axes_viewport = (
            AXES_GIZMO_MARGIN,
            (size.1 as f32 - gizmo_size - AXES_GIZMO_MARGIN).max(0.0),
            gizmo_size.max(1.0),
            gizmo_size.max(1.0),
        );

        let callback = ViewportCallback {
            camera_uniforms,
            axes_view_proj: self.camera.axes_gizmo_view_proj().to_cols_array_2d(),
            axes_viewport,
            uniform_buffer: self.gpu.uniform_buffer.clone(),
            uniform_bind_group: self.gpu.uniform_bind_group.clone(),
            axes_uniform_buffer: self.gpu.axes_uniform_buffer.clone(),
            axes_uniform_bind_group: self.gpu.axes_uniform_bind_group.clone(),
            scene_pipeline: self.gpu.scene_pipeline.clone(),
            background_pipeline: self.gpu.background_pipeline.clone(),
            hover_pipeline: self.gpu.hover_pipeline.clone(),
            select_pipeline: self.gpu.select_pipeline.clone(),
            axes_pipeline: self.gpu.axes_pipeline.clone(),
            pick_pipeline: self.gpu.pick_pipeline.clone(),
            blit_pipeline: self.gpu.blit_pipeline.clone(),
            axes_vertex_buffer: self.gpu.axes_mesh.vertex_buffer.clone(),
            axes_vertex_count: self.gpu.axes_mesh.vertex_count,
            instances,
            model_bind_group: self.gpu.models.bind_group.clone(),
            model_buffer: self.gpu.models.buffer.clone(),
            model_data,
            hover,
            select,
            color_view: offscreen.color_view.clone(),
            depth_view: offscreen.depth_view.clone(),
            blit_bind_group: offscreen.blit_bind_group.clone(),
            pick_color_view: offscreen.pick_color_view.clone(),
            pick_color_texture: offscreen.pick_color_texture.clone(),
            pick_depth_view: offscreen.pick_depth_view.clone(),
            pick: pick_pass,
        };
        ui.painter()
            .add(egui_wgpu::Callback::new_paint_callback(rect, callback));

        self.paint_overlay(ui, rect);

        // The projection label is render state a snapshot should show, so
        // it stays in the viewport corner; the wall-clock frame-time
        // readout lives in the app's status bar instead.
        let hud_color = if ui.visuals().dark_mode {
            egui::Color32::from_white_alpha(200)
        } else {
            egui::Color32::from_black_alpha(200)
        };
        let projection_label = match self.camera.projection {
            Projection::Perspective => "persp",
            Projection::Orthographic => "ortho",
        };
        ui.painter().text(
            rect.right_bottom() + egui::vec2(-8.0, -8.0),
            egui::Align2::RIGHT_BOTTOM,
            projection_label,
            egui::FontId::monospace(12.0),
            hud_color,
        );

        response
    }
}
