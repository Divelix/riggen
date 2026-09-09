//! The wgpu viewport: orbit camera, instance scene, ID-buffer picking,
//! ported from `robocad-viewport` (ADR-0001, docs/ARCHITECTURE.md).

mod camera;
mod gpu_mesh;
mod overlay;
pub mod pick_id;
mod scene;
mod viewport;

pub use camera::{
    CameraAnimation, CameraSample, OrbitCamera, Projection, StandardView, ViewOrientation,
    shortest_angular_delta,
};
pub use gpu_mesh::{AxesTriadMesh, ColorVertex, GpuMesh, PickVertex, Vertex};
pub use overlay::{HIDDEN_STRENGTH, Occlusion, Overlay, OverlayEntry, OverlayItem};
pub use scene::{
    DEFAULT_INSTANCE_COLOR, InstanceEntry, InstanceId, InstancePayload, MAX_INSTANCES, RenderGroup,
    Scene, SceneFull,
};
pub use viewport::depth::DepthImage;
pub use viewport::{InstanceState, Viewport};

/// What the cursor is over: one triangle of one instance
/// (docs/ARCHITECTURE.md §Picking and snapping).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PickHit {
    pub instance: InstanceId,
    pub triangle: u32,
}
