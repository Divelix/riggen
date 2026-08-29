//! The wgpu viewport: orbit camera, instance scene, ID-buffer picking,
//! ported from `robocad-viewport` (ADR-0001, docs/01-architecture.md).

mod camera;
mod gpu_mesh;
pub mod pick_id;
mod scene;
mod viewport;

pub use camera::{
    CameraAnimation, CameraSample, OrbitCamera, Projection, StandardView, ViewOrientation,
    shortest_angular_delta,
};
pub use gpu_mesh::{AxesTriadMesh, ColorVertex, GpuMesh, PickVertex, Vertex};
pub use scene::{InstanceEntry, InstanceId, InstancePayload, MAX_INSTANCES, Scene, SceneFull};
pub use viewport::{InstanceState, Viewport};

/// What the cursor is over: one triangle of one instance
/// (docs/01-architecture.md §Picking and snapping).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct PickHit {
    pub instance: InstanceId,
    pub triangle: u32,
}
