//! The wgpu viewport: orbit camera, instance scene, ID-buffer picking,
//! ported from `robocad-viewport` (ADR-0001, docs/01-architecture.md).

mod camera;

pub use camera::{
    CameraAnimation, CameraSample, OrbitCamera, Projection, StandardView, ViewOrientation,
    shortest_angular_delta,
};
