// ID-buffer pick pass: rasterizes each triangle's packed pick id (see
// `crate::pick_id`) into an R32Uint target instead of a shaded color
// (docs/01-architecture.md §Picking and snapping).

struct Uniforms {
    view_proj: mat4x4<f32>,
};
@group(0) @binding(0) var<uniform> u: Uniforms;
@group(1) @binding(0) var<uniform> model: mat4x4<f32>;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) pick_id: u32,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) @interpolate(flat) pick_id: u32,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.clip_position = u.view_proj * model * vec4<f32>(in.position, 1.0);
    out.pick_id = in.pick_id;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) u32 {
    return in.pick_id;
}
