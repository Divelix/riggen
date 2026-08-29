struct Uniforms {
    view_proj: mat4x4<f32>,
};
@group(0) @binding(0) var<uniform> u: Uniforms;

// Per-instance model transform, bound with a dynamic offset (see
// `crate::scene::Scene`). Instances are placed by rigid transforms, so the
// normal takes the same matrix rather than an inverse-transpose.
@group(1) @binding(0) var<uniform> model: mat4x4<f32>;

struct VertexInput {
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
};

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) normal: vec3<f32>,
};

@vertex
fn vs_main(in: VertexInput) -> VertexOutput {
    var out: VertexOutput;
    out.clip_position = u.view_proj * model * vec4<f32>(in.position, 1.0);
    out.normal = (model * vec4<f32>(in.normal, 0.0)).xyz;
    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let light_dir = normalize(vec3<f32>(0.4, -0.6, 0.8));
    let n = normalize(in.normal);
    let diffuse = max(dot(n, light_dir), 0.0);
    let ambient = 0.3;
    let base_color = vec3<f32>(0.55, 0.65, 0.78);
    let color = base_color * (ambient + diffuse * 0.7);
    return vec4<f32>(color, 1.0);
}
