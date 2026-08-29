struct BackgroundUniforms {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    eye: vec4<f32>,
    up: vec4<f32>,
    right: vec4<f32>,
    forward: vec4<f32>,
    params: vec4<f32>, // (is_ortho, aspect, is_dark_mode, 0.0)
};

@group(0) @binding(0) var<uniform> u: BackgroundUniforms;

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) world_dir: vec3<f32>,
};

@vertex
fn vs_main(@builtin(vertex_index) idx: u32) -> VertexOutput {
    var positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(3.0, -1.0),
        vec2<f32>(-1.0, 3.0),
    );
    let p = positions[idx];
    var out: VertexOutput;
    out.clip_position = vec4<f32>(p, 0.0, 1.0);

    if (u.params.x > 0.5) {
        // Orthographic projection:
        // Synthesize an intuitive perspective spread based on screen position
        // so that orientation (pitch, roll, yaw) is clearly visible across the viewport.
        let spread_y = 0.45;
        let spread_x = spread_y * u.params.y;
        out.world_dir = u.forward.xyz + u.up.xyz * (p.y * spread_y) + u.right.xyz * (p.x * spread_x);
    } else {
        // Perspective projection:
        // Unproject near and far plane points from NDC to get true world-space ray direction
        let p_near = u.inv_view_proj * vec4<f32>(p.x, p.y, 0.0, 1.0);
        let p_far = u.inv_view_proj * vec4<f32>(p.x, p.y, 1.0, 1.0);
        let world_near = p_near.xyz / p_near.w;
        let world_far = p_far.xyz / p_far.w;
        out.world_dir = world_far - world_near;
    }

    return out;
}

@fragment
fn fs_main(in: VertexOutput) -> @location(0) vec4<f32> {
    let dir = normalize(in.world_dir);

    // Directional anchor colors for the 6 cardinal 3D world directions:
    // +X (Right):  Warm Amber / Terracotta
    // -X (Left):   Ocean Cyan / Teal
    // +Y (Front):  Sage / Emerald
    // -Y (Back):   Plum / Violet
    // +Z (Top):    Slate Blue / Sky
    // -Z (Bottom): Deep Graphite / Charcoal

    // Dark theme directional palette:
    let dark_pos_x = vec3<f32>(0.20, 0.12, 0.10);
    let dark_neg_x = vec3<f32>(0.08, 0.18, 0.19);
    let dark_pos_y = vec3<f32>(0.10, 0.18, 0.12);
    let dark_neg_y = vec3<f32>(0.16, 0.10, 0.18);
    let dark_pos_z = vec3<f32>(0.09, 0.14, 0.22);
    let dark_neg_z = vec3<f32>(0.05, 0.055, 0.065);

    // Light theme directional palette:
    let light_pos_x = vec3<f32>(0.92, 0.88, 0.84);
    let light_neg_x = vec3<f32>(0.84, 0.90, 0.92);
    let light_pos_y = vec3<f32>(0.85, 0.91, 0.86);
    let light_neg_y = vec3<f32>(0.90, 0.85, 0.91);
    let light_pos_z = vec3<f32>(0.91, 0.94, 0.98);
    let light_neg_z = vec3<f32>(0.75, 0.77, 0.80);

    let is_dark = clamp(u.params.z, 0.0, 1.0);
    let c_pos_x = mix(light_pos_x, dark_pos_x, is_dark);
    let c_neg_x = mix(light_neg_x, dark_neg_x, is_dark);
    let c_pos_y = mix(light_pos_y, dark_pos_y, is_dark);
    let c_neg_y = mix(light_neg_y, dark_neg_y, is_dark);
    let c_pos_z = mix(light_pos_z, dark_pos_z, is_dark);
    let c_neg_z = mix(light_neg_z, dark_neg_z, is_dark);

    // Smooth quadratic directional blending:
    // sum of max(±d_i, 0)^2 = x^2 + y^2 + z^2 = 1.0 (exact partition of unity everywhere)
    let wx_pos = max(dir.x, 0.0) * max(dir.x, 0.0);
    let wx_neg = max(-dir.x, 0.0) * max(-dir.x, 0.0);
    let wy_pos = max(dir.y, 0.0) * max(dir.y, 0.0);
    let wy_neg = max(-dir.y, 0.0) * max(-dir.y, 0.0);
    let wz_pos = max(dir.z, 0.0) * max(dir.z, 0.0);
    let wz_neg = max(-dir.z, 0.0) * max(-dir.z, 0.0);

    let color = c_pos_x * wx_pos + c_neg_x * wx_neg
              + c_pos_y * wy_pos + c_neg_y * wy_neg
              + c_pos_z * wz_pos + c_neg_z * wz_neg;

    return vec4<f32>(color, 1.0);
}
