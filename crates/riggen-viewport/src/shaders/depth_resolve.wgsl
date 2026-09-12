// Multisampled depth -> single-sampled depth, so the overlay's readback has
// something `copy_texture_to_buffer` will accept (ADR-0020; WebGPU has no
// depth `resolve_target` and refuses a multisampled copy source).
//
// Sample 0, not an average of the samples: a blended depth would name a
// surface no sample actually wrote, which is the same reason the pick pass
// stays single-sampled. The overlay's `DEPTH_BIAS` already carries the slack
// a near-surface glyph needs, and one sample's depth is a real one.

@group(0) @binding(0) var t_depth: texture_depth_multisampled_2d;

@vertex
fn vs_main(@builtin(vertex_index) idx: u32) -> @builtin(position) vec4<f32> {
    var positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(3.0, -1.0),
        vec2<f32>(-1.0, 3.0),
    );
    return vec4<f32>(positions[idx], 0.0, 1.0);
}

@fragment
fn fs_main(@builtin(position) pos: vec4<f32>) -> @builtin(frag_depth) f32 {
    return textureLoad(t_depth, vec2<i32>(pos.xy), 0);
}
