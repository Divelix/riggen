// The ground: the world z = 0 plane, drawn as two lattices of lines so a part
// at the origin reads as standing on something instead of floating in the
// gradient.
//
// A fullscreen triangle rather than a quad of geometry: the plane is infinite,
// and any finite mesh for it would have an edge the camera could reach. Each
// pixel's ray is unprojected through `inv_view_proj` and intersected with the
// plane, which needs no orthographic/perspective split to be correct — the
// matrix already carries the difference. (`background.wgsl` splits the two,
// but for the look of its colour blend, not for correctness.)

struct CameraUniforms {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    eye: vec4<f32>,
    up: vec4<f32>,
    right: vec4<f32>,
    forward: vec4<f32>,
    params: vec4<f32>, // (is_ortho, aspect, is_dark_mode, 0.0)
};

@group(0) @binding(0) var<uniform> u: CameraUniforms;

// Metre lines, and a heavier one every ten of them. Two scales is what lets
// the eye read a distance off the floor without counting to twenty; both are
// in document units (AGENTS.md: meters), so a line is a metre wherever the
// camera is.
const MINOR: f32 = 1.0;
const MAJOR: f32 = 10.0;

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) ndc: vec2<f32>,
};

struct FragmentOutput {
    @location(0) color: vec4<f32>,
    // Written from the plane intersection, so the depth test against what the
    // opaque pass wrote is a real one: a part in front of the ground hides it.
    @builtin(frag_depth) depth: f32,
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
    out.ndc = p;
    return out;
}

// Coverage of one lattice of lines at `spacing` metres, antialiased to this
// pixel's footprint on the plane and `width` pixels thick.
//
// The screen-space derivative is doing two jobs. Dividing by it is what makes
// a line a constant thickness in pixels however far away it is; and when it
// grows, so that the lines crowd closer together than the pixels that would
// draw them, the lattice is faded out instead of turning into a wash. That
// second job is the horizon fade: a ray near grazing has an enormous
// footprint, so the floor thins to nothing where it runs away from the camera
// rather than ending at a rim. The fade starts while the cells are still
// about a dozen pixels apart, because a cell only a few pixels wide is
// already more line than gap.
fn lattice(coord: vec2<f32>, spacing: f32, width: f32) -> f32 {
    let c = coord / spacing;
    let footprint = max(fwidth(c), vec2<f32>(1e-8, 1e-8));
    let d = footprint * width;
    let distance_to_line = abs(fract(c - 0.5) - 0.5) / d;
    let line = 1.0 - clamp(min(distance_to_line.x, distance_to_line.y), 0.0, 1.0);
    return line * (1.0 - smoothstep(0.08, 0.25, max(footprint.x, footprint.y)));
}

@fragment
fn fs_main(in: VertexOutput) -> FragmentOutput {
    let p_near = u.inv_view_proj * vec4<f32>(in.ndc, 0.0, 1.0);
    let p_far = u.inv_view_proj * vec4<f32>(in.ndc, 1.0, 1.0);
    let near = p_near.xyz / p_near.w;
    let far = p_far.xyz / p_far.w;
    let dir = far - near;

    // `t` runs 0 to 1 from the near plane to the far one, so "behind the
    // camera" and "past the far clip" are exactly `t < 0` and `t > 1`.
    let crosses = abs(dir.z) > 1e-9;
    let t = select(-1.0, -near.z / dir.z, crosses);
    let hit = near + dir * t;

    // Both lattices before any `discard`: a quad straddling the horizon still
    // owes its neighbours real derivatives.
    let minor = lattice(hit.xy, MINOR, 1.0);
    let major = lattice(hit.xy, MAJOR, 1.6);

    if (!crosses || t < 0.0 || t > 1.0) {
        discard;
    }

    // Near-neutral, and quiet: the floor is furniture, and a coloured or
    // contrasty one would compete with the parts standing on it. Same
    // `is_dark_mode` switch the background reads.
    let is_dark = clamp(u.params.z, 0.0, 1.0);
    let minor_color = mix(vec3<f32>(0.35, 0.37, 0.40), vec3<f32>(0.52, 0.55, 0.60), is_dark);
    let major_color = mix(vec3<f32>(0.20, 0.22, 0.25), vec3<f32>(0.72, 0.76, 0.82), is_dark);
    let minor_alpha = 0.30 * minor;
    let major_alpha = 0.55 * major;

    let alpha = max(minor_alpha, major_alpha);
    let color = mix(minor_color, major_color, major_alpha / max(alpha, 1e-6));

    let clip = u.view_proj * vec4<f32>(hit, 1.0);

    var out: FragmentOutput;
    out.color = vec4<f32>(color, alpha);
    out.depth = clamp(clip.z / clip.w, 0.0, 1.0);
    return out;
}
