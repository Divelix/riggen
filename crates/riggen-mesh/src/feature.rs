//! Mesh features: what "click the bore, get the joint axis" is made of
//! (docs/02-data-model.md §Mesh features, docs/01-architecture.md
//! §Picking and snapping).
//!
//! There is no B-Rep. An STL is a triangle soup with the same coordinates
//! repeated verbatim per facet, so the only topology available is the one
//! recovered by **welding positions exactly** — bit-for-bit, no tolerance.
//! That is enough: every exporter writes the shared corner of two facets as
//! the same float, and a tolerance would instead merge two genuinely
//! distinct vertices a micron apart.
//!
//! From that adjacency, [`grow_region`] flood-fills a smooth region across
//! shared edges by dihedral angle, and [`fit_circle`] turns such a region
//! into a circle: a curved region (a bore or a shaft wall) gives its axis
//! from the adjacent normals' cross products and its circle from a
//! least-squares fit of the region's vertices; a planar region (a shaft's
//! end face) gives axis = face normal and fits its **boundary loop**.

use std::collections::{BTreeMap, HashMap};

use glam::DVec3;

use crate::TriMesh;

/// The dihedral angle two neighbouring triangles may differ by and still
/// count as one smooth region, in radians.
///
/// 70° is set by the coarsest cylinder [`MIN_SEGMENTS`] still accepts: six
/// segments turn by 60° per step, and coarse STL exports really are that
/// coarse. It stops well short of any square corner. A chamfer shallower
/// than this is absorbed into its neighbour, which is the right trade for a
/// snapping tool — an over-eager fit reports a bad `residual` and is
/// visible, a region that stops early silently halves the radius.
pub const DEFAULT_MAX_DIHEDRAL: f64 = 70.0 * std::f64::consts::PI / 180.0;

/// The fewest distinct angular positions a fit accepts as a circle.
///
/// Four coplanar corners of a square are exactly concyclic, and so is any
/// polygon a modeller drew — nothing in the residual distinguishes a cube
/// face from a very coarse bore. The segment count does: below six, a
/// "circle" is a polygon somebody meant as a polygon.
pub const MIN_SEGMENTS: usize = 6;

/// Welded topology of one mesh: which mesh vertices are the same point, and
/// which triangles share an edge.
///
/// Built once per mesh and reused — the app caches one beside the loaded
/// mesh, since a hover recomputes nothing.
#[derive(Debug, Clone)]
pub struct Adjacency {
    /// Welded index per mesh vertex; two entries are equal exactly when the
    /// positions are bit-identical.
    welded: Vec<u32>,
    welded_count: usize,
    /// Per triangle, the triangle across edge `e` (corners `e` and
    /// `e + 1 mod 3`). `None` at a boundary or a non-manifold edge.
    neighbors: Vec<[Option<u32>; 3]>,
}

impl Adjacency {
    /// The welded index of mesh vertex `vertex`.
    pub fn welded(&self, vertex: usize) -> u32 {
        self.welded[vertex]
    }

    /// How many distinct positions the mesh has.
    pub fn welded_count(&self) -> usize {
        self.welded_count
    }

    /// The three welded corners of triangle `i`.
    pub fn welded_triangle(&self, mesh: &TriMesh, i: usize) -> [u32; 3] {
        [
            self.welded[mesh.indices[3 * i] as usize],
            self.welded[mesh.indices[3 * i + 1] as usize],
            self.welded[mesh.indices[3 * i + 2] as usize],
        ]
    }

    /// The neighbours of triangle `i`, by edge.
    pub fn neighbors(&self, i: usize) -> [Option<u32>; 3] {
        self.neighbors[i]
    }

    /// Whether every edge of the mesh is shared by exactly two triangles.
    /// A closed mesh is what mass properties (M3) need; a fit does not care.
    pub fn is_closed(&self) -> bool {
        self.neighbors.iter().flatten().all(Option::is_some)
    }
}

/// `-0.0` and `0.0` are the same point but different bits; everything else
/// is hashed verbatim.
fn key_of(p: DVec3) -> [u64; 3] {
    [p.x, p.y, p.z].map(|c| if c == 0.0 { 0.0f64 } else { c }.to_bits())
}

/// Welds `mesh`'s vertices by exact position and pairs its triangles across
/// shared edges. See the module doc for why the welding has no tolerance.
pub fn adjacency(mesh: &TriMesh) -> Adjacency {
    let mut ids: HashMap<[u64; 3], u32> = HashMap::new();
    let mut welded = Vec::with_capacity(mesh.positions.len());
    for p in &mesh.positions {
        let next = ids.len() as u32;
        welded.push(*ids.entry(key_of(*p)).or_insert(next));
    }

    let count = mesh.triangle_count();
    let mut edges: HashMap<(u32, u32), Vec<(u32, u8)>> = HashMap::new();
    for t in 0..count {
        let corners = [
            welded[mesh.indices[3 * t] as usize],
            welded[mesh.indices[3 * t + 1] as usize],
            welded[mesh.indices[3 * t + 2] as usize],
        ];
        for e in 0..3 {
            let (a, b) = (corners[e], corners[(e + 1) % 3]);
            if a == b {
                continue; // degenerate edge of a zero-area triangle
            }
            edges
                .entry((a.min(b), a.max(b)))
                .or_default()
                .push((t as u32, e as u8));
        }
    }

    let mut neighbors = vec![[None; 3]; count];
    for sharers in edges.values() {
        // Exactly two: a manifold edge. One is a boundary, three or more is
        // a non-manifold seam; neither gets a neighbour, and growth stops.
        if let [(t0, e0), (t1, e1)] = sharers[..] {
            neighbors[t0 as usize][e0 as usize] = Some(t1);
            neighbors[t1 as usize][e1 as usize] = Some(t0);
        }
    }

    Adjacency {
        welded,
        welded_count: ids.len(),
        neighbors,
    }
}

/// The triangles reachable from `seed` across shared edges without ever
/// turning by more than `max_dihedral` in one step, in ascending order.
///
/// The angle is compared **locally**, between each triangle and the
/// neighbour it is entered from — that is what lets a cylinder wall grow all
/// the way round while a 90° corner stops it. A degenerate (zero-normal)
/// triangle is never entered.
pub fn grow_region(
    mesh: &TriMesh,
    adjacency: &Adjacency,
    seed: usize,
    max_dihedral: f64,
) -> Vec<usize> {
    let mut region = Vec::new();
    if seed >= mesh.triangle_count() || mesh.face_normal(seed) == DVec3::ZERO {
        return region;
    }
    let cos_limit = max_dihedral.cos();
    let mut seen = vec![false; mesh.triangle_count()];
    let mut stack = vec![seed];
    seen[seed] = true;
    while let Some(t) = stack.pop() {
        region.push(t);
        let normal = mesh.face_normal(t);
        for next in adjacency.neighbors(t).into_iter().flatten() {
            let next = next as usize;
            if seen[next] {
                continue;
            }
            let other = mesh.face_normal(next);
            if other != DVec3::ZERO && normal.dot(other) >= cos_limit {
                seen[next] = true;
                stack.push(next);
            }
        }
    }
    region.sort_unstable();
    region
}

/// A circle recovered from a mesh region: what the Place joint tool turns
/// into a joint origin and axis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CircleFit {
    /// On the axis, at the region's mean height.
    pub center: DVec3,
    /// Unit. For a planar region it is the face normal (so it points out of
    /// the material); for a curved one the sign is arbitrary and made
    /// deterministic by giving the largest component a positive sign.
    pub axis: DVec3,
    pub radius: f64,
    /// RMS distance of the fitted points from the circle, in mesh units. A
    /// clean bore reads ~0; the readout in the viewport shows it so a bad
    /// fit is obvious rather than silent.
    pub residual: f64,
    /// Distinct angular positions around the axis — the generator's segment
    /// count for a machine-made cylinder.
    pub segments: usize,
}

/// [`fit_circle_with`] building the adjacency itself. Convenient for tests
/// and one-off calls; the app keeps an [`Adjacency`] per mesh instead.
pub fn fit_circle(mesh: &TriMesh, triangle: usize) -> Option<CircleFit> {
    fit_circle_with(mesh, &adjacency(mesh), triangle)
}

/// Fits a circle to the smooth region around `triangle`, or `None` when
/// that region is not a circle (see [`MIN_SEGMENTS`]).
///
/// Curved region → the axis is the normalised sum of the adjacent normals'
/// cross products (each pair of neighbours turns about the cylinder's axis)
/// and the circle is a least-squares fit of every region vertex projected
/// into the plane ⟂ axis. Planar region → the axis is the face normal and
/// the fit runs on the region's boundary loop, which for a shaft's end face
/// is exactly its rim.
pub fn fit_circle_with(
    mesh: &TriMesh,
    adjacency: &Adjacency,
    triangle: usize,
) -> Option<CircleFit> {
    let region = grow_region(mesh, adjacency, triangle, DEFAULT_MAX_DIHEDRAL);
    if region.is_empty() {
        return None;
    }
    let seed = mesh.face_normal(triangle);
    let planar = region
        .iter()
        .all(|&t| mesh.face_normal(t).dot(seed) > 1.0 - 1e-12);

    let (axis, points) = if planar {
        (seed, boundary_points(mesh, adjacency, &region))
    } else {
        (
            curved_axis(mesh, adjacency, &region)?,
            region_points(mesh, adjacency, &region),
        )
    };
    fit(&points, axis)
}

/// Every distinct position the region's triangles use, in welded-id order
/// so the fit sums them the same way every run (a `HashMap`'s order is
/// per-process, and the last bits of a pivot then jittered between runs —
/// invisible at six decimals, a different `e-7` in every Properties field
/// that shows one).
fn region_points(mesh: &TriMesh, adjacency: &Adjacency, region: &[usize]) -> Vec<DVec3> {
    let mut seen: BTreeMap<u32, DVec3> = BTreeMap::new();
    for &t in region {
        for c in 0..3 {
            let vertex = mesh.indices[3 * t + c] as usize;
            seen.insert(adjacency.welded(vertex), mesh.positions[vertex]);
        }
    }
    seen.into_values().collect()
}

/// The distinct positions on the region's boundary: the edges whose
/// neighbour is outside the region (or missing). Welded-id order, as
/// [`region_points`].
fn boundary_points(mesh: &TriMesh, adjacency: &Adjacency, region: &[usize]) -> Vec<DVec3> {
    let inside: std::collections::HashSet<usize> = region.iter().copied().collect();
    let mut seen: BTreeMap<u32, DVec3> = BTreeMap::new();
    for &t in region {
        for (e, neighbor) in adjacency.neighbors(t).into_iter().enumerate() {
            let outside = match neighbor {
                Some(n) => !inside.contains(&(n as usize)),
                None => true,
            };
            if !outside {
                continue;
            }
            for c in [e, (e + 1) % 3] {
                let vertex = mesh.indices[3 * t + c] as usize;
                seen.insert(adjacency.welded(vertex), mesh.positions[vertex]);
            }
        }
    }
    seen.into_values().collect()
}

/// The axis a curved region turns about: `n_a × n_b` for every pair of
/// neighbours inside it, summed with a consistent sign.
///
/// Which way the sum points depends on the order the pairs happen to be
/// visited, so the result is flipped to give its largest component a
/// positive sign — deterministic, and a joint axis has no preferred
/// direction anyway.
fn curved_axis(mesh: &TriMesh, adjacency: &Adjacency, region: &[usize]) -> Option<DVec3> {
    let inside: std::collections::HashSet<usize> = region.iter().copied().collect();
    let mut sum = DVec3::ZERO;
    let mut reference: Option<DVec3> = None;
    for &t in region {
        let normal = mesh.face_normal(t);
        for next in adjacency.neighbors(t).into_iter().flatten() {
            let next = next as usize;
            // Each manifold pair is seen from both sides; take it once.
            if next <= t || !inside.contains(&next) {
                continue;
            }
            let cross = normal.cross(mesh.face_normal(next));
            if cross.length_squared() < 1e-24 {
                continue; // coplanar neighbours say nothing about the axis
            }
            let reference = *reference.get_or_insert(cross);
            sum += if cross.dot(reference) < 0.0 {
                -cross
            } else {
                cross
            };
        }
    }
    let axis = sum.normalize_or_zero();
    (axis != DVec3::ZERO).then(|| canonical(axis))
}

/// Flips `axis` so its largest-magnitude component is positive.
fn canonical(axis: DVec3) -> DVec3 {
    let largest = [axis.x, axis.y, axis.z]
        .into_iter()
        .max_by(|a, b| a.abs().total_cmp(&b.abs()))
        .expect("three components");
    if largest < 0.0 { -axis } else { axis }
}

/// Kåsa's algebraic circle fit in the plane ⟂ `axis`, with the points
/// shifted to their centroid first so the normal equations decouple:
/// `z = 2ax + 2by + c` with `Σx = Σy = 0` leaves a 2×2 solve and `c = z̄`.
fn fit(points: &[DVec3], axis: DVec3) -> Option<CircleFit> {
    if points.len() < 3 {
        return None;
    }
    let axis = axis.normalize_or_zero();
    if axis == DVec3::ZERO {
        return None;
    }
    let u = axis.any_orthonormal_vector();
    let v = axis.cross(u).normalize();

    let flat: Vec<[f64; 2]> = points.iter().map(|p| [p.dot(u), p.dot(v)]).collect();
    let n = flat.len() as f64;
    let mean = [
        flat.iter().map(|p| p[0]).sum::<f64>() / n,
        flat.iter().map(|p| p[1]).sum::<f64>() / n,
    ];

    let (mut sxx, mut syy, mut sxy, mut sxz, mut syz, mut sz) = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    for p in &flat {
        let (x, y) = (p[0] - mean[0], p[1] - mean[1]);
        let z = x * x + y * y;
        sxx += x * x;
        syy += y * y;
        sxy += x * y;
        sxz += x * z;
        syz += y * z;
        sz += z;
    }
    let det = sxx * syy - sxy * sxy;
    // Collinear (or single-point) input: no circle, and no NaN either.
    if det.abs() <= f64::EPSILON * (sxx * syy).max(1e-300) {
        return None;
    }
    let a = 0.5 * (sxz * syy - syz * sxy) / det;
    let b = 0.5 * (syz * sxx - sxz * sxy) / det;
    let c = sz / n;
    let radius_sq = c + a * a + b * b;
    if radius_sq <= 0.0 || !radius_sq.is_finite() {
        return None;
    }
    let radius = radius_sq.sqrt();

    let centre_2d = [a + mean[0], b + mean[1]];
    let residual = (flat
        .iter()
        .map(|p| {
            let d = ((p[0] - centre_2d[0]).powi(2) + (p[1] - centre_2d[1]).powi(2)).sqrt() - radius;
            d * d
        })
        .sum::<f64>()
        / n)
        .sqrt();

    let height = points.iter().map(|p| p.dot(axis)).sum::<f64>() / n;
    let center = u * centre_2d[0] + v * centre_2d[1] + axis * height;

    let segments = count_segments(&flat, centre_2d);
    if segments < MIN_SEGMENTS {
        return None;
    }
    Some(CircleFit {
        center,
        axis,
        radius,
        residual,
        segments,
    })
}

/// Distinct angular positions of `flat` around `centre`: the angles sorted
/// and clustered, with a gap threshold of half the spacing the points would
/// have if every one of them sat at its own angle. Two rings of a cylinder
/// wall therefore count once, and a coarse polygon counts its corners.
fn count_segments(flat: &[[f64; 2]], centre: [f64; 2]) -> usize {
    let mut angles: Vec<f64> = flat
        .iter()
        .map(|p| (p[1] - centre[1]).atan2(p[0] - centre[0]))
        .collect();
    angles.sort_by(f64::total_cmp);
    let threshold = std::f64::consts::PI / angles.len() as f64;
    let mut segments = 1;
    for pair in angles.windows(2) {
        if pair[1] - pair[0] > threshold {
            segments += 1;
        }
    }
    // The wrap-around gap: if the first and last angle are one cluster, the
    // walk above counted it twice.
    let wrap = angles[0] + std::f64::consts::TAU - angles[angles.len() - 1];
    if segments > 1 && wrap <= threshold {
        segments -= 1;
    }
    segments
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{DMat4, DQuat, DVec3};

    /// A few rigid poses that put the generator's +Z axis somewhere
    /// awkward, so nothing in the fit can be quietly axis-aligned.
    fn poses() -> Vec<DMat4> {
        [
            (DVec3::ZERO, DQuat::IDENTITY),
            (
                DVec3::new(0.4, -1.2, 3.0),
                DQuat::from_rotation_x(std::f64::consts::FRAC_PI_2),
            ),
            (
                DVec3::new(-2.5, 0.75, -0.25),
                DQuat::from_euler(glam::EulerRot::ZYX, 0.7, -1.1, 0.35),
            ),
            (
                DVec3::new(10.0, 10.0, 10.0),
                DQuat::from_axis_angle(DVec3::new(1.0, 2.0, 3.0).normalize(), 2.1),
            ),
        ]
        .into_iter()
        .map(|(t, r)| DMat4::from_rotation_translation(r, t))
        .collect()
    }

    fn posed(mesh: &TriMesh, m: &DMat4) -> TriMesh {
        let mut mesh = mesh.clone();
        mesh.transform(m);
        mesh
    }

    /// The first triangle whose centroid is on the wall at `radius` — i.e.
    /// not a cap — in the mesh's own (untransformed) frame.
    fn wall_triangle(mesh: &TriMesh, radius: f64) -> usize {
        (0..mesh.triangle_count())
            .find(|&i| {
                mesh.face_normal(i).z.abs() < 1e-9
                    && mesh
                        .triangle(i)
                        .iter()
                        .all(|p| (DVec3::new(p.x, p.y, 0.0).length() - radius).abs() < 1e-9)
            })
            .expect("a wall triangle")
    }

    fn cap_triangle(mesh: &TriMesh, z: f64) -> usize {
        (0..mesh.triangle_count())
            .find(|&i| {
                mesh.face_normal(i).z.abs() > 0.9
                    && mesh.triangle(i).iter().all(|p| (p.z - z).abs() < 1e-9)
            })
            .expect("a cap triangle")
    }

    /// Distance from `p` to the line through `origin` along unit `dir`.
    fn to_axis(p: DVec3, origin: DVec3, dir: DVec3) -> f64 {
        let d = p - origin;
        (d - dir * d.dot(dir)).length()
    }

    #[test]
    fn cylinder_wall_fits_its_axis_and_centre_at_any_pose() {
        let base = TriMesh::cylinder(0.3, 2.0, 32);
        let seed = wall_triangle(&base, 0.3);
        for m in poses() {
            let mesh = posed(&base, &m);
            let fit = fit_circle(&mesh, seed).expect("a bore fits");
            let axis = m.transform_vector3(DVec3::Z).normalize();
            let centre = m.transform_point3(DVec3::ZERO);
            assert!(
                fit.axis.cross(axis).length() < 1e-6,
                "axis {} vs {axis}",
                fit.axis
            );
            assert!((fit.radius - 0.3).abs() < 1e-9, "radius {}", fit.radius);
            assert!(to_axis(fit.center, centre, axis) < 1e-6, "{}", fit.center);
            // Mean height of both rings is the mid-plane.
            assert!((fit.center - centre).length() < 1e-6, "{}", fit.center);
            assert!(fit.residual < 1e-9, "residual {}", fit.residual);
            assert_eq!(fit.segments, 32);
        }
    }

    #[test]
    fn tube_inner_wall_fits_the_bore() {
        let base = TriMesh::tube(0.5, 0.2, 1.0, 24);
        let seed = wall_triangle(&base, 0.2);
        assert!(
            mesh_normal_points_inward(&base, seed),
            "the seed is the inner wall"
        );
        for m in poses() {
            let mesh = posed(&base, &m);
            let fit = fit_circle(&mesh, seed).expect("a bore fits");
            let axis = m.transform_vector3(DVec3::Z).normalize();
            assert!(fit.axis.cross(axis).length() < 1e-6, "axis {}", fit.axis);
            assert!((fit.radius - 0.2).abs() < 1e-9, "radius {}", fit.radius);
            assert!((fit.center - m.transform_point3(DVec3::ZERO)).length() < 1e-6);
            assert_eq!(fit.segments, 24);
        }
    }

    fn mesh_normal_points_inward(mesh: &TriMesh, triangle: usize) -> bool {
        let c = mesh.triangle(triangle).iter().sum::<DVec3>() / 3.0;
        mesh.face_normal(triangle).dot(DVec3::new(c.x, c.y, 0.0)) < 0.0
    }

    #[test]
    fn cap_loop_gives_the_face_normal_and_a_centre_on_the_cap() {
        let base = TriMesh::cylinder(0.4, 1.5, 20);
        let seed = cap_triangle(&base, 0.75);
        for m in poses() {
            let mesh = posed(&base, &m);
            let fit = fit_circle(&mesh, seed).expect("a cap fits");
            let normal = mesh.face_normal(seed);
            assert!((fit.axis - normal).length() < 1e-9, "axis {}", fit.axis);
            assert!((fit.radius - 0.4).abs() < 1e-9);
            let centre = m.transform_point3(DVec3::new(0.0, 0.0, 0.75));
            assert!((fit.center - centre).length() < 1e-9, "{}", fit.center);
            assert_eq!(fit.segments, 20);
        }
    }

    #[test]
    fn a_cube_face_is_not_a_circle() {
        let cube = TriMesh::cube(0.5);
        for i in 0..cube.triangle_count() {
            assert_eq!(fit_circle(&cube, i), None, "triangle {i}");
        }
        // The region is still found — it is the two coplanar triangles of
        // the face — the fit is what refuses it.
        let adj = adjacency(&cube);
        assert_eq!(grow_region(&cube, &adj, 0, DEFAULT_MAX_DIHEDRAL).len(), 2);
    }

    #[test]
    fn jitter_shows_up_in_the_residual() {
        let mut mesh = TriMesh::cylinder(0.25, 1.0, 32);
        let seed = wall_triangle(&mesh, 0.25);
        let clean = fit_circle(&mesh, seed).unwrap();
        assert!(clean.residual < 1e-12);

        // A deterministic ±1 mm wobble, radial so it is a radius error and
        // not a shift of the whole ring.
        const AMPLITUDE: f64 = 1e-3;
        let mut state = 12345u64;
        let mut jitter = || {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((state >> 33) as f64 / (1u64 << 31) as f64 - 1.0) * AMPLITUDE
        };
        // Welded: one wobble per distinct position, or the topology breaks.
        let adj = adjacency(&mesh);
        let mut offsets = vec![0.0; adj.welded_count()];
        for offset in &mut offsets {
            *offset = jitter();
        }
        for (vertex, p) in mesh.positions.iter_mut().enumerate() {
            let radial = DVec3::new(p.x, p.y, 0.0).normalize_or_zero();
            *p += radial * offsets[adj.welded(vertex) as usize];
        }

        let fit = fit_circle(&mesh, seed).expect("a wobbly bore still fits");
        assert!(
            (fit.radius - 0.25).abs() < AMPLITUDE,
            "radius {}",
            fit.radius
        );
        // RMS of a uniform ±a wobble is a/√3; allow a wide band, the point
        // is that the readout is the wobble's size and not zero.
        assert!(
            (AMPLITUDE / 6.0..AMPLITUDE).contains(&fit.residual),
            "residual {} for a ±{AMPLITUDE} wobble",
            fit.residual
        );
        assert_eq!(fit.segments, 32);
    }

    #[test]
    fn segments_follow_the_generator() {
        for n in [6, 8, 12, 17, 64] {
            let mesh = TriMesh::cylinder(1.0, 1.0, n);
            let wall = fit_circle(&mesh, wall_triangle(&mesh, 1.0)).unwrap();
            assert_eq!(wall.segments, n, "wall of a {n}-segment cylinder");
            let cap = fit_circle(&mesh, cap_triangle(&mesh, 0.5)).unwrap();
            assert_eq!(cap.segments, n, "cap of a {n}-segment cylinder");
        }
        // Below MIN_SEGMENTS the fit refuses: a pentagonal prism's wall is a
        // polygon somebody meant as a polygon. (Its 72° steps are past
        // DEFAULT_MAX_DIHEDRAL too, so the region never leaves the facet.)
        let prism = TriMesh::cylinder(1.0, 1.0, 5);
        assert_eq!(fit_circle(&prism, wall_triangle(&prism, 1.0)), None);
        assert_eq!(fit_circle(&prism, cap_triangle(&prism, 0.5)), None);
    }

    #[test]
    fn adjacency_welds_exact_positions_and_pairs_edges() {
        let cube = TriMesh::cube(0.5);
        let adj = adjacency(&cube);
        assert_eq!(cube.positions.len(), 36, "unwelded on the way in");
        assert_eq!(adj.welded_count(), 8, "a cube has eight corners");
        assert!(adj.is_closed());
        for i in 0..cube.triangle_count() {
            assert!(adj.neighbors(i).iter().all(Option::is_some));
        }

        let cylinder = TriMesh::cylinder(1.0, 1.0, 10);
        let adj = adjacency(&cylinder);
        assert_eq!(adj.welded_count(), 22, "two rings and two centres");
        assert!(adj.is_closed());

        // An open mesh: one loose triangle, three boundary edges.
        let loose = TriMesh {
            positions: vec![DVec3::ZERO, DVec3::X, DVec3::Y],
            normals: vec![],
            indices: vec![0, 1, 2],
        };
        let adj = adjacency(&loose);
        assert!(!adj.is_closed());
        assert_eq!(adj.neighbors(0), [None, None, None]);
    }

    #[test]
    fn growth_stops_at_a_corner_and_crosses_a_smooth_seam() {
        let cube = TriMesh::cube(0.5);
        let adj = adjacency(&cube);
        // 90° everywhere: one face, whatever the threshold below 90°.
        assert_eq!(grow_region(&cube, &adj, 3, 1.5).len(), 2);
        // Above 90° the whole cube is one region.
        assert_eq!(grow_region(&cube, &adj, 3, 1.6).len(), 12);

        let cylinder = TriMesh::cylinder(1.0, 2.0, 16);
        let adj = adjacency(&cylinder);
        let wall = grow_region(
            &cylinder,
            &adj,
            wall_triangle(&cylinder, 1.0),
            DEFAULT_MAX_DIHEDRAL,
        );
        assert_eq!(wall.len(), 32, "the wall, and not the caps");
        let cap = grow_region(
            &cylinder,
            &adj,
            cap_triangle(&cylinder, 1.0),
            DEFAULT_MAX_DIHEDRAL,
        );
        assert_eq!(cap.len(), 16, "one cap fan");

        // Out of range, and a degenerate seed, grow nothing rather than panic.
        assert!(grow_region(&cylinder, &adj, 9999, 1.0).is_empty());
    }
}
