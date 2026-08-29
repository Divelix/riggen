//! The convex hull of a point cloud by quickhull, our own ~150 lines rather
//! than a dependency (plans/m3-sim-ready): one hull per visual geom is the
//! `CollisionPolicy::ConvexHull` the exporters write as `<stem>_hull.stl`
//! and the viewport draws translucent.
//!
//! Standard quickhull: an initial tetrahedron from extreme points, then
//! repeatedly take a face with points outside it, the farthest such point,
//! delete every face it can see, and stitch the horizon to it. Faces are
//! triangles throughout — coplanar faces are not merged, which a triangle
//! mesh does not need — and the output is welded (one vertex per hull
//! point, `normals` empty) so `feature::adjacency` sees a closed surface.

use glam::DVec3;

use crate::{MeshError, TriMesh};

/// Points closer to a face plane than this fraction of the cloud's extent
/// count as on it, not outside. Keeps a flat cube face from sprouting
/// slivers.
const EPS_SCALE: f64 = 1e-10;

#[derive(Debug, Clone)]
struct Face {
    verts: [usize; 3],
    normal: DVec3,
    /// `normal · x = offset` on the plane.
    offset: f64,
    /// Input points strictly outside this face, still to be conquered.
    outside: Vec<usize>,
    alive: bool,
}

impl Face {
    fn new(points: &[DVec3], verts: [usize; 3]) -> Self {
        let [a, b, c] = verts.map(|i| points[i]);
        let normal = (b - a).cross(c - a).normalize_or_zero();
        Self {
            verts,
            normal,
            offset: normal.dot(a),
            outside: Vec::new(),
            alive: true,
        }
    }

    fn distance(&self, p: DVec3) -> f64 {
        self.normal.dot(p) - self.offset
    }
}

/// The convex hull of `points`, outward-wound. Duplicate points are fine;
/// fewer than four distinct points, or a cloud that is collinear or
/// coplanar, is [`MeshError::DegenerateHull`].
pub fn convex_hull(points: &[DVec3]) -> Result<TriMesh, MeshError> {
    // Exact dedup: an unwelded mesh repeats every corner, and a repeated
    // point must not become two hull vertices.
    let mut seen = std::collections::HashSet::new();
    let mut points: Vec<DVec3> = points
        .iter()
        .copied()
        .filter(|p| {
            seen.insert([p.x, p.y, p.z].map(|c| if c == 0.0 { 0.0f64 } else { c }.to_bits()))
        })
        .collect();
    // A point on a face of the final hull — a cap centre, a vertex inside
    // a planar CAD face — can become an apex while the partial hull is
    // still small and stay as a vertex fanning a coplanar face. It lies
    // inside the face its corners span, so dropping every such vertex and
    // hulling the rest again gives the minimal hull that still encloses
    // everything. The count strictly falls, so this terminates.
    loop {
        let hull = quickhull(&points)?;
        let flat = flat_vertices(&hull);
        if flat.is_empty() {
            return Ok(hull);
        }
        let keep: std::collections::HashSet<[u64; 3]> = hull
            .positions
            .iter()
            .enumerate()
            .filter(|(i, _)| !flat.contains(i))
            .map(|(_, p)| [p.x.to_bits(), p.y.to_bits(), p.z.to_bits()])
            .collect();
        points.retain(|p| keep.contains(&[p.x.to_bits(), p.y.to_bits(), p.z.to_bits()]));
    }
}

/// Vertices of `hull` whose incident faces are all coplanar: not corners.
fn flat_vertices(hull: &TriMesh) -> std::collections::HashSet<usize> {
    let mut normals: Vec<Vec<DVec3>> = vec![Vec::new(); hull.positions.len()];
    for i in 0..hull.triangle_count() {
        let n = hull.face_normal(i);
        for k in 0..3 {
            normals[hull.indices[3 * i + k] as usize].push(n);
        }
    }
    normals
        .iter()
        .enumerate()
        .filter(|(_, ns)| ns.iter().all(|n| n.dot(ns[0]) > 1.0 - 1e-9))
        .map(|(i, _)| i)
        .collect()
}

fn quickhull(points: &[DVec3]) -> Result<TriMesh, MeshError> {
    let (min, max) = points.iter().fold(
        (DVec3::splat(f64::INFINITY), DVec3::splat(f64::NEG_INFINITY)),
        |(lo, hi), p| (lo.min(*p), hi.max(*p)),
    );
    let extent = (max - min).max_element();
    if points.len() < 4 || !extent.is_finite() || extent <= 0.0 {
        return Err(MeshError::DegenerateHull {
            reason: "fewer than four distinct points".into(),
        });
    }
    let eps = EPS_SCALE * extent;

    let mut faces = initial_tetrahedron(points, eps)?;
    // Every point goes to the first face it is outside of.
    for (i, p) in points.iter().enumerate() {
        if let Some(f) = faces.iter_mut().find(|f| f.distance(*p) > eps) {
            f.outside.push(i);
        }
    }

    while let Some(fi) = faces.iter().position(|f| f.alive && !f.outside.is_empty()) {
        let apex = *faces[fi]
            .outside
            .iter()
            .max_by(|&&a, &&b| {
                faces[fi]
                    .distance(points[a])
                    .total_cmp(&faces[fi].distance(points[b]))
            })
            .expect("non-empty");
        let p = points[apex];

        // Everything that sees the apex goes; its orphaned outside points
        // are redistributed to the new faces.
        let visible: Vec<usize> = (0..faces.len())
            .filter(|&i| faces[i].alive && faces[i].distance(p) > eps)
            .collect();
        let mut orphans: Vec<usize> = Vec::new();
        for &i in &visible {
            faces[i].alive = false;
            orphans.append(&mut faces[i].outside);
        }
        // Horizon: directed edges of visible faces whose reverse is not an
        // edge of another visible face.
        let mut edges: Vec<(usize, usize)> = Vec::new();
        for &i in &visible {
            let v = faces[i].verts;
            edges.extend([(v[0], v[1]), (v[1], v[2]), (v[2], v[0])]);
        }
        let horizon: Vec<(usize, usize)> = edges
            .iter()
            .copied()
            .filter(|&(u, v)| !edges.contains(&(v, u)))
            .collect();
        let first_new = faces.len();
        for (u, v) in horizon {
            faces.push(Face::new(points, [u, v, apex]));
        }
        for i in orphans {
            if i == apex {
                continue;
            }
            if let Some(f) = faces[first_new..]
                .iter_mut()
                .find(|f| f.distance(points[i]) > eps)
            {
                f.outside.push(i);
            }
        }
    }

    // Compact to the vertices the hull uses, in first-use order.
    let mut remap = vec![u32::MAX; points.len()];
    let mut positions = Vec::new();
    let mut indices = Vec::new();
    for face in faces.iter().filter(|f| f.alive) {
        for &v in &face.verts {
            if remap[v] == u32::MAX {
                remap[v] = positions.len() as u32;
                positions.push(points[v]);
            }
            indices.push(remap[v]);
        }
    }
    Ok(TriMesh {
        positions,
        normals: Vec::new(),
        indices,
    })
}

/// Four points spanning a volume: the two farthest apart along the widest
/// axis, the farthest from that line, the farthest from that plane. Wound
/// so every face looks away from the tetrahedron's centroid.
fn initial_tetrahedron(points: &[DVec3], eps: f64) -> Result<Vec<Face>, MeshError> {
    let (a, b) = extreme_pair(points);
    let dir = (points[b] - points[a]).normalize_or_zero();
    let off_line = |p: DVec3| {
        let d = p - points[a];
        (d - dir * d.dot(dir)).length()
    };
    let c = (0..points.len())
        .max_by(|&i, &j| off_line(points[i]).total_cmp(&off_line(points[j])))
        .expect("non-empty");
    if off_line(points[c]) <= eps {
        return Err(MeshError::DegenerateHull {
            reason: "all points are collinear".into(),
        });
    }
    let normal = (points[b] - points[a])
        .cross(points[c] - points[a])
        .normalize_or_zero();
    let off_plane = |p: DVec3| normal.dot(p - points[a]);
    let d = (0..points.len())
        .max_by(|&i, &j| {
            off_plane(points[i])
                .abs()
                .total_cmp(&off_plane(points[j]).abs())
        })
        .expect("non-empty");
    if off_plane(points[d]).abs() <= eps {
        return Err(MeshError::DegenerateHull {
            reason: "all points are coplanar".into(),
        });
    }
    // If d is on the normal's side, (a, b, c) faces it and must be flipped
    // to face outward.
    let (a, b, c) = if off_plane(points[d]) > 0.0 {
        (a, c, b)
    } else {
        (a, b, c)
    };
    let faces = [[a, b, c], [a, d, b], [b, d, c], [c, d, a]];
    let centroid = (points[a] + points[b] + points[c] + points[d]) / 4.0;
    Ok(faces
        .into_iter()
        .map(|verts| {
            let f = Face::new(points, verts);
            debug_assert!(f.distance(centroid) < 0.0, "tetrahedron winds inward");
            f
        })
        .collect())
}

/// The min and max points along the axis of greatest spread.
fn extreme_pair(points: &[DVec3]) -> (usize, usize) {
    let mut best = (0, 0, f64::NEG_INFINITY);
    for axis in 0..3 {
        let lo = (0..points.len())
            .min_by(|&i, &j| points[i][axis].total_cmp(&points[j][axis]))
            .expect("non-empty");
        let hi = (0..points.len())
            .max_by(|&i, &j| points[i][axis].total_cmp(&points[j][axis]))
            .expect("non-empty");
        let spread = points[hi][axis] - points[lo][axis];
        if spread > best.2 {
            best = (lo, hi, spread);
        }
    }
    (best.0, best.1)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{feature, mass_properties};
    use std::path::Path;

    /// Largest signed distance of any of `points` outside `hull`'s faces.
    fn max_outside(hull: &TriMesh, points: &[DVec3]) -> f64 {
        let mut worst = f64::NEG_INFINITY;
        for i in 0..hull.triangle_count() {
            let [a, b, c] = hull.triangle(i);
            let n = (b - a).cross(c - a).normalize();
            for p in points {
                worst = worst.max(n.dot(*p - a));
            }
        }
        worst
    }

    fn assert_sound(hull: &TriMesh, points: &[DVec3]) {
        hull.validate().unwrap();
        assert!(feature::adjacency(hull).is_closed(), "hull is not closed");
        let props = mass_properties(hull, 1.0);
        assert!(!props.inward_winding, "hull winds inward");
        assert!(props.volume > 0.0);
        assert!(
            max_outside(hull, points) < 1e-9,
            "a point is outside the hull by {}",
            max_outside(hull, points)
        );
    }

    #[test]
    fn cube_with_interior_points_is_eight_vertices_twelve_triangles() {
        let mut points: Vec<DVec3> = TriMesh::cube(0.5).positions;
        points.extend([
            DVec3::ZERO,
            DVec3::new(0.1, -0.2, 0.3),
            DVec3::new(-0.49, 0.49, 0.0),
            DVec3::new(0.5, 0.0, 0.0), // on a face
        ]);
        let hull = convex_hull(&points).unwrap();
        assert_eq!(hull.positions.len(), 8);
        assert_eq!(hull.triangle_count(), 12);
        assert_sound(&hull, &points);
        let props = mass_properties(&hull, 1.0);
        assert!((props.volume - 1.0).abs() < 1e-12, "{}", props.volume);
    }

    #[test]
    fn tetrahedron_and_octahedron() {
        let tet = [DVec3::ZERO, DVec3::X, DVec3::Y, DVec3::Z];
        let hull = convex_hull(&tet).unwrap();
        assert_eq!((hull.positions.len(), hull.triangle_count()), (4, 4));
        assert_sound(&hull, &tet);

        let oct = [
            DVec3::X,
            DVec3::NEG_X,
            DVec3::Y,
            DVec3::NEG_Y,
            DVec3::Z,
            DVec3::NEG_Z,
        ];
        let hull = convex_hull(&oct).unwrap();
        assert_eq!((hull.positions.len(), hull.triangle_count()), (6, 8));
        assert_sound(&hull, &oct);
        let props = mass_properties(&hull, 1.0);
        assert!((props.volume - 4.0 / 3.0).abs() < 1e-12, "{}", props.volume);
    }

    #[test]
    fn hull_of_a_cylinder_is_the_cylinder() {
        let mesh = TriMesh::cylinder(0.3, 1.0, 32);
        let hull = convex_hull(&mesh.positions).unwrap();
        assert_sound(&hull, &mesh.positions);
        // The prism is already convex: same volume, its 64 ring vertices
        // (the cap centres are interior to the caps).
        assert_eq!(hull.positions.len(), 64);
        let (a, b) = (mass_properties(&hull, 1.0), mass_properties(&mesh, 1.0));
        assert!((a.volume - b.volume).abs() < 1e-12);
    }

    #[test]
    fn arm_parts_hull_contains_them() {
        let dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/fixtures/arm");
        for name in ["base.stl", "shoulder.stl", "upper.stl", "fore.stl"] {
            let mesh = crate::load_stl(&dir.join(name)).unwrap();
            let hull = convex_hull(&mesh.positions).unwrap();
            assert_sound(&hull, &mesh.positions);
            let (h, m) = (mass_properties(&hull, 1.0), mass_properties(&mesh, 1.0));
            assert!(
                h.volume >= m.volume - 1e-9,
                "{name}: {} < {}",
                h.volume,
                m.volume
            );
            assert!(
                hull.positions.len() < mesh.positions.len(),
                "{name}: the hull is welded and smaller"
            );
        }
    }

    /// Deterministic LCG so the test needs no `rand` and never flakes.
    fn lcg(seed: &mut u64) -> f64 {
        *seed = seed
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*seed >> 11) as f64 / (1u64 << 53) as f64) * 2.0 - 1.0
    }

    #[test]
    fn random_clouds_are_enclosed() {
        let mut seed = 7;
        for n in [5, 20, 200, 2000] {
            let points: Vec<DVec3> = (0..n)
                .map(|_| DVec3::new(lcg(&mut seed), lcg(&mut seed) * 0.1, lcg(&mut seed) * 3.0))
                .collect();
            let hull = convex_hull(&points).unwrap();
            assert_sound(&hull, &points);
            // Euler: a closed triangulated sphere has 2V - 4 faces.
            assert_eq!(hull.triangle_count(), 2 * hull.positions.len() - 4);
        }
    }

    #[test]
    fn degenerate_input_is_an_error() {
        let cases: [(&str, Vec<DVec3>); 5] = [
            ("empty", vec![]),
            ("one", vec![DVec3::X]),
            ("duplicates", vec![DVec3::X; 10]),
            ("collinear", (0..10).map(|i| DVec3::X * i as f64).collect()),
            (
                "coplanar",
                (0..20)
                    .map(|i| DVec3::new((i % 5) as f64, (i / 5) as f64, 0.0))
                    .collect(),
            ),
        ];
        for (name, points) in cases {
            let err = convex_hull(&points).unwrap_err();
            assert!(
                matches!(err, MeshError::DegenerateHull { .. }),
                "{name}: {err}"
            );
        }
    }
}
