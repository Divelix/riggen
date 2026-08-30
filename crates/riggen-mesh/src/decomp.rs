//! Approximate convex decomposition: a concave part becomes several convex
//! pieces that together keep its concavity, which is what a physics engine
//! needs — MuJoCo and every URDF consumer treat a collision mesh as its
//! convex hull, so a gripper finger, a C-bracket or a U-channel collides as
//! a solid block unless it is split first.
//!
//! The algorithm is V-HACD (voxelize, then split along the plane that most
//! reduces concavity, recursively), from `parry3d-f64`'s
//! `transformation::vhacd` — pure Rust, f64, and the same algorithm the C++
//! original implements (ADR-0011). This module is the boundary: parry's
//! types (and the glam 0.33 its `glamx` bridge pins) go in and out through
//! plain `[f64; 3]`, and no parry type appears in a signature here or
//! anywhere above.
//!
//! [`decompose`] sits beside [`crate::convex_hull`]: the hull is the
//! one-piece answer, this is the N-piece one. Neither is cheap enough for a
//! frame — the app runs this on a job thread
//! (docs/01-architecture.md §Jobs and threads).

use std::fmt;

use glam::DVec3;

use crate::TriMesh;

/// What [`decompose`] is allowed to spend and how closely it must fit.
/// Mirrored field-for-field by `riggen_core::CollisionPolicy::
/// ConvexDecomposition`, which is the document's copy of it (the document
/// stores the parameters, never the pieces — ADR-0008, ADR-0011).
///
/// A cache key wherever a decomposition is remembered (`riggen-export`'s
/// per-resolve map, the app's job cache), which is why `Eq` and `Hash` are
/// written by hand over `concavity`'s bits rather than derived: two
/// parameter sets are the same job when they are the same numbers, NaN
/// included.
#[derive(Debug, Clone, Copy)]
pub struct DecompParams {
    /// Ceiling on the number of pieces. V-HACD stops splitting at it.
    pub max_hulls: u32,
    /// Side of the voxel grid the mesh is rasterised into. Cost and memory
    /// are O(resolution³); detail thinner than one voxel is invisible to
    /// the algorithm.
    pub resolution: u32,
    /// How much of the part's volume a piece may fail to fill before
    /// V-HACD splits it again, as a fraction of the whole. Smaller means
    /// more pieces and a tighter fit.
    pub concavity: f64,
}

impl PartialEq for DecompParams {
    fn eq(&self, other: &Self) -> bool {
        (self.max_hulls, self.resolution, self.concavity.to_bits())
            == (other.max_hulls, other.resolution, other.concavity.to_bits())
    }
}

impl Eq for DecompParams {}

impl std::hash::Hash for DecompParams {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        (self.max_hulls, self.resolution, self.concavity.to_bits()).hash(state);
    }
}

impl Default for DecompParams {
    /// The defaults the properties panel starts from: a second, not a
    /// minute (docs/plans, OPEN 2). Well below V-HACD's own
    /// `max_convex_hulls: 1024`, which no robot link wants.
    fn default() -> Self {
        Self {
            max_hulls: 8,
            resolution: 64,
            concavity: 0.01,
        }
    }
}

/// Why a mesh has no decomposition.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecompError {
    /// No triangles to voxelize.
    EmptyMesh,
    /// V-HACD produced no piece that spans a volume: a surface with no
    /// inside, or a part thinner than one voxel at this `resolution`.
    NoParts { resolution: u32 },
}

impl fmt::Display for DecompError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyMesh => write!(f, "no convex decomposition: the mesh has no triangles"),
            Self::NoParts { resolution } => write!(
                f,
                "no convex decomposition: nothing solid at resolution {resolution} \
                 (an open surface, or a part thinner than one voxel)"
            ),
        }
    }
}

impl std::error::Error for DecompError {}

/// `mesh` as convex pieces whose union approximates it, keeping the
/// concavities a single [`crate::convex_hull`] would fill.
///
/// Every piece is convex, welded, closed and outward-wound — the same shape
/// of output as `convex_hull`, so the exporters write them with the same
/// code. The pieces do not partition the mesh exactly: they are the convex
/// hulls of a voxel partition, so they overlap slightly and bulge past the
/// surface by about one voxel. That is what a collision proxy is.
///
/// Cost is roughly O(`resolution`³) plus a hull per piece; seconds, not
/// milliseconds, on a real part.
pub fn decompose(mesh: &TriMesh, params: &DecompParams) -> Result<Vec<TriMesh>, DecompError> {
    if mesh.triangle_count() == 0 {
        return Err(DecompError::EmptyMesh);
    }

    let points: Vec<parry3d_f64::math::Vector> = mesh
        .positions
        .iter()
        .map(|p| parry3d_f64::math::Vector::new(p.x, p.y, p.z))
        .collect();
    // `validate()` guarantees a multiple of three, so the remainder is empty.
    let indices: Vec<[u32; 3]> = mesh.indices.as_chunks::<3>().0.to_vec();

    // `keep_voxel_to_primitives_map` feeds `compute_exact_convex_hulls`,
    // which hulls each part's share of the *original* triangles rather than
    // its voxels: the pieces then touch the real surface instead of a
    // staircase one voxel outside it.
    let vhacd = parry3d_f64::transformation::vhacd::VHACD::decompose(
        &vhacd_params(params),
        &points,
        &indices,
        true,
    );

    let pieces: Vec<Piece> = vhacd
        .compute_exact_convex_hulls(&points, &indices)
        .into_iter()
        // parry hulls a point set that can be flat or empty for a sliver
        // part; our own quickhull is the judge of what has volume, and it
        // returns the welded, closed, outward-wound mesh the exporters and
        // `feature::adjacency` expect.
        .filter_map(|(points, _)| {
            let points: Vec<DVec3> = points.iter().map(|p| DVec3::new(p.x, p.y, p.z)).collect();
            Piece::hull(&points)
        })
        .collect();

    if pieces.is_empty() {
        return Err(DecompError::NoParts {
            resolution: params.resolution,
        });
    }
    Ok(merge(pieces, params).into_iter().map(|p| p.mesh).collect())
}

/// One convex piece, kept as its mesh and the volume [`merge`] prices it by.
struct Piece {
    mesh: TriMesh,
    volume: f64,
}

impl Piece {
    /// The convex hull of `points` as a piece, or `None` if it spans no
    /// volume.
    fn hull(points: &[DVec3]) -> Option<Self> {
        let mesh = crate::convex_hull(points).ok()?;
        let volume = crate::mass_properties(&mesh, 1.0).volume;
        (volume > 0.0).then_some(Self { mesh, volume })
    }
}

/// V-HACD's merge step, which `parry3d-f64` describes in `do_compute_acd`
/// and does not implement: it splits a binary tree
/// `2·2^ceil(log2(max_hulls))` leaves deep and returns every leaf, so its
/// `max_convex_hulls` is a recursion depth and not the ceiling its name
/// promises — a convex cube comes back as nine pieces, and eight requested
/// hulls can be sixteen. Both are wrong for an exported collision model, so
/// the pieces are merged back here (ADR-0011).
///
/// Repeatedly joins the pair whose common hull adds the least volume,
/// relative to the part's own. A pair is joined while there are more pieces
/// than `max_hulls` whatever it costs, and after that only while it costs
/// less than `concavity` — the same threshold that decided the splits, so a
/// split that bought nothing is undone and a split across a real concavity
/// is kept.
fn merge(mut pieces: Vec<Piece>, params: &DecompParams) -> Vec<Piece> {
    let max_hulls = params.max_hulls.max(1) as usize;
    let total: f64 = pieces.iter().map(|p| p.volume).sum();
    if total <= 0.0 {
        return pieces;
    }
    // The hull vertices of each piece: the only points a joined hull needs.
    let mut points: Vec<Vec<DVec3>> = pieces.iter().map(|p| p.mesh.positions.clone()).collect();
    // `cost[i][j]` for `j < i`, recomputed only for the row that changes.
    let mut cost: Vec<Vec<f64>> = Vec::new();
    let joined = |a: &[DVec3], b: &[DVec3]| -> Option<Piece> {
        let mut both = a.to_vec();
        both.extend_from_slice(b);
        Piece::hull(&both)
    };
    // Infinity, never NaN: `best >= concavity` below must mean "do not
    // join" for a pair that cannot be joined at all.
    let price = |i: usize, j: usize, pieces: &[Piece], points: &[Vec<DVec3>]| -> f64 {
        match joined(&points[i], &points[j]) {
            Some(p) => match (p.volume - pieces[i].volume - pieces[j].volume) / total {
                c if c.is_finite() => c,
                _ => f64::INFINITY,
            },
            None => f64::INFINITY,
        }
    };
    for i in 0..pieces.len() {
        cost.push((0..i).map(|j| price(i, j, &pieces, &points)).collect());
    }

    while pieces.len() > 1 {
        let Some((i, j, best)) = (1..pieces.len())
            .flat_map(|i| (0..i).map(move |j| (i, j)))
            .map(|(i, j)| (i, j, cost[i][j]))
            .min_by(|a, b| a.2.total_cmp(&b.2))
        else {
            break;
        };
        if pieces.len() <= max_hulls && best >= params.concavity {
            break;
        }
        let Some(union) = joined(&points[i], &points[j]) else {
            break; // Cannot be joined at all; nothing else can be either.
        };
        points[j] = union.mesh.positions.clone();
        pieces[j] = union;
        points.remove(i);
        pieces.remove(i);
        cost.remove(i);
        for row in &mut cost[i..] {
            row.remove(i);
        }
        // Only row `j` and the column under it changed.
        cost[j] = (0..j).map(|k| price(j, k, &pieces, &points)).collect();
        let column: Vec<f64> = ((j + 1)..pieces.len())
            .map(|k| price(k, j, &pieces, &points))
            .collect();
        for (row, c) in cost[(j + 1)..].iter_mut().zip(column) {
            row[j] = c;
        }
    }
    pieces
}

fn vhacd_params(params: &DecompParams) -> parry3d_f64::transformation::vhacd::VHACDParameters {
    parry3d_f64::transformation::vhacd::VHACDParameters {
        // A zero here voxelizes into nothing and panics deep inside parry;
        // the panel clamps too, but the library boundary is where it counts.
        resolution: params.resolution.max(1),
        max_convex_hulls: params.max_hulls.max(1),
        concavity: params.concavity.clamp(0.0, 1.0),
        ..Default::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{convex_hull, feature, mass_properties};
    use std::path::Path;

    fn fixture(name: &str) -> std::path::PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/fixtures")
            .join(name)
    }

    /// An axis-aligned box as an unwelded [`TriMesh`], `center` ± `size`/2.
    fn boxed(center: DVec3, size: DVec3) -> TriMesh {
        let mut mesh = TriMesh::cube(0.5);
        mesh.transform(&glam::DMat4::from_scale_rotation_translation(
            size,
            glam::DQuat::IDENTITY,
            center,
        ));
        mesh
    }

    fn append(into: &mut TriMesh, other: TriMesh) {
        let base = into.positions.len() as u32;
        into.positions.extend(other.positions);
        into.normals.extend(other.normals);
        into.indices.extend(other.indices.iter().map(|i| i + base));
    }

    /// The U-channel of `assets/fixtures/bracket.stl`, in millimetres like
    /// the arm's parts: a 60 × 30 × 10 base slab with two 10 × 26 × 40
    /// prongs standing on its ends. The notch between the prongs —
    /// |x| < 20, 10 < z < 50 — is the concavity the whole plan exists for:
    /// inside the convex hull, outside the part.
    ///
    /// The prongs are narrower than the base in y on purpose: at 30 they
    /// would share four corner vertices with it, welding would pair edges
    /// across the two shells, and `feature::adjacency` would call the union
    /// open — which `inertial::computed_inertial` refuses. Three shells
    /// that only touch are each closed.
    fn bracket() -> TriMesh {
        let mut mesh = boxed(DVec3::new(0.0, 0.0, 5.0), DVec3::new(60.0, 30.0, 10.0));
        for x in [-25.0, 25.0] {
            append(
                &mut mesh,
                boxed(DVec3::new(x, 0.0, 30.0), DVec3::new(10.0, 26.0, 40.0)),
            );
        }
        mesh
    }

    /// A point deep in the notch, in the same millimetres.
    const NOTCH: DVec3 = DVec3::new(0.0, 0.0, 30.0);

    /// Regenerates `assets/fixtures/bracket.stl`. Ignored like the arm's
    /// and the cube's generators: the fixture is committed and the tests
    /// below read it. `cargo test -p riggen-mesh write_bracket_fixture --
    /// --ignored`.
    #[test]
    #[ignore = "writes the committed fixture; run on purpose"]
    fn write_bracket_fixture() {
        std::fs::write(fixture("bracket.stl"), crate::write_binary(&bracket())).unwrap();
    }

    #[test]
    fn fixture_matches_its_generator() {
        assert_eq!(
            std::fs::read(fixture("bracket.stl")).unwrap(),
            crate::write_binary(&bracket())
        );
    }

    /// The fixture is a closed solid, so a link built on it gets a computed
    /// inertial rather than `InertialError::OpenMesh` — what the export and
    /// MuJoCo tests need of it.
    #[test]
    fn the_bracket_is_a_closed_solid() {
        let mesh = crate::load_stl(&fixture("bracket.stl")).unwrap();
        assert!(feature::adjacency(&mesh).is_closed());
        let props = mass_properties(&mesh, 1.0);
        assert!(!props.inward_winding);
        // The three boxes only touch, so nothing is counted twice.
        assert!(
            (props.volume - (60.0 * 30.0 * 10.0 + 2.0 * 10.0 * 26.0 * 40.0)).abs() < 1e-6,
            "{}",
            props.volume
        );
    }

    /// Whether `p` is inside `piece`, which must be convex and
    /// outward-wound: inside every face plane, within `slack`.
    fn inside(piece: &TriMesh, p: DVec3, slack: f64) -> bool {
        (0..piece.triangle_count()).all(|i| {
            let [a, _, _] = piece.triangle(i);
            piece.face_normal(i).dot(p - a) <= slack
        })
    }

    /// Largest distance any vertex of `piece` sits outside its own faces:
    /// zero for a convex body, positive for a dented one.
    fn non_convexity(piece: &TriMesh) -> f64 {
        let mut worst: f64 = 0.0;
        for i in 0..piece.triangle_count() {
            let [a, _, _] = piece.triangle(i);
            let n = piece.face_normal(i);
            for p in &piece.positions {
                worst = worst.max(n.dot(*p - a));
            }
        }
        worst
    }

    #[test]
    fn the_bracket_keeps_its_notch() {
        let mesh = crate::load_stl(&fixture("bracket.stl")).unwrap();
        let pieces = decompose(&mesh, &DecompParams::default()).unwrap();

        assert!(pieces.len() > 1, "one piece is a convex hull, not a split");
        for (i, piece) in pieces.iter().enumerate() {
            piece.validate().unwrap();
            assert!(feature::adjacency(piece).is_closed(), "piece {i} is open");
            let props = mass_properties(piece, 1.0);
            assert!(!props.inward_winding, "piece {i} winds inward");
            assert!(props.volume > 0.0, "piece {i} is flat");
            assert!(
                non_convexity(piece) < 1e-9,
                "piece {i} is not convex: {}",
                non_convexity(piece)
            );
        }

        // The property the plan exists for. The pieces bulge by about a
        // voxel, so the notch point is tested with that much slack against
        // it (60 mm / 64 ≈ 1 mm) and none at all against the hull.
        let voxel = 60.0 / DecompParams::default().resolution as f64;
        for (i, piece) in pieces.iter().enumerate() {
            assert!(
                !inside(piece, NOTCH, -2.0 * voxel),
                "piece {i} fills the notch"
            );
        }
        let hull = convex_hull(&mesh.positions).unwrap();
        assert!(inside(&hull, NOTCH, 0.0), "the hull fills the notch");

        // Together the pieces still cover the part: every vertex of the
        // bracket is in one of them, within a voxel.
        for p in &mesh.positions {
            assert!(
                pieces.iter().any(|piece| inside(piece, *p, 2.0 * voxel)),
                "{p} is in no piece"
            );
        }
    }

    #[test]
    fn max_hulls_caps_the_pieces() {
        let mesh = crate::load_stl(&fixture("bracket.stl")).unwrap();
        for max_hulls in [1, 2, 3] {
            let pieces = decompose(
                &mesh,
                &DecompParams {
                    max_hulls,
                    ..Default::default()
                },
            )
            .unwrap();
            assert!(
                pieces.len() <= max_hulls as usize,
                "{max_hulls} allowed, {} returned",
                pieces.len()
            );
        }
    }

    #[test]
    fn a_convex_part_comes_back_in_one_piece() {
        let cube = TriMesh::cube(0.5);
        let pieces = decompose(&cube, &DecompParams::default()).unwrap();
        assert_eq!(pieces.len(), 1);
        let props = mass_properties(&pieces[0], 1.0);
        // A voxel grid rounds the cube outward by less than a voxel a side.
        assert!((props.volume - 1.0).abs() < 0.1, "{}", props.volume);
    }

    #[test]
    fn degenerate_input_is_an_error() {
        assert_eq!(
            decompose(&TriMesh::default(), &DecompParams::default()),
            Err(DecompError::EmptyMesh)
        );
        // A single triangle encloses nothing: the flood fill finds no
        // interior, so there is no part to hull.
        let plate = TriMesh {
            positions: vec![DVec3::ZERO, DVec3::X, DVec3::Y],
            normals: Vec::new(),
            indices: vec![0, 1, 2],
        };
        assert_eq!(
            decompose(&plate, &DecompParams::default()),
            Err(DecompError::NoParts { resolution: 64 })
        );
    }
}
