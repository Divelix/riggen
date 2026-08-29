use riggen_mesh::glam::Vec3;

/// Furthest the pitch may tilt before the up vector degenerates.
pub const MAX_PITCH: f32 = 1.5533; // ~89 degrees

/// True isometric pitch: `asin(1/sqrt(3))`, the classic engineering
/// isometric angle.
pub const ISO_PITCH: f32 = 0.615_479_7;

/// How the viewport is mapped onto the near plane.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Projection {
    Perspective,
    Orthographic,
}

impl Projection {
    /// Human-readable display label for the projection mode.
    pub fn label(self) -> &'static str {
        match self {
            Projection::Perspective => "Perspective",
            Projection::Orthographic => "Orthographic",
        }
    }

    /// Toggles between perspective and orthographic projections.
    pub fn toggled(self) -> Self {
        match self {
            Projection::Perspective => Projection::Orthographic,
            Projection::Orthographic => Projection::Perspective,
        }
    }
}

/// A keyboard-triggered standard view (numpad keys). Distance and target
/// are left untouched — only orientation snaps.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StandardView {
    Front,
    Back,
    Left,
    Right,
    Top,
    Bottom,
    Iso,
}

/// One of the 26 canonical view orientations of a ViewCube: 6 primary faces
/// (orthogonal 90°), 12 chamfered edges (two-axis 45°), and 8 chamfered
/// corners (three-axis isometric). Kept whole for the ViewCube port later;
/// M0 only uses it through [`StandardView`] and `closest_orientation`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ViewOrientation {
    // 6 Primary Faces
    Front,
    Back,
    Left,
    Right,
    Top,
    Bottom,

    // 12 Chamfered Edges
    FrontTop,
    FrontBottom,
    FrontLeft,
    FrontRight,
    BackTop,
    BackBottom,
    BackLeft,
    BackRight,
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,

    // 8 Chamfered Corners
    FrontTopLeft,
    FrontTopRight,
    FrontBottomLeft,
    FrontBottomRight,
    BackTopLeft,
    BackTopRight,
    BackBottomLeft,
    BackBottomRight,
}

impl ViewOrientation {
    pub const ALL: [Self; 26] = [
        Self::Front,
        Self::Back,
        Self::Left,
        Self::Right,
        Self::Top,
        Self::Bottom,
        Self::FrontTop,
        Self::FrontBottom,
        Self::FrontLeft,
        Self::FrontRight,
        Self::BackTop,
        Self::BackBottom,
        Self::BackLeft,
        Self::BackRight,
        Self::TopLeft,
        Self::TopRight,
        Self::BottomLeft,
        Self::BottomRight,
        Self::FrontTopLeft,
        Self::FrontTopRight,
        Self::FrontBottomLeft,
        Self::FrontBottomRight,
        Self::BackTopLeft,
        Self::BackTopRight,
        Self::BackBottomLeft,
        Self::BackBottomRight,
    ];

    pub const FACES: [Self; 6] = [
        Self::Front,
        Self::Back,
        Self::Left,
        Self::Right,
        Self::Top,
        Self::Bottom,
    ];

    pub const EDGES: [Self; 12] = [
        Self::FrontTop,
        Self::FrontBottom,
        Self::FrontLeft,
        Self::FrontRight,
        Self::BackTop,
        Self::BackBottom,
        Self::BackLeft,
        Self::BackRight,
        Self::TopLeft,
        Self::TopRight,
        Self::BottomLeft,
        Self::BottomRight,
    ];

    pub const CORNERS: [Self; 8] = [
        Self::FrontTopLeft,
        Self::FrontTopRight,
        Self::FrontBottomLeft,
        Self::FrontBottomRight,
        Self::BackTopLeft,
        Self::BackTopRight,
        Self::BackBottomLeft,
        Self::BackBottomRight,
    ];

    /// Outward unit normal vector pointing from the target towards the
    /// camera eye for this orientation (Z-up, +Y Front, +X Right).
    pub fn normal(self) -> Vec3 {
        match self {
            Self::Front => Vec3::new(0.0, 1.0, 0.0),
            Self::Back => Vec3::new(0.0, -1.0, 0.0),
            Self::Left => Vec3::new(-1.0, 0.0, 0.0),
            Self::Right => Vec3::new(1.0, 0.0, 0.0),
            Self::Top => Vec3::new(0.0, 0.0, 1.0),
            Self::Bottom => Vec3::new(0.0, 0.0, -1.0),

            Self::FrontTop => Vec3::new(0.0, 1.0, 1.0).normalize(),
            Self::FrontBottom => Vec3::new(0.0, 1.0, -1.0).normalize(),
            Self::FrontLeft => Vec3::new(-1.0, 1.0, 0.0).normalize(),
            Self::FrontRight => Vec3::new(1.0, 1.0, 0.0).normalize(),
            Self::BackTop => Vec3::new(0.0, -1.0, 1.0).normalize(),
            Self::BackBottom => Vec3::new(0.0, -1.0, -1.0).normalize(),
            Self::BackLeft => Vec3::new(-1.0, -1.0, 0.0).normalize(),
            Self::BackRight => Vec3::new(1.0, -1.0, 0.0).normalize(),
            Self::TopLeft => Vec3::new(-1.0, 0.0, 1.0).normalize(),
            Self::TopRight => Vec3::new(1.0, 0.0, 1.0).normalize(),
            Self::BottomLeft => Vec3::new(-1.0, 0.0, -1.0).normalize(),
            Self::BottomRight => Vec3::new(1.0, 0.0, -1.0).normalize(),

            Self::FrontTopLeft => Vec3::new(-1.0, 1.0, 1.0).normalize(),
            Self::FrontTopRight => Vec3::new(1.0, 1.0, 1.0).normalize(),
            Self::FrontBottomLeft => Vec3::new(-1.0, 1.0, -1.0).normalize(),
            Self::FrontBottomRight => Vec3::new(1.0, 1.0, -1.0).normalize(),
            Self::BackTopLeft => Vec3::new(-1.0, -1.0, 1.0).normalize(),
            Self::BackTopRight => Vec3::new(1.0, -1.0, 1.0).normalize(),
            Self::BackBottomLeft => Vec3::new(-1.0, -1.0, -1.0).normalize(),
            Self::BackBottomRight => Vec3::new(1.0, -1.0, -1.0).normalize(),
        }
    }

    /// Target `(yaw, pitch)` angles in radians for this orientation.
    pub fn yaw_pitch(self) -> (f32, f32) {
        use std::f32::consts::{FRAC_PI_2, FRAC_PI_4, PI};
        const FRAC_3_PI_4: f32 = 3.0 * std::f32::consts::FRAC_PI_4;
        match self {
            Self::Front => (FRAC_PI_2, 0.0),
            Self::Back => (-FRAC_PI_2, 0.0),
            Self::Left => (PI, 0.0),
            Self::Right => (0.0, 0.0),
            Self::Top => (FRAC_PI_2, FRAC_PI_2),
            Self::Bottom => (FRAC_PI_2, -FRAC_PI_2),

            Self::FrontTop => (FRAC_PI_2, FRAC_PI_4),
            Self::FrontBottom => (FRAC_PI_2, -FRAC_PI_4),
            Self::FrontLeft => (FRAC_3_PI_4, 0.0),
            Self::FrontRight => (FRAC_PI_4, 0.0),
            Self::BackTop => (-FRAC_PI_2, FRAC_PI_4),
            Self::BackBottom => (-FRAC_PI_2, -FRAC_PI_4),
            Self::BackLeft => (-FRAC_3_PI_4, 0.0),
            Self::BackRight => (-FRAC_PI_4, 0.0),
            Self::TopLeft => (PI, FRAC_PI_4),
            Self::TopRight => (0.0, FRAC_PI_4),
            Self::BottomLeft => (PI, -FRAC_PI_4),
            Self::BottomRight => (0.0, -FRAC_PI_4),

            Self::FrontTopLeft => (FRAC_3_PI_4, ISO_PITCH),
            Self::FrontTopRight => (FRAC_PI_4, ISO_PITCH),
            Self::FrontBottomLeft => (FRAC_3_PI_4, -ISO_PITCH),
            Self::FrontBottomRight => (FRAC_PI_4, -ISO_PITCH),
            Self::BackTopLeft => (-FRAC_3_PI_4, ISO_PITCH),
            Self::BackTopRight => (-FRAC_PI_4, ISO_PITCH),
            Self::BackBottomLeft => (-FRAC_3_PI_4, -ISO_PITCH),
            Self::BackBottomRight => (-FRAC_PI_4, -ISO_PITCH),
        }
    }

    /// Resolves the closest [`ViewOrientation`] from an arbitrary direction
    /// vector (e.g. from camera target to camera eye).
    pub fn from_direction(dir: Vec3) -> Self {
        let mag = dir.length();
        if mag < 1e-6 {
            return Self::Front;
        }
        let unit_dir = dir / mag;
        let mut best = Self::Front;
        let mut max_dot = f32::NEG_INFINITY;
        for &orientation in &Self::ALL {
            let dot = orientation.normal().dot(unit_dir);
            if dot > max_dot {
                max_dot = dot;
                best = orientation;
            }
        }
        best
    }

    /// Whether this is one of the 6 primary orthogonal faces.
    pub fn is_face(self) -> bool {
        matches!(
            self,
            Self::Front | Self::Back | Self::Left | Self::Right | Self::Top | Self::Bottom
        )
    }

    /// Whether this is one of the 12 chamfered edges.
    pub fn is_edge(self) -> bool {
        matches!(
            self,
            Self::FrontTop
                | Self::FrontBottom
                | Self::FrontLeft
                | Self::FrontRight
                | Self::BackTop
                | Self::BackBottom
                | Self::BackLeft
                | Self::BackRight
                | Self::TopLeft
                | Self::TopRight
                | Self::BottomLeft
                | Self::BottomRight
        )
    }

    /// Whether this is one of the 8 chamfered corners.
    pub fn is_corner(self) -> bool {
        matches!(
            self,
            Self::FrontTopLeft
                | Self::FrontTopRight
                | Self::FrontBottomLeft
                | Self::FrontBottomRight
                | Self::BackTopLeft
                | Self::BackTopRight
                | Self::BackBottomLeft
                | Self::BackBottomRight
        )
    }

    /// Label text rendered on primary faces, if any.
    pub fn label(self) -> Option<&'static str> {
        match self {
            Self::Front => Some("FRONT"),
            Self::Back => Some("BACK"),
            Self::Left => Some("LEFT"),
            Self::Right => Some("RIGHT"),
            Self::Top => Some("TOP"),
            Self::Bottom => Some("BOTTOM"),
            _ => None,
        }
    }

    /// Human-readable name for this orientation.
    pub fn name(self) -> &'static str {
        match self {
            Self::Front => "Front",
            Self::Back => "Back",
            Self::Left => "Left",
            Self::Right => "Right",
            Self::Top => "Top",
            Self::Bottom => "Bottom",
            Self::FrontTop => "FrontTop",
            Self::FrontBottom => "FrontBottom",
            Self::FrontLeft => "FrontLeft",
            Self::FrontRight => "FrontRight",
            Self::BackTop => "BackTop",
            Self::BackBottom => "BackBottom",
            Self::BackLeft => "BackLeft",
            Self::BackRight => "BackRight",
            Self::TopLeft => "TopLeft",
            Self::TopRight => "TopRight",
            Self::BottomLeft => "BottomLeft",
            Self::BottomRight => "BottomRight",
            Self::FrontTopLeft => "FrontTopLeft",
            Self::FrontTopRight => "FrontTopRight",
            Self::FrontBottomLeft => "FrontBottomLeft",
            Self::FrontBottomRight => "FrontBottomRight",
            Self::BackTopLeft => "BackTopLeft",
            Self::BackTopRight => "BackTopRight",
            Self::BackBottomLeft => "BackBottomLeft",
            Self::BackBottomRight => "BackBottomRight",
        }
    }

    /// Converts to [`StandardView`] if this orientation is one of the
    /// standard views.
    pub fn as_standard_view(self) -> Option<StandardView> {
        match self {
            Self::Front => Some(StandardView::Front),
            Self::Back => Some(StandardView::Back),
            Self::Left => Some(StandardView::Left),
            Self::Right => Some(StandardView::Right),
            Self::Top => Some(StandardView::Top),
            Self::Bottom => Some(StandardView::Bottom),
            Self::BackTopRight => Some(StandardView::Iso),
            _ => None,
        }
    }
}

impl From<StandardView> for ViewOrientation {
    fn from(view: StandardView) -> Self {
        match view {
            StandardView::Front => Self::Front,
            StandardView::Back => Self::Back,
            StandardView::Left => Self::Left,
            StandardView::Right => Self::Right,
            StandardView::Top => Self::Top,
            StandardView::Bottom => Self::Bottom,
            StandardView::Iso => Self::BackTopRight,
        }
    }
}
