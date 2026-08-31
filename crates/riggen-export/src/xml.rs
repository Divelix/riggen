//! The two halves of XML, in one file.
//!
//! **Writing** is forty lines: the output of all three writers is
//! fixed-shape, so an XML crate would only add a dependency and a way to
//! emit something MuJoCo cannot read. Escaping is the whole job — of an
//! attribute value, and, since SDF puts its numbers in element bodies, of
//! a body too ([`Xml::text`], [`pose6`] — ADR-0016 §3).
//!
//! **Reading** is not, because the file was written by somebody else:
//! [`parse`] is a read-only DOM over `quick-xml` (ADR-0015 §2), and beside
//! it live MJCF's five spellings of one rotation — `quat`, `euler`,
//! `axisangle`, `xyaxes`, `zaxis` — collapsing to one `DQuat`. That is the
//! mirror of [`quat_wxyz`], and obeys the same rule: one place, tested.

use std::collections::BTreeMap;
use std::fmt;
use std::fmt::Write as _;

use riggen_core::Pose;
use riggen_core::glam::{DMat3, DQuat, DVec3};

#[derive(Default)]
pub struct Xml {
    out: String,
    depth: usize,
}

impl Xml {
    pub fn new() -> Self {
        Self {
            out: String::from("<?xml version=\"1.0\" encoding=\"utf-8\"?>\n"),
            depth: 0,
        }
    }

    pub fn finish(self) -> String {
        self.out
    }

    /// `<tag a="b">` and one level of indentation.
    pub fn open(&mut self, tag: &str, attrs: &[(&str, String)]) {
        self.line(tag, attrs, ">");
        self.depth += 1;
    }

    /// `<tag a="b"/>`.
    pub fn empty(&mut self, tag: &str, attrs: &[(&str, String)]) {
        self.line(tag, attrs, "/>");
    }

    /// `<tag a="b">text</tag>` on one line. SDF puts its numbers in
    /// element bodies where MJCF and URDF use attributes — `<mass>2.7</mass>`,
    /// `<pose>x y z r p y</pose>` — so the body is escaped exactly as an
    /// attribute value is (ADR-0016 §3).
    pub fn text(&mut self, tag: &str, attrs: &[(&str, String)], text: &str) {
        self.indent();
        let _ = write!(self.out, "<{tag}");
        for (k, v) in attrs {
            let _ = write!(self.out, " {k}=\"{}\"", escape(v));
        }
        let _ = writeln!(self.out, ">{}</{tag}>", escape(text));
    }

    pub fn close(&mut self, tag: &str) {
        self.depth -= 1;
        self.indent();
        let _ = writeln!(self.out, "</{tag}>");
    }

    /// `<!-- text -->`; a `--` inside the text is not XML, so it is spaced.
    pub fn comment(&mut self, text: &str) {
        self.indent();
        let _ = writeln!(self.out, "<!-- {} -->", text.replace("--", "- -"));
    }

    fn line(&mut self, tag: &str, attrs: &[(&str, String)], end: &str) {
        self.indent();
        let _ = write!(self.out, "<{tag}");
        for (k, v) in attrs {
            let _ = write!(self.out, " {k}=\"{}\"", escape(v));
        }
        let _ = writeln!(self.out, "{end}");
    }

    fn indent(&mut self) {
        for _ in 0..self.depth {
            self.out.push_str("  ");
        }
    }
}

/// The five XML entities; everything else passes through.
pub fn escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '&' => out.push_str("&amp;"),
            '<' => out.push_str("&lt;"),
            '>' => out.push_str("&gt;"),
            '"' => out.push_str("&quot;"),
            '\'' => out.push_str("&apos;"),
            _ => out.push(c),
        }
    }
    out
}

/// A number as both formats want it: twelve decimals, trailing zeros and a
/// bare point trimmed, `-0` folded to `0`. Twelve is far inside what the
/// round-trip tests compare (1e-9) and keeps the files readable.
pub fn num(v: f64) -> String {
    let s = format!("{v:.12}");
    let s = s.trim_end_matches('0').trim_end_matches('.');
    match s {
        "" | "-0" | "-" => "0".to_owned(),
        _ => s.to_owned(),
    }
}

pub fn vec3(v: DVec3) -> String {
    format!("{} {} {}", num(v.x), num(v.y), num(v.z))
}

/// MJCF quaternion order, `w x y z`; `glam::DQuat` is `x y z w`. The one
/// place the conversion happens (ADR-0004 §3).
pub fn quat_wxyz(q: DQuat) -> [f64; 4] {
    let q = q.normalize();
    [q.w, q.x, q.y, q.z]
}

pub fn quat(q: DQuat) -> String {
    let [w, x, y, z] = quat_wxyz(q);
    format!("{} {} {} {}", num(w), num(x), num(y), num(z))
}

/// SDF's `<pose>`: `x y z roll pitch yaw`, six numbers in one element
/// body. The angles come out of [`Pose::to_xyz_rpy`] — the same helper
/// `urdf::origin_attrs` uses, because SDF's RPY *is* URDF's, `Rz·Ry·Rx`
/// about fixed axes (ADR-0016 §2).
pub fn pose6(pose: &Pose) -> String {
    let (xyz, rpy) = pose.to_xyz_rpy();
    format!("{} {}", vec3(xyz), vec3(rpy))
}

// ---------------------------------------------------------------------------
// The reading half (ADR-0015 §2).
// ---------------------------------------------------------------------------

/// A parsed element: its tag, its attributes and its children in document
/// order. Text nodes, comments and processing instructions are dropped —
/// neither format we read puts meaning in them.
///
/// The fields are public because resolving MJCF's `<default>` classes is
/// merging attribute maps (ADR-0015 §3), and a merged element is a `Node`
/// like any other.
#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct Node {
    pub tag: String,
    pub attrs: BTreeMap<String, String>,
    pub children: Vec<Node>,
}

/// The five ways MJCF spells one rotation, in the order [`Node::orientation`]
/// looks for them. An element carrying two of them is an error, as it is in
/// MuJoCo.
pub const ORIENTATION_ATTRS: [&str; 5] = ["quat", "euler", "axisangle", "xyaxes", "zaxis"];

impl Node {
    pub fn attr(&self, name: &str) -> Option<&str> {
        self.attrs.get(name).map(String::as_str)
    }

    /// The first child with this tag.
    pub fn child(&self, tag: &str) -> Option<&Node> {
        self.children.iter().find(|c| c.tag == tag)
    }

    /// Every child with this tag, in document order.
    pub fn kids<'a>(&'a self, tag: &'a str) -> impl Iterator<Item = &'a Node> + 'a {
        self.children.iter().filter(move |c| c.tag == tag)
    }

    /// One number; `None` when the attribute is absent. Infinities and NaN
    /// are refused here rather than reaching the document.
    pub fn num(&self, name: &str) -> Result<Option<f64>, String> {
        Ok(self.nums::<1>(name)?.map(|[v]| v))
    }

    /// Exactly `N` whitespace-separated numbers.
    pub fn nums<const N: usize>(&self, name: &str) -> Result<Option<[f64; N]>, String> {
        let Some(v) = self.numbers(name)? else {
            return Ok(None);
        };
        <[f64; N]>::try_from(v.as_slice())
            .map(Some)
            .map_err(|_| self.bad(name, &format!("expected {N} numbers, got {}", v.len())))
    }

    /// Any number of whitespace-separated numbers.
    pub fn numbers(&self, name: &str) -> Result<Option<Vec<f64>>, String> {
        let Some(text) = self.attr(name) else {
            return Ok(None);
        };
        let mut out = Vec::new();
        for word in text.split_whitespace() {
            match word.parse::<f64>() {
                Ok(v) if v.is_finite() => out.push(v),
                _ => return Err(self.bad(name, &format!("{word:?} is not a finite number"))),
            }
        }
        Ok(Some(out))
    }

    pub fn vec3(&self, name: &str) -> Result<Option<DVec3>, String> {
        Ok(self.nums::<3>(name)?.map(DVec3::from_array))
    }

    /// An MJCF boolean: `true` / `false`, and the `1` / `0` MuJoCo also
    /// takes.
    pub fn flag(&self, name: &str) -> Result<Option<bool>, String> {
        match self.attr(name) {
            None => Ok(None),
            Some("true" | "1") => Ok(Some(true)),
            Some("false" | "0") => Ok(Some(false)),
            Some(_) => Err(self.bad(name, "expected true or false")),
        }
    }

    /// Whichever of [`ORIENTATION_ATTRS`] this element carries, as one
    /// rotation; `None` when it carries none, an error when it carries two.
    ///
    /// `conv` is what `<compiler angle eulerseq>` said, which is why the
    /// compiler is read before any body is.
    pub fn orientation(&self, conv: AngleConvention) -> Result<Option<DQuat>, String> {
        let present: Vec<&str> = ORIENTATION_ATTRS
            .into_iter()
            .filter(|a| self.attrs.contains_key(*a))
            .collect();
        if present.len() > 1 {
            return Err(format!(
                "<{}>: {} are two spellings of one rotation; keep one",
                self.tag,
                present.join(" and ")
            ));
        }
        if let Some([w, x, y, z]) = self.nums::<4>("quat")? {
            let q = DQuat::from_xyzw(x, y, z, w);
            if q.length_squared() < 1e-24 {
                return Err(self.bad("quat", "is zero"));
            }
            return Ok(Some(q.normalize()));
        }
        if let Some(angles) = self.nums::<3>("euler")? {
            let mut q = DQuat::IDENTITY;
            for (letter, angle) in conv.eulerseq.into_iter().zip(angles) {
                let axis = match letter.to_ascii_lowercase() {
                    b'x' => DVec3::X,
                    b'y' => DVec3::Y,
                    b'z' => DVec3::Z,
                    _ => return Err(format!("eulerseq: {:?} is not an axis", letter as char)),
                };
                let r = DQuat::from_axis_angle(axis, conv.radians(angle));
                // A lowercase letter turns about the axes the rotations
                // before it carried along; an uppercase one about the fixed
                // frame. MuJoCo's default `xyz` is the first kind.
                q = if letter.is_ascii_lowercase() {
                    q * r
                } else {
                    r * q
                };
            }
            return Ok(Some(q));
        }
        if let Some([x, y, z, angle]) = self.nums::<4>("axisangle")? {
            let axis = DVec3::new(x, y, z);
            if axis.length_squared() < 1e-24 {
                return Err(self.bad("axisangle", "has a zero axis"));
            }
            return Ok(Some(DQuat::from_axis_angle(
                axis.normalize(),
                conv.radians(angle),
            )));
        }
        // `xyaxes` is the X axis and *a vector in the XY plane*: the second
        // is Gram-Schmidted against the first, exactly as MuJoCo does it.
        if let Some([x1, y1, z1, x2, y2, z2]) = self.nums::<6>("xyaxes")? {
            let x = DVec3::new(x1, y1, z1);
            if x.length_squared() < 1e-24 {
                return Err(self.bad("xyaxes", "has a zero x axis"));
            }
            let x = x.normalize();
            let y = DVec3::new(x2, y2, z2);
            let y = y - x * x.dot(y);
            if y.length_squared() < 1e-24 {
                return Err(self.bad("xyaxes", "has two parallel axes"));
            }
            let y = y.normalize();
            return Ok(Some(DQuat::from_mat3(&DMat3::from_cols(x, y, x.cross(y)))));
        }
        // `zaxis` names only where Z points, so it means the *minimal*
        // rotation that takes it there — the one MuJoCo picks too.
        if let Some(z) = self.vec3("zaxis")? {
            if z.length_squared() < 1e-24 {
                return Err(self.bad("zaxis", "is zero"));
            }
            return Ok(Some(DQuat::from_rotation_arc(DVec3::Z, z.normalize())));
        }
        Ok(None)
    }

    fn bad(&self, name: &str, what: &str) -> String {
        format!(
            "<{}> {name}=\"{}\": {what}",
            self.tag,
            self.attr(name).unwrap_or_default()
        )
    }
}

/// What `<compiler angle eulerseq>` said about the numbers in an
/// orientation attribute. [`Default`] is MJCF's own: **degrees**, and an
/// intrinsic `xyz` sequence — which is why a file that omits the
/// `<compiler>` still reads correctly and ours, which always writes it,
/// reads in radians.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct AngleConvention {
    pub degrees: bool,
    /// Three of `xyzXYZ`; lowercase turns about the moving axes.
    pub eulerseq: [u8; 3],
}

impl Default for AngleConvention {
    fn default() -> Self {
        Self {
            degrees: true,
            eulerseq: *b"xyz",
        }
    }
}

impl AngleConvention {
    /// What our own writer emits: `angle="radian"`, the default `eulerseq`.
    pub const RADIAN: Self = Self {
        degrees: false,
        eulerseq: *b"xyz",
    };

    pub fn radians(self, angle: f64) -> f64 {
        if self.degrees {
            angle.to_radians()
        } else {
            angle
        }
    }
}

/// A file that is not XML, or not one element deep.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseError {
    pub message: String,
    /// Byte offset where the reader stopped.
    pub at: u64,
}

impl fmt::Display for ParseError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} (byte {})", self.message, self.at)
    }
}

impl std::error::Error for ParseError {}

/// The document's single root element and everything under it.
pub fn parse(text: &str) -> Result<Node, ParseError> {
    use quick_xml::events::Event;

    let mut reader = quick_xml::Reader::from_str(text);
    let mut stack: Vec<Node> = Vec::new();
    let mut root: Option<Node> = None;
    loop {
        let at = reader.buffer_position();
        let fail = move |message: String| ParseError { message, at };
        match reader.read_event() {
            Err(e) => return Err(fail(e.to_string())),
            Ok(Event::Eof) => break,
            Ok(Event::Start(e)) => stack.push(element(&e).map_err(fail)?),
            Ok(Event::Empty(e)) => {
                let node = element(&e).map_err(fail)?;
                place(&mut stack, &mut root, node).map_err(fail)?;
            }
            Ok(Event::End(e)) => {
                let name = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                let node = stack
                    .pop()
                    .ok_or_else(|| fail(format!("</{name}> closes nothing")))?;
                if node.tag != name {
                    return Err(fail(format!("</{name}> closes <{}>", node.tag)));
                }
                place(&mut stack, &mut root, node).map_err(fail)?;
            }
            Ok(_) => {}
        }
    }
    if let Some(open) = stack.last() {
        return Err(ParseError {
            message: format!("<{}> is never closed", open.tag),
            at: reader.buffer_position(),
        });
    }
    root.ok_or_else(|| ParseError {
        message: "no root element".to_owned(),
        at: 0,
    })
}

fn element(e: &quick_xml::events::BytesStart<'_>) -> Result<Node, String> {
    let tag = String::from_utf8(e.name().as_ref().to_vec())
        .map_err(|_| "an element name is not UTF-8".to_owned())?;
    let mut attrs = BTreeMap::new();
    for a in e.attributes() {
        let a = a.map_err(|err| format!("<{tag}>: {err}"))?;
        let key = String::from_utf8(a.key.as_ref().to_vec())
            .map_err(|_| format!("<{tag}>: an attribute name is not UTF-8"))?;
        let value = a
            .unescape_value()
            .map_err(|err| format!("<{tag}> {key}: {err}"))?
            .into_owned();
        attrs.insert(key, value);
    }
    Ok(Node {
        tag,
        attrs,
        children: Vec::new(),
    })
}

fn place(stack: &mut [Node], root: &mut Option<Node>, node: Node) -> Result<(), String> {
    match stack.last_mut() {
        Some(parent) => parent.children.push(node),
        None if root.is_none() => *root = Some(node),
        None => return Err(format!("<{}> is a second root element", node.tag)),
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    /// The rotation every spelling below names: 90° about +X, which takes
    /// Y to Z and Z to −Y. It is picked because `zaxis` can only ever name
    /// the *minimal* rotation onto a direction, and this one is minimal.
    fn rx90() -> DQuat {
        DQuat::from_rotation_x(FRAC_PI_2)
    }

    fn node(attr: &str, value: &str) -> Node {
        Node {
            tag: "body".to_owned(),
            attrs: BTreeMap::from([(attr.to_owned(), value.to_owned())]),
            children: Vec::new(),
        }
    }

    #[track_caller]
    fn assert_same(q: DQuat, expected: DQuat) {
        // A quaternion and its negation are one rotation.
        let close = (q.x - expected.x).abs()
            + (q.y - expected.y).abs()
            + (q.z - expected.z).abs()
            + (q.w - expected.w).abs();
        let flipped = (q.x + expected.x).abs()
            + (q.y + expected.y).abs()
            + (q.z + expected.z).abs()
            + (q.w + expected.w).abs();
        assert!(close.min(flipped) < 1e-12, "{q:?} is not {expected:?}");
    }

    #[test]
    fn every_spelling_of_one_rotation_agrees() {
        let deg = AngleConvention::default();
        let h = FRAC_PI_2 / 2.0;
        for (attr, value) in [
            ("quat", format!("{} {} 0 0", h.cos(), h.sin())),
            ("euler", "90 0 0".to_owned()),
            ("axisangle", "1 0 0 90".to_owned()),
            // The X axis, and the image of Y — a vector in the new XY plane.
            ("xyaxes", "1 0 0 0 0 1".to_owned()),
            // Only where Z points: the minimal rotation that takes it there.
            ("zaxis", "0 -1 0".to_owned()),
        ] {
            let q = node(attr, &value)
                .orientation(deg)
                .unwrap()
                .unwrap_or_else(|| panic!("{attr} read as no rotation"));
            assert_same(q, rx90());
        }
        // `xyaxes` does not need the second vector to be perpendicular: it
        // is Gram-Schmidted, as MuJoCo does it.
        assert_same(
            node("xyaxes", "1 0 0 0.5 0 2")
                .orientation(deg)
                .unwrap()
                .unwrap(),
            rx90(),
        );
        // Nothing named is no rotation at all, not the identity by accident.
        assert_eq!(node("pos", "1 2 3").orientation(deg).unwrap(), None);
    }

    #[test]
    fn the_compiler_decides_degrees_and_the_euler_sequence() {
        let deg = AngleConvention::default();
        let rad = AngleConvention::RADIAN;
        // The same text is two different rotations under the two
        // conventions, and MJCF's *default* is the degree one.
        assert_same(
            node("euler", "90 0 0").orientation(deg).unwrap().unwrap(),
            rx90(),
        );
        assert_same(
            node("euler", "1.5707963267948966 0 0")
                .orientation(rad)
                .unwrap()
                .unwrap(),
            rx90(),
        );
        assert_same(
            node("axisangle", "1 0 0 1.5707963267948966")
                .orientation(rad)
                .unwrap()
                .unwrap(),
            rx90(),
        );
        // `quat`, `xyaxes` and `zaxis` carry no angle, so `angle` cannot
        // touch them.
        for (attr, value) in [("xyaxes", "1 0 0 0 0 1"), ("zaxis", "0 -1 0")] {
            assert_same(node(attr, value).orientation(rad).unwrap().unwrap(), rx90());
        }
        // Lowercase turns about the moving axes, uppercase about the fixed
        // ones, so `xyz` and `XYZ` are the same product in opposite order.
        let seq = |letters: &[u8; 3]| {
            let conv = AngleConvention {
                degrees: true,
                eulerseq: *letters,
            };
            node("euler", "90 90 0").orientation(conv).unwrap().unwrap()
        };
        let (rx, ry) = (
            DQuat::from_rotation_x(FRAC_PI_2),
            DQuat::from_rotation_y(FRAC_PI_2),
        );
        assert_same(seq(b"xyz"), rx * ry);
        assert_same(seq(b"XYZ"), ry * rx);
        assert!(
            (seq(b"xyz").dot(seq(b"XYZ")).abs() - 1.0).abs() > 1e-6,
            "the two differ"
        );
        // A non-default sequence is honoured.
        assert_same(seq(b"zyx"), DQuat::from_rotation_z(FRAC_PI_2) * ry);
    }

    #[test]
    fn a_bad_orientation_is_named_not_guessed() {
        let deg = AngleConvention::default();
        let two = Node {
            tag: "geom".to_owned(),
            attrs: BTreeMap::from([
                ("quat".to_owned(), "1 0 0 0".to_owned()),
                ("euler".to_owned(), "0 0 0".to_owned()),
            ]),
            children: Vec::new(),
        };
        assert_eq!(
            two.orientation(deg).unwrap_err(),
            "<geom>: quat and euler are two spellings of one rotation; keep one"
        );
        for (attr, value, message) in [
            ("quat", "0 0 0 0", "<body> quat=\"0 0 0 0\": is zero"),
            (
                "axisangle",
                "0 0 0 1",
                "<body> axisangle=\"0 0 0 1\": has a zero axis",
            ),
            (
                "xyaxes",
                "1 0 0 2 0 0",
                "<body> xyaxes=\"1 0 0 2 0 0\": has two parallel axes",
            ),
            ("zaxis", "0 0 0", "<body> zaxis=\"0 0 0\": is zero"),
            (
                "quat",
                "1 0 0",
                "<body> quat=\"1 0 0\": expected 4 numbers, got 3",
            ),
            (
                "euler",
                "1 x 0",
                "<body> euler=\"1 x 0\": \"x\" is not a finite number",
            ),
            (
                "euler",
                "1 inf 0",
                "<body> euler=\"1 inf 0\": \"inf\" is not a finite number",
            ),
        ] {
            assert_eq!(node(attr, value).orientation(deg).unwrap_err(), message);
        }
    }

    #[test]
    fn attribute_helpers_read_what_is_there_and_name_what_is_not() {
        let n = Node {
            tag: "geom".to_owned(),
            attrs: BTreeMap::from([
                ("size".to_owned(), " 0.05  0.1\n0.15 ".to_owned()),
                ("mass".to_owned(), "2.7".to_owned()),
                ("limited".to_owned(), "true".to_owned()),
                ("contype".to_owned(), "0".to_owned()),
            ]),
            children: Vec::new(),
        };
        assert_eq!(n.vec3("size").unwrap(), Some(DVec3::new(0.05, 0.1, 0.15)));
        assert_eq!(n.num("mass").unwrap(), Some(2.7));
        assert_eq!(n.num("density").unwrap(), None);
        assert_eq!(n.numbers("size").unwrap().unwrap().len(), 3);
        assert_eq!(n.flag("limited").unwrap(), Some(true));
        assert_eq!(n.flag("contype").unwrap(), Some(false));
        assert_eq!(n.flag("conaffinity").unwrap(), None);
        assert_eq!(
            n.flag("size").unwrap_err(),
            "<geom> size=\" 0.05  0.1\n0.15 \": expected true or false"
        );
        assert_eq!(
            n.nums::<2>("size").unwrap_err(),
            "<geom> size=\" 0.05  0.1\n0.15 \": expected 2 numbers, got 3"
        );
    }

    #[test]
    fn the_writers_golden_parses_back_to_what_it_wrote() {
        let root = parse(crate::mjcf::tests::GOLDEN).unwrap();
        assert_eq!(root.tag, "mujoco");
        assert_eq!(root.attr("model"), Some("test"));
        // The comment and the XML declaration are gone; the elements are not.
        let compiler = root.child("compiler").unwrap();
        assert_eq!(compiler.attr("angle"), Some("radian"));
        assert_eq!(compiler.attr("meshdir"), Some("meshes"));
        assert_eq!(compiler.flag("autolimits").unwrap(), Some(true));
        let conv = AngleConvention::RADIAN;

        let world = root.child("worldbody").unwrap();
        assert_eq!(world.kids("body").count(), 1, "one root body");
        let base = world.child("body").unwrap();
        assert_eq!(base.attr("name"), Some("base_link"));
        assert_eq!(base.orientation(conv).unwrap(), None, "no pose at identity");
        assert_eq!(base.kids("geom").count(), 2);
        assert_eq!(
            base.child("inertial")
                .unwrap()
                .nums::<6>("fullinertia")
                .unwrap(),
            Some([0.0045, 0.0045, 0.0045, 0.0, 0.0, 0.0])
        );
        // The `<site>` the frame was written as, with its quaternion back
        // through the same helper the writer's `quat` came out of.
        let site = base.child("site").unwrap();
        assert_eq!(site.attr("name"), Some("camera_mount"));
        assert_eq!(site.vec3("pos").unwrap(), Some(DVec3::new(0.0, 0.03, 0.04)));
        assert_same(
            site.orientation(conv).unwrap().unwrap(),
            DQuat::from_rotation_x(FRAC_PI_2),
        );

        // Four bodies deep, each nested in the one before.
        let upper = base.child("body").unwrap();
        assert_eq!(upper.attr("name"), Some("upper"));
        assert_eq!(upper.vec3("pos").unwrap(), Some(DVec3::Z * 0.1));
        let joint = upper.child("joint").unwrap();
        assert_eq!(joint.attr("type"), Some("hinge"));
        assert_eq!(joint.vec3("axis").unwrap(), Some(DVec3::Y));
        assert_eq!(joint.nums::<2>("range").unwrap(), Some([-1.0, 1.0]));
        assert_eq!(joint.num("damping").unwrap(), Some(0.1));
        let tip = upper
            .child("body")
            .unwrap()
            .child("body")
            .unwrap()
            .child("body")
            .unwrap();
        assert_eq!(tip.attr("name"), Some("tip"));
        assert_eq!(tip.child("site").unwrap().attr("name"), Some("tcp"));

        // The two blocks after `</worldbody>`.
        let equality = root.child("equality").unwrap().child("joint").unwrap();
        assert_eq!(equality.attr("joint1"), Some("slider_joint"));
        assert_eq!(
            equality.nums::<5>("polycoef").unwrap(),
            Some([0.1, -0.5, 0.0, 0.0, 0.0])
        );
        let actuator = root.child("actuator").unwrap();
        assert_eq!(actuator.children.len(), 2);
        assert_eq!(actuator.children[0].tag, "position");
        assert_eq!(actuator.children[0].num("kp").unwrap(), Some(100.0));
        assert_eq!(actuator.children[1].tag, "velocity");
    }

    #[test]
    fn escaping_survives_the_round_trip_and_a_broken_file_is_refused() {
        let mut x = Xml::new();
        x.open("mujoco", &[("model", "a<b>&\"c\"'d'".into())]);
        x.comment("dropped on the way back");
        x.empty("body", &[("name", "é".into())]);
        x.close("mujoco");
        let root = parse(&x.finish()).unwrap();
        assert_eq!(root.attr("model"), Some("a<b>&\"c\"'d'"));
        assert_eq!(root.children.len(), 1, "the comment is not a child");
        assert_eq!(root.child("body").unwrap().attr("name"), Some("é"));

        for (text, message) in [
            ("<a><b></a>", "expected `</b>`, but `</a>` was found"),
            ("<a>", "<a> is never closed"),
            ("<?xml version=\"1.0\"?>", "no root element"),
            ("<a/><b/>", "<b> is a second root element"),
        ] {
            let err = parse(text).unwrap_err();
            assert!(
                err.message.contains(message),
                "{text:?} gave {:?}, wanted {message:?}",
                err.message
            );
        }
    }

    #[test]
    fn escapes_the_five_entities_only() {
        assert_eq!(
            escape("a<b>&\"c\"'d' é"),
            "a&lt;b&gt;&amp;&quot;c&quot;&apos;d&apos; é"
        );
        let mut xml = Xml::new();
        xml.open("a", &[("k", "x<y".into())]);
        xml.comment("effort 10 -- not written");
        xml.empty("b", &[]);
        xml.close("a");
        assert_eq!(
            xml.finish(),
            "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n<a k=\"x&lt;y\">\n  <!-- effort 10 - - not written -->\n  <b/>\n</a>\n"
        );
    }

    /// The body of an element is escaped like an attribute value, and
    /// comes back through the reading half unchanged in meaning — the
    /// text node itself is dropped, as [`Node`] documents.
    #[test]
    fn a_text_element_escapes_its_body_and_closes_on_one_line() {
        let mut x = Xml::new();
        x.open("sdf", &[("version", "1.11".into())]);
        x.text("mass", &[], &num(2.7));
        x.text("uri", &[], "meshes/a<b>&\"c\".stl");
        x.text("multiplier", &[("note", "a&b".into())], &num(-0.5));
        x.text("empty", &[], "");
        x.close("sdf");
        let out = x.finish();
        assert_eq!(
            out,
            "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n\
             <sdf version=\"1.11\">\n\
             \x20 <mass>2.7</mass>\n\
             \x20 <uri>meshes/a&lt;b&gt;&amp;&quot;c&quot;.stl</uri>\n\
             \x20 <multiplier note=\"a&amp;b\">-0.5</multiplier>\n\
             \x20 <empty></empty>\n\
             </sdf>\n"
        );
        // Still XML, and the escaping survives a parser that is not ours.
        let root = parse(&out).unwrap();
        assert_eq!(root.attr("version"), Some("1.11"));
        assert_eq!(root.children.len(), 4);
    }

    /// SDF's six-number `<pose>` is URDF's `xyz` and `rpy` in one string,
    /// through the one helper both go through — so a pose that survives
    /// the URDF round trip survives this one.
    #[test]
    fn pose6_is_xyz_then_rpy_with_the_same_number_rules() {
        assert_eq!(pose6(&Pose::IDENTITY), "0 0 0 0 0 0");
        let p = Pose {
            t: DVec3::new(0.0, 0.03, 0.04),
            r: DQuat::from_rotation_x(FRAC_PI_2),
        };
        assert_eq!(pose6(&p), "0 0.03 0.04 1.570796326795 0 0");
        // The `-0` folding and the twelve-decimal trim `num` does apply
        // inside the body exactly as they do inside an attribute.
        let (xyz, rpy) = p.to_xyz_rpy();
        assert_eq!(pose6(&p), format!("{} {}", vec3(xyz), vec3(rpy)));
        assert_eq!(
            pose6(&Pose {
                t: DVec3::new(-0.0, 1e-15, 1.0),
                r: DQuat::IDENTITY,
            }),
            "0 0 1 0 0 0"
        );
    }

    #[test]
    fn numbers_are_trimmed_and_negative_zero_folded() {
        assert_eq!(num(0.0), "0");
        assert_eq!(num(-0.0), "0");
        assert_eq!(num(1.0), "1");
        assert_eq!(num(-2.5), "-2.5");
        assert_eq!(num(0.1 + 0.2), "0.3");
        assert_eq!(num(1e-15), "0");
        assert_eq!(num(-1e-15), "0");
        assert_eq!(num(0.004500000000000001), "0.0045");
        assert_eq!(num(1234.5678), "1234.5678");
        assert_eq!(vec3(DVec3::new(1.0, -0.0, 0.5)), "1 0 0.5");
    }

    #[test]
    fn quaternion_is_w_first() {
        assert_eq!(quat_wxyz(DQuat::IDENTITY), [1.0, 0.0, 0.0, 0.0]);
        let q = DQuat::from_rotation_z(FRAC_PI_2);
        let [w, x, y, z] = quat_wxyz(q);
        let h = FRAC_PI_2 / 2.0;
        assert!((w - h.cos()).abs() < 1e-15);
        assert_eq!((x, y), (0.0, 0.0));
        assert!((z - h.sin()).abs() < 1e-15);
        assert_eq!(quat(DQuat::IDENTITY), "1 0 0 0");
        // Not normalised in: normalised out.
        assert_eq!(quat(DQuat::from_xyzw(0.0, 0.0, 0.0, 2.0)), "1 0 0 0");
    }
}
