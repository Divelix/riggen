//! A 30-line XML writer: the output of both writers is fixed-shape, so an
//! XML crate would only add a dependency and a way to emit something
//! MuJoCo cannot read. Escaping is the whole job.

use std::fmt::Write as _;

use riggen_core::glam::{DQuat, DVec3};

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

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

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
