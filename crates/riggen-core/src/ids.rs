//! Document ids: `u32` newtypes handed out by a per-document [`IdGen`]
//! counter, stored in `BTreeMap`s and serialised as `"l3"` / `"j7"` strings
//! (ADR-0005). Stable across edits and across save/load, never reused within
//! a document's life.

use std::fmt;
use std::str::FromStr;

use serde::de::{self, Visitor};
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// Shared by every id kind so [`IdGen::alloc`] can be generic. Sealed: the
/// only implementors are the newtypes below.
pub trait Id: Copy + Ord + fmt::Debug + fmt::Display + private::Sealed {
    /// Letter that prefixes the serialised form (`'l'` for `"l3"`).
    const PREFIX: char;
    /// What the id is called in error messages ("link", "joint", …).
    const KIND: &'static str;
    fn from_raw(raw: u32) -> Self;
    fn raw(self) -> u32;
}

mod private {
    pub trait Sealed {}
}

/// Why a string is not an id of the expected kind.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ParseIdError {
    pub text: String,
    pub expected_prefix: char,
}

impl fmt::Display for ParseIdError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "\"{}\" is not an id of the form \"{}<number>\"",
            self.text, self.expected_prefix
        )
    }
}

impl std::error::Error for ParseIdError {}

macro_rules! id_type {
    ($(#[$doc:meta])* $name:ident, $prefix:literal, $kind:literal) => {
        $(#[$doc])*
        #[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
        pub struct $name(u32);

        impl private::Sealed for $name {}

        impl Id for $name {
            const PREFIX: char = $prefix;
            const KIND: &'static str = $kind;
            fn from_raw(raw: u32) -> Self {
                Self(raw)
            }
            fn raw(self) -> u32 {
                self.0
            }
        }

        impl fmt::Display for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}{}", $prefix, self.0)
            }
        }

        impl fmt::Debug for $name {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                write!(f, "{}({}{})", stringify!($name), $prefix, self.0)
            }
        }

        impl FromStr for $name {
            type Err = ParseIdError;
            fn from_str(s: &str) -> Result<Self, ParseIdError> {
                let error = || ParseIdError {
                    text: s.to_owned(),
                    expected_prefix: $prefix,
                };
                let digits = s.strip_prefix($prefix).ok_or_else(error)?;
                // `u32::from_str` accepts a leading `+`; ids never carry one.
                if digits.is_empty() || !digits.bytes().all(|b| b.is_ascii_digit()) {
                    return Err(error());
                }
                digits.parse().map(Self).map_err(|_| error())
            }
        }

        impl Serialize for $name {
            fn serialize<S: Serializer>(&self, serializer: S) -> Result<S::Ok, S::Error> {
                serializer.collect_str(self)
            }
        }

        impl<'de> Deserialize<'de> for $name {
            fn deserialize<D: Deserializer<'de>>(deserializer: D) -> Result<Self, D::Error> {
                struct IdVisitor;
                impl Visitor<'_> for IdVisitor {
                    type Value = $name;
                    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                        write!(f, "an id string like \"{}3\"", $prefix)
                    }
                    fn visit_str<E: de::Error>(self, v: &str) -> Result<$name, E> {
                        v.parse().map_err(de::Error::custom)
                    }
                }
                deserializer.deserialize_str(IdVisitor)
            }
        }
    };
}

id_type!(
    /// A link, the node of the kinematic tree.
    LinkId, 'l', "link"
);
id_type!(
    /// A joint, the edge between a parent and a child link.
    JointId, 'j', "joint"
);
id_type!(
    /// A visual geom inside a link; `(LinkId, GeomId)` keys viewport instances.
    GeomId, 'g', "geom"
);
id_type!(
    /// A registered mesh file (`MeshAsset`).
    MeshId, 'm', "mesh"
);
id_type!(
    /// A named frame attached to a link: a TCP, a sensor mount (ADR-0012).
    FrameId, 'f', "frame"
);

/// The document's id counter. One counter for every kind, so an id is
/// unique across kinds too; serialised as a bare number.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(transparent)]
pub struct IdGen {
    next: u32,
}

impl IdGen {
    pub fn new() -> Self {
        Self { next: 0 }
    }

    /// Hands out the next id; never returns the same number twice.
    pub fn alloc<I: Id>(&mut self) -> I {
        let id = I::from_raw(self.next);
        self.next = self
            .next
            .checked_add(1)
            .expect("more than u32::MAX ids in one document");
        id
    }

    /// The raw value the next [`alloc`](Self::alloc) will return.
    pub fn peek(&self) -> u32 {
        self.next
    }
}

impl Default for IdGen {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeMap;

    #[test]
    fn display_and_parse_round_trip() {
        let id = LinkId::from_raw(3);
        assert_eq!(id.to_string(), "l3");
        assert_eq!("l3".parse::<LinkId>(), Ok(id));
        assert_eq!(format!("{id:?}"), "LinkId(l3)");
        assert_eq!(JointId::from_raw(7).to_string(), "j7");
        assert_eq!(GeomId::from_raw(2).to_string(), "g2");
        assert_eq!(MeshId::from_raw(1).to_string(), "m1");
        assert_eq!(FrameId::from_raw(0).to_string(), "f0");
    }

    #[test]
    fn parse_rejects_wrong_prefix_and_garbage() {
        for text in ["j3", "l", "l-1", "l+1", "l 1", "3", "", "l3x"] {
            assert!(text.parse::<LinkId>().is_err(), "{text:?} should not parse");
        }
        assert_eq!(
            "j3".parse::<LinkId>().unwrap_err().to_string(),
            "\"j3\" is not an id of the form \"l<number>\""
        );
    }

    #[test]
    fn serde_as_string_and_as_map_key() {
        let id = LinkId::from_raw(3);
        assert_eq!(serde_json::to_string(&id).unwrap(), "\"l3\"");
        assert_eq!(serde_json::from_str::<LinkId>("\"l3\"").unwrap(), id);
        assert!(serde_json::from_str::<LinkId>("\"j3\"").is_err());
        assert!(serde_json::from_str::<LinkId>("3").is_err());

        let mut map = BTreeMap::new();
        map.insert(JointId::from_raw(10), "ten");
        map.insert(JointId::from_raw(2), "two");
        let json = serde_json::to_string(&map).unwrap();
        assert_eq!(json, "{\"j2\":\"two\",\"j10\":\"ten\"}");
        let back: BTreeMap<JointId, &str> = serde_json::from_str(&json).unwrap();
        assert_eq!(back, map);
    }

    #[test]
    fn id_gen_never_repeats_and_serialises_as_a_number() {
        let mut ids = IdGen::new();
        let a: LinkId = ids.alloc();
        let b: JointId = ids.alloc();
        let c: LinkId = ids.alloc();
        assert_eq!((a.raw(), b.raw(), c.raw()), (0, 1, 2));
        assert_eq!(ids.peek(), 3);
        assert_eq!(serde_json::to_string(&ids).unwrap(), "3");
        let back: IdGen = serde_json::from_str("3").unwrap();
        assert_eq!(back, ids);
    }
}
