//! The sample robot compiled into the build.
//!
//! Target-independent on purpose (docs/plans/web-demo.md step 4): the
//! desktop's `--example arm` unpacks these bytes to a temp directory and
//! opens the file, and the web demo hands the very same bytes to
//! `RiggenApp::load_dropped` as if they had been dropped on the page. One
//! sample robot, one copy of it, two ways in.

#[cfg(not(target_arch = "wasm32"))]
use std::path::Path;
use std::path::PathBuf;

/// A sample robot compiled into the binary, so the first run after
/// `uv tool install riggen` needs nothing downloaded.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Example {
    /// `assets/fixtures/arm/`: the M3 sample arm, four STL parts and the
    /// document that assembles them.
    Arm,
}

impl Example {
    pub const NAMES: &[&str] = &["arm"];

    pub fn from_name(name: &str) -> Option<Self> {
        match name {
            "arm" => Some(Self::Arm),
            _ => None,
        }
    }

    /// `(file name, bytes)` for every file the example needs.
    pub fn files(self) -> &'static [(&'static str, &'static [u8])] {
        match self {
            Self::Arm => &[
                (
                    "arm.riggen",
                    include_bytes!("../../../assets/fixtures/arm/arm.riggen"),
                ),
                (
                    "base.stl",
                    include_bytes!("../../../assets/fixtures/arm/base.stl"),
                ),
                (
                    "shoulder.stl",
                    include_bytes!("../../../assets/fixtures/arm/shoulder.stl"),
                ),
                (
                    "upper.stl",
                    include_bytes!("../../../assets/fixtures/arm/upper.stl"),
                ),
                (
                    "fore.stl",
                    include_bytes!("../../../assets/fixtures/arm/fore.stl"),
                ),
            ],
        }
    }

    /// The document file to open, once extracted.
    pub fn document(self) -> &'static str {
        match self {
            Self::Arm => "arm.riggen",
        }
    }

    pub fn name(self) -> &'static str {
        match self {
            Self::Arm => "arm",
        }
    }

    /// The same files as one drop gesture would deliver them: what the web
    /// build opens at startup.
    pub fn dropped(self) -> Vec<(PathBuf, Vec<u8>)> {
        self.files()
            .iter()
            .map(|(name, bytes)| (PathBuf::from(name), bytes.to_vec()))
            .collect()
    }

    /// Writes the files to `<temp>/riggen-example-<name>/` (overwriting a
    /// previous extraction: they are 64 KB) and returns the document path.
    #[cfg(not(target_arch = "wasm32"))]
    pub fn extract(self) -> Result<PathBuf, String> {
        self.extract_into(&std::env::temp_dir())
    }

    #[cfg(not(target_arch = "wasm32"))]
    pub fn extract_into(self, temp: &Path) -> Result<PathBuf, String> {
        let dir = temp.join(format!("riggen-example-{}", self.name()));
        std::fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        for (name, bytes) in self.files() {
            let path = dir.join(name);
            std::fs::write(&path, bytes).map_err(|e| format!("{}: {e}", path.display()))?;
        }
        Ok(dir.join(self.document()))
    }
}
