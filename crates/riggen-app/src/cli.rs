//! The command line: `riggen [FILE...]` opens the window, `riggen --export
//! mjcf|urdf|sdf|both|all --out DIR INPUT` is the headless export
//! (ADR-0008, ADR-0016) that returns before eframe starts — it is what
//! CI's `mujoco` and `sdf` jobs run, so it must need no display.
//! `--help`, `--version` and `--example arm` are the rest
//! (docs/01-architecture.md §Crates). `INPUT` is a `.riggen` document, a
//! `.urdf` or an MJCF `.xml` (imported through `riggen_export::urdf_in` or
//! `mjcf_in` first).
//!
//! Hand-rolled on purpose: half a dozen flags do not earn `clap` and its
//! compile time. What keeps it honest is [`FLAGS`]: the parser accepts only
//! what is listed there, and [`help`] is generated from the same table, so
//! a flag cannot exist without its help line.

use std::ffi::OsString;
use std::path::{Path, PathBuf};

use riggen_export::{ExportOptions, Format, MeshStore};

/// The one-line reminder under every parse error.
pub const USAGE: &str = "usage: riggen [FILE...] | riggen --example arm | \
riggen --export mjcf|urdf|sdf|both|all --out DIR [--fk-samples] INPUT\ntry `riggen --help`";

/// One command-line flag: its spelling, an optional short form, the name
/// of the value it takes (if any) and the help line.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Flag {
    pub long: &'static str,
    pub short: Option<&'static str>,
    pub value: Option<&'static str>,
    pub doc: &'static str,
}

/// Every flag the parser accepts, in the order `--help` lists them.
pub const FLAGS: &[Flag] = &[
    Flag {
        long: "--example",
        short: None,
        value: Some("NAME"),
        doc: "open a bundled example: arm (the five-link sample robot)",
    },
    Flag {
        long: "--export",
        short: None,
        value: Some("FORMAT"),
        doc: "headless export of INPUT (.riggen, .urdf or .xml): mjcf, urdf, sdf, both or all",
    },
    Flag {
        long: "--out",
        short: None,
        value: Some("DIR"),
        doc: "where --export writes; created if missing",
    },
    Flag {
        long: "--fk-samples",
        short: None,
        value: None,
        doc: "with --export: also write <name>.fk.json, five sampled joint configurations",
    },
    Flag {
        long: "--timing",
        short: None,
        value: None,
        doc: "print the time from launch to the first frame on stderr",
    },
    Flag {
        long: "--help",
        short: Some("-h"),
        value: None,
        doc: "print this help",
    },
    Flag {
        long: "--version",
        short: Some("-V"),
        value: None,
        doc: "print the version and the git commit it was built from",
    },
];

/// `riggen 0.1.0 (2b60ae4 2026-08-29)`: the crate version, then the git
/// hash and commit date `build.rs` recorded (`unknown` for a build that had
/// neither `.git` nor `RIGGEN_GIT_HASH`, such as one from the sdist).
pub fn version() -> String {
    format!(
        "riggen {} ({} {})",
        env!("CARGO_PKG_VERSION"),
        env!("RIGGEN_GIT_HASH"),
        env!("RIGGEN_BUILD_DATE")
    )
}

/// The `--help` text: usage forms, then one line per [`FLAGS`] entry.
pub fn help() -> String {
    let mut out = String::new();
    out.push_str(&version());
    out.push_str(
        "\nThe robot assembler for RL researchers: meshes in, MJCF, URDF and SDF out.\n\n",
    );
    out.push_str("usage:\n");
    out.push_str(
        "  riggen [FILE...]        open a .riggen document, or drop meshes (.stl, .obj) as links\n",
    );
    out.push_str("  riggen --example arm    open the bundled sample arm\n");
    out.push_str("  riggen --export mjcf|urdf|sdf|both|all --out DIR [--fk-samples] INPUT\n");
    out.push_str(
        "                          write INPUT's export to DIR without opening a window\n\n",
    );
    out.push_str("options:\n");
    for flag in FLAGS {
        let mut spelling = String::new();
        if let Some(short) = flag.short {
            spelling.push_str(short);
            spelling.push_str(", ");
        }
        spelling.push_str(flag.long);
        if let Some(value) = flag.value {
            spelling.push(' ');
            spelling.push_str(value);
        }
        out.push_str(&format!("  {spelling:<22}  {}\n", flag.doc));
    }
    out
}

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

    fn from_name(name: &str) -> Option<Self> {
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
    fn document(self) -> &'static str {
        match self {
            Self::Arm => "arm.riggen",
        }
    }

    /// Writes the files to `<temp>/riggen-example-<name>/` (overwriting a
    /// previous extraction: they are 64 KB) and returns the document path.
    pub fn extract(self) -> Result<PathBuf, String> {
        self.extract_into(&std::env::temp_dir())
    }

    pub fn extract_into(self, temp: &Path) -> Result<PathBuf, String> {
        let dir = temp.join(format!("riggen-example-{}", self.name()));
        std::fs::create_dir_all(&dir).map_err(|e| format!("{}: {e}", dir.display()))?;
        for (name, bytes) in self.files() {
            let path = dir.join(name);
            std::fs::write(&path, bytes).map_err(|e| format!("{}: {e}", path.display()))?;
        }
        Ok(dir.join(self.document()))
    }

    fn name(self) -> &'static str {
        match self {
            Self::Arm => "arm",
        }
    }
}

/// What the command line asked for.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Invocation {
    /// Open the window on `files` (possibly none) and, first, the example.
    Open(OpenArgs),
    /// The headless export; never opens a window.
    Export(ExportArgs),
    Help,
    Version,
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct OpenArgs {
    pub files: Vec<PathBuf>,
    pub example: Option<Example>,
    /// `--timing`: report the first frame on stderr.
    pub timing: bool,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExportArgs {
    pub format: Format,
    pub out: PathBuf,
    pub input: PathBuf,
    /// Also write `<name>.fk.json` (`riggen_export::fk_samples`).
    pub fk_samples: bool,
}

/// Parses everything after the program name. `--help` and `--version` win
/// wherever they appear; an `--export` anywhere makes it the export form,
/// whose other arguments are checked; otherwise every positional is a file
/// to open.
pub fn parse(args: &[OsString]) -> Result<Invocation, String> {
    let mut format = None;
    let mut out = None;
    let mut fk_samples = false;
    let mut example = None;
    let mut timing = false;
    let mut positional = Vec::new();
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.to_str() {
            Some("--help" | "-h") => return Ok(Invocation::Help),
            Some("--version" | "-V") => return Ok(Invocation::Version),
            Some("--export") => {
                let value = it.next().and_then(|v| v.to_str());
                format = Some(match value.map(str::parse::<Format>) {
                    Some(Ok(format)) => format,
                    _ => {
                        return Err(format!(
                            "--export expects one of {}, got {value:?}\n{USAGE}",
                            Format::NAMES.join(", ")
                        ));
                    }
                });
            }
            Some("--out") => {
                out = Some(PathBuf::from(
                    it.next()
                        .ok_or(format!("--out expects a directory\n{USAGE}"))?,
                ));
            }
            Some("--fk-samples") => fk_samples = true,
            Some("--timing") => timing = true,
            Some("--example") => {
                let name = it.next().and_then(|v| v.to_str());
                example = Some(name.and_then(Example::from_name).ok_or_else(|| {
                    format!(
                        "--example expects one of {}, got {name:?}\n{USAGE}",
                        Example::NAMES.join(", ")
                    )
                })?);
            }
            Some(flag) if flag.starts_with('-') && flag.len() > 1 => {
                return Err(format!("unknown flag {flag}\n{USAGE}"));
            }
            _ => positional.push(PathBuf::from(arg)),
        }
    }
    let Some(format) = format else {
        if out.is_some() || fk_samples {
            return Err(format!("--out and --fk-samples need --export\n{USAGE}"));
        }
        return Ok(Invocation::Open(OpenArgs {
            files: positional,
            example,
            timing,
        }));
    };
    if example.is_some() || timing {
        return Err(format!(
            "--example and --timing are for the window; --export opens none\n{USAGE}"
        ));
    }
    let mut inputs = positional.into_iter();
    let input = inputs.next().ok_or(format!("INPUT is required\n{USAGE}"))?;
    if inputs.next().is_some() {
        return Err(format!("only one INPUT is exported at a time\n{USAGE}"));
    }
    Ok(Invocation::Export(ExportArgs {
        format,
        out: out.ok_or(format!("--out is required\n{USAGE}"))?,
        input,
        fk_samples,
    }))
}

fn warn_all(warnings: &[impl std::fmt::Display]) {
    for w in warnings {
        eprintln!("warning: {w}");
    }
}

/// Loads, resolves and writes. The `Err` is what the user reads on stderr:
/// every resolve error, one per line.
pub fn run(args: &ExportArgs) -> Result<Vec<PathBuf>, String> {
    let extension = args
        .input
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or_default()
        .to_ascii_lowercase();
    // A `.urdf` or an `.xml` is imported first (02 §URDF import, §MJCF
    // import); anything else is read as a document.
    let robot = match extension.as_str() {
        "urdf" => {
            let (robot, warnings) =
                riggen_export::urdf_in::load(&args.input, &riggen_export::PackageMap::default())
                    .map_err(|e| e.to_string())?;
            warn_all(&warnings);
            robot
        }
        "xml" => {
            let (robot, warnings) =
                riggen_export::mjcf_in::load(&args.input).map_err(|e| e.to_string())?;
            warn_all(&warnings);
            robot
        }
        _ => {
            let (robot, warnings) = riggen_core::load(&args.input).map_err(|e| e.to_string())?;
            warn_all(&warnings);
            robot
        }
    };
    let (store, load_errors) = MeshStore::load(&robot);
    let options = ExportOptions {
        format: args.format,
        ..Default::default()
    };
    let resolved =
        match riggen_export::resolve(&robot, &store, &riggen_export::ComputeNow, &options) {
            Ok(r) if load_errors.is_empty() => r,
            Ok(_) => return Err(join_errors(&load_errors)),
            Err(mut errors) => {
                errors.extend(load_errors);
                return Err(join_errors(&errors));
            }
        };
    let mut written =
        riggen_export::export(&resolved, &options, &args.out).map_err(|e| e.to_string())?;
    if args.fk_samples {
        let path = args.out.join(format!("{}.fk.json", robot.name));
        std::fs::write(&path, riggen_export::fk_samples::to_json(&robot))
            .map_err(|e| format!("{}: {e}", path.display()))?;
        written.push(path);
    }
    Ok(written)
}

fn join_errors(errors: &[riggen_export::ExportError]) -> String {
    errors
        .iter()
        .map(|e| format!("cannot export: {e}"))
        .collect::<Vec<_>>()
        .join("\n")
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::Path;

    fn args(list: &[&str]) -> Vec<OsString> {
        list.iter().map(OsString::from).collect()
    }

    #[test]
    fn parse_recognises_the_export_form_and_leaves_files_alone() {
        assert_eq!(
            parse(&args(&["a.stl", "b.riggen"])).unwrap(),
            Invocation::Open(OpenArgs {
                files: vec!["a.stl".into(), "b.riggen".into()],
                example: None,
                timing: false,
            })
        );
        assert_eq!(
            parse(&args(&[])).unwrap(),
            Invocation::Open(OpenArgs::default())
        );
        let parsed = parse(&args(&[
            "--export", "mjcf", "--out", "target/x", "r.riggen",
        ]))
        .unwrap();
        assert_eq!(
            parsed,
            Invocation::Export(ExportArgs {
                format: Format::MJCF,
                out: "target/x".into(),
                input: "r.riggen".into(),
                fk_samples: false,
            })
        );
        // Order does not matter.
        let Invocation::Export(parsed) = parse(&args(&[
            "r.riggen",
            "--fk-samples",
            "--out",
            "o",
            "--export",
            "both",
        ]))
        .unwrap() else {
            panic!("export form")
        };
        assert_eq!(parsed.format, Format::BOTH);
        assert!(parsed.fk_samples);

        // Every spelling `--help` offers, and the set each names. `both`
        // survives from when there were two writers (ADR-0016 §Consequences).
        for (spelling, expected) in [
            ("mjcf", Format::MJCF),
            ("urdf", Format::URDF),
            ("sdf", Format::SDF),
            ("both", Format::BOTH),
            ("all", Format::ALL),
        ] {
            assert!(Format::NAMES.contains(&spelling));
            let Invocation::Export(parsed) =
                parse(&args(&["--export", spelling, "--out", "o", "r"])).unwrap()
            else {
                panic!("export form")
            };
            assert_eq!(parsed.format, expected, "--export {spelling}");
            // `Display` names the set back with the same word.
            assert_eq!(expected.to_string(), spelling);
        }
        // The unknown one lists them all rather than the two it used to.
        let err = parse(&args(&["--export", "usd", "--out", "o", "r"])).unwrap_err();
        assert!(err.contains("mjcf, urdf, sdf, both, all"), "{err}");
    }

    #[test]
    fn parse_accepts_every_flag_in_its_long_and_short_form() {
        assert_eq!(parse(&args(&["--help"])).unwrap(), Invocation::Help);
        assert_eq!(parse(&args(&["-h"])).unwrap(), Invocation::Help);
        assert_eq!(parse(&args(&["--version"])).unwrap(), Invocation::Version);
        assert_eq!(parse(&args(&["-V"])).unwrap(), Invocation::Version);
        // Help wins over everything else, wherever it is.
        assert_eq!(
            parse(&args(&["--export", "mjcf", "--help"])).unwrap(),
            Invocation::Help
        );
        assert_eq!(
            parse(&args(&["--example", "arm", "extra.stl"])).unwrap(),
            Invocation::Open(OpenArgs {
                files: vec!["extra.stl".into()],
                example: Some(Example::Arm),
                timing: false,
            })
        );
        assert_eq!(
            parse(&args(&["--timing"])).unwrap(),
            Invocation::Open(OpenArgs {
                files: vec![],
                example: None,
                timing: true,
            })
        );
        // Every listed flag parses: a value-taking one with a plausible
        // value, a bare one on its own (in the form that makes it legal).
        for flag in FLAGS {
            let line: Vec<&str> = match (flag.long, flag.value) {
                ("--example", _) => vec!["--example", "arm"],
                ("--export", _) => vec!["--export", "mjcf", "--out", "o", "r"],
                ("--out", _) => vec!["--export", "urdf", "--out", "o", "r"],
                ("--fk-samples", _) => vec!["--export", "both", "--out", "o", "r", "--fk-samples"],
                (long, None) => vec![long],
                (long, Some(_)) => panic!("no test line for the new value flag {long}"),
            };
            parse(&args(&line)).unwrap_or_else(|e| panic!("{line:?}: {e}"));
            if let Some(short) = flag.short {
                parse(&args(&[short])).unwrap_or_else(|e| panic!("{short}: {e}"));
            }
        }
    }

    /// `help()` is generated from `FLAGS`, so a flag the parser knows always
    /// has a help line; this pins that every spelling shows up verbatim.
    #[test]
    fn help_lists_every_flag() {
        let help = help();
        for flag in FLAGS {
            assert!(
                help.contains(flag.long),
                "{} missing from:\n{help}",
                flag.long
            );
            if let Some(short) = flag.short {
                assert!(help.contains(short), "{short} missing from:\n{help}");
            }
            assert!(
                help.contains(flag.doc),
                "{:?} missing from:\n{help}",
                flag.doc
            );
        }
        assert!(help.starts_with("riggen "), "{help}");
        assert!(help.contains("usage:"), "{help}");
        for name in Example::NAMES {
            assert!(help.contains(name), "example {name} missing from:\n{help}");
        }
        // …and every format the parser takes, in both places that spell
        // the list: the usage form and the short `USAGE` a parse error
        // prints. A writer added without touching these is a writer the
        // user cannot find (this is how `sdf` was missed once).
        let spelling = Format::NAMES.join("|");
        for text in [&help, &USAGE.to_owned()] {
            assert!(
                text.contains(&format!("--export {spelling}")),
                "{spelling:?} missing from:\n{text}"
            );
        }
    }

    #[test]
    fn version_has_the_crate_version_hash_and_date() {
        let v = version();
        assert!(
            v.starts_with(&format!("riggen {} (", env!("CARGO_PKG_VERSION"))),
            "{v}"
        );
        assert!(v.ends_with(')'), "{v}");
        let inner = &v[v.find('(').unwrap() + 1..v.len() - 1];
        let (hash, date) = inner.split_once(' ').expect("hash and date");
        let hash_ok = hash == "unknown"
            || hash
                .trim_end_matches("-dirty")
                .chars()
                .all(|c| c.is_ascii_hexdigit());
        assert!(hash_ok, "{v}");
        assert!(date == "unknown" || date.len() == 10, "{v}");
    }

    #[test]
    fn parse_reports_each_mistake_with_the_usage() {
        for bad in [
            vec!["--export"],
            vec!["--export", "usd", "--out", "o", "r"],
            vec!["--export", "mjcf", "r"],
            vec!["--export", "mjcf", "--out", "o"],
            vec!["--export", "mjcf", "--out", "o", "r", "s"],
            vec!["--export", "mjcf", "--out", "o", "r", "--bogus"],
            vec!["--out", "o", "r"],
            vec!["--fk-samples"],
            vec!["--example"],
            vec!["--example", "spaceship"],
            vec!["--example", "arm", "--export", "mjcf", "--out", "o", "r"],
            vec!["--timing", "--export", "mjcf", "--out", "o", "r"],
            vec!["-x"],
        ] {
            let err = parse(&args(&bad)).unwrap_err();
            assert!(err.contains("usage:"), "{bad:?}: {err}");
        }
        // A lone `-` is a file name, as everywhere.
        assert!(matches!(parse(&args(&["-"])).unwrap(), Invocation::Open(_)));
    }

    #[test]
    fn the_arm_example_extracts_five_files_that_load() {
        let temp = std::env::temp_dir().join(format!("riggen-example-test-{}", std::process::id()));
        let document = Example::Arm.extract_into(&temp).unwrap();
        assert_eq!(document, temp.join("riggen-example-arm/arm.riggen"));
        let mut names: Vec<_> = std::fs::read_dir(document.parent().unwrap())
            .unwrap()
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .collect();
        names.sort();
        assert_eq!(
            names,
            [
                "arm.riggen",
                "base.stl",
                "fore.stl",
                "shoulder.stl",
                "upper.stl"
            ]
        );
        // Extracting again over the top is fine.
        Example::Arm.extract_into(&temp).unwrap();
        // The document loads with every mesh beside it.
        let (robot, warnings) = riggen_core::load(&document).unwrap();
        assert!(warnings.is_empty(), "{warnings:?}");
        let (_, errors) = MeshStore::load(&robot);
        assert!(errors.is_empty(), "{errors:?}");
        std::fs::remove_dir_all(&temp).unwrap();
    }

    #[test]
    fn export_of_the_pendulum_fixture_writes_the_files() {
        let fixture =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/fixtures/pendulum.riggen");
        let out = std::env::temp_dir().join(format!("riggen-cli-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&out);
        let written = run(&ExportArgs {
            format: Format::MJCF,
            out: out.clone(),
            input: fixture,
            fk_samples: true,
        })
        .unwrap();
        let names: Vec<String> = written
            .iter()
            .map(|p| p.strip_prefix(&out).unwrap().display().to_string())
            .collect();
        assert_eq!(
            names,
            [
                "pendulum.xml",
                "meshes/cube_ascii.stl",
                "meshes/cube_binary.stl",
                "pendulum.fk.json"
            ]
        );
        let json = std::fs::read_to_string(out.join("pendulum.fk.json")).unwrap();
        assert!(json.contains("\"hinge\""), "{json}");
        let xml = std::fs::read_to_string(out.join("pendulum.xml")).unwrap();
        assert!(
            xml.contains("<joint name=\"hinge\" type=\"hinge\""),
            "{xml}"
        );
        std::fs::remove_dir_all(&out).unwrap();
    }

    #[test]
    fn export_of_the_urdf_fixture_writes_the_files() {
        let fixture =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/fixtures/arm/arm.urdf");
        let out = std::env::temp_dir().join(format!("riggen-cli-urdf-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&out);
        let written = run(&ExportArgs {
            format: Format::MJCF,
            out: out.clone(),
            input: fixture,
            fk_samples: true,
        })
        .unwrap();
        assert!(written.contains(&out.join("arm.xml")));
        assert!(written.contains(&out.join("meshes/fore_hull.stl")));
        assert!(written.contains(&out.join("arm.fk.json")));
        std::fs::remove_dir_all(&out).unwrap();
    }

    /// The plan's acceptance route, in one test: export the arm to MJCF,
    /// then export *that* `.xml` again. The second run is the import
    /// (ADR-0015), and it has to write the same files.
    #[test]
    fn an_mjcf_input_is_imported_and_re_exported() {
        let fixture =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/fixtures/arm/arm.riggen");
        let out = std::env::temp_dir().join(format!("riggen-cli-mjcf-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&out);
        let first = run(&ExportArgs {
            format: Format::MJCF,
            out: out.join("one"),
            input: fixture,
            fk_samples: true,
        })
        .unwrap();
        let second = run(&ExportArgs {
            format: Format::MJCF,
            out: out.join("two"),
            input: out.join("one/arm.xml"),
            fk_samples: true,
        })
        .unwrap();
        let names = |written: &[PathBuf], root: &Path| {
            written
                .iter()
                .map(|p| p.strip_prefix(root).unwrap().to_string_lossy().into_owned())
                .collect::<Vec<_>>()
        };
        assert_eq!(
            names(&first, &out.join("one")),
            names(&second, &out.join("two"))
        );
        // The MJCF is a fixed point but for one line: everything in it
        // survived the read, and the twelve decimals it is written with are
        // already what the document holds the second time round.
        let one = std::fs::read_to_string(out.join("one/arm.xml")).unwrap();
        let two = std::fs::read_to_string(out.join("two/arm.xml")).unwrap();
        let apology = "need an <actuator>";
        let without = |text: &str| {
            text.lines()
                .filter(|l| !l.contains(apology))
                .collect::<Vec<_>>()
                .join("\n")
        };
        assert_eq!(without(&one), without(&two));
        // The line that does differ is the whole of what MJCF cannot hold:
        // `effort` and `velocity` live on an `<actuator>`, and `fore_joint`
        // — a mimic follower — may not have one (ADR-0004 §4 as amended by
        // ADR-0014), so its two numbers were only ever in this comment.
        assert!(
            one.contains("joint fore_joint: effort 5 velocity 3"),
            "{one}"
        );
        assert!(
            two.contains("joint fore_joint: effort 0 velocity 0"),
            "{two}"
        );
        // The FK samples are the oracle both directions share (ADR-0004).
        // Not byte-identical: the first run's joint limits are the
        // document's full precision and the second's are the twelve
        // decimals the file carries, which moves the sampled `q` — and
        // every pose with it — in the last few digits.
        assert_close_json(
            &std::fs::read_to_string(out.join("one/arm.fk.json")).unwrap(),
            &std::fs::read_to_string(out.join("two/arm.fk.json")).unwrap(),
        );
        std::fs::remove_dir_all(&out).unwrap();
    }

    /// Two `--fk-samples` files, equal to 1e-9 — the acceptance tolerance.
    #[track_caller]
    fn assert_close_json(a: &str, b: &str) {
        fn walk(a: &serde_json::Value, b: &serde_json::Value, at: &str) {
            match (a, b) {
                (serde_json::Value::Number(x), serde_json::Value::Number(y)) => {
                    let (x, y) = (x.as_f64().unwrap(), y.as_f64().unwrap());
                    assert!((x - y).abs() < 1e-9, "{at}: {x} vs {y}");
                }
                (serde_json::Value::Array(x), serde_json::Value::Array(y)) => {
                    assert_eq!(x.len(), y.len(), "{at}");
                    for (i, (x, y)) in x.iter().zip(y).enumerate() {
                        walk(x, y, &format!("{at}[{i}]"));
                    }
                }
                (serde_json::Value::Object(x), serde_json::Value::Object(y)) => {
                    assert_eq!(
                        x.keys().collect::<Vec<_>>(),
                        y.keys().collect::<Vec<_>>(),
                        "{at}"
                    );
                    for (k, x) in x {
                        walk(x, &y[k], &format!("{at}.{k}"));
                    }
                }
                _ => assert_eq!(a, b, "{at}"),
            }
        }
        walk(
            &serde_json::from_str(a).unwrap(),
            &serde_json::from_str(b).unwrap(),
            "",
        );
    }

    #[test]
    fn a_missing_input_is_an_error_not_a_panic() {
        let err = run(&ExportArgs {
            format: Format::MJCF,
            out: std::env::temp_dir(),
            input: "/nowhere/none.riggen".into(),
            fk_samples: false,
        })
        .unwrap_err();
        assert!(err.contains("none.riggen"), "{err}");
    }
}
