//! `riggen --export mjcf|urdf|both --out DIR INPUT`: the headless export
//! (ADR-0008), which returns before eframe starts. It is what CI's `mujoco`
//! job runs, so it must need no display. `INPUT` is a `.riggen` document
//! (a `.urdf` from step 13).

use std::ffi::OsString;
use std::path::PathBuf;

use riggen_export::{ExportOptions, Format, MeshStore};

pub const USAGE: &str = "usage: riggen --export mjcf|urdf|both --out DIR INPUT";

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExportArgs {
    pub format: Format,
    pub out: PathBuf,
    pub input: PathBuf,
}

/// `Ok(None)` when the arguments are not an export invocation (the GUI
/// opens them as files); `Err` when they try to be one and fail.
pub fn parse(args: &[OsString]) -> Result<Option<ExportArgs>, String> {
    if !args.iter().any(|a| a == "--export") {
        return Ok(None);
    }
    let mut format = None;
    let mut out = None;
    let mut input = None;
    let mut it = args.iter();
    while let Some(arg) = it.next() {
        match arg.to_str() {
            Some("--export") => {
                format = Some(match it.next().and_then(|v| v.to_str()) {
                    Some("mjcf") => Format::Mjcf,
                    Some("urdf") => Format::Urdf,
                    Some("both") => Format::Both,
                    other => {
                        return Err(format!(
                            "--export expects mjcf, urdf or both, got {other:?}\n{USAGE}"
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
            Some(flag) if flag.starts_with("--") => {
                return Err(format!("unknown flag {flag}\n{USAGE}"));
            }
            _ => {
                if input.replace(PathBuf::from(arg)).is_some() {
                    return Err(format!("only one INPUT is exported at a time\n{USAGE}"));
                }
            }
        }
    }
    Ok(Some(ExportArgs {
        format: format.ok_or(USAGE)?,
        out: out.ok_or(format!("--out is required\n{USAGE}"))?,
        input: input.ok_or(format!("INPUT is required\n{USAGE}"))?,
    }))
}

/// Loads, resolves and writes. The `Err` is what the user reads on stderr:
/// every resolve error, one per line.
pub fn run(args: &ExportArgs) -> Result<Vec<PathBuf>, String> {
    let (robot, warnings) = riggen_core::load(&args.input).map_err(|e| e.to_string())?;
    for w in &warnings {
        eprintln!("warning: {w}");
    }
    let (store, load_errors) = MeshStore::load(&robot);
    let options = ExportOptions {
        format: args.format,
        ..Default::default()
    };
    let resolved = match riggen_export::resolve(&robot, &store, &options) {
        Ok(r) if load_errors.is_empty() => r,
        Ok(_) => return Err(join_errors(&load_errors)),
        Err(mut errors) => {
            errors.extend(load_errors);
            return Err(join_errors(&errors));
        }
    };
    riggen_export::export(&resolved, &options, &args.out).map_err(|e| e.to_string())
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
        assert_eq!(parse(&args(&["a.stl", "b.riggen"])).unwrap(), None);
        assert_eq!(parse(&args(&[])).unwrap(), None);
        let parsed = parse(&args(&[
            "--export", "mjcf", "--out", "target/x", "r.riggen",
        ]))
        .unwrap()
        .unwrap();
        assert_eq!(
            parsed,
            ExportArgs {
                format: Format::Mjcf,
                out: "target/x".into(),
                input: "r.riggen".into(),
            }
        );
        // Order does not matter.
        let parsed = parse(&args(&["r.riggen", "--out", "o", "--export", "both"]))
            .unwrap()
            .unwrap();
        assert_eq!(parsed.format, Format::Both);
    }

    #[test]
    fn parse_reports_each_mistake_with_the_usage() {
        for bad in [
            vec!["--export"],
            vec!["--export", "sdf", "--out", "o", "r"],
            vec!["--export", "mjcf", "r"],
            vec!["--export", "mjcf", "--out", "o"],
            vec!["--export", "mjcf", "--out", "o", "r", "s"],
            vec!["--export", "mjcf", "--out", "o", "r", "--bogus"],
        ] {
            let err = parse(&args(&bad)).unwrap_err();
            assert!(err.contains("usage:"), "{bad:?}: {err}");
        }
    }

    #[test]
    fn export_of_the_pendulum_fixture_writes_the_files() {
        let fixture =
            Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/fixtures/pendulum.riggen");
        let out = std::env::temp_dir().join(format!("riggen-cli-{}", std::process::id()));
        let _ = std::fs::remove_dir_all(&out);
        let written = run(&ExportArgs {
            format: Format::Mjcf,
            out: out.clone(),
            input: fixture,
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
                "meshes/cube_binary.stl"
            ]
        );
        let xml = std::fs::read_to_string(out.join("pendulum.xml")).unwrap();
        assert!(
            xml.contains("<joint name=\"hinge\" type=\"hinge\""),
            "{xml}"
        );
        std::fs::remove_dir_all(&out).unwrap();
    }

    #[test]
    fn a_missing_input_is_an_error_not_a_panic() {
        let err = run(&ExportArgs {
            format: Format::Mjcf,
            out: std::env::temp_dir(),
            input: "/nowhere/none.riggen".into(),
        })
        .unwrap_err();
        assert!(err.contains("none.riggen"), "{err}");
    }
}
