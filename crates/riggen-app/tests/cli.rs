//! The built `riggen` binary from the outside: what a shell sees.
//!
//! `CARGO_BIN_EXE_riggen` is the path cargo built for this test run, so
//! these exercise `main.rs`'s dispatch — exit codes and streams — which the
//! unit tests in `cli.rs` cannot.

use std::process::Command;

fn riggen(args: &[&str]) -> std::process::Output {
    Command::new(env!("CARGO_BIN_EXE_riggen"))
        .args(args)
        .output()
        .expect("run riggen")
}

#[test]
fn version_prints_one_line_with_the_hash() {
    for flag in ["--version", "-V"] {
        let out = riggen(&[flag]);
        assert!(out.status.success(), "{out:?}");
        let text = String::from_utf8(out.stdout).unwrap();
        assert_eq!(text.lines().count(), 1, "{text}");
        assert_eq!(text, format!("{}\n", riggen_app::cli::version()));
        assert!(
            text.starts_with(&format!("riggen {} (", env!("CARGO_PKG_VERSION"))),
            "{text}"
        );
        assert!(out.stderr.is_empty());
    }
}

#[test]
fn help_goes_to_stdout_and_exits_zero() {
    for flag in ["--help", "-h"] {
        let out = riggen(&[flag]);
        assert!(out.status.success(), "{out:?}");
        assert_eq!(
            String::from_utf8(out.stdout).unwrap(),
            riggen_app::cli::help()
        );
    }
}

#[test]
fn a_bad_flag_exits_two_with_the_usage_on_stderr() {
    let out = riggen(&["--bogus"]);
    assert_eq!(out.status.code(), Some(2));
    assert!(out.stdout.is_empty());
    let err = String::from_utf8(out.stderr).unwrap();
    assert!(err.contains("unknown flag --bogus"), "{err}");
    assert!(err.contains("usage:"), "{err}");
}

/// A static link exported without mass is a `warning:` line on stderr, one
/// per link, and the export still succeeds (ADR-0032 §2). The import corpus
/// has one: `tool`, a mesh and no `<inertial>`. Copied out of the tree
/// first, because importing it writes its inline mesh beside it.
#[test]
fn a_static_link_without_mass_is_a_warning_and_the_export_succeeds() {
    let fixtures = std::path::Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/fixtures");
    let dir = std::env::temp_dir().join(format!("riggen-cli-massless-{}", std::process::id()));
    let _ = std::fs::remove_dir_all(&dir);
    std::fs::create_dir_all(dir.join("arm")).unwrap();
    for name in ["menagerie_style.xml", "menagerie_style_arm.xml"] {
        std::fs::copy(fixtures.join(name), dir.join(name)).unwrap();
    }
    for name in ["base.stl", "shoulder.stl", "thing.msh"] {
        std::fs::copy(fixtures.join("arm").join(name), dir.join("arm").join(name)).unwrap();
    }
    let out_dir = dir.join("out");
    let out = riggen(&[
        "--export",
        "mjcf",
        "--out",
        out_dir.to_str().unwrap(),
        dir.join("menagerie_style.xml").to_str().unwrap(),
    ]);
    assert!(out.status.success(), "{out:?}");
    let err = String::from_utf8(out.stderr).unwrap();
    let massless: Vec<&str> = err
        .lines()
        .filter(|l| l.contains("carries no mass"))
        .collect();
    assert_eq!(
        massless,
        ["warning: link \"tool\" is static and carries no mass; written without <inertial>"],
        "{err}"
    );
    let xml = std::fs::read_to_string(out_dir.join("menagerie_style.xml")).unwrap();
    assert!(xml.contains("<body name=\"tool\""), "{xml}");
    let _ = std::fs::remove_dir_all(&dir);
}
