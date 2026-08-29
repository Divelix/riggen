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
