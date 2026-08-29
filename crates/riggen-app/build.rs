//! Records which commit the binary was built from, for `riggen --version`
//! (`riggen 0.1.0 (2b60ae4 2026-08-29)`).
//!
//! Three sources, first one wins: the `RIGGEN_GIT_HASH` / `RIGGEN_BUILD_DATE`
//! environment variables (the release workflow sets them from `github.sha`;
//! a build from the sdist has no `.git` to ask), then `git` on the checkout
//! (`-dirty` appended when the tree has uncommitted changes), then
//! `unknown` for the hash and today's date for the date. Never fails the
//! build: a missing git is a fact to report, not an error.

use std::path::Path;
use std::process::Command;

fn main() {
    println!("cargo:rerun-if-env-changed=RIGGEN_GIT_HASH");
    println!("cargo:rerun-if-env-changed=RIGGEN_BUILD_DATE");
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    // Rebuild when HEAD moves: `.git/HEAD` itself, and the ref it points at
    // (on a branch, HEAD's content never changes; the ref file does).
    let head = root.join(".git/HEAD");
    if head.is_file() {
        println!("cargo:rerun-if-changed={}", head.display());
        if let Ok(content) = std::fs::read_to_string(&head)
            && let Some(reference) = content.trim().strip_prefix("ref: ")
        {
            println!(
                "cargo:rerun-if-changed={}",
                root.join(".git").join(reference).display()
            );
        }
    }

    let hash = std::env::var("RIGGEN_GIT_HASH")
        .ok()
        .filter(|s| !s.is_empty())
        .or_else(|| {
            let short = git(&root, &["rev-parse", "--short", "HEAD"])?;
            let dirty = Command::new("git")
                .current_dir(&root)
                .args(["diff", "--quiet", "HEAD"])
                .status()
                .map(|s| !s.success())
                .unwrap_or(false);
            Some(if dirty {
                format!("{short}-dirty")
            } else {
                short
            })
        })
        .unwrap_or_else(|| "unknown".into());
    let date = std::env::var("RIGGEN_BUILD_DATE")
        .ok()
        .filter(|s| !s.is_empty())
        .or_else(|| git(&root, &["log", "-1", "--format=%cs"]))
        .unwrap_or_else(today);
    println!("cargo:rustc-env=RIGGEN_GIT_HASH={hash}");
    println!("cargo:rustc-env=RIGGEN_BUILD_DATE={date}");
}

/// `YYYY-MM-DD` in UTC from the system clock, for a build with no git to
/// ask (the sdist). Howard Hinnant's `civil_from_days`; no chrono for a
/// build script.
fn today() -> String {
    let secs = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0);
    let z = secs.div_euclid(86_400) + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z - era * 146_097;
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = doy - (153 * mp + 2) / 5 + 1;
    let m = if mp < 10 { mp + 3 } else { mp - 9 };
    let y = if m <= 2 { y + 1 } else { y };
    format!("{y:04}-{m:02}-{d:02}")
}

fn git(root: &Path, args: &[&str]) -> Option<String> {
    let output = Command::new("git")
        .current_dir(root)
        .args(args)
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    let text = String::from_utf8(output.stdout).ok()?.trim().to_string();
    (!text.is_empty()).then_some(text)
}
