//! The commit a registry build came from, for `riggen --version`.
//!
//! `cargo package` writes `.cargo_vcs_info.json` into the crate it packs:
//! `{"git": {"sha1": "…", "dirty": true}, "path_in_vcs": "…"}`, with
//! `dirty` only under `--allow-dirty`. A crate built from crates.io has
//! this file and no `.git`. `build.rs` includes this module by path, and
//! the lib compiles it under `cfg(test)` so the parsing is tested
//! (docs/ARCHITECTURE.md §Crates.io distribution). The format is fixed and
//! small, so a string search stands in for a JSON parser that a build
//! script would have to depend on.

/// The short hash (7 characters) from the file's contents, with `-dirty`
/// when Cargo packed an unclean tree; `None` when there is no `sha1`.
pub fn short_hash(json: &str) -> Option<String> {
    let sha = string_field(json, "sha1")?;
    if sha.len() < 7 || !sha.bytes().all(|b| b.is_ascii_hexdigit()) {
        return None;
    }
    let dirty = json.split_once("\"dirty\"").is_some_and(|(_, rest)| {
        rest.trim_start()
            .trim_start_matches(':')
            .trim_start()
            .starts_with("true")
    });
    let short = &sha[..7];
    Some(if dirty {
        format!("{short}-dirty")
    } else {
        short.to_string()
    })
}

/// The string value of `"key": "value"`, the first occurrence.
fn string_field<'a>(json: &'a str, key: &str) -> Option<&'a str> {
    let (_, rest) = json.split_once(&format!("\"{key}\""))?;
    let rest = rest.trim_start().strip_prefix(':')?.trim_start();
    let rest = rest.strip_prefix('"')?;
    Some(&rest[..rest.find('"')?])
}

#[cfg(test)]
mod tests {
    use super::short_hash;

    #[test]
    fn a_packaged_crate_names_its_commit() {
        let json = r#"{
  "git": {
    "sha1": "48f1845d0c3e2b9a7f6e5d4c3b2a1908f7e6d5c4"
  },
  "path_in_vcs": "crates/riggen-app"
}"#;
        assert_eq!(short_hash(json).as_deref(), Some("48f1845"));
    }

    #[test]
    fn an_allow_dirty_package_says_so() {
        let json = r#"{"git":{"sha1":"48f1845d0c3e2b9a7f6e5d4c3b2a1908f7e6d5c4","dirty":true},"path_in_vcs":""}"#;
        assert_eq!(short_hash(json).as_deref(), Some("48f1845-dirty"));
        let clean = json.replace("true", "false");
        assert_eq!(short_hash(&clean).as_deref(), Some("48f1845"));
    }

    #[test]
    fn no_hash_is_none_not_a_guess() {
        assert_eq!(short_hash(r#"{"path_in_vcs": ""}"#), None);
        assert_eq!(short_hash(r#"{"git": {"sha1": "xyz"}}"#), None);
        assert_eq!(short_hash(""), None);
    }
}
