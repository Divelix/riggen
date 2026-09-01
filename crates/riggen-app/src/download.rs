//! Getting files *out* of a browser (ADR-0017 §6).
//!
//! There is nowhere to write, so the only way out is a download the user
//! accepts: a `Blob`, an object URL, and a synthetic click on an
//! `<a download>`. An export is a directory, and a download is one file, so
//! the directory becomes a **stored** zip — no compression, because these
//! are STL and XML going straight into a simulator and the seconds are
//! better spent elsewhere.
//!
//! [`stored_zip`] is built for every target, not only wasm: the test that
//! reads the archive back and compares it to `export_files` is native.

use std::io::{Cursor, Write as _};
use std::path::Path;

/// The MIME types the three downloads carry. A `.riggen` is JSON, and a
/// browser that is told so shows it rather than warning about an unknown
/// type.
#[cfg(target_arch = "wasm32")]
pub(crate) const JSON: &str = "application/json";
#[cfg(target_arch = "wasm32")]
pub(crate) const ZIP: &str = "application/zip";

/// `files` as one uncompressed zip, entry names relative to `root`.
///
/// A file outside `root` keeps its own name, which cannot happen for an
/// export — every path `export_files` returns is under the directory it was
/// given — but a zip with an absolute entry name is a trap, so the fallback
/// is the file name rather than the path.
pub(crate) fn stored_zip(files: &[(std::path::PathBuf, Vec<u8>)], root: &Path) -> Vec<u8> {
    let mut writer = zip::ZipWriter::new(Cursor::new(Vec::new()));
    let options = zip::write::SimpleFileOptions::default()
        .compression_method(zip::CompressionMethod::Stored)
        // The archive is generated, not archived from disk: a
        // last-modified time would make two exports of one document differ.
        .last_modified_time(zip::DateTime::default());
    for (path, bytes) in files {
        let name = path
            .strip_prefix(root)
            .unwrap_or_else(|_| Path::new(path.file_name().unwrap_or(path.as_os_str())));
        // Zip entry names are `/`-separated by the format, whatever the
        // host's separator is.
        let name = name
            .components()
            .map(|c| c.as_os_str().to_string_lossy())
            .collect::<Vec<_>>()
            .join("/");
        writer
            .start_file(name, options)
            .and_then(|()| writer.write_all(bytes).map_err(Into::into))
            .expect("writing a stored zip into memory cannot fail");
    }
    writer
        .finish()
        .expect("finishing a stored zip in memory cannot fail")
        .into_inner()
}

/// Hands the browser `bytes` as a file called `name`.
#[cfg(target_arch = "wasm32")]
pub(crate) fn offer(name: &str, bytes: &[u8], mime: &str) -> Result<(), String> {
    use wasm_bindgen::JsCast as _;

    fn js(what: &'static str) -> impl Fn(wasm_bindgen::JsValue) -> String {
        move |_| format!("the browser refused to {what}")
    }
    let array = js_sys::Uint8Array::from(bytes);
    let parts = js_sys::Array::new();
    parts.push(&array);
    let properties = web_sys::BlobPropertyBag::new();
    properties.set_type(mime);
    let blob = web_sys::Blob::new_with_u8_array_sequence_and_options(&parts, &properties)
        .map_err(js("build the file"))?;
    let url = web_sys::Url::create_object_url_with_blob(&blob).map_err(js("name the file"))?;

    let document = web_sys::window()
        .and_then(|w| w.document())
        .ok_or("there is no page to download from")?;
    let anchor = document
        .create_element("a")
        .map_err(js("make a link"))?
        .dyn_into::<web_sys::HtmlAnchorElement>()
        .map_err(|_| "that was not a link".to_owned())?;
    anchor.set_href(&url);
    anchor.set_download(name);
    anchor.click();
    // The blob stays alive until this is revoked, and the click has already
    // taken its copy of the URL.
    let _ = web_sys::Url::revoke_object_url(&url);
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    /// The archive is the export directory (ADR-0017 §6): every file
    /// `export_files` produced, under its path relative to the export
    /// directory, byte for byte, and readable by a zip reader that never
    /// saw ours.
    #[test]
    fn the_zip_is_the_export_directory() {
        let (robot, _) = riggen_core::load(
            &Path::new(env!("CARGO_MANIFEST_DIR")).join("../../assets/fixtures/arm/arm.riggen"),
        )
        .unwrap();
        let (store, errors) = riggen_export::MeshStore::load(&robot, &riggen_core::Disk);
        assert!(errors.is_empty(), "{errors:?}");
        let options = riggen_export::ExportOptions::default();
        let resolved =
            riggen_export::resolve(&robot, &store, &riggen_export::ComputeNow, &options).unwrap();

        let root = PathBuf::from("/export");
        let files = riggen_export::export_files(&resolved, &options, &root);
        let archive = stored_zip(&files, &root);

        let mut zip = zip::ZipArchive::new(Cursor::new(archive)).unwrap();
        assert_eq!(zip.len(), files.len());
        for (path, bytes) in &files {
            let name = path
                .strip_prefix(&root)
                .unwrap()
                .to_string_lossy()
                .replace('\\', "/");
            let mut entry = zip.by_name(&name).unwrap_or_else(|e| panic!("{name}: {e}"));
            assert_eq!(entry.compression(), zip::CompressionMethod::Stored);
            let mut read = Vec::new();
            std::io::Read::read_to_end(&mut entry, &mut read).unwrap();
            assert_eq!(&read, bytes, "{name}");
        }
        // The names are the directory's, not the host's absolute paths.
        let names: Vec<String> = zip.file_names().map(str::to_owned).collect();
        assert!(names.contains(&"arm.xml".to_owned()), "{names:?}");
        assert!(names.contains(&"meshes/base.stl".to_owned()), "{names:?}");
        assert!(!names.iter().any(|n| n.starts_with('/')), "{names:?}");
    }

    /// Two exports of one document are the same bytes: nothing in the
    /// archive is a clock reading.
    #[test]
    fn the_zip_is_reproducible() {
        let files = vec![
            (PathBuf::from("/export/a.xml"), b"<a/>".to_vec()),
            (PathBuf::from("/export/meshes/b.stl"), vec![0u8; 32]),
        ];
        let root = Path::new("/export");
        assert_eq!(stored_zip(&files, root), stored_zip(&files, root));
    }
}
