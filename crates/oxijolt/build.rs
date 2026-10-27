//! Sets `cfg(repository_docs)` when the crate builds inside its repository, where the README and
//! the guides in `docs/` sit next to it. Their examples run as doctests through items in
//! `src/lib.rs` that include those files, so the published package, which ships neither, leaves
//! them out.

use std::path::Path;

fn main() {
    println!("cargo::rerun-if-changed=build.rs");
    println!("cargo::rustc-check-cfg=cfg(repository_docs)");
    let repository = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    if repository.join("README.md").is_file() && repository.join("docs/guide.md").is_file() {
        println!("cargo::rustc-cfg=repository_docs");
    }
}
