#!/usr/bin/env bash
# Build the web demo into `web/dist/` (docs/01-architecture.md §The web
# build, ADR-0017).
#
#   web/build.sh            release, the bundle that is deployed
#   web/build.sh --dev      an unoptimized build, for a fast local loop
#
# The result is static: `python3 -m http.server -d web/dist` serves it, and
# so does GitHub Pages. `wasm-bindgen-cli` is pinned to the `wasm-bindgen`
# version in `Cargo.lock` — the two halves of the ABI have to agree, and a
# mismatch fails at load with an unhelpful message.
set -euo pipefail

here=$(cd "$(dirname "$0")" && pwd)
root=$(dirname "$here")
dist="$here/dist"

# The `web` profile: `opt-level = "s"` and fat LTO, because a browser pays
# for the download (root Cargo.toml).
profile=web
cargo_profile_flag=(--profile web)
if [[ ${1:-} == "--dev" ]]; then
  profile=debug
  cargo_profile_flag=()
elif [[ $# -gt 0 ]]; then
  echo "usage: $0 [--dev]" >&2
  exit 2
fi

# The version cargo will link, straight out of the lock file: one place to
# change it, and no chance of the script and the build disagreeing.
wanted=$(
  awk '/^name = "wasm-bindgen"$/ { getline; gsub(/[",]/, "", $3); print $3; exit }' \
    "$root/Cargo.lock"
)
have=$(wasm-bindgen --version 2>/dev/null | awk '{print $2}' || true)
if [[ $have != "$wanted" ]]; then
  echo "installing wasm-bindgen-cli $wanted (have: ${have:-none})" >&2
  cargo install wasm-bindgen-cli --version "$wanted" --locked
fi

cargo build "${cargo_profile_flag[@]}" --target wasm32-unknown-unknown -p riggen-app \
  --manifest-path "$root/Cargo.toml"

rm -rf "$dist"
mkdir -p "$dist"
wasm-bindgen \
  --target web \
  --no-typescript \
  --out-dir "$dist" \
  "$root/target/wasm32-unknown-unknown/$profile/riggen_app.wasm"

cp "$here/index.html" "$here/main.js" "$dist/"

echo "web/dist:"
ls -lh "$dist" | tail -n +2
