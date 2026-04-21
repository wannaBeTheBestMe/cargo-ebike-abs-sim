#!/usr/bin/env bash
set -euo pipefail

# Regenerate docs/block-diagram.svg and docs/block-diagram.png from the
# hand-authored docs/block-diagram.dot. Requires `dot` (graphviz) on PATH;
# install with `brew install graphviz` on macOS.

cd "$(dirname "$0")/.."

dot -Tsvg docs/block-diagram.dot -o docs/block-diagram.svg
dot -Tpng -Gdpi=200 docs/block-diagram.dot -o docs/block-diagram.png

echo "wrote docs/block-diagram.svg docs/block-diagram.png"
