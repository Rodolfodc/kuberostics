#!/usr/bin/env bash
# Regenerates docs/diagrams/render/*.svg from docs/diagrams/*.mmd with the official mermaid-cli Docker image
# (Docker-first: no Node.js or Chromium on the host). Run it whenever a .mmd file changes and commit the SVGs.
# Usage: scripts/render-diagrams.sh
# Writes both .svg and .png (scale 2). puppeteer-config.json points at the image Chromium and disables its sandbox (it cannot run inside a container).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${MERMAID_IMAGE:-minlag/mermaid-cli:12.0.1}"
mkdir -p "$REPO/docs/diagrams/render"

shopt -s nullglob
for src in "$REPO"/docs/diagrams/*.mmd; do
  name="$(basename "${src%.mmd}")"
  echo ">> $name"
  # mermaid-config.json: htmlLabels false, so labels are plain SVG text (no foreignObject) and open in any viewer.
  for ext in svg png; do
    docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$REPO/docs/diagrams":/data "$IMAGE" \
      -p /data/puppeteer-config.json -c /data/mermaid-config.json -s 2 \
      -i "/data/$name.mmd" -o "/data/render/$name.$ext" >/dev/null
  done
done
echo "done: $(ls "$REPO"/docs/diagrams/render/*.svg | wc -l) SVG files, $(ls "$REPO"/docs/diagrams/render/*.png | wc -l) PNG files"
