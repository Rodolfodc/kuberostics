#!/usr/bin/env bash
# Regenerates docs/diagrams/render/*.svg from docs/diagrams/*.mmd with the official mermaid-cli Docker image
# (Docker-first: no Node.js or Chromium on the host). Run it whenever a .mmd file changes and commit the SVGs.
# Usage: scripts/render-diagrams.sh
# puppeteer-config.json points at the image Chromium and disables its sandbox (it cannot run inside a container).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${MERMAID_IMAGE:-minlag/mermaid-cli:12.0.1}"
mkdir -p "$REPO/docs/diagrams/render"

shopt -s nullglob
for src in "$REPO"/docs/diagrams/*.mmd; do
  name="$(basename "${src%.mmd}")"
  echo ">> $name"
  docker run --rm -u "$(id -u):$(id -g)" -e HOME=/tmp -v "$REPO/docs/diagrams":/data "$IMAGE" \
    -p /data/puppeteer-config.json -i "/data/$name.mmd" -o "/data/render/$name.svg" >/dev/null
done
echo "done: $(ls "$REPO"/docs/diagrams/render/*.svg | wc -l) SVG files"
