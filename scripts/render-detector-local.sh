#!/usr/bin/env bash
# Renders deploy/k8s/detector.yaml.tpl (the same template Terraform applies) into a kubectl List for the LOCAL k3d cluster.
# Read-only: it evaluates the module with `terraform console` in a pinned container; it applies nothing and needs no cluster.
# The namespace object is added here (in the real module it is a Terraform resource).
# Usage: scripts/render-detector-local.sh [--distro jazzy] [--model yolo11x.pt] [--weights-url URL] [--out FILE]
# Output default: tools/chaos/local-logs/detector-local.json (gitignored). Apply it with: tools/k3s.sh apply -f FILE
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TF_IMG="${TF_IMG:-hashicorp/terraform:1.13.5}"
DISTRO=jazzy MODEL=yolo11x.pt URL="" OUT="$REPO/tools/chaos/local-logs/detector-local.json"
while [[ $# -gt 0 ]]; do
  case "$1" in
    --distro) DISTRO="$2"; shift 2;;
    --model) MODEL="$2"; shift 2;;
    --weights-url) URL="$2"; shift 2;;
    --out) OUT="$2"; shift 2;;
    *) echo "unknown arg $1" >&2; exit 2;;
  esac
done
mkdir -p "$(dirname "$OUT")"
RAW="$(docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -e TF_IN_AUTOMATION=1 -v "$REPO":/repo:ro --entrypoint sh "$TF_IMG" -c '
  set -eu; mkdir /tmp/work && cp -r /repo/infra /repo/deploy /tmp/work/ && cd /tmp/work/infra
  terraform init -backend=false -input=false >/dev/null
  echo "jsonencode(module.workloads.manifests)" | terraform console \
    -var=ros_distro='"$DISTRO"' -var=transport=bridge -var=model_file='"$MODEL"' -var=weights_url='"$URL"'')"
python3 -I - "$OUT" "$RAW" <<'EOF'
import json, sys
out, raw = sys.argv[1], sys.argv[2]
manifests = json.loads(json.loads(raw))
items = [{"apiVersion": "v1", "kind": "Namespace", "metadata": {"name": "vacuum-cloud"}}] + list(manifests.values())
json.dump({"apiVersion": "v1", "kind": "List", "items": items}, open(out, "w"), indent=1)
print(f"rendered {len(items)} objects -> {out}")
EOF
