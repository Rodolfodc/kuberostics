#!/usr/bin/env bash
# (009a-R17, R19; Docker only, no cluster, no apply) Validates the cloud side offline:
#   1. terraform fmt -check, init -backend=false, validate (infra/), in a pinned hashicorp/terraform image;
#   2. renders the manifests for several variants (transport bridge|rmw_zenoh, cloud sync on|off) with `terraform console`;
#   3. checks the policy rules in scripts/check_cloud_manifests.py (livenessProbe on every container, no privileged or
#      hostPath, no secret values, R19 allow-lists = the two perception topics of contracts/topics.yaml, nothing with mux or
#      cmd_vel) and validates each rendered object with kubeconform (-strict).
# Usage: scripts/check-cloud.sh [--update-lock]   (--update-lock rewrites infra/.terraform.lock.hcl for linux and macOS)
# Rendered output goes to .tf-render/ (gitignored). The repo is mounted read-only except infra/ for --update-lock.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TF_IMG="${TF_IMG:-hashicorp/terraform:1.13.5}"
KUBECONFORM_IMG="${KUBECONFORM_IMG:-ghcr.io/yannh/kubeconform:v0.7.0}"
PY_IMG="${PY_IMG:-python:3.13-slim}"
OUT="$REPO/.tf-render"

# name|terraform -var flags
VARIANTS=(
  "bridge-jazzy|-var=ros_distro=jazzy -var=transport=bridge -var=enable_cloud_sync=true"
  "bridge-lyrical|-var=ros_distro=lyrical -var=transport=bridge -var=weights_url=https://example.invalid/yolo11x.pt"
  "rmw-zenoh-jazzy|-var=ros_distro=jazzy -var=transport=rmw_zenoh"
)

if [[ "${1:-}" == "--update-lock" ]]; then
  echo ">> updating infra/.terraform.lock.hcl"
  docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$REPO/infra":/work -w /work --entrypoint sh "$TF_IMG" -c '
    terraform init -backend=false -input=false >/dev/null
    terraform providers lock -platform=linux_amd64 -platform=linux_arm64 -platform=darwin_arm64 -platform=darwin_amd64
    rm -rf .terraform'
  exit 0
fi

rm -rf "$OUT"; mkdir -p "$OUT"
echo ">> terraform fmt, init, validate, render ($TF_IMG)"
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -v "$REPO":/repo:ro -v "$OUT":/out -e TF_IN_AUTOMATION=1 --entrypoint sh "$TF_IMG" -c '
  set -eu; set -o pipefail
  mkdir /tmp/work && cp -r /repo/infra /repo/deploy /tmp/work/ && cd /tmp/work/infra
  terraform fmt -check -recursive -diff
  terraform init -backend=false -input=false >/dev/null
  terraform validate
  for spec in "$@"; do
    name="${spec%%|*}"; flags="${spec#*|}"
    echo "jsonencode(module.workloads.manifests)" | terraform console $flags > "/out/$name.raw"
  done
' sh "${VARIANTS[@]}"

echo ">> policy checks and object split ($PY_IMG)"
docker run --rm --user "$(id -u):$(id -g)" -v "$REPO":/repo:ro -v "$OUT":/out "$PY_IMG" python3 -I /repo/scripts/check_cloud_manifests.py \
  /out /repo/specs/000-system/contracts/topics.yaml /repo/deploy/bridge

echo ">> kubeconform ($KUBECONFORM_IMG)"
docker run --rm -v "$OUT":/out:ro "$KUBECONFORM_IMG" -strict -summary -kubernetes-version 1.31.0 /out/objects

echo ">> cloud checks passed"
