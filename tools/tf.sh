#!/usr/bin/env bash
# Terraform pinned to infra/ and to the local k3d cluster. State stays inside the project.
# Usage: tools/tf.sh <terraform args...>   (apply/destroy need the user's confirmation)
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
command -v terraform >/dev/null 2>&1 || { echo "terraform is not installed" >&2; exit 127; }
[[ -d "$REPO/infra" ]] || { echo "infra/ does not exist yet (task T911)" >&2; exit 1; }
[[ -f "$REPO/.k3s/kubeconfig" ]] || { echo "local cluster missing; run: tools/k3s.sh up" >&2; exit 1; }

export KUBE_CONFIG_PATH="$REPO/.k3s/kubeconfig"
export KUBE_CTX="k3d-aspirador"
cd "$REPO/infra"
exec terraform "$@"
