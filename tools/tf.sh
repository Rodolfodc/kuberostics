#!/usr/bin/env bash
# Terraform pinned to infra/ and to the local k3d cluster. State stays inside the project.
# Usage: tools/tf.sh <terraform args...>   (apply/destroy need the user's confirmation)
# Offline verbs (fmt, validate, version, init -backend=false) need no cluster. Everything else needs `tools/k3s.sh up`.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
command -v terraform >/dev/null 2>&1 || { echo "terraform is not installed" >&2; exit 127; }
[[ -d "$REPO/infra" ]] || { echo "infra/ does not exist" >&2; exit 1; }

# Offline verbs never contact a cluster, so they run without the local kubeconfig: fmt, validate, version, init -backend=false.
OFFLINE=0
case "${1:-}" in
  fmt|validate|version) OFFLINE=1;;
  init) [[ " $* " == *" -backend=false "* ]] && OFFLINE=1;;
esac
if [[ $OFFLINE -eq 0 && ! -f "$REPO/.k3s/kubeconfig" ]]; then
  echo "local cluster missing; run: tools/k3s.sh up" >&2; exit 1
fi

export KUBE_CONFIG_PATH="$REPO/.k3s/kubeconfig"
export KUBE_CTX="k3d-kuberostics"
cd "$REPO/infra"
exec terraform "$@"
