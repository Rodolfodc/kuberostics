#!/usr/bin/env bash
# kubectl pinned to the local k3d cluster "aspirador". Never touches the user's own kubeconfig.
# Usage: tools/k3s.sh up | down | <kubectl args...>
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
NAME="aspirador"
export KUBECONFIG="$REPO/.k3s/kubeconfig"
mkdir -p "$REPO/.k3s"

need() { command -v "$1" >/dev/null 2>&1 || { echo "$1 is not installed (needed for the local K3s cluster)" >&2; exit 127; }; }

case "${1:-}" in
  up)   need k3d; k3d cluster list "$NAME" >/dev/null 2>&1 || k3d cluster create "$NAME" --kubeconfig-update-default=false --kubeconfig-switch-context=false
        k3d kubeconfig get "$NAME" >"$KUBECONFIG"; chmod 600 "$KUBECONFIG";;
  down) need k3d; k3d cluster delete "$NAME"; rm -f "$KUBECONFIG";;
  "")   echo "usage: $0 up|down|<kubectl args>" >&2; exit 2;;
  *)    need kubectl
        [[ -f "$KUBECONFIG" ]] || { echo "cluster not created; run: tools/k3s.sh up" >&2; exit 1; }
        exec kubectl --context "k3d-$NAME" "$@";;
esac
