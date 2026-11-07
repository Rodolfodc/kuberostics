#!/usr/bin/env bash
# Installs kubectl, k3d and terraform into /usr/local/bin and closes the CoppeliaSim
# Remote API ports (23000-23099) to non-loopback traffic.
# Usage: sudo scripts/setup-tools.sh [--no-firewall]
# Idempotent: tools already installed are skipped. The firewall rule lasts until reboot.
set -euo pipefail

[[ $EUID -eq 0 ]] || { echo "run with sudo: sudo $0" >&2; exit 1; }
for c in curl jq unzip; do command -v "$c" >/dev/null || { echo "missing dependency: $c" >&2; exit 1; }; done

BIN=/usr/local/bin
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
FIREWALL=1; [[ "${1:-}" == "--no-firewall" ]] && FIREWALL=0

# kubectl: official release, with the checksum published by Kubernetes.
if ! command -v kubectl >/dev/null; then
  v="$(curl -fsSL https://dl.k8s.io/release/stable.txt)"
  echo ">> kubectl $v"
  curl -fsSL -o "$TMP/kubectl" "https://dl.k8s.io/release/$v/bin/linux/amd64/kubectl"
  echo "$(curl -fsSL "https://dl.k8s.io/release/$v/bin/linux/amd64/kubectl.sha256")  $TMP/kubectl" | sha256sum -c -
  install -m 0755 "$TMP/kubectl" "$BIN/kubectl"
else echo ">> kubectl already installed, skipping"; fi

# k3d: GitHub release binary, checked against the release's checksums.txt.
if ! command -v k3d >/dev/null; then
  echo ">> k3d (latest)"
  base="https://github.com/k3d-io/k3d/releases/latest/download"
  curl -fsSL -o "$TMP/k3d-linux-amd64" "$base/k3d-linux-amd64"
  if curl -fsSL -o "$TMP/checksums.txt" "$base/checksums.txt" 2>/dev/null; then
    # entries look like "<hash>  _dist/k3d-linux-amd64"
    want="$(awk '/k3d-linux-amd64$/ {print $1}' "$TMP/checksums.txt")"
    [[ -n "$want" ]] || { echo "no checksum entry for k3d-linux-amd64" >&2; exit 1; }
    echo "$want  $TMP/k3d-linux-amd64" | sha256sum -c -
  else echo "   (checksums.txt not available; skipping verification)" >&2; fi
  install -m 0755 "$TMP/k3d-linux-amd64" "$BIN/k3d"
else echo ">> k3d already installed, skipping"; fi

# terraform: HashiCorp release zip, checked against the signed SHA256SUMS file's entry.
if ! command -v terraform >/dev/null; then
  v="$(curl -fsSL https://checkpoint-api.hashicorp.com/v1/check/terraform | jq -r .current_version)"
  echo ">> terraform $v"
  z="terraform_${v}_linux_amd64.zip"
  curl -fsSL -o "$TMP/$z" "https://releases.hashicorp.com/terraform/$v/$z"
  curl -fsSL -o "$TMP/SHA256SUMS" "https://releases.hashicorp.com/terraform/$v/terraform_${v}_SHA256SUMS"
  (cd "$TMP" && grep " $z\$" SHA256SUMS | sha256sum -c -)
  unzip -qo "$TMP/$z" terraform -d "$TMP" && install -m 0755 "$TMP/terraform" "$BIN/terraform"
else echo ">> terraform already installed, skipping"; fi

# Firewall: the CoppeliaSim ZeroMQ/WebSocket Remote API binds on all interfaces and can run code.
if [[ $FIREWALL -eq 1 ]]; then
  if iptables -C INPUT -p tcp --dport 23000:23099 ! -i lo -j DROP 2>/dev/null; then
    echo ">> firewall rule already present"
  else
    iptables -I INPUT -p tcp --dport 23000:23099 ! -i lo -j DROP
    echo ">> firewall: ports 23000-23099 closed to non-loopback traffic (until reboot)"
  fi
fi

echo; echo "== installed versions =="
kubectl version --client 2>/dev/null | head -1 || true
k3d version | head -1
terraform version | head -1
