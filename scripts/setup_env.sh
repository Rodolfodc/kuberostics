#!/usr/bin/env bash
# Prepares an Ubuntu machine (24.04 or 26.04) for Kuberostics. Idempotent: safe to run again.
#   scripts/setup_env.sh             installs what is missing (asks for sudo when needed) and verifies
#   scripts/setup_env.sh --check     only verifies, installs nothing
#   scripts/setup_env.sh --pull-jazzy  also pulls the osrf/ros:jazzy-desktop-full image (~3 GB) to test on Jazzy
#   scripts/setup_env.sh --no-coppelia skips the CoppeliaSim download (~250 MB)
# Installs: apt packages, Docker, kubectl/k3d/terraform + firewall (scripts/setup-tools.sh), CoppeliaSim 4.10.0,
# .venv with requirements.txt and the workspace (scripts/setup.sh: vcs import + rosdep).
# Does NOT install ROS 2 (use the official Lyrical or Jazzy guide) nor llama.cpp (optional, operational help only).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"
CHECK=0; PULL_JAZZY=0; COPPELIA=1
for a in "$@"; do case "$a" in
  --check) CHECK=1;; --pull-jazzy) PULL_JAZZY=1;; --no-coppelia) COPPELIA=0;;
  -h|--help) sed -n '2,10p' "$0"; exit 0;; *) echo "unknown option: $a" >&2; exit 2;; esac; done

COPPELIA_VER="V4_10_0_rev0"
COPPELIA_DIR="${COPPELIA_DIR:-$HOME/Tools/CoppeliaSim_Edu_${COPPELIA_VER}_Ubuntu24_04}"
COPPELIA_URL="https://downloads.coppeliarobotics.com/${COPPELIA_VER}/CoppeliaSim_Edu_${COPPELIA_VER}_Ubuntu24_04.tar.xz"
APT_PKGS=(curl jq unzip git iproute2 poppler-utils python3-venv python3-pip python3-pytest
          python3-colcon-common-extensions python3-vcstool python3-rosdep)
FAIL=0
say()  { printf '\n>> %s\n' "$*"; }
ok()   { printf '  [ok]      %s\n' "$*"; }
miss() { printf '  [MISSING] %s\n' "$*"; FAIL=1; }
have() { command -v "$1" >/dev/null 2>&1; }

[[ $EUID -ne 0 ]] || { echo "run as a regular user; the script calls sudo only where needed" >&2; exit 1; }
. /etc/os-release
[[ "$ID" == "ubuntu" ]] || echo "warning: only tested on Ubuntu (detected: $ID $VERSION_ID)" >&2

# ---------------------------------------------------------------- installation
if [[ $CHECK -eq 0 ]]; then
  say "apt packages"
  need=(); for p in "${APT_PKGS[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || need+=("$p"); done
  if ((${#need[@]})); then sudo apt-get update -qq && sudo apt-get install -y "${need[@]}"; else echo "  nothing to install"; fi

  say "Docker"
  if ! have docker; then sudo apt-get install -y docker.io; fi
  if ! id -nG | tr ' ' '\n' | grep -qx docker; then
    sudo usermod -aG docker "$USER"; echo "  you joined the docker group: log out and back in for it to apply"
  fi
  [[ $PULL_JAZZY -eq 1 ]] && { say "Jazzy image"; docker pull osrf/ros:jazzy-desktop-full; }

  say "kubectl, k3d, terraform and firewall for ports 23000-23099"
  sudo "$REPO/scripts/setup-tools.sh"

  if [[ $COPPELIA -eq 1 ]]; then
    say "CoppeliaSim $COPPELIA_VER"
    if [[ -x "$COPPELIA_DIR/coppeliaSim.sh" ]]; then echo "  already installed in $COPPELIA_DIR"
    else
      mkdir -p "$(dirname "$COPPELIA_DIR")"; t="$(mktemp -d)"; trap 'rm -rf "$t"' EXIT
      curl -fL --retry 3 -o "$t/c.tar.xz" "$COPPELIA_URL"
      tar -xf "$t/c.tar.xz" -C "$(dirname "$COPPELIA_DIR")"
    fi
  fi

  say "Python environment (.venv with requirements.txt)"
  # /usr/bin/python3 on purpose: a python3.x in ~/.local/bin breaks rosidl.
  [[ -d .venv ]] || /usr/bin/python3 -m venv --system-site-packages .venv
  .venv/bin/pip install -q -r requirements.txt && echo "  done"

  say "workspace (vcs import + rosdep)"
  ros=""; for d in lyrical jazzy; do [[ -f /opt/ros/$d/setup.bash ]] && { ros=$d; break; }; done
  if [[ -n "$ros" ]]; then
    [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
    rosdep update --rosdistro "$ros" >/dev/null
    bash -c "source /opt/ros/$ros/setup.bash && scripts/setup.sh"
  else echo "  ROS 2 not found: install Lyrical or Jazzy and run this script again"; fi
fi

# ---------------------------------------------------------------- verification
say "verification"
for c in git curl jq unzip pdfinfo ss colcon vcs rosdep docker kubectl k3d terraform systemd-run; do
  have "$c" && ok "$c" || miss "$c"
done
d=""; for x in lyrical jazzy; do [[ -f /opt/ros/$x/setup.bash ]] && d="$d $x"; done
[[ -n "$d" ]] && ok "ROS 2 on the host:$d" || miss "ROS 2 is REQUIRED and not installed by this script: install Lyrical (or Jazzy) in /opt/ros from the official guide"
docker image inspect osrf/ros:jazzy-desktop-full >/dev/null 2>&1 && ok "image osrf/ros:jazzy-desktop-full" \
  || echo "  [optional] Jazzy image missing (use --pull-jazzy)"
if [[ $COPPELIA -eq 1 ]]; then
  [[ -x "$COPPELIA_DIR/coppeliaSim.sh" ]] && ok "CoppeliaSim in $COPPELIA_DIR" || miss "CoppeliaSim in $COPPELIA_DIR"
fi
[[ -x .venv/bin/python ]] && .venv/bin/python -c "import coppeliasim_zmqremoteapi_client, zmq, pytest" 2>/dev/null \
  && ok ".venv with ZMQ client, pyzmq and pytest" || miss ".venv with the requirements.txt packages"
(have llama-server || [[ -x "$HOME/Tools/llama.cpp/build/bin/llama-server" ]]) && ok "llama.cpp (optional)" \
  || echo "  [optional] llama.cpp missing: the project works without it"
[[ -d src/deps/IA368II ]] && ok "src/deps/IA368II (third-party code, read-only)" || miss "src/deps (run scripts/setup.sh)"
sudo -n iptables -C INPUT -p tcp --dport 23000:23099 ! -i lo -j DROP 2>/dev/null && ok "firewall 23000-23099 closed" \
  || echo "  [warning] firewall rule not confirmed (lasts until reboot; checking needs sudo: run without --check)"
[[ -f ../scene_house.ttt ]] && ok "scene ../scene_house.ttt" || miss "scene ../scene_house.ttt (kept outside the repo; copy it from the course material)"

echo; if ((FAIL)); then echo "Items above are missing."; exit 1; else echo "Environment ready."; fi
