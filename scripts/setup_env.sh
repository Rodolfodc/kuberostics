#!/usr/bin/env bash
# Prepara uma máquina Ubuntu (24.04 ou 26.04) para o Kuberostics. Idempotente: roda de novo sem estragar nada.
#   scripts/setup_env.sh             instala o que falta (pede sudo quando precisar) e verifica
#   scripts/setup_env.sh --check     só verifica, não instala nada
#   scripts/setup_env.sh --pull-jazzy  também baixa a imagem osrf/ros:jazzy-desktop-full (~3 GB) para testar no Jazzy
#   scripts/setup_env.sh --no-coppelia pula o download do CoppeliaSim (~250 MB)
# Instala: pacotes apt, Docker, kubectl/k3d/terraform + firewall (scripts/setup-tools.sh), CoppeliaSim 4.10.0,
# .venv com requirements.txt e o workspace (scripts/setup.sh: vcs import + rosdep).
# NÃO instala o ROS 2 (use o guia oficial do Lyrical ou do Jazzy) nem o llama.cpp (opcional, só ajuda de operação).
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO"
CHECK=0; PULL_JAZZY=0; COPPELIA=1
for a in "$@"; do case "$a" in
  --check) CHECK=1;; --pull-jazzy) PULL_JAZZY=1;; --no-coppelia) COPPELIA=0;;
  -h|--help) sed -n '2,10p' "$0"; exit 0;; *) echo "opção desconhecida: $a" >&2; exit 2;; esac; done

COPPELIA_VER="V4_10_0_rev0"
COPPELIA_DIR="${COPPELIA_DIR:-$HOME/Tools/CoppeliaSim_Edu_${COPPELIA_VER}_Ubuntu24_04}"
COPPELIA_URL="https://downloads.coppeliarobotics.com/${COPPELIA_VER}/CoppeliaSim_Edu_${COPPELIA_VER}_Ubuntu24_04.tar.xz"
APT_PKGS=(curl jq unzip git iproute2 poppler-utils python3-venv python3-pip python3-pytest
          python3-colcon-common-extensions python3-vcstool python3-rosdep)
FAIL=0
say()  { printf '\n>> %s\n' "$*"; }
ok()   { printf '  [ok]      %s\n' "$*"; }
miss() { printf '  [FALTA]   %s\n' "$*"; FAIL=1; }
have() { command -v "$1" >/dev/null 2>&1; }

[[ $EUID -ne 0 ]] || { echo "rode como usuário comum; o script chama sudo só onde precisa" >&2; exit 1; }
. /etc/os-release
[[ "$ID" == "ubuntu" ]] || echo "aviso: testado só em Ubuntu (detectado: $ID $VERSION_ID)" >&2

# ---------------------------------------------------------------- instalação
if [[ $CHECK -eq 0 ]]; then
  say "pacotes apt"
  need=(); for p in "${APT_PKGS[@]}"; do dpkg -s "$p" >/dev/null 2>&1 || need+=("$p"); done
  if ((${#need[@]})); then sudo apt-get update -qq && sudo apt-get install -y "${need[@]}"; else echo "  nada a instalar"; fi

  say "Docker"
  if ! have docker; then sudo apt-get install -y docker.io; fi
  if ! id -nG | tr ' ' '\n' | grep -qx docker; then
    sudo usermod -aG docker "$USER"; echo "  você entrou no grupo docker: faça logout/login para valer"
  fi
  [[ $PULL_JAZZY -eq 1 ]] && { say "imagem Jazzy"; docker pull osrf/ros:jazzy-desktop-full; }

  say "kubectl, k3d, terraform e firewall das portas 23000-23099"
  sudo "$REPO/scripts/setup-tools.sh"

  if [[ $COPPELIA -eq 1 ]]; then
    say "CoppeliaSim $COPPELIA_VER"
    if [[ -x "$COPPELIA_DIR/coppeliaSim.sh" ]]; then echo "  já instalado em $COPPELIA_DIR"
    else
      mkdir -p "$(dirname "$COPPELIA_DIR")"; t="$(mktemp -d)"; trap 'rm -rf "$t"' EXIT
      curl -fL --retry 3 -o "$t/c.tar.xz" "$COPPELIA_URL"
      tar -xf "$t/c.tar.xz" -C "$(dirname "$COPPELIA_DIR")"
    fi
  fi

  say "ambiente Python (.venv com requirements.txt)"
  # /usr/bin/python3 de propósito: um python3.x em ~/.local/bin quebra o rosidl.
  [[ -d .venv ]] || /usr/bin/python3 -m venv --system-site-packages .venv
  .venv/bin/pip install -q -r requirements.txt && echo "  ok"

  say "workspace (vcs import + rosdep)"
  ros=""; for d in lyrical jazzy; do [[ -f /opt/ros/$d/setup.bash ]] && { ros=$d; break; }; done
  if [[ -n "$ros" ]]; then
    [[ -f /etc/ros/rosdep/sources.list.d/20-default.list ]] || sudo rosdep init
    rosdep update --rosdistro "$ros" >/dev/null
    bash -c "source /opt/ros/$ros/setup.bash && scripts/setup.sh"
  else echo "  ROS 2 não encontrado: instale o Lyrical ou o Jazzy e rode este script de novo"; fi
fi

# ---------------------------------------------------------------- verificação
say "verificação"
for c in git curl jq unzip pdfinfo ss colcon vcs rosdep docker kubectl k3d terraform systemd-run; do
  have "$c" && ok "$c" || miss "$c"
done
d=""; for x in lyrical jazzy; do [[ -f /opt/ros/$x/setup.bash ]] && d="$d $x"; done
[[ -n "$d" ]] && ok "ROS 2 no host:$d" || miss "ROS 2 (/opt/ros/{lyrical,jazzy}); o Jazzy também roda por Docker"
docker image inspect osrf/ros:jazzy-desktop-full >/dev/null 2>&1 && ok "imagem osrf/ros:jazzy-desktop-full" \
  || echo "  [opcional] imagem Jazzy ausente (use --pull-jazzy)"
if [[ $COPPELIA -eq 1 ]]; then
  [[ -x "$COPPELIA_DIR/coppeliaSim.sh" ]] && ok "CoppeliaSim em $COPPELIA_DIR" || miss "CoppeliaSim em $COPPELIA_DIR"
fi
[[ -x .venv/bin/python ]] && .venv/bin/python -c "import coppeliasim_zmqremoteapi_client, zmq, pytest" 2>/dev/null \
  && ok ".venv com cliente ZMQ, pyzmq e pytest" || miss ".venv com os pacotes de requirements.txt"
[[ -d src/deps/lucianovk/IA368II ]] && ok "src/deps/lucianovk/IA368II (código do Luciano, somente leitura)" || miss "src/deps (rode scripts/setup.sh)"
sudo -n iptables -C INPUT -p tcp --dport 23000:23099 ! -i lo -j DROP 2>/dev/null && ok "firewall 23000-23099 fechado" \
  || echo "  [aviso]   regra de firewall não confirmada (vale até o reboot; precisa de sudo para checar: rode sem --check)"
[[ -f ../scene_house.ttt ]] && ok "cena ../scene_house.ttt" || miss "cena ../scene_house.ttt (fica fora do repo; copie da entrega da disciplina)"

echo; if ((FAIL)); then echo "Faltam itens acima."; exit 1; else echo "Ambiente pronto."; fi
