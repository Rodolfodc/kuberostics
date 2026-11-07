#!/usr/bin/env bash
# T001: baixa a dependência fixada e instala as dependências ROS do workspace.
# Uso: scripts/setup.sh   (rodar na raiz do repositório, com o ROS 2 carregado)
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p src/deps
vcs import src/deps < deps.repos
rosdep install --from-paths src --ignore-src -r -y
