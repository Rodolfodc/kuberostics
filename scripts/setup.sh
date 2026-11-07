#!/usr/bin/env bash
# T001: fetches the pinned dependency and installs the workspace ROS dependencies.
# Usage: scripts/setup.sh   (run from the repository root, with ROS 2 sourced)
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/.."
mkdir -p src/deps
vcs import src/deps < deps.repos
rosdep install --from-paths src --ignore-src -r -y
