#!/usr/bin/env bash
# Unit tests of the pure chaos analysis (tools/chaos/offload_analysis.py), in a Python container: no ROS, no cluster.
set -euo pipefail
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp -e PYTHONDONTWRITEBYTECODE=1 -v "$REPO/tools/chaos":/chaos:ro \
  -w /tmp "${PY_IMG:-python:3.13-slim}" sh -c 'pip install -q --user pytest 2>&1 | tail -n 1; python3 -m pytest -q -p no:cacheprovider /chaos'
