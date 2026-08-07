#!/usr/bin/env bash
set -euo pipefail
SPEC="${1:-specs/scoop_cycle_v1.yaml}"
python3 scoop_trajectory_client.py --spec "$SPEC"
