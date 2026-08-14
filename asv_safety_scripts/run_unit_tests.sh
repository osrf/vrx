#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
ROOT_DIR=$(cd "${SCRIPT_DIR}/.." && pwd)
PYTHONPATH="${ROOT_DIR}/asv_actuator_safety_gateway" \
  python3 -m unittest discover -s "${ROOT_DIR}/asv_actuator_safety_gateway/test" -v
