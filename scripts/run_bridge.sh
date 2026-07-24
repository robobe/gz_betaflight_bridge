#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG_FILE="${1:-${PROJECT_ROOT}/config/bridge.yaml}"

exec "${PROJECT_ROOT}/build/debug/betaflight_gazebo_bridge" --config "${CONFIG_FILE}"

