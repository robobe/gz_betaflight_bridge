#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
WORLD_FILE="${PROJECT_ROOT}/worlds/quadcopter_sensor.sdf"

source "${SCRIPT_DIR}/setup_gazebo_env.sh"

exec gz sim -v 4 "${WORLD_FILE}" "$@"
