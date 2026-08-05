#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
SITL_BIN="${PROJECT_ROOT}/bin/betaflight_SITL.elf"

if [[ ! -x "${SITL_BIN}" ]]; then
  echo "Missing executable: ${SITL_BIN}" >&2
  echo "Build or copy Betaflight SITL to bin/betaflight_SITL.elf first." >&2
  exit 1
fi

exec "${SITL_BIN}" "$@"
