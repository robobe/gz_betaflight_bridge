#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
BETAFLIGHT_REPOSITORY="${BETAFLIGHT_REPOSITORY:-https://github.com/betaflight/betaflight.git}"
SOURCE_DIR="${BETAFLIGHT_SOURCE_DIR:-${PROJECT_ROOT}/external/betaflight}"
OUTPUT_DIR="${PROJECT_ROOT}/bin"
OUTPUT_BINARY="${OUTPUT_DIR}/betaflight_SITL.elf"
BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"

for command in git make curl tar; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    echo "Required command not found: ${command}" >&2
    exit 1
  fi
done

echo "Finding the latest stable Betaflight release..."
LATEST_STABLE_TAG="$({
  git ls-remote --tags --refs "${BETAFLIGHT_REPOSITORY}" \
    | awk -F/ '{print $3}' \
    | grep -E '^[0-9]+\.[0-9]+\.[0-9]+$' \
    | sort -V \
    | tail -n 1
} || true)"

if [[ -z "${LATEST_STABLE_TAG}" ]]; then
  echo "Could not determine the latest stable Betaflight release." >&2
  exit 1
fi

echo "Latest stable release: ${LATEST_STABLE_TAG}"

if [[ -d "${SOURCE_DIR}/.git" ]]; then
  if [[ -n "$(git -C "${SOURCE_DIR}" status --porcelain)" ]]; then
    echo "Refusing to update modified source tree: ${SOURCE_DIR}" >&2
    exit 1
  fi
  git -C "${SOURCE_DIR}" fetch --depth 1 origin "refs/tags/${LATEST_STABLE_TAG}:refs/tags/${LATEST_STABLE_TAG}"
  git -C "${SOURCE_DIR}" checkout --detach "${LATEST_STABLE_TAG}"
elif [[ -e "${SOURCE_DIR}" ]]; then
  echo "Source path exists but is not a Git repository: ${SOURCE_DIR}" >&2
  exit 1
else
  mkdir -p "$(dirname -- "${SOURCE_DIR}")"
  git clone --branch "${LATEST_STABLE_TAG}" --depth 1 \
    "${BETAFLIGHT_REPOSITORY}" "${SOURCE_DIR}"
fi

# Betaflight validates its pinned ARM toolchain before selecting the native
# compiler used by SITL. This target is idempotent after the first installation.
echo "Ensuring Betaflight's pinned build toolchain is installed..."
make -C "${SOURCE_DIR}" arm_sdk_install

echo "Building Betaflight ${LATEST_STABLE_TAG} for SITL..."
make -C "${SOURCE_DIR}" TARGET=SITL -j"${BUILD_JOBS}"

BUILT_BINARY="${SOURCE_DIR}/obj/main/betaflight_SITL.elf"
if [[ ! -x "${BUILT_BINARY}" ]]; then
  echo "Build completed without the expected binary: ${BUILT_BINARY}" >&2
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"
install -m 0755 "${BUILT_BINARY}" "${OUTPUT_BINARY}"

echo "Betaflight SITL binary: ${OUTPUT_BINARY}"
