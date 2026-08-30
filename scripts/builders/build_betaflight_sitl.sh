#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
BETAFLIGHT_REPOSITORY="${BETAFLIGHT_REPOSITORY:-https://github.com/betaflight/betaflight.git}"
SOURCE_DIR="${BETAFLIGHT_SOURCE_DIR:-${PROJECT_ROOT}/external/betaflight}"
OUTPUT_DIR="${PROJECT_ROOT}/bin"
OUTPUT_BINARY="${OUTPUT_DIR}/betaflight_SITL.elf"
GPS_PATCH="${PROJECT_ROOT}/config/betaflight/virtual_gps.patch"
BUILD_JOBS="${BUILD_JOBS:-$(nproc)}"

log() {
  local level="$1"
  shift
  printf '[%s] [%-5s] %s\n' "$(date '+%Y-%m-%d %H:%M:%S')" "${level}" "$*"
}

log_info() {
  log "INFO" "$@"
}

log_warn() {
  log "WARN" "$@" >&2
}

log_error() {
  log "ERROR" "$@" >&2
}

BUILD_START_SECONDS="${SECONDS}"
trap 'log_error "Build failed at line ${LINENO}."' ERR

for command in git make curl tar; do
  if ! command -v "${command}" >/dev/null 2>&1; then
    log_error "Required command not found: ${command}"
    exit 1
  fi
done

log_info "Finding the three newest stable Betaflight releases..."
mapfile -t STABLE_TAGS < <({
  git ls-remote --tags --refs "${BETAFLIGHT_REPOSITORY}" \
    | awk -F/ '{print $3}' \
    | grep -E '^[0-9]+\.[0-9]+\.[0-9]+$' \
    | sort -V \
    | tail -n 3 \
    | sort -Vr
} || true)

if ((${#STABLE_TAGS[@]} < 3)); then
  log_error "Expected at least three stable Betaflight releases, found ${#STABLE_TAGS[@]}."
  exit 1
fi

log_info "Available stable releases (alpha, beta, and RC tags excluded):"
for index in "${!STABLE_TAGS[@]}"; do
  printf '  %d) %s\n' "$((index + 1))" "${STABLE_TAGS[${index}]}"
done

while true; do
  printf 'Select the Betaflight version to build [1-3]: '
  if ! read -r selection; then
    log_error "No version selected because input was closed."
    exit 1
  fi

  case "${selection}" in
    1|2|3)
      SELECTED_TAG="${STABLE_TAGS[$((selection - 1))]}"
      break
      ;;
    *)
      log_warn "Invalid selection '${selection}'. Enter 1, 2, or 3."
      ;;
  esac
done

log_info "Selected Betaflight release: ${SELECTED_TAG}"

if [[ -d "${SOURCE_DIR}/.git" ]]; then
  if git -C "${SOURCE_DIR}" apply --reverse --check "${GPS_PATCH}" 2>/dev/null; then
    git -C "${SOURCE_DIR}" apply --reverse "${GPS_PATCH}"
  fi
  if [[ -n "$(git -C "${SOURCE_DIR}" status --porcelain)" ]]; then
    log_error "Refusing to update modified source tree: ${SOURCE_DIR}"
    exit 1
  fi
  log_info "Fetching ${SELECTED_TAG} into the existing source tree."
  git -C "${SOURCE_DIR}" fetch --depth 1 origin "refs/tags/${SELECTED_TAG}:refs/tags/${SELECTED_TAG}"
  git -C "${SOURCE_DIR}" checkout --detach "${SELECTED_TAG}"
elif [[ -e "${SOURCE_DIR}" ]]; then
  log_error "Source path exists but is not a Git repository: ${SOURCE_DIR}"
  exit 1
else
  log_info "Cloning Betaflight ${SELECTED_TAG} into ${SOURCE_DIR}."
  mkdir -p "$(dirname -- "${SOURCE_DIR}")"
  git clone --branch "${SELECTED_TAG}" --depth 1 \
    "${BETAFLIGHT_REPOSITORY}" "${SOURCE_DIR}"
fi

git -C "${SOURCE_DIR}" submodule update --init --recursive
git -C "${SOURCE_DIR}" apply "${GPS_PATCH}"

# Betaflight validates its pinned ARM toolchain before selecting the native
# compiler used by SITL. This target is idempotent after the first installation.
log_info "Ensuring Betaflight's pinned build toolchain is installed."
make -C "${SOURCE_DIR}" arm_sdk_install

log_info "Building Betaflight ${SELECTED_TAG} for SITL with GPS and ${BUILD_JOBS} jobs."
make -C "${SOURCE_DIR}" TARGET=SITL EXTRA_FLAGS=
-DUSE_GPS -j"${BUILD_JOBS}"

BUILT_BINARY="${SOURCE_DIR}/obj/main/betaflight_SITL.elf"
if [[ ! -x "${BUILT_BINARY}" ]]; then
  log_error "Build completed without the expected binary: ${BUILT_BINARY}"
  exit 1
fi

mkdir -p "${OUTPUT_DIR}"
install -m 0755 "${BUILT_BINARY}" "${OUTPUT_BINARY}"

trap - ERR
log_info "Betaflight SITL binary: ${OUTPUT_BINARY}"
log_info "Build completed successfully in $((SECONDS - BUILD_START_SECONDS)) seconds."
