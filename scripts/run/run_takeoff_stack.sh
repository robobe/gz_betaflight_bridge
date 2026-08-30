#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
CONFIG_FILE="${PROJECT_ROOT}/config/bridge.yaml"
LOG_DIR="${PROJECT_ROOT}/logs/takeoff-stack-$(date +%Y%m%d-%H%M%S)"
HEADLESS=false
START_RC=false
RC_ARGS=(--takeoff-sequence)

usage() {
  cat <<EOF
Usage: scripts/run/run_takeoff_stack.sh [options] [-- rc-args...]

Starts Gazebo, Betaflight SITL, and the bridge.

Options:
  --headless          Run Gazebo server-only with -r -s.
  --udp-rc           Also start the legacy UDP RC takeoff test.
  --config <file>    Bridge YAML config file. Default: config/bridge.yaml.
  --ramp-end <us>    RC takeoff ramp end value. Default: send_rc_test.py default.
  --hold-duration <s>
                     RC hold duration after ramp. Default: send_rc_test.py default.
  -h, --help         Show this help.

Examples:
  scripts/run/run_takeoff_stack.sh
  scripts/run/run_takeoff_stack.sh --headless
  scripts/run/run_takeoff_stack.sh --udp-rc --ramp-end 1600 --hold-duration 20
  scripts/run/run_takeoff_stack.sh -- --takeoff-sequence --ramp-end 1700
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)
      HEADLESS=true
      shift
      ;;
    --udp-rc)
      START_RC=true
      shift
      ;;
    --config)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --ramp-end)
      START_RC=true
      RC_ARGS+=(--ramp-end "$2")
      shift 2
      ;;
    --hold-duration)
      START_RC=true
      RC_ARGS+=(--hold-duration "$2")
      shift 2
      ;;
    --)
      shift
      START_RC=true
      RC_ARGS=("$@")
      break
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

mkdir -p "${LOG_DIR}"
source "${PROJECT_ROOT}/scripts/worlds/setup_gazebo_env.sh" >/dev/null

PIDS=()
NAMES=()

cleanup() {
  local status=$?
  trap - INT TERM EXIT

  if [[ ${#PIDS[@]} -gt 0 ]]; then
    echo
    echo "Stopping stack..."
    for pid in "${PIDS[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        kill "${pid}" 2>/dev/null || true
      fi
    done
    sleep 1
    for pid in "${PIDS[@]}"; do
      if kill -0 "${pid}" 2>/dev/null; then
        kill -9 "${pid}" 2>/dev/null || true
      fi
    done
  fi

  echo "Logs: ${LOG_DIR}"
  exit "${status}"
}

trap cleanup INT TERM EXIT

start_process() {
  local name="$1"
  local logfile="$2"
  shift 2

  echo "Starting ${name}..."
  "$@" >"${logfile}" 2>&1 &
  local pid=$!
  PIDS+=("${pid}")
  NAMES+=("${name}")
  echo "  pid=${pid} log=${logfile}"
}

wait_for_topic() {
  local topic="$1"
  local timeout_s="$2"
  local start
  start="$(date +%s)"

  while true; do
    if gz topic -l 2>/dev/null | grep -Eq "^${topic}$"; then
      return 0
    fi

    if (( $(date +%s) - start >= timeout_s )); then
      echo "Timed out waiting for Gazebo topic: ${topic}" >&2
      return 1
    fi

    sleep 0.5
  done
}

wait_for_udp_port() {
  local port="$1"
  local timeout_s="$2"
  local start
  start="$(date +%s)"

  while true; do
    if ss -lun 2>/dev/null | grep -Eq "[:.]${port}\\b"; then
      return 0
    fi

    if (( $(date +%s) - start >= timeout_s )); then
      echo "Timed out waiting for UDP port: ${port}" >&2
      return 1
    fi

    sleep 0.5
  done
}

wait_for_tcp_port() {
  local port="$1"
  local timeout_s="$2"
  local start
  start="$(date +%s)"

  until (echo >"/dev/tcp/127.0.0.1/${port}") 2>/dev/null; do
    if (( $(date +%s) - start >= timeout_s )); then
      echo "Timed out waiting for TCP port: ${port}" >&2
      return 1
    fi
    sleep 0.5
  done
}

check_alive() {
  local index
  for index in "${!PIDS[@]}"; do
    if ! kill -0 "${PIDS[index]}" 2>/dev/null; then
      echo "${NAMES[index]} exited early. Check ${LOG_DIR}/${NAMES[index]}.log" >&2
      return 1
    fi
  done
}

if [[ ! -x "${PROJECT_ROOT}/build/debug/betaflight_gazebo_bridge" ]]; then
  echo "Missing bridge executable. Build first:" >&2
  echo "  cmake --preset debug && cmake --build --preset debug" >&2
  exit 1
fi

if [[ ! -x "${PROJECT_ROOT}/bin/betaflight_SITL.elf" ]]; then
  echo "Missing Betaflight SITL executable: bin/betaflight_SITL.elf" >&2
  exit 1
fi

if [[ ! -f "${PROJECT_ROOT}/eeprom.bin" ]]; then
  echo "Warning: eeprom.bin not found. Generate it once with:" >&2
  echo "  scripts/run/run_betaflight_sitl.sh --config config/betaflight/sitl_modes.cli" >&2
fi

GAZEBO_ARGS=(-r)
if [[ "${HEADLESS}" == true ]]; then
  GAZEBO_ARGS+=(-s)
fi

start_process "gazebo" "${LOG_DIR}/gazebo.log" \
  "${PROJECT_ROOT}/scripts/worlds/run_quadcopter_world.sh" "${GAZEBO_ARGS[@]}"
wait_for_topic "/imu" 30
wait_for_topic "/altimeter" 30
wait_for_topic "/X3/gazebo/command/motor_speed" 30
check_alive

start_process "sitl" "${LOG_DIR}/sitl.log" \
  "${PROJECT_ROOT}/scripts/run/run_betaflight_sitl.sh"
wait_for_udp_port 9003 15
wait_for_udp_port 9004 15
wait_for_tcp_port 5761 15
check_alive

start_process "bridge" "${LOG_DIR}/bridge.log" \
  "${PROJECT_ROOT}/scripts/run/run_bridge.sh" "${CONFIG_FILE}"
sleep 3
check_alive

if [[ "${START_RC}" == true ]]; then
  start_process "rc" "${LOG_DIR}/rc.log" \
    "${PROJECT_ROOT}/scripts/tests/send_rc_test.py" "${RC_ARGS[@]}"
fi

echo
echo "Stack is running."
echo "Logs: ${LOG_DIR}"
echo "Tail logs with:"
if [[ "${START_RC}" == true ]]; then
  echo "  tail -f ${LOG_DIR}/gazebo.log ${LOG_DIR}/sitl.log ${LOG_DIR}/bridge.log ${LOG_DIR}/rc.log"
else
  echo "  tail -f ${LOG_DIR}/gazebo.log ${LOG_DIR}/sitl.log ${LOG_DIR}/bridge.log"
fi
echo
echo "Run MSP hover in another terminal with:"
echo "  scripts/msp_hover/hover_msp_controller.py --target-altitude 5"
echo
echo "Press Ctrl+C here to stop Gazebo, SITL, bridge, and optional RC."

while true; do
  sleep 1

  for index in "${!PIDS[@]}"; do
    if ! kill -0 "${PIDS[index]}" 2>/dev/null; then
      if [[ "${NAMES[index]}" == "rc" ]]; then
        continue
      fi

      echo "${NAMES[index]} exited. Check ${LOG_DIR}/${NAMES[index]}.log" >&2
      exit 1
    fi
  done
done
