#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
CONFIG_FILE="${PROJECT_ROOT}/config/bridge.yaml"
LOG_DIR="${PROJECT_ROOT}/logs/msp-hover-stack-$(date +%Y%m%d-%H%M%S)"
HEADLESS=false
MISSION="hover"
MISSION_ARGS=(--target-altitude 5)

usage() {
  cat <<EOF
Usage: scripts/run_msp_hover_stack.sh [options] [-- mission-args...]

Starts Gazebo, Betaflight SITL, the C++ bridge, and an MSP mission controller.

Options:
  --headless              Run Gazebo server-only with -r -s.
  --config <file>         Bridge YAML config file. Default: config/bridge.yaml.
  --mission <name>        MSP mission to run: hover or square. Default: hover.
  --target-altitude <m>   Hover target altitude. Default: 5.
  --takeoff-altitude <m>  Square mission takeoff altitude. Default: 4.
  --start-square-altitude <m>
                          Start square legs once this altitude is reached, even
                          if --takeoff-altitude has not settled. Default: 0
                          means wait for --takeoff-altitude.
  --square-side <m>       Square mission side length. Default: 6.
  --max-horizontal-speed <m/s>
                          Square mission horizontal speed cap. Default: 1.
  --rc-us-per-mps <us>   Square mission RC stick authority per m/s. Default: 250.
  --ki-position <value>  Square mission position integral gain. Default: 0.05.
  --roll-min <us>        Square mission minimum roll command. Default: 1200.
  --roll-max <us>        Square mission maximum roll command. Default: 1800.
  --pitch-min <us>       Square mission minimum pitch command. Default: 1200.
  --pitch-max <us>       Square mission maximum pitch command. Default: 1800.
  --duration <s>          Stop hover after this many seconds.
  --kp <value>            Hover proportional gain.
  --kd <value>            Hover derivative gain.
  --hover-throttle <us>   Base hover throttle.
  --min-throttle <us>     Minimum hover throttle.
  --max-throttle <us>     Maximum hover throttle.
  --prearm-duration <s>   Disarmed RC period before arming.
  --arm-low-duration <s>  Armed low-throttle period before hover.
  --angle-mode            Enable ANGLE mode through AUX2. Default.
  --no-angle-mode         Keep AUX2 low for acro/rate mode.
  -h, --help              Show this help.

Examples:
  scripts/run_msp_hover_stack.sh
  scripts/run_msp_hover_stack.sh --headless --target-altitude 5
  scripts/run_msp_hover_stack.sh --mission square
  scripts/run_msp_hover_stack.sh --mission square --takeoff-altitude 4 --start-square-altitude 1.5 --square-side 6
  scripts/run_msp_hover_stack.sh --duration 30 --kp 60 --kd 45 --max-throttle 1650
  scripts/run_msp_hover_stack.sh -- --target-altitude 5 --host 127.0.0.1 --port 5761
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)
      HEADLESS=true
      shift
      ;;
    --config)
      CONFIG_FILE="$2"
      shift 2
      ;;
    --mission)
      MISSION="$2"
      if [[ "${MISSION}" != "hover" && "${MISSION}" != "square" ]]; then
        echo "Unknown mission: ${MISSION}" >&2
        usage >&2
        exit 2
      fi
      if [[ "${MISSION}" == "hover" ]]; then
        MISSION_ARGS=(--target-altitude 5)
      else
        MISSION_ARGS=()
      fi
      shift 2
      ;;
    --target-altitude|--duration|--kp|--kd|--hover-throttle|--min-throttle|--max-throttle|--prearm-duration|--arm-low-duration)
      MISSION_ARGS+=("$1" "$2")
      shift 2
      ;;
    --takeoff-altitude|--start-square-altitude|--square-side|--max-horizontal-speed|--descent-rate|--position-tolerance|--max-leg-duration|--max-mission-duration|--rc-us-per-mps|--roll-min|--roll-max|--pitch-min|--pitch-max|--kp-position|--ki-position|--kd-position|--position-integral-limit)
      MISSION_ARGS+=("$1" "$2")
      shift 2
      ;;
    --angle-mode|--no-angle-mode)
      MISSION_ARGS+=("$1")
      shift
      ;;
    --)
      shift
      MISSION_ARGS=("$@")
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
source "${SCRIPT_DIR}/setup_gazebo_env.sh" >/dev/null

PIDS=()
NAMES=()

cleanup() {
  local status=$?
  trap - INT TERM EXIT

  if [[ ${#PIDS[@]} -gt 0 ]]; then
    echo
    echo "Stopping MSP hover stack..."
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

start_process_with_console_log() {
  local name="$1"
  local logfile="$2"
  shift 2

  echo "Starting ${name}..."
  "$@" > >(tee "${logfile}") 2>&1 &
  local pid=$!
  PIDS+=("${pid}")
  NAMES+=("${name}")
  echo "  pid=${pid} log=${logfile} console=on"
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

  while true; do
    if ss -ltn 2>/dev/null | grep -Eq "[:.]${port}\\b"; then
      return 0
    fi

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
  echo "  scripts/run_betaflight_sitl.sh --config config/betaflight/sitl_modes.cli" >&2
fi

GAZEBO_ARGS=(-r)
if [[ "${HEADLESS}" == true ]]; then
  GAZEBO_ARGS+=(-s)
fi

start_process "gazebo" "${LOG_DIR}/gazebo.log" \
  "${SCRIPT_DIR}/run_quadcopter_world.sh" "${GAZEBO_ARGS[@]}"
wait_for_topic "/imu" 30
wait_for_topic "/altimeter" 30
wait_for_topic "/X3/gazebo/command/motor_speed" 30
if [[ "${MISSION}" == "square" ]]; then
  wait_for_topic "/world/quadcopter/dynamic_pose/info" 30
fi
check_alive

start_process "sitl" "${LOG_DIR}/sitl.log" \
  "${SCRIPT_DIR}/run_betaflight_sitl.sh"
wait_for_udp_port 9003 15
wait_for_udp_port 9004 15
wait_for_tcp_port 5761 15
check_alive

start_process "bridge" "${LOG_DIR}/bridge.log" \
  "${SCRIPT_DIR}/run_bridge.sh" "${CONFIG_FILE}"
sleep 3
check_alive

if [[ "${MISSION}" == "hover" ]]; then
  MISSION_NAME="hover"
  MISSION_SCRIPT="${SCRIPT_DIR}/hover_msp_controller.py"
else
  MISSION_NAME="square"
  MISSION_SCRIPT="${SCRIPT_DIR}/msp_square_mission.py"
fi

start_process_with_console_log "${MISSION_NAME}" "${LOG_DIR}/${MISSION_NAME}.log" \
  "${MISSION_SCRIPT}" "${MISSION_ARGS[@]}"
sleep 1
check_alive

echo
echo "MSP ${MISSION} stack is running."
echo "Logs: ${LOG_DIR}"
echo "${MISSION_NAME} state logs are also printed in this terminal."
echo "Tail logs with:"
echo "  tail -f ${LOG_DIR}/gazebo.log ${LOG_DIR}/sitl.log ${LOG_DIR}/bridge.log ${LOG_DIR}/${MISSION_NAME}.log"
echo
echo "Press Ctrl+C here to stop Gazebo, SITL, bridge, and ${MISSION_NAME}."

while true; do
  sleep 1

  for index in "${!PIDS[@]}"; do
    if ! kill -0 "${PIDS[index]}" 2>/dev/null; then
      if [[ "${NAMES[index]}" == "hover" || "${NAMES[index]}" == "square" ]]; then
        status=0
        wait "${PIDS[index]}" || status=$?
        echo "${NAMES[index]} exited with status ${status}. Stopping the rest of the stack."
        exit "${status}"
      fi

      echo "${NAMES[index]} exited. Check ${LOG_DIR}/${NAMES[index]}.log" >&2
      exit 1
    fi
  done
done
