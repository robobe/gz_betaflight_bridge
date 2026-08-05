#!/usr/bin/env bash

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

append_env_path() {
  local variable_name="$1"
  local path_to_add="$2"
  local current_value="${!variable_name:-}"

  if [[ -z "${current_value}" ]]; then
    export "${variable_name}=${path_to_add}"
    return
  fi

  case ":${current_value}:" in
    *":${path_to_add}:"*) ;;
    *) export "${variable_name}=${current_value}:${path_to_add}" ;;
  esac
}

append_env_path GZ_SIM_RESOURCE_PATH "${PROJECT_ROOT}"
append_env_path GZ_SIM_RESOURCE_PATH "${PROJECT_ROOT}/worlds"
append_env_path GZ_SIM_RESOURCE_PATH "${PROJECT_ROOT}/models"

append_env_path GZ_SIM_SYSTEM_PLUGIN_PATH "${PROJECT_ROOT}/build/debug"
append_env_path GZ_SIM_SYSTEM_PLUGIN_PATH "${PROJECT_ROOT}/build/release"

echo "GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"
echo "GZ_SIM_SYSTEM_PLUGIN_PATH=${GZ_SIM_SYSTEM_PLUGIN_PATH}"
