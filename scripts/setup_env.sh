#!/usr/bin/env bash
set -euo pipefail

# FastNav 通用环境初始化。
# 重点是让 Gazebo Sim 能从新的 config/sim/gz 路径找到 FastNav 自定义模型和世界。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTNAV_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_ROOT="${PX4_ROOT:-/home/shukun/ThirdPackages/PX4-Autopilot}"

FASTNAV_GZ_MODELS="${FASTNAV_ROOT}/config/sim/gz/models"
FASTNAV_GZ_WORLDS="${FASTNAV_ROOT}/config/sim/gz/worlds"

prepend_path() {
  local var_name="$1"
  local new_path="$2"
  local old_value="${!var_name:-}"

  if [[ -z "${old_value}" ]]; then
    export "${var_name}=${new_path}"
  elif [[ ":${old_value}:" != *":${new_path}:"* ]]; then
    export "${var_name}=${new_path}:${old_value}"
  fi
}

if [[ -d "${PX4_ROOT}/Tools/simulation/gz/models" ]]; then
  prepend_path GZ_SIM_RESOURCE_PATH "${PX4_ROOT}/Tools/simulation/gz/models"
  prepend_path IGN_GAZEBO_RESOURCE_PATH "${PX4_ROOT}/Tools/simulation/gz/models"
fi

if [[ -d "${PX4_ROOT}/Tools/simulation/gz/worlds" ]]; then
  prepend_path GZ_SIM_RESOURCE_PATH "${PX4_ROOT}/Tools/simulation/gz/worlds"
  prepend_path IGN_GAZEBO_RESOURCE_PATH "${PX4_ROOT}/Tools/simulation/gz/worlds"
fi

# FastNav 自定义资源最后 prepend，确保同名模型和世界优先使用 config/sim/gz 中的版本。
prepend_path GZ_SIM_RESOURCE_PATH "${FASTNAV_GZ_MODELS}"
prepend_path GZ_SIM_RESOURCE_PATH "${FASTNAV_GZ_WORLDS}"

# 部分 GZ / Ignition 版本仍读取 IGN_GAZEBO_RESOURCE_PATH，两个变量一起设置更稳。
prepend_path IGN_GAZEBO_RESOURCE_PATH "${FASTNAV_GZ_MODELS}"
prepend_path IGN_GAZEBO_RESOURCE_PATH "${FASTNAV_GZ_WORLDS}"

echo "[FastNav][Env] FASTNAV_ROOT=${FASTNAV_ROOT}"
echo "[FastNav][Env] PX4_ROOT=${PX4_ROOT}"
echo "[FastNav][Env] GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH}"
