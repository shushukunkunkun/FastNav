#!/usr/bin/env bash
set -euo pipefail

# 启动 PX4 SITL + Gazebo Sim。
# 新结构下，FastNav 自定义模型和世界来自 config/sim/gz/models 与 config/sim/gz/worlds。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTNAV_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PX4_ROOT="${PX4_ROOT:-/home/shukun/ThirdPackages/PX4-Autopilot}"

WORLD="${FASTNAV_GZ_WORLD:-forest_easy}"
MODEL="${FASTNAV_PX4_MODEL:-x250_mid360}"
MAKE_TARGET="${FASTNAV_PX4_MAKE_TARGET:-gz_x250_mid360}"

source "${FASTNAV_ROOT}/scripts/setup_env.sh"

if [[ ! -d "${PX4_ROOT}" ]]; then
  echo "[FastNav][PX4GZ] PX4_ROOT does not exist: ${PX4_ROOT}" >&2
  echo "[FastNav][PX4GZ] Set PX4_ROOT=/path/to/PX4-Autopilot and retry." >&2
  exit 1
fi

echo "[FastNav][PX4GZ] world=${WORLD}, model=${MODEL}, make target=${MAKE_TARGET}"
echo "[FastNav][PX4GZ] cd ${PX4_ROOT}"
cd "${PX4_ROOT}"

PX4_GZ_WORLD="${WORLD}" PX4_SIM_MODEL="${MODEL}" make px4_sitl "${MAKE_TARGET}"
