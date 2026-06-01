#!/usr/bin/env bash
set -euo pipefail

# 启动 FastNav ROS 侧完整仿真链路。
# PX4 + GZ 本体建议先单独用 scripts/start_px4_gz.sh 启动。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTNAV_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

source "${FASTNAV_ROOT}/scripts/setup_env.sh"

if [[ -f "${FASTNAV_ROOT}/ros1_ws/devel/setup.bash" ]]; then
  source "${FASTNAV_ROOT}/ros1_ws/devel/setup.bash"
fi

roslaunch fastnav_bringup fastnav_full_sim.launch
