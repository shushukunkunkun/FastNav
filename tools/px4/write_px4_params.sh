#!/usr/bin/env bash
set -euo pipefail

# 将 FastNav 维护的 PX4 .params 文件逐项写入当前连接的 PX4。
# 这个脚本只负责 adapter/配置层工作，不改变 FastNav planner/control 的算法参数读取逻辑。
# 使用前需要先启动 roscore、MAVROS，并确认 /mavros/state 处于 connected。

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTNAV_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
# NOTE: 修改这个文件
PARAM_FILE="${FASTNAV_ROOT}/config/sim/px4/multicopter_limits_x250.params"
MAVROS_NS="/mavros"
DRY_RUN=0

usage() {
  cat <<EOF
Usage:
  $(basename "$0") [--sim | --real | -f PARAM_FILE] [--namespace MAVROS_NS] [--dry-run]

Examples:
  $(basename "$0") --sim
  $(basename "$0") --real
  $(basename "$0") -f config/sim/px4/multicopter_limits_x250.params
  $(basename "$0") --sim --dry-run

Options:
  --sim                 Use config/sim/px4/multicopter_limits_x250.params.
  --real                Use config/real/px4/multicopter_limits.params.
  -f, --file PATH       Use a custom PX4 .params file.
  --namespace NS        MAVROS namespace, default: /mavros.
  --dry-run             Print parsed params without writing to PX4.
  -h, --help            Show this help.
EOF
}

resolve_path() {
  local path="$1"
  if [[ "${path}" = /* ]]; then
    printf '%s\n' "${path}"
  else
    printf '%s\n' "${FASTNAV_ROOT}/${path}"
  fi
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --sim)
      PARAM_FILE="${FASTNAV_ROOT}/config/sim/px4/multicopter_limits_x250.params"
      shift
      ;;
    --real)
      PARAM_FILE="${FASTNAV_ROOT}/config/real/px4/multicopter_limits.params"
      shift
      ;;
    -f|--file)
      PARAM_FILE="$(resolve_path "$2")"
      shift 2
      ;;
    --namespace)
      MAVROS_NS="$2"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=1
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "[FastNav][PX4Params] Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -f "${PARAM_FILE}" ]]; then
  echo "[FastNav][PX4Params] Param file does not exist: ${PARAM_FILE}" >&2
  exit 1
fi

if [[ "${DRY_RUN}" -eq 0 ]] && ! command -v rosrun >/dev/null 2>&1; then
  echo "[FastNav][PX4Params] rosrun not found. Source ROS and the FastNav workspace first." >&2
  exit 1
fi

run_mavparam_set() {
  local name="$1"
  local value="$2"

  if [[ "${DRY_RUN}" -eq 1 ]]; then
    printf '[dry-run] rosrun mavros mavparam set %s %s\n' "${name}" "${value}"
    return 0
  fi

  # MAVROS 的 mavparam 工具通过 ROS_NAMESPACE 定位 /mavros/param/* 服务。
  ROS_NAMESPACE="${MAVROS_NS}" rosrun mavros mavparam set "${name}" "${value}"
}

echo "[FastNav][PX4Params] Param file: ${PARAM_FILE}"
echo "[FastNav][PX4Params] MAVROS namespace: ${MAVROS_NS}"

count=0
while IFS= read -r raw_line || [[ -n "${raw_line}" ]]; do
  line="${raw_line%%#*}"
  line="$(echo "${line}" | xargs)"
  [[ -z "${line}" ]] && continue

  # 支持两种格式：
  # 1. PX4 导出格式：$vehicle_id $component_id $name $value $type$
  # 2. 简化格式：$name $value$
  read -r first second third fourth fifth _ <<< "${line}"
  if [[ -n "${fifth:-}" && "${first}" =~ ^[0-9]+$ && "${second}" =~ ^[0-9]+$ ]]; then
    name="${third}"
    value="${fourth}"
  else
    name="${first}"
    value="${second}"
  fi

  if [[ -z "${name:-}" || -z "${value:-}" ]]; then
    echo "[FastNav][PX4Params] Skip invalid line: ${raw_line}" >&2
    continue
  fi

  echo "[FastNav][PX4Params] set ${name} = ${value}"
  run_mavparam_set "${name}" "${value}"
  count=$((count + 1))
done < "${PARAM_FILE}"

echo "[FastNav][PX4Params] Done. Wrote ${count} params."
echo "[FastNav][PX4Params] Tip: for repeatable SITL startup, also mirror these values in the PX4 airframe startup script."
