#!/usr/bin/env bash
set -euo pipefail

# Install FastNav custom PX4 SITL airframes into a local PX4-Autopilot tree.
# This keeps the canonical airframe files inside FastNav config/, while still
# making targets such as $make px4_sitl gz_x250_mid360$ available to PX4.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FASTNAV_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
PX4_ROOT="${PX4_ROOT:-/home/shukun/ThirdPackages/PX4-Autopilot}"

SRC_DIR="${FASTNAV_ROOT}/config/sim/px4/airframes"
DST_DIR="${PX4_ROOT}/ROMFS/px4fmu_common/init.d-posix/airframes"
CMAKE_FILE="${DST_DIR}/CMakeLists.txt"

install_airframe() {
  local name="$1"
  local src="${SRC_DIR}/${name}"
  local dst="${DST_DIR}/${name}"

  if [[ ! -f "${src}" ]]; then
    echo "[FastNav][PX4Airframe] Missing source airframe: ${src}" >&2
    exit 1
  fi

  install -m 755 "${src}" "${dst}"
  echo "[FastNav][PX4Airframe] Installed ${dst}"

  if ! grep -q "^[[:space:]]*${name}[[:space:]]*$" "${CMAKE_FILE}"; then
    local tmp
    tmp="$(mktemp)"
    awk -v insert_name="${name}" '
      {
        print $0
        if ($0 ~ /^[[:space:]]*4022_gz_x500_mid360[[:space:]]*$/) {
          print "\t" insert_name
        }
      }
    ' "${CMAKE_FILE}" > "${tmp}"
    mv "${tmp}" "${CMAKE_FILE}"
    echo "[FastNav][PX4Airframe] Registered ${name} in ${CMAKE_FILE}"
  else
    echo "[FastNav][PX4Airframe] ${name} already registered in ${CMAKE_FILE}"
  fi
}

if [[ ! -d "${PX4_ROOT}" ]]; then
  echo "[FastNav][PX4Airframe] PX4_ROOT does not exist: ${PX4_ROOT}" >&2
  exit 1
fi

if [[ ! -f "${CMAKE_FILE}" ]]; then
  echo "[FastNav][PX4Airframe] PX4 airframe CMakeLists not found: ${CMAKE_FILE}" >&2
  exit 1
fi

install_airframe "4023_gz_x250_mid360"

echo "[FastNav][PX4Airframe] Done."
echo "[FastNav][PX4Airframe] Re-run PX4 cmake/make if the target was not previously generated."
