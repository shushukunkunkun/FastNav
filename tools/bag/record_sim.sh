#!/usr/bin/env bash
set -e

BAG_DIR="${BAG_DIR:-$HOME/fastnav_bags}"
mkdir -p "${BAG_DIR}"

STAMP="$(date +%Y%m%d_%H%M%S)"
BAG_FILE="${BAG_DIR}/fastnav_sim_${STAMP}.bag"

echo "[FastNav][Bag] Recording simulation topics to ${BAG_FILE}"
echo "[FastNav][Bag] Press Ctrl-C to stop."

rosbag record -O "${BAG_FILE}" \
  /fastnav/state/odom \
  /fastnav/lidar/points \
  /fastnav/perception/cloud_odom \
  /fastnav/perception/cloud_filtered \
  /fastnav/map/occupied_cloud \
  /fastnav/map/inflated_cloud \
  /fastnav/planner/goal \
  /fastnav/planner/path \
  /fastnav/planner/fsm_state \
  /fastnav/planner/local_target \
  /fastnav/planner/minco_trajectory \
  /fastnav/planner/heartbeat \
  /fastnav/control/fsm_state \
  /fastnav/control_cmd \
  /mavros/state \
  /mavros/local_position/odom \
  /tf \
  /tf_static
