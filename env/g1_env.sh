#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  printf '%s\n' \
    'Please source this script:' \
    'source env/g1_env.sh <network_interface>' >&2
  exit 1
fi

NET_IFACE="${1:-${UNITREE_NET_IFACE:-enP8p1s0}}"
export UNITREE_NET_IFACE="${NET_IFACE}"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi

if [[ -f "${HOME}/unitree_ros2/cyclonedds_ws/install/setup.bash" ]]; then
  source "${HOME}/unitree_ros2/cyclonedds_ws/install/setup.bash"
fi

if [[ -f "${HOME}/Robot/ros2_interactive_marker_pid/install/setup.bash" ]]; then
  source "${HOME}/Robot/ros2_interactive_marker_pid/install/setup.bash"
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY=0
export CYCLONEDDS_URI="<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface
          name=\"${NET_IFACE}\"
          priority=\"default\"
          multicast=\"default\"/>
      </Interfaces>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</CycloneDDS>"

printf '[g1_env] Network interface: %s\n' "${NET_IFACE}"
