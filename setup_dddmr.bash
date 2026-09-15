#!/usr/bin/env bash

_DDDMR_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
_DDDMR_ROS_DISTRO="${ROS_DISTRO:-humble}"
_DDDMR_NOUNSET_WAS_ON="False"
case "$-" in
  *u*) _DDDMR_NOUNSET_WAS_ON="True" ;;
esac

if [ ! -f "/opt/ros/${_DDDMR_ROS_DISTRO}/setup.bash" ]; then
  echo "未找到 ROS 2 ${_DDDMR_ROS_DISTRO}。" >&2
  return 1 2>/dev/null || exit 1
fi
if [ ! -f "${_DDDMR_ROOT}/install/setup.bash" ]; then
  echo "DDDMR 尚未编译，请先运行 ${_DDDMR_ROOT}/build.sh。" >&2
  return 1 2>/dev/null || exit 1
fi

set +u
# shellcheck disable=SC1090
source "/opt/ros/${_DDDMR_ROS_DISTRO}/setup.bash"
export CMAKE_PREFIX_PATH="${_DDDMR_ROOT}/.deps/install${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"
export LD_LIBRARY_PATH="${_DDDMR_ROOT}/.deps/install/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
# shellcheck disable=SC1090
source "${_DDDMR_ROOT}/install/setup.bash"
if [ "$_DDDMR_NOUNSET_WAS_ON" = "True" ]; then
  set -u
else
  set +u
fi

unset _DDDMR_ROOT _DDDMR_ROS_DISTRO _DDDMR_NOUNSET_WAS_ON
