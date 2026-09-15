#!/usr/bin/env bash
set -Eeuo pipefail

trap 'echo "DDDMR 编译失败：第 ${LINENO} 行，请检查上方错误。"' ERR

if [ "${EUID}" -eq 0 ]; then
  echo "请不要使用 sudo 运行本脚本。"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${SCRIPT_DIR}/src"
DEPS_PREFIX="${SCRIPT_DIR}/.deps/install"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
BUILD_JOBS="${DDDMR_BUILD_JOBS:-}"
AVAILABLE_THREADS="$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo 1)"
TRT_ENABLED="OFF"
PACKAGE_MODE="all"
PACKAGE_NAMES=()

usage() {
  echo "用法：./build.sh [--jobs N] [--trt] [--packages-select 包...] [--packages-up-to 包...]"
  echo "未指定 --jobs 时交互选择线程数；默认增量编译全部包并关闭 TensorRT。"
}

choose_build_jobs() {
  local default_jobs=2
  local answer

  if [ "$AVAILABLE_THREADS" -lt "$default_jobs" ]; then
    default_jobs="$AVAILABLE_THREADS"
  fi

  if [ -n "$BUILD_JOBS" ]; then
    return
  fi

  echo "检测到可用 CPU 线程数：${AVAILABLE_THREADS}"
  if [ ! -t 0 ]; then
    BUILD_JOBS="$default_jobs"
    echo "当前不是交互式终端，使用默认线程数：${BUILD_JOBS}"
    return
  fi

  while true; do
    read -r -p "请选择编译线程数 [1-${AVAILABLE_THREADS}，默认 ${default_jobs}]：" answer
    answer="${answer:-$default_jobs}"
    if [[ "$answer" =~ ^[1-9][0-9]*$ ]] && [ "$answer" -le "$AVAILABLE_THREADS" ]; then
      BUILD_JOBS="$answer"
      return
    fi
    echo "请输入 1 到 ${AVAILABLE_THREADS} 之间的整数。"
  done
}

while [ "$#" -gt 0 ]; do
  case "$1" in
    --jobs)
      [ "$#" -ge 2 ] || { usage; exit 2; }
      BUILD_JOBS="$2"; shift 2 ;;
    --trt) TRT_ENABLED="ON"; shift ;;
    --packages-select|--packages-up-to)
      PACKAGE_MODE="${1#--packages-}"
      shift
      while [ "$#" -gt 0 ] && [[ "$1" != --* ]]; do
        PACKAGE_NAMES+=("$1"); shift
      done
      [ "${#PACKAGE_NAMES[@]}" -gt 0 ] || { usage; exit 2; } ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$1"; usage; exit 2 ;;
  esac
done

choose_build_jobs

if ! [[ "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]] || [ "$BUILD_JOBS" -gt "$AVAILABLE_THREADS" ]; then
  echo "编译线程数必须是 1 到 ${AVAILABLE_THREADS} 之间的整数。"
  exit 2
fi
if [ ! -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]; then
  echo "未找到 ROS 2 ${ROS_DISTRO_NAME}。"
  exit 1
fi
if [ ! -f "${DEPS_PREFIX}/share/pcl-1.15/PCLConfig.cmake" ] || \
   [ ! -f "${DEPS_PREFIX}/lib/cmake/small_gicp/small_gicp-config.cmake" ]; then
  echo "未找到本地 PCL 1.15 或 small_gicp，请先运行：./install.sh"
  exit 1
fi

set +u
# shellcheck disable=SC1090
source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
set -u

MPI_HEADER_DIR=""
if command -v dpkg-architecture >/dev/null 2>&1; then
  MULTIARCH="$(dpkg-architecture -qDEB_HOST_MULTIARCH 2>/dev/null || true)"
  if [ -f "/usr/lib/${MULTIARCH}/openmpi/include/mpi.h" ]; then
    MPI_HEADER_DIR="/usr/lib/${MULTIARCH}/openmpi/include"
  fi
fi

export MAKEFLAGS="-j${BUILD_JOBS}"
export CMAKE_BUILD_PARALLEL_LEVEL="$BUILD_JOBS"
export CMAKE_PREFIX_PATH="${DEPS_PREFIX}${CMAKE_PREFIX_PATH:+:${CMAKE_PREFIX_PATH}}"

COLCON_ARGS=(
  --log-base "${SCRIPT_DIR}/log"
  build
  --base-paths "$SOURCE_DIR"
  --build-base "${SCRIPT_DIR}/build"
  --install-base "${SCRIPT_DIR}/install"
  --symlink-install
  --parallel-workers 1
)
if [ "$PACKAGE_MODE" != "all" ]; then
  COLCON_ARGS+=("--packages-${PACKAGE_MODE}" "${PACKAGE_NAMES[@]}")
fi
COLCON_ARGS+=(--cmake-args
  -DCMAKE_BUILD_TYPE=Release
  "-DTRT_ENABLED=${TRT_ENABLED}"
  "-DPCL_DIR=${DEPS_PREFIX}/share/pcl-1.15"
  "-Dsmall_gicp_DIR=${DEPS_PREFIX}/lib/cmake/small_gicp"
  "-DGTSAM_DIR=/opt/ros/${ROS_DISTRO_NAME}/lib/cmake/GTSAM"
  "-DGTSAM_INCLUDE_DIRS=/opt/ros/${ROS_DISTRO_NAME}/include"
  "-DGTSAM_LIBRARIES=gtsam"
  "-DCMAKE_PREFIX_PATH=${DEPS_PREFIX};/opt/ros/${ROS_DISTRO_NAME}"
)
if [ -n "$MPI_HEADER_DIR" ]; then
  COLCON_ARGS+=(
    "-DMPI_C_HEADER_DIR=${MPI_HEADER_DIR}"
    "-DMPI_CXX_HEADER_DIR=${MPI_HEADER_DIR}"
  )
fi

echo "源码：${SOURCE_DIR}"
echo "依赖前缀：${DEPS_PREFIX}"
echo "线程：${BUILD_JOBS}（包顺序编译，单包最多 ${BUILD_JOBS} 线程）"
echo "TensorRT：${TRT_ENABLED}"
colcon "${COLCON_ARGS[@]}"

echo
echo "编译完成。使用前执行：source ${SCRIPT_DIR}/setup_dddmr.bash"
