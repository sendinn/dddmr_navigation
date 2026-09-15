#!/usr/bin/env bash
set -Eeuo pipefail

trap 'echo "依赖安装失败：第 ${LINENO} 行，请检查上方错误。"' ERR

if [ "${EUID}" -eq 0 ]; then
  echo "请不要用 sudo 运行本脚本；仅 apt 步骤会调用 sudo。"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOURCE_DIR="${SCRIPT_DIR}/src"
DEPS_DIR="${SCRIPT_DIR}/.deps"
PCL_SOURCE="${DEPS_DIR}/src/pcl"
PCL_BUILD="${DEPS_DIR}/build/pcl"
SMALL_GICP_SOURCE="${DEPS_DIR}/src/small_gicp"
SMALL_GICP_BUILD="${DEPS_DIR}/build/small_gicp"
DEPS_PREFIX="${DEPS_DIR}/install"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
PCL_TAG="pcl-1.15.0"
SMALL_GICP_TAG="v1.0.1"
BUILD_JOBS="${DDDMR_BUILD_JOBS:-2}"
SKIP_APT="False"

usage() {
  echo "用法：./install.sh [--skip-apt]"
  echo "  默认优先安装 Ubuntu/ROS ARM64 二进制依赖。"
  echo "  PCL 1.15 和 small_gicp 无 Ubuntu 22.04 ARM64 二进制，固定版本安装到 .deps。"
}

for arg in "$@"; do
  case "$arg" in
    --skip-apt) SKIP_APT="True" ;;
    -h|--help) usage; exit 0 ;;
    *) echo "未知参数：$arg"; usage; exit 2 ;;
  esac
done

if [ ! -f "/opt/ros/${ROS_DISTRO_NAME}/setup.bash" ]; then
  echo "未找到 ROS 2 ${ROS_DISTRO_NAME}。"
  exit 1
fi
if [ ! -d "$SOURCE_DIR" ]; then
  echo "未找到新仓库源码目录：${SOURCE_DIR}"
  exit 1
fi

if [ "$SKIP_APT" = "False" ]; then
  sudo apt-get update
  sudo apt-get install -y \
    build-essential cmake git pkg-config \
    python3-colcon-common-extensions python3-rosdep python3-catkin-pkg-modules \
    libeigen3-dev libboost-all-dev libmetis-dev libpcl-dev \
    libyaml-cpp-dev \
    libflann-dev libqhull-dev libvtk9-dev libopenni-dev libopenni2-dev \
    libpng-dev libjpeg-dev libtiff-dev liblz4-dev \
    libopenmpi-dev openmpi-bin libpcap-dev libusb-1.0-0-dev \
    libopencv-dev python3-opencv libfreeimage-dev libasio-dev can-utils \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    "ros-${ROS_DISTRO_NAME}-gtsam" \
    "ros-${ROS_DISTRO_NAME}-ackermann-msgs" \
    "ros-${ROS_DISTRO_NAME}-yaml-cpp-vendor"

  if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
  fi
fi

mkdir -p "${DEPS_DIR}/src" "${DEPS_DIR}/build" "$DEPS_PREFIX"

# Jammy 的 ARM64 仓库只有 PCL 1.12，而当前 DDDMR 原源码明确要求 1.15。
# 使用上游固定 tag 本地安装，既不修改 DDDMR 源码，也不覆盖系统 PCL。
if [ -f "${DEPS_PREFIX}/share/pcl-1.15/PCLConfig.cmake" ]; then
  echo "PCL 1.15.0 已安装：${DEPS_PREFIX}"
else
  if [ ! -d "${PCL_SOURCE}/.git" ]; then
    if [ -e "$PCL_SOURCE" ]; then
      echo "${PCL_SOURCE} 已存在但不是 Git 仓库，请先人工检查。"
      exit 1
    fi
    git clone --branch "$PCL_TAG" --depth 1 \
      https://github.com/PointCloudLibrary/pcl.git "$PCL_SOURCE"
  fi
  cmake -S "$PCL_SOURCE" -B "$PCL_BUILD" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$DEPS_PREFIX" \
    -DPCL_ENABLE_AVX=OFF -DPCL_ENABLE_SSE=OFF -DPCL_ENABLE_MARCHNATIVE=OFF \
    -DBUILD_apps=OFF -DBUILD_examples=OFF -DBUILD_global_tests=OFF \
    -DBUILD_tools=OFF -DBUILD_simulation=OFF -DBUILD_cuda=OFF -DBUILD_gpu=OFF
  cmake --build "$PCL_BUILD" --parallel "$BUILD_JOBS"
  cmake --install "$PCL_BUILD"
fi

if [ -f "${DEPS_PREFIX}/lib/cmake/small_gicp/small_gicp-config.cmake" ]; then
  echo "small_gicp ${SMALL_GICP_TAG} 已安装：${DEPS_PREFIX}"
else
  if [ ! -d "${SMALL_GICP_SOURCE}/.git" ]; then
    if [ -e "$SMALL_GICP_SOURCE" ]; then
      echo "${SMALL_GICP_SOURCE} 已存在但不是 Git 仓库，请先人工检查。"
      exit 1
    fi
    git clone --branch "$SMALL_GICP_TAG" --depth 1 \
      https://github.com/koide3/small_gicp.git "$SMALL_GICP_SOURCE"
  fi
  cmake -S "$SMALL_GICP_SOURCE" -B "$SMALL_GICP_BUILD" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$DEPS_PREFIX" \
    -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF -DBUILD_BENCHMARKS=OFF \
    -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_WITH_MARCH_NATIVE=OFF
  cmake --build "$SMALL_GICP_BUILD" --parallel "$BUILD_JOBS"
  cmake --install "$SMALL_GICP_BUILD"
fi

if [ "$SKIP_APT" = "False" ]; then
  rosdep update
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO_NAME}/setup.bash"
  set -u
  # Local CMake-only dependencies are intentionally outside rosdep.
  rosdep install --from-paths "$SOURCE_DIR" --ignore-src \
    --rosdistro "$ROS_DISTRO_NAME" \
    --skip-keys "libyaml-cpp-dev yaml-cpp_vendor" -r -y
fi

echo
echo "依赖准备完成："
echo "  GTSAM/OpenMPI 等：Ubuntu/ROS ARM64 二进制包"
echo "  PCL 1.15.0：${DEPS_PREFIX}（Jammy 无所需版本二进制）"
echo "  small_gicp ${SMALL_GICP_TAG}：${DEPS_PREFIX}（无 apt 二进制）"
echo "下一步：./build.sh --jobs ${BUILD_JOBS}"
