#!/bin/bash
# 交叉编译: 在 WSL(Ubuntu-22.04) 上用 NVIDIA gcc9.3 工具链编 aarch64 版本
set -e
ROOT_PWD=$(cd "$(dirname "$0")" && pwd)
BUILD_DIR=${ROOT_PWD}/build-cross
TC_BIN=/home/sura/toolchain/toolchain/nx_toolchains/nvidia-gcc93/bin
PSDK=${PSDK_ROOT:-/home/sura/work/Payload-SDK-3.17.0}

rm -rf "${BUILD_DIR}"
mkdir -p "${BUILD_DIR}"
cd "${BUILD_DIR}"

cmake .. \
  -DCMAKE_SYSTEM_NAME=Linux \
  -DCMAKE_SYSTEM_PROCESSOR=aarch64 \
  -DCMAKE_C_COMPILER=${TC_BIN}/aarch64-buildroot-linux-gnu-gcc \
  -DCMAKE_CXX_COMPILER=${TC_BIN}/aarch64-buildroot-linux-gnu-g++ \
  -DPSDK_ROOT=${PSDK}

make -j"$(nproc)"
file "${BUILD_DIR}/m3_widget_gimbal"
echo "cross build ok: ${BUILD_DIR}/m3_widget_gimbal"
