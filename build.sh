#!/bin/bash
# 编译: 在妙算3上直接执行 ./build.sh
set -e
ROOT_PWD=$(cd "$(dirname "$0")" && pwd)
BUILD_DIR=${ROOT_PWD}/build

mkdir -p ${BUILD_DIR}
cd ${BUILD_DIR}
cmake .. -DPSDK_ROOT=${PSDK_ROOT:-/home/dji/Payload-SDK-3.17.0}
make -j4
echo "build ok: ${BUILD_DIR}/m3_widget_gimbal"
