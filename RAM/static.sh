#!/bin/bash
BASEDIR=$(cd $(dirname $0) && pwd)
rm -rf build/static
cmake --fresh -DCMAKE_BUILD_TYPE=Static -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE="${BASEDIR}/cmake/gcc-arm-none-eabi.cmake" -S"${BASEDIR}" -B"${BASEDIR}/build/static" -G Ninja
cmake --build "${BASEDIR}/build/static" --target RAM --