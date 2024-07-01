#!/bin/bash
BASEDIR=$(cd $(dirname $0) && pwd)
cmake=/usr/bin/cmake/bin/cmake
$cmake -DCMAKE_BUILD_TYPE=Static -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE="${BASEDIR}/cmake/gcc-arm-none-eabi.cmake" -S"${BASEDIR}" -B"${BASEDIR}/build/debug" -G Ninja
$cmake --build "${BASEDIR}/build/debug" --target RAM --
