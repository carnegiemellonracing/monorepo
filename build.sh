#!/bin/bash
BASEDIR=$(cd $(dirname $0) && pwd)
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_TOOLCHAIN_FILE="${BASEDIR}\cmake\gcc-arm-none-eabi.cmake" -SC:"${BASEDIR}" -BC:"${BASEDIR}\build\debug" -G Ninja
cmake --build "${BASEDIR}\build\debug" --target RAM --