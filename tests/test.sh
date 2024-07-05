BASEDIR=$(cd $(dirname $0) && pwd)
cmake=/usr/bin/cmake/bin/cmake

cmake -S . -B build -DCMAKE_CXX_COMPILER=g++-13 -DCMAKE_C_COMPILER=gcc-13
cmake --build build -j12
(cd build && ctest)