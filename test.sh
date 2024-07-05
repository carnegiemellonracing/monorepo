cmake -S . -B build -DENABLE_TESTING=TRUE "-D__WFI()=__asm volatile ("nop")"  -DCMAKE_CXX_COMPILER=g++-13 -DCMAKE_C_COMPILER=gcc-13
cmake --build build -j12