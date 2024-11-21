# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/HVC"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/HVC/src/HVC-build"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/HVC"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/HVC/tmp"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/HVC/src/HVC-stamp"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/HVC/src"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/HVC/src/HVC-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/HVC/src/HVC-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/HVC/src/HVC-stamp${cfgdir}") # cfgdir has leading slash
endif()
