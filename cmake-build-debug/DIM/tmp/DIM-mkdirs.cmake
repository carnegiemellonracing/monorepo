# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/DIM"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/DIM/src/DIM-build"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/DIM"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/DIM/tmp"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/DIM/src/DIM-stamp"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/DIM/src"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/DIM/src/DIM-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/DIM/src/DIM-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/jeffreyshen/Desktop/Vscode/monorepo/cmake-build-debug/DIM/src/DIM-stamp${cfgdir}") # cfgdir has leading slash
endif()
