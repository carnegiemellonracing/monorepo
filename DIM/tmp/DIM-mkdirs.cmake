# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/jeffreyshen/Desktop/Vscode/monorepo/DIM")
  file(MAKE_DIRECTORY "/Users/jeffreyshen/Desktop/Vscode/monorepo/DIM")
endif()
file(MAKE_DIRECTORY
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/DIM/src/DIM-build"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/DIM"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/DIM/tmp"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/DIM/src/DIM-stamp"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/DIM/src"
  "/Users/jeffreyshen/Desktop/Vscode/monorepo/DIM/src/DIM-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/jeffreyshen/Desktop/Vscode/monorepo/DIM/src/DIM-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/jeffreyshen/Desktop/Vscode/monorepo/DIM/src/DIM-stamp${cfgdir}") # cfgdir has leading slash
endif()
