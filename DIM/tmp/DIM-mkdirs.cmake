# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/DIM"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/DIM/src/DIM-build"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/DIM"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/DIM/tmp"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/DIM/src/DIM-stamp"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/DIM/src"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/DIM/src/DIM-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/nidhivadlamudi/Desktop/CMR/monorepo/DIM/src/DIM-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/nidhivadlamudi/Desktop/CMR/monorepo/DIM/src/DIM-stamp${cfgdir}") # cfgdir has leading slash
endif()
