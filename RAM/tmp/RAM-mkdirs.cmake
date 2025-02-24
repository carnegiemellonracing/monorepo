# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/RAM"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/RAM/src/RAM-build"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/RAM"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/RAM/tmp"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/RAM/src/RAM-stamp"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/RAM/src"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/RAM/src/RAM-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/nidhivadlamudi/Desktop/CMR/monorepo/RAM/src/RAM-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/nidhivadlamudi/Desktop/CMR/monorepo/RAM/src/RAM-stamp${cfgdir}") # cfgdir has leading slash
endif()
