# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/johnli/CMR/monorepo/RAM"
  "/Users/johnli/CMR/monorepo/RAM/src/RAM-build"
  "/Users/johnli/CMR/monorepo/RAM"
  "/Users/johnli/CMR/monorepo/RAM/tmp"
  "/Users/johnli/CMR/monorepo/RAM/src/RAM-stamp"
  "/Users/johnli/CMR/monorepo/RAM/src"
  "/Users/johnli/CMR/monorepo/RAM/src/RAM-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/johnli/CMR/monorepo/RAM/src/RAM-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/johnli/CMR/monorepo/RAM/src/RAM-stamp${cfgdir}") # cfgdir has leading slash
endif()
