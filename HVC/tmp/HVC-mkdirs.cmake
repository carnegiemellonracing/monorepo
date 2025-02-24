# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/HVC"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/HVC/src/HVC-build"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/HVC"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/HVC/tmp"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/HVC/src/HVC-stamp"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/HVC/src"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/HVC/src/HVC-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/nidhivadlamudi/Desktop/CMR/monorepo/HVC/src/HVC-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/nidhivadlamudi/Desktop/CMR/monorepo/HVC/src/HVC-stamp${cfgdir}") # cfgdir has leading slash
endif()
