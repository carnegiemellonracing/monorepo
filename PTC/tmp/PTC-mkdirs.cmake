# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/PTC"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/PTC/src/PTC-build"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/PTC"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/PTC/tmp"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/PTC/src/PTC-stamp"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/PTC/src"
  "/Users/nidhivadlamudi/Desktop/CMR/monorepo/PTC/src/PTC-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/nidhivadlamudi/Desktop/CMR/monorepo/PTC/src/PTC-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/nidhivadlamudi/Desktop/CMR/monorepo/PTC/src/PTC-stamp${cfgdir}") # cfgdir has leading slash
endif()
