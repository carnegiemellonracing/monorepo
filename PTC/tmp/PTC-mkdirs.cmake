# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/kavehfayyazi/monorepo/PTC"
  "/Users/kavehfayyazi/monorepo/PTC/src/PTC-build"
  "/Users/kavehfayyazi/monorepo/PTC"
  "/Users/kavehfayyazi/monorepo/PTC/tmp"
  "/Users/kavehfayyazi/monorepo/PTC/src/PTC-stamp"
  "/Users/kavehfayyazi/monorepo/PTC/src"
  "/Users/kavehfayyazi/monorepo/PTC/src/PTC-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/kavehfayyazi/monorepo/PTC/src/PTC-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/kavehfayyazi/monorepo/PTC/src/PTC-stamp${cfgdir}") # cfgdir has leading slash
endif()
