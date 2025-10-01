# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/ayushgarg/Documents/GutHub/monorepo/VSM"
  "/Users/ayushgarg/Documents/GutHub/monorepo/VSM/src/VSM-build"
  "/Users/ayushgarg/Documents/GutHub/monorepo/VSM"
  "/Users/ayushgarg/Documents/GutHub/monorepo/VSM/tmp"
  "/Users/ayushgarg/Documents/GutHub/monorepo/VSM/src/VSM-stamp"
  "/Users/ayushgarg/Documents/GutHub/monorepo/VSM/src"
  "/Users/ayushgarg/Documents/GutHub/monorepo/VSM/src/VSM-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/ayushgarg/Documents/GutHub/monorepo/VSM/src/VSM-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/ayushgarg/Documents/GutHub/monorepo/VSM/src/VSM-stamp${cfgdir}") # cfgdir has leading slash
endif()
