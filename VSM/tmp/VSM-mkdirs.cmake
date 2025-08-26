# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/anvitanadgir/Documents/GitHub/monorepo/VSM"
  "/Users/anvitanadgir/Documents/GitHub/monorepo/VSM/src/VSM-build"
  "/Users/anvitanadgir/Documents/GitHub/monorepo/VSM"
  "/Users/anvitanadgir/Documents/GitHub/monorepo/VSM/tmp"
  "/Users/anvitanadgir/Documents/GitHub/monorepo/VSM/src/VSM-stamp"
  "/Users/anvitanadgir/Documents/GitHub/monorepo/VSM/src"
  "/Users/anvitanadgir/Documents/GitHub/monorepo/VSM/src/VSM-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/anvitanadgir/Documents/GitHub/monorepo/VSM/src/VSM-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/anvitanadgir/Documents/GitHub/monorepo/VSM/src/VSM-stamp${cfgdir}") # cfgdir has leading slash
endif()
