# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file LICENSE.rst or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION ${CMAKE_VERSION}) # this file comes with cmake

# If CMAKE_DISABLE_SOURCE_CHANGES is set to true and the source directory is an
# existing directory in our source tree, calling file(MAKE_DIRECTORY) on it
# would cause a fatal error, even though it would be a no-op.
if(NOT EXISTS "/Users/matthewmao/Desktop/monorepo/VSM")
  file(MAKE_DIRECTORY "/Users/matthewmao/Desktop/monorepo/VSM")
endif()
file(MAKE_DIRECTORY
  "/Users/matthewmao/Desktop/monorepo/VSM/src/VSM-build"
  "/Users/matthewmao/Desktop/monorepo/VSM"
  "/Users/matthewmao/Desktop/monorepo/VSM/tmp"
  "/Users/matthewmao/Desktop/monorepo/VSM/src/VSM-stamp"
  "/Users/matthewmao/Desktop/monorepo/VSM/src"
  "/Users/matthewmao/Desktop/monorepo/VSM/src/VSM-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/matthewmao/Desktop/monorepo/VSM/src/VSM-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/matthewmao/Desktop/monorepo/VSM/src/VSM-stamp${cfgdir}") # cfgdir has leading slash
endif()
