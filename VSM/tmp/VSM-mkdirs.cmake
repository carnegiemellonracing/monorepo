# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "C:/Users/nosla/Downloads/monorepo/VSM"
  "C:/Users/nosla/Downloads/monorepo/VSM/src/VSM-build"
  "C:/Users/nosla/Downloads/monorepo/VSM"
  "C:/Users/nosla/Downloads/monorepo/VSM/tmp"
  "C:/Users/nosla/Downloads/monorepo/VSM/src/VSM-stamp"
  "C:/Users/nosla/Downloads/monorepo/VSM/src"
  "C:/Users/nosla/Downloads/monorepo/VSM/src/VSM-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Users/nosla/Downloads/monorepo/VSM/src/VSM-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Users/nosla/Downloads/monorepo/VSM/src/VSM-stamp${cfgdir}") # cfgdir has leading slash
endif()
