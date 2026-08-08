# Specify the version being used aswell as the language
cmake_minimum_required(VERSION 3.24)


#****************************************************************************************
# Project configuration
#****************************************************************************************
# Specify the project name
project(FlashCli)

# Build debug version by default
set(CMAKE_BUILD_TYPE "Debug")

#****************************************************************************************
# Directories
#****************************************************************************************
# Set the output directory
set (PROJECT_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(OPENBLT_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../openblt/")


# Set OpenBLT library related directory locations
set(LIBOPENBLT_INC ${OPENBLT_ROOT}/Host/Source/LibOpenBLT/)
set(LIBOPENBLT_CMAKE_DIR ${OPENBLT_ROOT}/Host/Source/LibOpenBLT)

set(CMR_DRIVERS "${CMAKE_CURRENT_SOURCE_DIR}/../../stm32f413-drivers/CMR")

# Build OpenBLT as a static library from this project.
set(BUILD_SHARED OFF CACHE BOOL "Disable shared OpenBLT library" FORCE)
set(BUILD_STATIC ON CACHE BOOL "Enable static OpenBLT library" FORCE)
add_subdirectory(${LIBOPENBLT_CMAKE_DIR} ${CMAKE_BINARY_DIR}/openblt)


#****************************************************************************************
# Compiler flags
#****************************************************************************************
# Set platform specific compiler macros PLATFORM_XXX
if(WIN32)
  if(CMAKE_C_COMPILER_ID MATCHES GNU) 
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPLATFORM_WINDOWS -DPLATFORM_32BIT -D_CRT_SECURE_NO_WARNINGS -std=gnu99")
    else()
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPLATFORM_WINDOWS -DPLATFORM_64BIT -D_CRT_SECURE_NO_WARNINGS -std=gnu99")
    endif()
  elseif(CMAKE_C_COMPILER_ID MATCHES MSVC)
    if(CMAKE_SIZEOF_VOID_P EQUAL 4)
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPLATFORM_WINDOWS -DPLATFORM_32BIT -D_CRT_SECURE_NO_WARNINGS")
    else()
      set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPLATFORM_WINDOWS -DPLATFORM_64BIT -D_CRT_SECURE_NO_WARNINGS")
    endif()
  endif()
elseif(UNIX)
  if(CMAKE_SIZEOF_VOID_P EQUAL 4)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPLATFORM_LINUX -DPLATFORM_32BIT -D_TIME_BITS=64 -D_FILE_OFFSET_BITS=64 -pthread -std=gnu99")
  else()
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DPLATFORM_LINUX -DPLATFORM_64BIT -D_TIME_BITS=64 -D_FILE_OFFSET_BITS=64 -pthread -std=gnu99")
  endif()
endif(WIN32)

# Configure a statically linked run-time library for msvc
if(WIN32)
  if(CMAKE_C_COMPILER_ID MATCHES MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
  endif()
endif(WIN32)


#***************************************************************************************
# Includes
#****************************************************************************************
# Set include directories
include_directories("${PROJECT_SOURCE_DIR}/Inc" "${LIBOPENBLT_INC}" "${CMR_DRIVERS}/include")


#***************************************************************************************
# Files
#****************************************************************************************
# Get header files from the root directory
file(GLOB INCS_ROOT "Inc/*.h")
set(INCS ${INCS_ROOT})

# Add sources
file(GLOB SRC_ROOT "Src/*.c")

set(
  PROG_SRCS
  ${INCS}
  ${SRC_ROOT}
)


#***************************************************************************************
# Search path
#****************************************************************************************
# Add the path of the executable to the library search path. This way the LibOpenBLT
# shared library can simply be in the same directory as the FlashCli executable.
set(CMAKE_SKIP_BUILD_RPATH  FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH "\$ORIGIN")


#***************************************************************************************
# Targets
#****************************************************************************************
add_executable(
  FlashCli
  ${PROG_SRCS}
)

# Add libraries.
target_link_libraries(FlashCli openblt_static)

if(WIN32)
  target_link_libraries(FlashCli ws2_32 setupapi winusb)
endif()

# Only generate the PC-lint taget if the option is enabled. Use "make FlashCli_LINT"
# to lint the project sources
if(LINT_ENABLED)
  # Include PC-lint configuration file for the correct compiler. Currently GNU GCC and
  # Microsoft Visual Studio are supported.
  if(CMAKE_C_COMPILER_ID MATCHES GNU)
    include(${PROJECT_SOURCE_DIR}/lint/gnu/pc_lint.cmake)
  elseif(CMAKE_C_COMPILER_ID MATCHES MSVC)
    include(${PROJECT_SOURCE_DIR}/lint/msvc/pc_lint.cmake)
  endif()

  # Generate the PC-lint target.
  if(COMMAND add_pc_lint)
    add_pc_lint(FlashCli ${PROG_SRCS})
  endif(COMMAND add_pc_lint)
endif(LINT_ENABLED)
