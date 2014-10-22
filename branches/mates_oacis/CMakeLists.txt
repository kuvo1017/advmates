# cmake version
cmake_minimum_required(VERSION 2.6)

# project
project(AdvMATES)
set(serial "0.3.2")

# start cmake
message(STATUS "*** cmake: ${CMAKE_CURRENT_SOURCE_DIR} ***")
set(CMAKE_VERBOSE_MAKEFILE TRUE)

#----------------------------------------------------------------------
# platform detection
if(NOT PLATFORM)
  set(PLATFORM ${CMAKE_SYSTEM_NAME})
endif(NOT PLATFORM)
string(TOLOWER ${PLATFORM} PLATFORM)

if(${PLATFORM} STREQUAL "windows")
  if(${CYGWIN})
    set(PLATFORM "cygwin")
  elseif(${MINGW})
    set(PLATFORM "mingw")
  endif(${CYGWIN})
endif(${PLATFORM} STREQUAL "windows")

message(STATUS "PLATFORM: ${PLATFORM}")

#----------------------------------------------------------------------
# duplicate custom targets
set_property(GLOBAL PROPERTY ALLOW_DUPLICATE_CUSTOM_TARGETS TRUE)

#----------------------------------------------------------------------
# subdirectories
# - build libraries before solvers
# - build solvers before pre modules
if(${PLATFORM} STREQUAL "novis")
  subdirs(
    lib/mt19937
    solver
    )
else(${PLATFORM} STREQUAL "novis")
  subdirs(
    lib/autogl lib/mt19937
    solver
    )
endif(${PLATFORM} STREQUAL "novis")
