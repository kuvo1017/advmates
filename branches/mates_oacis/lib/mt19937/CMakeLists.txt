#----------------------------------------------------------------------
#
# Mersenne Twister : a pseudorandom number generator
#
#----------------------------------------------------------------------
# cmake version
cmake_minimum_required(VERSION 2.6)

# start cmake
message(STATUS "*** cmake: ${CMAKE_CURRENT_SOURCE_DIR} ***")
set(CMAKE_VERBOSE_MAKEFILE TRUE)

# library
set(LIBMT19937 "mt19937")

# make test program (default: false)
set(BUILD_TEST_EXEC FALSE)
set(MT19937_TEST_EXEC "mtTest")

#----------------------------------------------------------------------
# compiler
set(CMAKE_C_COMPILER gcc)

# compile options
# build type = Debug (defualt)
if(NOT CMAKE_BUILD_TYPE)
  message(STATUS "build type is not specified, set to debug build")
  set(CMAKE_BUILD_TYPE "debug")
endif(NOT CMAKE_BUILD_TYPE)
string(TOLOWER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)

if(${CMAKE_BUILD_TYPE} STREQUAL "winnative"
    AND NOT ${PLATFORM} STREQUAL "mingw")
  message(WARNING "winnative mode is only valid on mingw platform")
  set(CMAKE_BUILD_TYPE "debug")
endif(${CMAKE_BUILD_TYPE} STREQUAL "winnative"
  AND NOT ${PLATFORM} STREQUAL "mingw")

message(STATUS "configuring for ${CMAKE_BUILD_TYPE}...")

# options for release build
set(CMAKE_C_FLAGS_RELEASE   "-O3 -Wno-sign-compare")
#if(${CMAKE_BUILD_TYPE} STREQUAL "release")
#endif(${CMAKE_BUILD_TYPE} STREQUAL "release")

# options for windows native application
# #compile on MinGW
set(CMAKE_C_FLAGS_WINNATIVE   "-O3 -Wno-sign-compare -mwindows -static-libgcc")
#if(${CMAKE_BUILD_TYPE} STREQUAL "winnative")
#endif(${CMAKE_BUILD_TYPE} STREQUAL "winnative")

# options for debug build
set(CMAKE_C_FLAGS_DEBUG   "-g -Wall -Wno-sign-compare")
#if(${CMAKE_BUILD_TYPE} STREQUAL "debug")
#endif(${CMAKE_BUILD_TYPE} STREQUAL "debug")

#----------------------------------------------------------------------
# program
set(MT19937_HDRS
  mt19937ar.h
  )

set(MT19937_SRCS
  mt19937ar.c
  )

set(MT19937_TEST_SRCS
  mtTest.c
  )

#----------------------------------------------------------------------
# build library
add_library(${LIBMT19937} STATIC ${MT19937_SRCS})

# build test program
if(BUILD_TEST_EXEC)
  add_executable(${MT19937_TEST_EXEC} ${MT19937_TEST_SRCS})
  target_link_libraries(${MT19937_TEST_EXEC} ${LIBMT19937})
endif(BUILD_TEST_EXEC)

#----------------------------------------------------------------------
# clean target
set(CLEAN_TARGETS
  ${CMAKE_CURRENT_BINARY_DIR}/CMakeCache.txt
  ${CMAKE_CURRENT_BINARY_DIR}/cmake_install.cmake
  )

set_directory_properties(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES "${CLEAN_TARGETS}"
  )
