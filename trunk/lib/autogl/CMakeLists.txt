#----------------------------------------------------------------------
#
# ADV_AutoGL : a simple graphics & GUI library 
#              for making your own viewer
# (pronounced as "O-te-ga-lu")
#
#----------------------------------------------------------------------

# cmake version
cmake_minimum_required(VERSION 2.6)

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
# make autogl
set(BUILD_AUTOGL TRUE)

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
# find packages 

#
# for Linux
#
if(${PLATFORM} STREQUAL "linux")
  # OpenGL
  # The following variables will be defined:
  #   OPENGL_FOUND
  #   OPENGL_INCLUDE_DIR
  #   OPENGL_LIBRARIES
  message(STATUS "Finding OpenGL ... ${OPENGL_FOUND}")
  find_package(OpenGL)
  if(OPENGL_FOUND)
    include_directories(${OPENGL_INCLUDE_DIR})
  else(OPENGL_FOUND)
    set(BUILD_AUTOGL FALSE)
  endif(OPENGL_FOUND)

  # GTK2
  # The following variables will be defined:
  #   GTK2_FOUND
  #   GTK2_INCLUDE_DIRS
  #   GTK2_LIBRARIES
  message(STATUS "Finding GTK2 ... ${GTK2_FOUND}")
  find_package(GTK2)
  if(GTK2_FOUND)
    include_directories(${GTK2_INCLUDE_DIRS})
  else(GTK2_FOUND)
    set(BUILD_AUTOGL FALSE)
  endif(GTK2_FOUND)
endif(${PLATFORM} STREQUAL "linux")

#
# for Unix
#
if(${PLATFORM} STREQUAL "unix")
  # OpenGL
  # The following variables will be defined:
  #   OPENGL_FOUND
  #   OPENGL_INCLUDE_DIR
  #   OPENGL_LIBRARIES
  message(STATUS "Finding OpenGL ... ${OPENGL_FOUND}")
  find_package(OpenGL)
  if(OPENGL_FOUND)
    include_directories(${OPENGL_INCLUDE_DIR})
  endif(OPENGL_FOUND)

  # Motif
  # The folloing variables will be defined:
  #   MOTIF_FOUND
  #   MOTIF_INCLUDE_DIR
  #   MOTIF_LIBRARIES
  message(STATUS "Finding Motif ... ${MOTIF_FOUND}")
  find_package(Motif)
  if(MOTIF_FOUND)
    include_directories(${MOTIF_INCLUDE_DIR})
  endif(MOTIF_FOUND)
endif(${PLATFORM} STREQUAL "unix")

#
# for Darwin (OSX)
#
if(${PLATFORM} STREQUAL "darwin")
  # OpenGL
  # The following variables will be defined:
  #   OPENGL_FOUND
  #   OPENGL_INCLUDE_DIR
  #   OPENGL_LIBRARIES
  message(STATUS "Finding OpenGL ... ${OPENGL_FOUND}")
  find_package(OpenGL)
  if(OPENGL_FOUND)
  else(OPENGL_FOUND)
    set(BUILD_AUTOGL FALSE)
  endif(OPENGL_FOUND)

  # GTK2
  # The following variables will be defined:
  #   GTK2_FOUND
  #   GTK2_INCLUDE_DIRS
  #   GTK2_LIBRARIES
  message(STATUS "Finding GTK2 ... ${GTK2_FOUND}")
  find_package(GTK2)
  if(GTK2_FOUND)
    include_directories(${GTK2_INCLUDE_DIRS})
  else(GTK2_FOUND)
    set(BUILD_AUTOGL FALSE)
  endif(GTK2_FOUND)

  # X11
  # The following variables will be defined:
  #   X11_FOUND
  #   X11_INCLUDE_DIRS
  #   X11_LIBRARIES
  message(STATUS "Finding X11 ... ${X11_FOUND}")
  find_package(X11)
  if(X11_FOUND)
    include_directories(${X11_INCLUDE_DIR})
  message(STATUS "Found X11 ... ${X11_FOUND}") 
  else(X11_FOUND)
    set(BUILD_AUTOGL FALSE)
 message(STATUS "Not found X11 ... ${X11_FOUND}") 
  endif(X11_FOUND)
endif(${PLATFORM} STREQUAL "darwin")

#
# for Cygwin on Windows
#
if(PLATFORM STREQUAL "cygwin")
  set(CMAKE_C_FLAGS_RELEASE
    "${CMAKE_C_FLAGS_RELEASE} -DUSE_OPENGL32")
  set(CMAKE_C_FLAGS_DEBUG
    "${CMAKE_C_FLAGS_DEBUG} -DUSE_OPENGL32")
endif(PLATFORM STREQUAL "cygwin")

#
# for MinGW on Windows
#
if(PLATFORM STREQUAL "mingw")
  # OpenGL
  message(STATUS "Finding OpenGL ... ${OPENGL_FOUND}")
  find_package(OpenGL)
endif(PLATFORM STREQUAL "mingw")

#----------------------------------------------------------------------
# program
set(AUTOGL_HDRS
  autogl_os.h
  autogl_math.h
  autogl_uims.h
  autogl_graphics.h
  autogl_gui.h
  autogl_cui.h
  autogl_utility.h
  autogl_contour.h
  autogl_vislow.h
  autogl_visualization.h
  autogl_utility2d.h
  autogl_utility3d.h
  autogl.h 
  )

set(AUTOGL_SRCS
  autogl_os.c
  autogl_math.c
  autogl_uims.c
  autogl_graphics.c
  autogl_gui.c
  autogl_callback.c
  autogl_cui.c
  autogl_utility.c
  autogl_contour.c
  autogl_vislow.c
  autogl_visualization.c
  autogl_utility2d.c
  autogl_utility3d.c
)

if(${PLATFORM} STREQUAL "linux")
  set(AUTOGL_SRCS
    ${AUTOGL_SRCS}
    autogl_graphics_opengl.c
    autogl_gui_gtk_glx.c
    )
elseif(${PLATFORM} STREQUAL "unix")
  set(AUTOGL_SRCS
    ${AUTOGL_SRCS}
    autogl_graphics_opengl.c
    autogl_gui_motif.c
    )
elseif(${PLATFORM} STREQUAL "darwin")
  set(AUTOGL_SRCS
    ${AUTOGL_SRCS}
    autogl_graphics_opengl.c
    autogl_gui_gtk_glx.c
    )
elseif(${PLATFORM} STREQUAL "cygwin")
  set(AUTOGL_SRCS
    ${AUTOGL_SRCS}
    autogl_graphics_opengl.c
    autogl_gui_windows.c
    )
elseif(${PLATFORM} STREQUAL "mingw")
  set(AUTOGL_SRCS
    ${AUTOGL_SRCS}
    autogl_graphics_opengl.c
    autogl_gui_windows.c
    )
endif(${PLATFORM} STREQUAL "linux")

#----------------------------------------------------------------------
# add library
if(BUILD_AUTOGL)
  add_library(autogl STATIC ${AUTOGL_SRCS})
endif(BUILD_AUTOGL)

# copy advautogl-config
if(${PLATFORM} STREQUAL "linux")
  execute_process(
    COMMAND
    cp ${CMAKE_CURRENT_SOURCE_DIR}/advautogl-config__Linux
    ${CMAKE_CURRENT_SOURCE_DIR}/advautogl-config
    )
elseif(${PLATFORM} STREQUAL "unix")
  execute_process(
    COMMAND
    cp ${CMAKE_CURRENT_SOURCE_DIR}/advautogl-config__Unix
    ${CMAKE_CURRENT_SOURCE_DIR}/advautogl-config
    )
elseif(${PLATFORM} STREQUAL "darwin")
  execute_process(
    COMMAND
    cp ${CMAKE_CURRENT_SOURCE_DIR}/advautogl-config__Darwin
    ${CMAKE_CURRENT_SOURCE_DIR}/advautogl-config
    )
elseif(${PLATFORM} STREQUAL "cygwin")
  execute_process(
    COMMAND
    cp ${CMAKE_CURRENT_SOURCE_DIR}/advautogl-config__Cygwin
    ${CMAKE_CURRENT_SOURCE_DIR}/advautogl-config
    )
endif(${PLATFORM} STREQUAL "linux")

#----------------------------------------------------------------------
# clean target
set(CLEAN_TARGETS
  ${CMAKE_CURRENT_BINARY_DIR}/CMakeCache.txt
  ${CMAKE_CURRENT_BINARY_DIR}/cmake_install.cmake
  )

set_directory_properties(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES "${CLEAN_TARGETS}"
  )
