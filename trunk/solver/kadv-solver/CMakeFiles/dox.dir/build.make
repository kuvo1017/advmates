# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Produce verbose output by default.
VERBOSE = 1

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/2.8.11.2/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/2.8.11.2/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/local/Cellar/cmake/2.8.11.2/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/shikichi_takuya/kadv-mates/trunk/solver

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/shikichi_takuya/kadv-mates/trunk/solver

# Utility rule file for dox.

# Include the progress variables for this target.
include CMakeFiles/dox.dir/progress.make

CMakeFiles/dox:
	cd /Users/shikichi_takuya/kadv-mates/trunk/doc/doxygen && doxygen /Users/shikichi_takuya/kadv-mates/trunk/solver/../doc/doxygen/advmates-dox-min

dox: CMakeFiles/dox
dox: CMakeFiles/dox.dir/build.make
.PHONY : dox

# Rule to build all files generated by this target.
CMakeFiles/dox.dir/build: dox
.PHONY : CMakeFiles/dox.dir/build

CMakeFiles/dox.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/dox.dir/cmake_clean.cmake
.PHONY : CMakeFiles/dox.dir/clean

CMakeFiles/dox.dir/depend:
	cd /Users/shikichi_takuya/kadv-mates/trunk/solver && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/shikichi_takuya/kadv-mates/trunk/solver /Users/shikichi_takuya/kadv-mates/trunk/solver /Users/shikichi_takuya/kadv-mates/trunk/solver /Users/shikichi_takuya/kadv-mates/trunk/solver /Users/shikichi_takuya/kadv-mates/trunk/solver/CMakeFiles/dox.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/dox.dir/depend
