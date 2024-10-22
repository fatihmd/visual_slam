# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

# Default target executed when no arguments are given to make.
default_target: all

.PHONY : default_target

# Allow only one "make -f Makefile2" at a time, but pass parallelism.
.NOTPARALLEL:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/administrator/slam_test/visual_slam

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/administrator/slam_test/visual_slam

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache

.PHONY : rebuild_cache/fast

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake cache editor..."
	/usr/bin/cmake-gui -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache

.PHONY : edit_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start /home/administrator/slam_test/visual_slam/CMakeFiles /home/administrator/slam_test/visual_slam/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start /home/administrator/slam_test/visual_slam/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean

.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named tools

# Build rule for target.
tools: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 tools
.PHONY : tools

# fast build rule for target.
tools/fast:
	$(MAKE) -f CMakeFiles/tools.dir/build.make CMakeFiles/tools.dir/build
.PHONY : tools/fast

#=============================================================================
# Target rules for targets named main

# Build rule for target.
main: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 main
.PHONY : main

# fast build rule for target.
main/fast:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/build
.PHONY : main/fast

src/sfm.o: src/sfm.cpp.o

.PHONY : src/sfm.o

# target to build an object file
src/sfm.cpp.o:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/src/sfm.cpp.o
.PHONY : src/sfm.cpp.o

src/sfm.i: src/sfm.cpp.i

.PHONY : src/sfm.i

# target to preprocess a source file
src/sfm.cpp.i:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/src/sfm.cpp.i
.PHONY : src/sfm.cpp.i

src/sfm.s: src/sfm.cpp.s

.PHONY : src/sfm.s

# target to generate assembly for a file
src/sfm.cpp.s:
	$(MAKE) -f CMakeFiles/main.dir/build.make CMakeFiles/main.dir/src/sfm.cpp.s
.PHONY : src/sfm.cpp.s

src/tools.o: src/tools.cpp.o

.PHONY : src/tools.o

# target to build an object file
src/tools.cpp.o:
	$(MAKE) -f CMakeFiles/tools.dir/build.make CMakeFiles/tools.dir/src/tools.cpp.o
.PHONY : src/tools.cpp.o

src/tools.i: src/tools.cpp.i

.PHONY : src/tools.i

# target to preprocess a source file
src/tools.cpp.i:
	$(MAKE) -f CMakeFiles/tools.dir/build.make CMakeFiles/tools.dir/src/tools.cpp.i
.PHONY : src/tools.cpp.i

src/tools.s: src/tools.cpp.s

.PHONY : src/tools.s

# target to generate assembly for a file
src/tools.cpp.s:
	$(MAKE) -f CMakeFiles/tools.dir/build.make CMakeFiles/tools.dir/src/tools.cpp.s
.PHONY : src/tools.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... rebuild_cache"
	@echo "... tools"
	@echo "... edit_cache"
	@echo "... main"
	@echo "... src/sfm.o"
	@echo "... src/sfm.i"
	@echo "... src/sfm.s"
	@echo "... src/tools.o"
	@echo "... src/tools.i"
	@echo "... src/tools.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

