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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = /usr/bin/ccmake

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /usr/users/localuser/softdev/vsa1control

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /usr/users/localuser/softdev/vsa1control/build

# Utility rule file for deb.

# Include the progress variables for this target.
include CMakeFiles/deb.dir/progress.make

CMakeFiles/deb:
	$(CMAKE_COMMAND) -E cmake_progress_report /usr/users/localuser/softdev/vsa1control/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating Debian package..."
	cd /usr/users/localuser/softdev/vsa1control && git-buildpackage --git-debian-branch=debian --git-builder="debuild\ -S\ -i.git\ -I.git"

deb: CMakeFiles/deb
deb: CMakeFiles/deb.dir/build.make
.PHONY : deb

# Rule to build all files generated by this target.
CMakeFiles/deb.dir/build: deb
.PHONY : CMakeFiles/deb.dir/build

CMakeFiles/deb.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/deb.dir/cmake_clean.cmake
.PHONY : CMakeFiles/deb.dir/clean

CMakeFiles/deb.dir/depend:
	cd /usr/users/localuser/softdev/vsa1control/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/users/localuser/softdev/vsa1control /usr/users/localuser/softdev/vsa1control /usr/users/localuser/softdev/vsa1control/build /usr/users/localuser/softdev/vsa1control/build /usr/users/localuser/softdev/vsa1control/build/CMakeFiles/deb.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/deb.dir/depend

