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

# Include any dependencies generated for this target.
include src/CMakeFiles/vsa1control.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/vsa1control.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/vsa1control.dir/flags.make

src/CMakeFiles/vsa1control.dir/clientudp3.cpp.o: src/CMakeFiles/vsa1control.dir/flags.make
src/CMakeFiles/vsa1control.dir/clientudp3.cpp.o: ../src/clientudp3.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /usr/users/localuser/softdev/vsa1control/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/vsa1control.dir/clientudp3.cpp.o"
	cd /usr/users/localuser/softdev/vsa1control/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/vsa1control.dir/clientudp3.cpp.o -c /usr/users/localuser/softdev/vsa1control/src/clientudp3.cpp

src/CMakeFiles/vsa1control.dir/clientudp3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vsa1control.dir/clientudp3.cpp.i"
	cd /usr/users/localuser/softdev/vsa1control/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /usr/users/localuser/softdev/vsa1control/src/clientudp3.cpp > CMakeFiles/vsa1control.dir/clientudp3.cpp.i

src/CMakeFiles/vsa1control.dir/clientudp3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vsa1control.dir/clientudp3.cpp.s"
	cd /usr/users/localuser/softdev/vsa1control/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /usr/users/localuser/softdev/vsa1control/src/clientudp3.cpp -o CMakeFiles/vsa1control.dir/clientudp3.cpp.s

src/CMakeFiles/vsa1control.dir/clientudp3.cpp.o.requires:
.PHONY : src/CMakeFiles/vsa1control.dir/clientudp3.cpp.o.requires

src/CMakeFiles/vsa1control.dir/clientudp3.cpp.o.provides: src/CMakeFiles/vsa1control.dir/clientudp3.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/vsa1control.dir/build.make src/CMakeFiles/vsa1control.dir/clientudp3.cpp.o.provides.build
.PHONY : src/CMakeFiles/vsa1control.dir/clientudp3.cpp.o.provides

src/CMakeFiles/vsa1control.dir/clientudp3.cpp.o.provides.build: src/CMakeFiles/vsa1control.dir/clientudp3.cpp.o

src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.o: src/CMakeFiles/vsa1control.dir/flags.make
src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.o: ../src/pidcontroller.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /usr/users/localuser/softdev/vsa1control/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.o"
	cd /usr/users/localuser/softdev/vsa1control/build/src && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/vsa1control.dir/pidcontroller.cpp.o -c /usr/users/localuser/softdev/vsa1control/src/pidcontroller.cpp

src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vsa1control.dir/pidcontroller.cpp.i"
	cd /usr/users/localuser/softdev/vsa1control/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /usr/users/localuser/softdev/vsa1control/src/pidcontroller.cpp > CMakeFiles/vsa1control.dir/pidcontroller.cpp.i

src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vsa1control.dir/pidcontroller.cpp.s"
	cd /usr/users/localuser/softdev/vsa1control/build/src && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /usr/users/localuser/softdev/vsa1control/src/pidcontroller.cpp -o CMakeFiles/vsa1control.dir/pidcontroller.cpp.s

src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.o.requires:
.PHONY : src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.o.requires

src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.o.provides: src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.o.requires
	$(MAKE) -f src/CMakeFiles/vsa1control.dir/build.make src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.o.provides.build
.PHONY : src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.o.provides

src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.o.provides.build: src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.o

# Object files for target vsa1control
vsa1control_OBJECTS = \
"CMakeFiles/vsa1control.dir/clientudp3.cpp.o" \
"CMakeFiles/vsa1control.dir/pidcontroller.cpp.o"

# External object files for target vsa1control
vsa1control_EXTERNAL_OBJECTS =

src/libvsa1control.so.UNKNOWN-dirty: src/CMakeFiles/vsa1control.dir/clientudp3.cpp.o
src/libvsa1control.so.UNKNOWN-dirty: src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.o
src/libvsa1control.so.UNKNOWN-dirty: src/CMakeFiles/vsa1control.dir/build.make
src/libvsa1control.so.UNKNOWN-dirty: src/CMakeFiles/vsa1control.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library libvsa1control.so"
	cd /usr/users/localuser/softdev/vsa1control/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vsa1control.dir/link.txt --verbose=$(VERBOSE)
	cd /usr/users/localuser/softdev/vsa1control/build/src && $(CMAKE_COMMAND) -E cmake_symlink_library libvsa1control.so.UNKNOWN-dirty libvsa1control.so.UNKNOWN-dirty libvsa1control.so

src/libvsa1control.so: src/libvsa1control.so.UNKNOWN-dirty

# Rule to build all files generated by this target.
src/CMakeFiles/vsa1control.dir/build: src/libvsa1control.so
.PHONY : src/CMakeFiles/vsa1control.dir/build

src/CMakeFiles/vsa1control.dir/requires: src/CMakeFiles/vsa1control.dir/clientudp3.cpp.o.requires
src/CMakeFiles/vsa1control.dir/requires: src/CMakeFiles/vsa1control.dir/pidcontroller.cpp.o.requires
.PHONY : src/CMakeFiles/vsa1control.dir/requires

src/CMakeFiles/vsa1control.dir/clean:
	cd /usr/users/localuser/softdev/vsa1control/build/src && $(CMAKE_COMMAND) -P CMakeFiles/vsa1control.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/vsa1control.dir/clean

src/CMakeFiles/vsa1control.dir/depend:
	cd /usr/users/localuser/softdev/vsa1control/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /usr/users/localuser/softdev/vsa1control /usr/users/localuser/softdev/vsa1control/src /usr/users/localuser/softdev/vsa1control/build /usr/users/localuser/softdev/vsa1control/build/src /usr/users/localuser/softdev/vsa1control/build/src/CMakeFiles/vsa1control.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/vsa1control.dir/depend

