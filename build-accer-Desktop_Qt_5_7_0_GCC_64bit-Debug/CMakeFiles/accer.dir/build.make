# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.2

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
CMAKE_SOURCE_DIR = /home/hao/Desktop/accer/accer

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hao/Desktop/accer/build-accer-Desktop_Qt_5_7_0_GCC_64bit-Debug

# Include any dependencies generated for this target.
include CMakeFiles/accer.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/accer.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/accer.dir/flags.make

CMakeFiles/accer.dir/main.cpp.o: CMakeFiles/accer.dir/flags.make
CMakeFiles/accer.dir/main.cpp.o: /home/hao/Desktop/accer/accer/main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/hao/Desktop/accer/build-accer-Desktop_Qt_5_7_0_GCC_64bit-Debug/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/accer.dir/main.cpp.o"
	/usr/bin/g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/accer.dir/main.cpp.o -c /home/hao/Desktop/accer/accer/main.cpp

CMakeFiles/accer.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/accer.dir/main.cpp.i"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/hao/Desktop/accer/accer/main.cpp > CMakeFiles/accer.dir/main.cpp.i

CMakeFiles/accer.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/accer.dir/main.cpp.s"
	/usr/bin/g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/hao/Desktop/accer/accer/main.cpp -o CMakeFiles/accer.dir/main.cpp.s

CMakeFiles/accer.dir/main.cpp.o.requires:
.PHONY : CMakeFiles/accer.dir/main.cpp.o.requires

CMakeFiles/accer.dir/main.cpp.o.provides: CMakeFiles/accer.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/accer.dir/build.make CMakeFiles/accer.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/accer.dir/main.cpp.o.provides

CMakeFiles/accer.dir/main.cpp.o.provides.build: CMakeFiles/accer.dir/main.cpp.o

# Object files for target accer
accer_OBJECTS = \
"CMakeFiles/accer.dir/main.cpp.o"

# External object files for target accer
accer_EXTERNAL_OBJECTS =

accer: CMakeFiles/accer.dir/main.cpp.o
accer: CMakeFiles/accer.dir/build.make
accer: /usr/local/lib/libceres.a
accer: /usr/local/lib/libglog.so
accer: /usr/lib/x86_64-linux-gnu/libspqr.so
accer: /usr/lib/libtbb.so
accer: /usr/lib/libtbbmalloc.so
accer: /usr/lib/x86_64-linux-gnu/libcholmod.so
accer: /usr/lib/x86_64-linux-gnu/libccolamd.so
accer: /usr/lib/x86_64-linux-gnu/libcamd.so
accer: /usr/lib/x86_64-linux-gnu/libcolamd.so
accer: /usr/lib/x86_64-linux-gnu/libamd.so
accer: /usr/lib/liblapack.so
accer: /usr/lib/libblas.so
accer: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.a
accer: /usr/lib/x86_64-linux-gnu/librt.so
accer: /usr/lib/liblapack.so
accer: /usr/lib/libblas.so
accer: /usr/lib/x86_64-linux-gnu/libsuitesparseconfig.a
accer: /usr/lib/x86_64-linux-gnu/librt.so
accer: CMakeFiles/accer.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable accer"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/accer.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/accer.dir/build: accer
.PHONY : CMakeFiles/accer.dir/build

CMakeFiles/accer.dir/requires: CMakeFiles/accer.dir/main.cpp.o.requires
.PHONY : CMakeFiles/accer.dir/requires

CMakeFiles/accer.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/accer.dir/cmake_clean.cmake
.PHONY : CMakeFiles/accer.dir/clean

CMakeFiles/accer.dir/depend:
	cd /home/hao/Desktop/accer/build-accer-Desktop_Qt_5_7_0_GCC_64bit-Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hao/Desktop/accer/accer /home/hao/Desktop/accer/accer /home/hao/Desktop/accer/build-accer-Desktop_Qt_5_7_0_GCC_64bit-Debug /home/hao/Desktop/accer/build-accer-Desktop_Qt_5_7_0_GCC_64bit-Debug /home/hao/Desktop/accer/build-accer-Desktop_Qt_5_7_0_GCC_64bit-Debug/CMakeFiles/accer.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/accer.dir/depend

