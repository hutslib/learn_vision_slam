# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_COMMAND = /home/hts/software/clion-2020.1.2/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/hts/software/clion-2020.1.2/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/hts/01Vision_slam/ch1_t4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hts/01Vision_slam/ch1_t4/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/sayhello.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/sayhello.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sayhello.dir/flags.make

CMakeFiles/sayhello.dir/useHello.cpp.o: CMakeFiles/sayhello.dir/flags.make
CMakeFiles/sayhello.dir/useHello.cpp.o: ../useHello.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hts/01Vision_slam/ch1_t4/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sayhello.dir/useHello.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sayhello.dir/useHello.cpp.o -c /home/hts/01Vision_slam/ch1_t4/useHello.cpp

CMakeFiles/sayhello.dir/useHello.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sayhello.dir/useHello.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hts/01Vision_slam/ch1_t4/useHello.cpp > CMakeFiles/sayhello.dir/useHello.cpp.i

CMakeFiles/sayhello.dir/useHello.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sayhello.dir/useHello.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hts/01Vision_slam/ch1_t4/useHello.cpp -o CMakeFiles/sayhello.dir/useHello.cpp.s

# Object files for target sayhello
sayhello_OBJECTS = \
"CMakeFiles/sayhello.dir/useHello.cpp.o"

# External object files for target sayhello
sayhello_EXTERNAL_OBJECTS =

sayhello: CMakeFiles/sayhello.dir/useHello.cpp.o
sayhello: CMakeFiles/sayhello.dir/build.make
sayhello: libhello.so
sayhello: CMakeFiles/sayhello.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hts/01Vision_slam/ch1_t4/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sayhello"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sayhello.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sayhello.dir/build: sayhello

.PHONY : CMakeFiles/sayhello.dir/build

CMakeFiles/sayhello.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sayhello.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sayhello.dir/clean

CMakeFiles/sayhello.dir/depend:
	cd /home/hts/01Vision_slam/ch1_t4/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hts/01Vision_slam/ch1_t4 /home/hts/01Vision_slam/ch1_t4 /home/hts/01Vision_slam/ch1_t4/cmake-build-debug /home/hts/01Vision_slam/ch1_t4/cmake-build-debug /home/hts/01Vision_slam/ch1_t4/cmake-build-debug/CMakeFiles/sayhello.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sayhello.dir/depend
