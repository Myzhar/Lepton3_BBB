# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB

# Include any dependencies generated for this target.
include grabber_lib/CMakeFiles/lepton3_grabber.dir/depend.make

# Include the progress variables for this target.
include grabber_lib/CMakeFiles/lepton3_grabber.dir/progress.make

# Include the compile flags for this target's objects.
include grabber_lib/CMakeFiles/lepton3_grabber.dir/flags.make

grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o: grabber_lib/CMakeFiles/lepton3_grabber.dir/flags.make
grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o: grabber_lib/src/Lepton3.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o"
	cd /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o -c /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib/src/Lepton3.cpp

grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.i"
	cd /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib/src/Lepton3.cpp > CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.i

grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.s"
	cd /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib/src/Lepton3.cpp -o CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.s

grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o.requires:

.PHONY : grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o.requires

grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o.provides: grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o.requires
	$(MAKE) -f grabber_lib/CMakeFiles/lepton3_grabber.dir/build.make grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o.provides.build
.PHONY : grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o.provides

grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o.provides.build: grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o


grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o: grabber_lib/CMakeFiles/lepton3_grabber.dir/flags.make
grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o: grabber_lib/src/stopwatch.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o"
	cd /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o -c /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib/src/stopwatch.cpp

grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.i"
	cd /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib/src/stopwatch.cpp > CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.i

grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.s"
	cd /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib/src/stopwatch.cpp -o CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.s

grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o.requires:

.PHONY : grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o.requires

grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o.provides: grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o.requires
	$(MAKE) -f grabber_lib/CMakeFiles/lepton3_grabber.dir/build.make grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o.provides.build
.PHONY : grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o.provides

grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o.provides.build: grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o


# Object files for target lepton3_grabber
lepton3_grabber_OBJECTS = \
"CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o" \
"CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o"

# External object files for target lepton3_grabber
lepton3_grabber_EXTERNAL_OBJECTS =

grabber_lib/liblepton3_grabber.a: grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o
grabber_lib/liblepton3_grabber.a: grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o
grabber_lib/liblepton3_grabber.a: grabber_lib/CMakeFiles/lepton3_grabber.dir/build.make
grabber_lib/liblepton3_grabber.a: grabber_lib/CMakeFiles/lepton3_grabber.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library liblepton3_grabber.a"
	cd /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib && $(CMAKE_COMMAND) -P CMakeFiles/lepton3_grabber.dir/cmake_clean_target.cmake
	cd /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lepton3_grabber.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
grabber_lib/CMakeFiles/lepton3_grabber.dir/build: grabber_lib/liblepton3_grabber.a

.PHONY : grabber_lib/CMakeFiles/lepton3_grabber.dir/build

grabber_lib/CMakeFiles/lepton3_grabber.dir/requires: grabber_lib/CMakeFiles/lepton3_grabber.dir/src/Lepton3.cpp.o.requires
grabber_lib/CMakeFiles/lepton3_grabber.dir/requires: grabber_lib/CMakeFiles/lepton3_grabber.dir/src/stopwatch.cpp.o.requires

.PHONY : grabber_lib/CMakeFiles/lepton3_grabber.dir/requires

grabber_lib/CMakeFiles/lepton3_grabber.dir/clean:
	cd /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib && $(CMAKE_COMMAND) -P CMakeFiles/lepton3_grabber.dir/cmake_clean.cmake
.PHONY : grabber_lib/CMakeFiles/lepton3_grabber.dir/clean

grabber_lib/CMakeFiles/lepton3_grabber.dir/depend:
	cd /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib /home/debian/devel/BeagleBoneBlue_Lepton3/Lepton3_BBB/grabber_lib/CMakeFiles/lepton3_grabber.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : grabber_lib/CMakeFiles/lepton3_grabber.dir/depend

