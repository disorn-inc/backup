# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/mycoss/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mycoss/catkin_ws/build

# Include any dependencies generated for this target.
include scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/depend.make

# Include the progress variables for this target.
include scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/progress.make

# Include the compile flags for this target's objects.
include scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/flags.make

scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o: scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/flags.make
scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o: /home/mycoss/catkin_ws/src/scan_tools/laser_scan_sparsifier/src/laser_scan_sparsifier.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mycoss/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o"
	cd /home/mycoss/catkin_ws/build/scan_tools/laser_scan_sparsifier && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o -c /home/mycoss/catkin_ws/src/scan_tools/laser_scan_sparsifier/src/laser_scan_sparsifier.cpp

scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.i"
	cd /home/mycoss/catkin_ws/build/scan_tools/laser_scan_sparsifier && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mycoss/catkin_ws/src/scan_tools/laser_scan_sparsifier/src/laser_scan_sparsifier.cpp > CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.i

scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.s"
	cd /home/mycoss/catkin_ws/build/scan_tools/laser_scan_sparsifier && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mycoss/catkin_ws/src/scan_tools/laser_scan_sparsifier/src/laser_scan_sparsifier.cpp -o CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.s

scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o.requires:

.PHONY : scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o.requires

scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o.provides: scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o.requires
	$(MAKE) -f scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/build.make scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o.provides.build
.PHONY : scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o.provides

scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o.provides.build: scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o


# Object files for target laser_scan_sparsifier
laser_scan_sparsifier_OBJECTS = \
"CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o"

# External object files for target laser_scan_sparsifier
laser_scan_sparsifier_EXTERNAL_OBJECTS =

/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/build.make
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /opt/ros/melodic/lib/libnodeletlib.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /opt/ros/melodic/lib/libbondcpp.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /opt/ros/melodic/lib/libclass_loader.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/libPocoFoundation.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libdl.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /opt/ros/melodic/lib/libroslib.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /opt/ros/melodic/lib/librospack.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libpython2.7.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libboost_program_options.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libtinyxml2.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /opt/ros/melodic/lib/libroscpp.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libboost_filesystem.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libboost_signals.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /opt/ros/melodic/lib/librosconsole.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /opt/ros/melodic/lib/librosconsole_log4cxx.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /opt/ros/melodic/lib/librosconsole_backend_interface.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/liblog4cxx.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libboost_regex.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /opt/ros/melodic/lib/libxmlrpcpp.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /opt/ros/melodic/lib/libroscpp_serialization.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /opt/ros/melodic/lib/librostime.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /opt/ros/melodic/lib/libcpp_common.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libboost_system.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libboost_thread.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libboost_chrono.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libboost_date_time.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libboost_atomic.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libpthread.so
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: /usr/lib/aarch64-linux-gnu/libconsole_bridge.so.0.4
/home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so: scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mycoss/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library /home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so"
	cd /home/mycoss/catkin_ws/build/scan_tools/laser_scan_sparsifier && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/laser_scan_sparsifier.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/build: /home/mycoss/catkin_ws/devel/lib/liblaser_scan_sparsifier.so

.PHONY : scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/build

scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/requires: scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/src/laser_scan_sparsifier.cpp.o.requires

.PHONY : scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/requires

scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/clean:
	cd /home/mycoss/catkin_ws/build/scan_tools/laser_scan_sparsifier && $(CMAKE_COMMAND) -P CMakeFiles/laser_scan_sparsifier.dir/cmake_clean.cmake
.PHONY : scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/clean

scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/depend:
	cd /home/mycoss/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mycoss/catkin_ws/src /home/mycoss/catkin_ws/src/scan_tools/laser_scan_sparsifier /home/mycoss/catkin_ws/build /home/mycoss/catkin_ws/build/scan_tools/laser_scan_sparsifier /home/mycoss/catkin_ws/build/scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : scan_tools/laser_scan_sparsifier/CMakeFiles/laser_scan_sparsifier.dir/depend

