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
CMAKE_SOURCE_DIR = /home/hapticpc/nao/nao_teleop/TeleopModule

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hapticpc/nao/nao_teleop/TeleopModule/build-crosstc

# Include any dependencies generated for this target.
include CMakeFiles/testTeleopModule.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/testTeleopModule.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/testTeleopModule.dir/flags.make

CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o: CMakeFiles/testTeleopModule.dir/flags.make
CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o: ../testTeleopModule.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hapticpc/nao/nao_teleop/TeleopModule/build-crosstc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o"
	/home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o -c /home/hapticpc/nao/nao_teleop/TeleopModule/testTeleopModule.cpp

CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.i"
	/home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hapticpc/nao/nao_teleop/TeleopModule/testTeleopModule.cpp > CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.i

CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.s"
	/home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/cross/bin/i686-aldebaran-linux-gnu-g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hapticpc/nao/nao_teleop/TeleopModule/testTeleopModule.cpp -o CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.s

CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o.requires:

.PHONY : CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o.requires

CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o.provides: CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o.requires
	$(MAKE) -f CMakeFiles/testTeleopModule.dir/build.make CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o.provides.build
.PHONY : CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o.provides

CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o.provides.build: CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o


# Object files for target testTeleopModule
testTeleopModule_OBJECTS = \
"CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o"

# External object files for target testTeleopModule
testTeleopModule_EXTERNAL_OBJECTS =

sdk/bin/testTeleopModule: CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o
sdk/bin/testTeleopModule: CMakeFiles/testTeleopModule.dir/build.make
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libalproxies.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libalcommon.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_signals-mt-1_55.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/librttools.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libalvalue.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libalerror.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libqimessaging.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libqitype.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/libnaoqi/lib/libqi.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_chrono-mt-1_55.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_filesystem-mt-1_55.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_program_options-mt-1_55.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_regex-mt-1_55.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/lib/libdl.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/lib/librt.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_date_time-mt-1_55.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_system-mt-1_55.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_locale-mt-1_55.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/boost/lib/libboost_thread-mt-1_55.so
sdk/bin/testTeleopModule: /home/hapticpc/nao/ctc-linux64-atom-2.1.4.13/cross/i686-aldebaran-linux-gnu/sysroot/usr/lib/libpthread.so
sdk/bin/testTeleopModule: CMakeFiles/testTeleopModule.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hapticpc/nao/nao_teleop/TeleopModule/build-crosstc/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sdk/bin/testTeleopModule"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/testTeleopModule.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/testTeleopModule.dir/build: sdk/bin/testTeleopModule

.PHONY : CMakeFiles/testTeleopModule.dir/build

CMakeFiles/testTeleopModule.dir/requires: CMakeFiles/testTeleopModule.dir/testTeleopModule.cpp.o.requires

.PHONY : CMakeFiles/testTeleopModule.dir/requires

CMakeFiles/testTeleopModule.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/testTeleopModule.dir/cmake_clean.cmake
.PHONY : CMakeFiles/testTeleopModule.dir/clean

CMakeFiles/testTeleopModule.dir/depend:
	cd /home/hapticpc/nao/nao_teleop/TeleopModule/build-crosstc && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hapticpc/nao/nao_teleop/TeleopModule /home/hapticpc/nao/nao_teleop/TeleopModule /home/hapticpc/nao/nao_teleop/TeleopModule/build-crosstc /home/hapticpc/nao/nao_teleop/TeleopModule/build-crosstc /home/hapticpc/nao/nao_teleop/TeleopModule/build-crosstc/CMakeFiles/testTeleopModule.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/testTeleopModule.dir/depend
