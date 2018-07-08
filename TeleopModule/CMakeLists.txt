cmake_minimum_required(VERSION 2.8)

# Give a name to the project.
project(TeleopModule)

# You need this to find the qiBuild CMake framework
find_package(qibuild)

# Create a list of source files
set(_srcs
    TeleopModule.cpp
    TeleopModule.h
    main.cpp)

# Create a plugin, that is a shared library, and make
# sure it is built in lib/naoqi
qi_create_lib(TeleopModule SHARED ${_srcs} SUBFOLDER naoqi)

# Tell CMake that TeleopModule depends on ALCOMMON and ALPROXIES.
# This will set the libraries to link TeleopModule with,
# the include paths, and so on
qi_use_lib(TeleopModule ALCOMMON ALPROXIES BOOST)
