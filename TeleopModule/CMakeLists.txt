cmake_minimum_required(VERSION 2.8)

# Give a name to the project.
project(TeleopModule)

# You need this to find the qiBuild CMake framework
find_package(qibuild)

option(TELEOPMODULE_IS_REMOTE
  "module is compiled as a remote module (ON or OFF)"
  OFF)


add_definitions(-DBOOST_SIGNALS_NO_DEPRECATION_WARNING )

include_directories(".")

# Create a list of source files
set(_srcs
    cameraserver.h
    cameraserver.cpp
    TeleopModule.h
    TeleopModule.cpp
    main.cpp)

if(TELEOPMODULE_IS_REMOTE)
  add_definitions( " -DTELEOPMODULE_IS_REMOTE ")
  # Create an executable
  qi_create_bin(TeleopModule ${_srcs})
else()
  qi_create_lib(TeleopModule SHARED ${_srcs} SUBFOLDER naoqi)
endif()


# Tell CMake that TeleopModule depends on ALCOMMON and ALPROXIES.
# This will set the libraries to link TeleopModule with,
# the include paths, and so on
qi_use_lib(TeleopModule ALCOMMON ALVISION ALPROXIES ALMATH BOOST )

#qi_create_bin(testTeleopModule testTeleopModule.cpp)
#qi_use_lib(testTeleopModule ALCOMMON ALVISION ALPROXIES BOOST)
qi_create_bin(testTeleopModule testTeleopModule.cpp)
qi_use_lib(testTeleopModule ALCOMMON ALPROXIES ALMATH BOOST)
