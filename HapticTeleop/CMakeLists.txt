#--------------------------------------------------------------
#
# Zenom Hard Real-Time Simulation Enviroment
# Copyright (C) 2013
#
#--------------------------------------------------------------

cmake_minimum_required(VERSION 2.8)
project(HapticTeleop)

find_package(qibuild)

set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=gnu++0x")

add_definitions(-DBOOST_SIGNALS_NO_DEPRECATION_WARNING )

include_directories(".")

set(_srcs
    positioncontroller.h
    positioncontroller.cpp
    setpoint.h
    setpoint.cpp
    squarewavegenerator.h
    squarewavegenerator.cpp
    main.cpp)

# Create a executable named HapticTeleop
# with the source file: main.cpp
qi_create_bin(HapticTeleop ${_srcs})

qi_use_lib(HapticTeleop ZENOM HAPTICWAND ALCOMMON ALPROXIES)
