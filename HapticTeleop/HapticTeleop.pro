#--------------------------------------------------------------
#
# Zenom Hard Real-Time Simulation Enviroment
# Copyright (C) 2013
#
#--------------------------------------------------------------

# Zenom
INCLUDEPATH += /usr/local/include/zenom
LIBS += -L/usr/local/lib64 -lznm-controlbase -lznm-core -lznm-tools

# HapticWand
INCLUDEPATH += /usr/local/include/hapticWand
LIBS += -lHapticWand

TEMPLATE = app
CONFIG += console
#CONFIG -= qt
QMAKE_CXXFLAGS += -std=c++11
CONFIG += c++11

SOURCES += main.cpp \
    positioncontroller.cpp \
    setpoint.cpp \
    squarewavegenerator.cpp

HEADERS += \
    positioncontroller.h \
    setpoint.h \
    squarewavegenerator.h

