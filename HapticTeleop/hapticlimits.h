#ifndef HAPTICLIMITS_H
#define HAPTICLIMITS_H

#define HAPTIC_MIN_X  -0.24 // meters
#define HAPTIC_MAX_X  0.24 // meters
#define HAPTIC_MIN_Y  0.085 // meters
#define HAPTIC_MAX_Y  0.335 // meters
#define HAPTIC_MIN_Z  -0.215 // meters
#define HAPTIC_MAX_Z  0.235 // meters
#define HAPTIC_MIN_PITCH  -1.483 // radians
#define HAPTIC_MAX_PITCH  1.483 // radians
#define HAPTIC_MIN_ROLL  -1.134 // radians
#define HAPTIC_MAX_ROLL 1.134 // radians

#define HAPTIC_START_YPOS ((HAPTIC_MAX_Y - HAPTIC_MIN_Y) * 0.75 + HAPTIC_MIN_Y)
#define HAPTIC_MID_YPOS ((HAPTIC_MAX_Y - HAPTIC_MIN_Y) / 2 + HAPTIC_MIN_Y)

#endif
