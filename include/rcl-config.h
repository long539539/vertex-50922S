#pragma once

#include "RclTracking.h"

// Created in src/rcl-config.cpp
extern vex::distance rear_dist;
extern vex::distance left_dist;
extern vex::distance right_dist;

extern RclSensor right_rcl;
extern RclSensor rear_rcl;
extern RclSensor left_rcl;

extern RclTracking RclMain;
