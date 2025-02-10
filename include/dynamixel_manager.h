#ifndef DYNAMIXEL_MANAGER_H
#define DYNAMIXEL_MANAGER_H

#include <Dynamixel2Arduino.h>
#include "config.h"

void setupDynamixels();

// Para que outros módulos (ex.: sync_rw) possam usar os objetos
extern Dynamixel2Arduino dxl1;
extern Dynamixel2Arduino dxl2;
extern Dynamixel2Arduino dxl3;
extern Dynamixel2Arduino dxl5;

#endif  // DYNAMIXEL_MANAGER_H
