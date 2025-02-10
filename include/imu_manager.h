#ifndef IMU_MANAGER_H
#define IMU_MANAGER_H

#include "SparkFun_BNO080_Arduino_Library.h"
#include <ros.h>
#include <sensor_msgs/Imu.h>

void setupIMU();
void updateIMU();

#endif  // IMU_MANAGER_H
