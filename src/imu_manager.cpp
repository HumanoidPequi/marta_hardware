#include "imu_manager.h"
#include "ros_comm.h"  // para usar o NodeHandle e possivelmente o publisher
#include "config.h"
#include <Arduino.h>
#include <Wire.h>

// Criação do objeto IMU
BNO080 myIMU;

// Defina o frame_id da IMU (ex.: "/imu")
char imu_frame_id[] = "/imu";

// Mensagem a ser publicada
sensor_msgs::Imu inertial;
geometry_msgs::Vector3 lin;
geometry_msgs::Vector3 ang;

void setupIMU() {
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  digitalWrite(LED_BUILTIN, LOW);
  if (!myIMU.begin()) {
    // Em caso de falha, pisca o LED e trava o sistema
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    while (1);
  }
  delay(10);
  Wire.setClock(400000);
  delay(10);
  myIMU.enableLinearAccelerometer(8);
  myIMU.enableRotationVector(4);
  myIMU.enableGyro(12);
  digitalWrite(LED_BUILTIN, LOW);
}

void updateIMU() {
  inertial.header.frame_id = imu_frame_id;
  inertial.header.stamp = nh.now();
  
  if (myIMU.dataAvailable()) {
    digitalWrite(LED_BUILTIN, HIGH);
    byte linAccuracy = 0;
    byte gyroAccuracy = 0;
    float quatRadianAccuracy = 0;
    byte quatAccuracy = 0;
    
    myIMU.getLinAccel(lin.x, lin.y, lin.z, linAccuracy);
    myIMU.getGyro(ang.x, ang.y, ang.z, gyroAccuracy);
    myIMU.getQuat(inertial.orientation.x, inertial.orientation.y, inertial.orientation.z,
                   inertial.orientation.w, quatRadianAccuracy, quatAccuracy);
    inertial.angular_velocity = ang;
    inertial.linear_acceleration = lin;
  }
  pub_imu.publish(&inertial);
  delay(1);
}
