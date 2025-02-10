#ifndef ROS_COMM_H
#define ROS_COMM_H

#include <ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>

extern ros::NodeHandle nh;

// Publishers usados para publicar estados dos atuadores e IMU
extern ros::Publisher right_leg_pub;
extern ros::Publisher left_leg_pub;
extern ros::Publisher arm_r_pub;
extern ros::Publisher pub_imu;

void setupROS();
void spinROS();

// Subscribers e callbacks
void right_leg_cb(const std_msgs::Float64MultiArray &cmd_msg);
void left_leg_cb(const std_msgs::Float64MultiArray &cmd_msg);
void arm_r_cb(const std_msgs::Float64MultiArray &cmd_msg);
void arm_l_head_cb(const std_msgs::Float64MultiArray &cmd_msg);

#endif  // ROS_COMM_H
