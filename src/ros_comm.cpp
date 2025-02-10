#include "ros_comm.h"
#include "config.h"  // para acessar arrays e funções de conversão
#include <Arduino.h>

// Declare o nó ROS
ros::NodeHandle nh;

// Declare os publishers
std_msgs::Float64MultiArray dummy_msg;  // mensagem dummy para construção dos publishers
ros::Publisher right_leg_pub("/marta/right_leg/state", &dummy_msg);
ros::Publisher left_leg_pub("/marta/left_leg/state", &dummy_msg);
ros::Publisher arm_r_pub("/marta/arm_r/state", &dummy_msg);
ros::Publisher pub_imu("/micro/IMU", &dummy_msg);  // use o tipo de mensagem adequado em seu caso

// Declarando os subscribers e seus callbacks
ros::Subscriber<std_msgs::Float64MultiArray> right_leg_sub("/marta/right_leg/command", right_leg_cb);
ros::Subscriber<std_msgs::Float64MultiArray> left_leg_sub("/marta/left_leg/command", left_leg_cb);
ros::Subscriber<std_msgs::Float64MultiArray> arm_r_sub("/marta/arm_r/command", arm_r_cb);
ros::Subscriber<std_msgs::Float64MultiArray> arm_l_head_sub("/marta/arm_l_head/command", arm_l_head_cb);

void setupROS() {
  nh.initNode();
  nh.subscribe(right_leg_sub);
  nh.subscribe(left_leg_sub);
  nh.subscribe(arm_r_sub);
  nh.subscribe(arm_l_head_sub);
  nh.advertise(right_leg_pub);
  nh.advertise(left_leg_pub);
  nh.advertise(arm_r_pub);
  nh.advertise(pub_imu);
}

void spinROS() {
  nh.spinOnce();
}

// --- Callbacks ---
// Estes callbacks convertem os comandos recebidos (em radianos) em posições para os atuadores,
// utilizando as funções de conversão definidas em config.h.

void right_leg_cb(const std_msgs::Float64MultiArray &cmd_msg) {
  for (int i = 0; i < DXL_ID_CNT; i++) {
    int32_t d_theta = convert_command(cmd_msg.data[i]);
    goal_position_R[i] = goal_position_initial_R[i] + d_theta * Signal_right[i];
  }
}

void left_leg_cb(const std_msgs::Float64MultiArray &cmd_msg) {
  for (int i = 0; i < DXL_ID_CNT; i++) {
    int32_t d_theta = convert_command(cmd_msg.data[i]);
    goal_position_L[i] = goal_position_initial_L[i] + d_theta * Signal_left[i];
  }
}

void arm_r_cb(const std_msgs::Float64MultiArray &cmd_msg) {
  for (int i = 0; i < DXL_ID_CNT_ARM_R; i++) {
    int32_t d_theta = convert_command(cmd_msg.data[i]);
    goal_position_arm_r[i] = goal_position_initial_arm_r[i] + d_theta * Signal_arm_r[i];
  }
}

void arm_l_head_cb(const std_msgs::Float64MultiArray &cmd_msg) {
  for (int i = 0; i < DXL_ID_CNT_ARM_L_HEAD; i++) {
    int32_t d_theta = convert_command(cmd_msg.data[i]);
    goal_position_arm_l_head[i] = goal_position_initial_arm_l_head[i] + d_theta * Signal_arm_l_head[i];
  }
}
