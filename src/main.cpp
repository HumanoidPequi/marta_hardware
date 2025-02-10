#include <Arduino.h>
#include "ros_comm.h"
#include "dynamixel_manager.h"
#include "sync_rw.h"
#include "imu_manager.h"
#include "config.h"

// Para ajuste de delay
int delay1 = 1;

void setup() {
  setupROS();
  setupDynamixels();
  setupSyncReadWrite();
  setupIMU();
}

void loop() {
  // Atualiza os dados da IMU
  updateIMU();
  
  // Realiza o sync read/write dos motores
  performSyncReadWrite();
  
  // Publica os estados de cada grupo com base na Sync Read:
  {
    uint8_t recv_cnt1 = dxl1.syncRead(&sr_infos1);
    if (recv_cnt1 > 0) {
      std_msgs::Float64MultiArray msg_R;
      msg_R.data_length = recv_cnt1;
      msg_R.data = (float*)malloc(recv_cnt1 * sizeof(float));
      for (uint8_t i = 0; i < recv_cnt1; i++) {
        msg_R.data[i] = convert_state(sr_data1[i].present_position - goal_position_initial_R[i]) * Signal_right[i];
      }
      right_leg_pub.publish(&msg_R);
      free(msg_R.data);
    }
  }
  
  {
    uint8_t recv_cnt2 = dxl2.syncRead(&sr_infos2);
    if (recv_cnt2 > 0) {
      std_msgs::Float64MultiArray msg_L;
      msg_L.data_length = recv_cnt2;
      msg_L.data = (float*)malloc(recv_cnt2 * sizeof(float));
      for (uint8_t i = 0; i < recv_cnt2; i++) {
        msg_L.data[i] = convert_state(sr_data2[i].present_position - goal_position_initial_L[i]) * Signal_left[i];
      }
      left_leg_pub.publish(&msg_L);
      free(msg_L.data);
    }
  }
  
  {
    uint8_t recv_cnt3 = dxl3.syncRead(&sr_infos3);
    if (recv_cnt3 > 0) {
      std_msgs::Float64MultiArray msg_arm_R;
      msg_arm_R.data_length = recv_cnt3;
      msg_arm_R.data = (float*)malloc(recv_cnt3 * sizeof(float));
      for (uint8_t i = 0; i < recv_cnt3; i++) {
        msg_arm_R.data[i] = convert_state(sr_data3[i].present_position - goal_position_initial_arm_r[i]) * Signal_arm_r[i];
      }
      arm_r_pub.publish(&msg_arm_R);
      free(msg_arm_R.data);
    }
  }
  
  // Pisca o LED integrado
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  
  spinROS();
  delay(3 * delay1);
}
