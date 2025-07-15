#include <Dynamixel2Arduino.h>

#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"

#include <ros.h>
#include <ros/time.h>

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

ros::NodeHandle nh;

//IMU ros config
sensor_msgs::Imu inertial;
geometry_msgs::Vector3 lin;
geometry_msgs::Vector3 ang;


ros::Publisher pub_imu("micro/IMU", &inertial);
char in[] = "/imu";

//#define DEBUG_SERIAL Serial
#define DXL_SERIAL1 Serial1 // right leg
#define DXL_SERIAL2 Serial3 // left lef
#define DXL_SERIAL3 Serial2 // right arm
#define DXL_SERIAL5 Serial5 // left arm and head

const int DXL_DIR_PIN1 = 2; // pino de controle Serial1
const int DXL_DIR_PIN2 = 22; // pino de controle Serial2
const int DXL_DIR_PIN3 = 9; // pino de controle Serial3
const int DXL_DIR_PIN5 = 23; // pino de controle Serial5

const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION = 2.0;
const uint8_t DXL_ID_CNT = 6;
const uint8_t DXL_ID_CNT_ARM_R = DXL_ID_CNT/2;
const uint8_t DXL_ID_CNT_ARM_L_HEAD = DXL_ID_CNT - 1;
const uint8_t DXL_ID_LIST_R[DXL_ID_CNT] = {1,2,3,4,5,6};
const uint8_t DXL_ID_LIST_L[DXL_ID_CNT] = {7,8,9,10,11,12};
const uint8_t DXL_ID_LIST_ARM_R[DXL_ID_CNT_ARM_R] = {13,14,15};
const uint8_t DXL_ID_LIST_ARM_L_HEAD[DXL_ID_CNT_ARM_L_HEAD] = {19,20,16,17,18};// 3 últimos são do braço esquerdo e os dois primeiros são da cabeça
const int8_t Signal_left[DXL_ID_CNT] = {-1,-1,1,1,-1,1};
const int8_t Signal_right[DXL_ID_CNT] = {-1,-1,-1,-1,1,1};
const int8_t Signal_arm_r[DXL_ID_CNT_ARM_R] = {-1, 1, -1}; 
const int8_t Signal_arm_l_head[DXL_ID_CNT_ARM_L_HEAD] = {1, 1, 1, -1, 1}; 
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf1[user_pkt_buf_cap];
uint8_t user_pkt_buf2[user_pkt_buf_cap];
uint8_t user_pkt_buf3[user_pkt_buf_cap];
uint8_t user_pkt_buf5[user_pkt_buf_cap];

//perna direita
const int32_t rHipYaw  =    2048; //1
const int32_t rHipRoll  =   2020; //2
const int32_t rHipPitch  =  2020; //3
const int32_t rKneePitch =  2048; //4
const int32_t rAnklePitch = 2048; //5
const int32_t rAnkleRoll =  2048;  //6

//perna esquerda
const int32_t lHipYaw   =   2048; //7
const int32_t lHipRoll  =   2048; //8
const int32_t lHipPitch =   1130; //9
const int32_t lKneePitch =  2048; //10
const int32_t lAnklePitch = 2020; //11
const int32_t lAnkleRoll  = 2048; //12

unsigned int minPosR[DXL_ID_CNT] = {rHipYaw-341, rHipRoll-148, rHipPitch-455, rKneePitch-1024, rAnklePitch-455, rAnkleRoll-341}; // angulos minimos (12Bits)
                                  
unsigned int minPosL[DXL_ID_CNT] = {lHipYaw-341, lHipRoll-455, lHipPitch-1024, lKneePitch-1024, lAnklePitch-455, lAnkleRoll-341}; // angulos minimos (12Bits)                              
                                  
unsigned int maxPosR[DXL_ID_CNT] = {rHipYaw+341, rHipRoll+455, rHipPitch+1024, rKneePitch+1024, rAnklePitch+455, rAnkleRoll+341};

unsigned int maxPosL[DXL_ID_CNT] = {lHipYaw+341, lHipRoll+148, lHipPitch+455, lKneePitch+1024, lAnklePitch+455, lAnkleRoll+341};       

// Starting address of the Data to read; Present Position = 132
const uint16_t SR_START_ADDR = 132;
// Length of the Data to read; Length of Position data of X series is 4 byte
const uint16_t SR_ADDR_LEN = 4;
// Starting address of the Data to write; Goal Position = 116
const uint16_t SW_START_ADDR = 116;
// Length of the Data to write; Length of Position data of X series is 4 byte
const uint16_t SW_ADDR_LEN = 4;

typedef struct sr_data {
  int32_t present_position;
} __attribute__((packed)) sr_data_t;

typedef struct sw_data {
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

sr_data_t sr_data1[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos1;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr1[DXL_ID_CNT];

sw_data_t sw_data1[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos1;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw1[DXL_ID_CNT];

sr_data_t sr_data2[DXL_ID_CNT];
DYNAMIXEL::InfoSyncReadInst_t sr_infos2;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr2[DXL_ID_CNT];

sw_data_t sw_data2[DXL_ID_CNT];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos2;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw2[DXL_ID_CNT];

sr_data_t sr_data3[DXL_ID_CNT_ARM_R];
DYNAMIXEL::InfoSyncReadInst_t sr_infos3;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr3[DXL_ID_CNT_ARM_R];

sw_data_t sw_data3[DXL_ID_CNT_ARM_R];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos3;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw3[DXL_ID_CNT_ARM_R];

sr_data_t sr_data5[DXL_ID_CNT_ARM_L_HEAD];
DYNAMIXEL::InfoSyncReadInst_t sr_infos5;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr5[DXL_ID_CNT_ARM_L_HEAD];

sw_data_t sw_data5[DXL_ID_CNT_ARM_L_HEAD];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos5;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw5[DXL_ID_CNT_ARM_L_HEAD];

Dynamixel2Arduino dxl1(DXL_SERIAL1, DXL_DIR_PIN1);
Dynamixel2Arduino dxl2(DXL_SERIAL2, DXL_DIR_PIN2);
Dynamixel2Arduino dxl3(DXL_SERIAL3, DXL_DIR_PIN3);
Dynamixel2Arduino dxl5(DXL_SERIAL5, DXL_DIR_PIN5);

BNO080 myIMU;

// This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

int32_t goal_position_initial_R[DXL_ID_CNT] = {rHipYaw,rHipRoll,rHipPitch,rKneePitch,rAnklePitch,rAnkleRoll};
int32_t goal_position_R[DXL_ID_CNT] = {rHipYaw,rHipRoll,rHipPitch,rKneePitch,rAnklePitch,rAnkleRoll};

int32_t goal_position_initial_L[DXL_ID_CNT] = {lHipYaw,lHipRoll,lHipPitch,lKneePitch,lAnklePitch,lAnkleRoll};
int32_t goal_position_L[DXL_ID_CNT] = {lHipYaw,lHipRoll,lHipPitch,lKneePitch,lAnklePitch,lAnkleRoll};

int32_t goal_position_initial_arm_r[DXL_ID_CNT_ARM_R] = {2048,2048,2048};
int32_t goal_position_arm_r[DXL_ID_CNT_ARM_R] = {2048,2048,2048};

int32_t goal_position_initial_arm_l_head[DXL_ID_CNT_ARM_L_HEAD] = {2048,2048,2048,2048,2048};
int32_t goal_position_arm_l_head[DXL_ID_CNT_ARM_L_HEAD] = {2048,2048,2048,2048,2048};

int convert_command(float ang_command) {
  //Converte os valores de 16 bits recebidos pelo rosserial para os valores de 12 bits
  return (int)((ang_command / M_PI) * 2048);
}

int convert_command_degree(int ang_command) {
  //Converte os valores de 16 bits recebidos pelo rosserial para os valores de 12 bits
  int position_command = map(ang_command, -1800, 1800, -2048, 2048);
  
  return position_command;
}

float convert_state(int raw_diff) {
  // Ajusta o valor para considerar o wrap-around (12 bits: 0 a 4095)
  if (raw_diff > 2048)
    raw_diff -= 4096;
  else if (raw_diff < -2048)
    raw_diff += 4096;
  
  // Converte a diferença para radianos
  return (float)raw_diff * M_PI / 2048.0;
}

void right_leg_cb(const std_msgs::Float64MultiArray& cmd_msg) {
   for (int i = 0; i < DXL_ID_CNT; i++) {
      int32_t d_theta = convert_command(cmd_msg.data[i]);
      goal_position_R[i] = goal_position_initial_R[i] + d_theta*Signal_right[i];
   }
}

void left_leg_cb(const std_msgs::Float64MultiArray& cmd_msg) {
   for (int i = 0; i < DXL_ID_CNT; i++) {
      int32_t d_theta = convert_command(cmd_msg.data[i]);
      goal_position_L[i] = goal_position_initial_L[i] + d_theta*Signal_left[i];
  }
} 

void arm_r_cb(const std_msgs::Float64MultiArray& cmd_msg) {
   for (int i = 0; i < DXL_ID_CNT_ARM_R; i++) {
      int32_t d_theta = convert_command(cmd_msg.data[i]);
      goal_position_arm_r[i] = goal_position_initial_arm_r[i] + d_theta*Signal_arm_r[i];
   }
}

void arm_l_head_cb(const std_msgs::Float64MultiArray& cmd_msg) {
   for (int i = 0; i < DXL_ID_CNT_ARM_L_HEAD; i++) {
      int32_t d_theta = convert_command(cmd_msg.data[i]);
      goal_position_arm_l_head[i] = goal_position_initial_arm_l_head[i] + d_theta*Signal_arm_l_head[i];
   }
}

int delay1 = 1; // Ajuste de delay

std_msgs::Float64MultiArray msg_R;
std_msgs::Float64MultiArray msg_L;
std_msgs::Float64MultiArray msg_arm_R;
std_msgs::Float64MultiArray msg_arm_L_head;

ros::Publisher right_leg_pub("/marta/right_leg/state", &msg_R);
ros::Publisher left_leg_pub("/marta/left_leg/state", &msg_L);
ros::Publisher arm_r_pub("/marta/arm_r/state", &msg_arm_R);
ros::Publisher arm_l_head_pub("/marta/arm_l_head/state", &msg_arm_L_head);
ros::Subscriber<std_msgs::Float64MultiArray> right_leg_sub("/marta/right_leg/command", right_leg_cb);
ros::Subscriber<std_msgs::Float64MultiArray> left_leg_sub("/marta/left_leg/command", left_leg_cb);
ros::Subscriber<std_msgs::Float64MultiArray> arm_r_sub("/marta/arm_r/command", arm_r_cb);
ros::Subscriber<std_msgs::Float64MultiArray> arm_l_head_sub("/marta/arm_l_head/command", arm_l_head_cb);

void setup() {
  nh.initNode();
  nh.subscribe(right_leg_sub);
  nh.subscribe(left_leg_sub);
  nh.subscribe(arm_r_sub);
  nh.subscribe(arm_l_head_sub);
  nh.advertise(right_leg_pub);
  nh.advertise(left_leg_pub);
  nh.advertise(arm_r_pub);
  nh.advertise(arm_l_head_pub);

  dynamixel_setup();

  sync_read_write_setup();

  bno_setup();
}

void loop() {
  
  bno_loop();
  
  static uint32_t try_count = 0;
  uint8_t i, recv_cnt;

  // Insert a new Goal Position to the SyncWrite Packet
  for (i = 0; i < DXL_ID_CNT; i++) {
    sw_data1[i].goal_position = goal_position_R[i];
    sw_data2[i].goal_position = goal_position_L[i];
  }

  sw_infos1.is_info_changed = true;
  sw_infos2.is_info_changed = true;
  
  for (i = 0; i < DXL_ID_CNT_ARM_R; i++) {
    sw_data3[i].goal_position = goal_position_arm_r[i];
  }
  sw_infos3.is_info_changed = true;

  for (i = 0; i < DXL_ID_CNT_ARM_L_HEAD; i++) {
    sw_data5[i].goal_position = goal_position_arm_l_head[i];
  }
  sw_infos5.is_info_changed = true;
  
  dxl1.syncWrite(&sw_infos1);
  dxl2.syncWrite(&sw_infos2);
  dxl3.syncWrite(&sw_infos3);
  dxl5.syncWrite(&sw_infos5);

  // Transmit predefined SyncRead instruction packet
  // and receive a status packet from each DYNAMIXEL
  uint8_t recv_cnt1 = dxl1.syncRead(&sr_infos1);
  uint8_t recv_cnt2 = dxl2.syncRead(&sr_infos2);
  uint8_t recv_cnt3 = dxl3.syncRead(&sr_infos3);
  uint8_t recv_cnt5 = dxl5.syncRead(&sr_infos5);
  
  if (recv_cnt1 > 0) {
    // Prepare msg data
    msg_R.data_length = recv_cnt1;
    msg_R.data = (float*) malloc(recv_cnt1 * sizeof(float));

    for (i = 0; i < recv_cnt1; i++) {
      msg_R.data[i] = convert_state(sr_data1[i].present_position - goal_position_initial_R[i])*Signal_right[i];
      //Serial.println(msg_R.data[i]); // Debug print to Serial
    }

    right_leg_pub.publish(&msg_R);
    free(msg_R.data); // Free the allocated memory
  }
  
  if (recv_cnt2 > 0) {
    // Prepare msg data
    msg_L.data_length = recv_cnt2;
    msg_L.data = (float*) malloc(recv_cnt2 * sizeof(float));
    
    for (i = 0; i < recv_cnt2; i++) {
      msg_L.data[i] = convert_state(sr_data2[i].present_position - goal_position_initial_L[i])*Signal_left[i];
      //Serial.println(msg_L.data[i]); // Debug print to Serial
    }
    
    left_leg_pub.publish(&msg_L);
    free(msg_L.data); // Free the allocated memory
  }

  if (recv_cnt3 > 0) {
    // Prepare msg data
    msg_arm_R.data_length = recv_cnt3;
    msg_arm_R.data = (float*) malloc(recv_cnt3 * sizeof(float));
    
    for (i = 0; i < recv_cnt3; i++) {
      msg_arm_R.data[i] = convert_state(sr_data3[i].present_position - goal_position_initial_arm_r[i])*Signal_arm_r[i];
      //Serial.println(msg_L.data[i]); // Debug print to Serial
    }
    
    arm_r_pub.publish(&msg_arm_R);
    free(msg_arm_R.data); // Free the allocated memory
  }

  if (recv_cnt5 > 0) {
    // Prepare msg data
    msg_arm_L_head.data_length = recv_cnt5;
    msg_arm_L_head.data = (float*) malloc(recv_cnt5 * sizeof(float));
    
    for (i = 0; i < recv_cnt5; i++) {
      msg_arm_L_head.data[i] = convert_state(sr_data5[i].present_position - goal_position_initial_arm_l_head[i])*Signal_arm_l_head[i];
      //Serial.println(msg_L.data[i]); // Debug print to Serial
    }
    
    arm_l_head_pub.publish(&msg_arm_L_head);
    free(msg_arm_L_head.data); // Free the allocated memory
  }

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

  nh.spinOnce();
  delay(3 * delay1);
}

void dynamixel_setup(){
  int i;

  Serial.begin(1000000);
  dxl1.begin(1000000);
  dxl1.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  dxl2.begin(1000000);
  dxl2.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  dxl3.begin(1000000);
  dxl3.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  dxl5.begin(1000000);
  dxl5.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);

  // Prepare the SyncRead structure
  for (i = 0; i < DXL_ID_CNT; i++) {
    dxl1.torqueOff(DXL_ID_LIST_R[i]);
    dxl1.setOperatingMode(DXL_ID_LIST_R[i], OP_POSITION);
  }
  dxl1.torqueOn(BROADCAST_ID);

  // Prepare the SyncRead structure
  for (i = 0; i < DXL_ID_CNT; i++) {
    dxl2.torqueOff(DXL_ID_LIST_L[i]);
    dxl2.setOperatingMode(DXL_ID_LIST_L[i], OP_POSITION);
  }
  dxl2.torqueOn(BROADCAST_ID);


  // Prepare the SyncRead structure
  for (i = 0; i < DXL_ID_CNT_ARM_R; i++) {
    dxl3.torqueOff(DXL_ID_LIST_ARM_R[i]);
    dxl3.setOperatingMode(DXL_ID_LIST_ARM_R[i], OP_POSITION);
  }
  dxl3.torqueOn(BROADCAST_ID);

    // Prepare the SyncRead structure
  for (i = 0; i < DXL_ID_CNT_ARM_L_HEAD; i++) {
    dxl5.torqueOff(DXL_ID_LIST_ARM_L_HEAD[i]);
    dxl5.setOperatingMode(DXL_ID_LIST_ARM_L_HEAD[i], OP_POSITION);
  }
  dxl5.torqueOn(BROADCAST_ID);
}

void sync_read_write_setup(){
  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos1.packet.p_buf = user_pkt_buf1;
  sr_infos1.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos1.packet.is_completed = false;
  sr_infos1.addr = SR_START_ADDR;
  sr_infos1.addr_length = SR_ADDR_LEN;
  sr_infos1.p_xels = info_xels_sr1;
  sr_infos1.xel_count = 0;

  for (int i = 0; i < DXL_ID_CNT; i++) {
    info_xels_sr1[i].id = DXL_ID_LIST_R[i];
    info_xels_sr1[i].p_recv_buf = (uint8_t*)&sr_data1[i];
    //Serial.println(info_xels_sr1[i].id);
    sr_infos1.xel_count++;
  }
  sr_infos1.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos1.packet.p_buf = nullptr;
  sw_infos1.packet.is_completed = false;
  sw_infos1.addr = SW_START_ADDR;
  sw_infos1.addr_length = SW_ADDR_LEN;
  sw_infos1.p_xels = info_xels_sw1;
  sw_infos1.xel_count = 0;

  for (int i = 0; i < DXL_ID_CNT; i++) {
    info_xels_sw1[i].id = DXL_ID_LIST_R[i];
    info_xels_sw1[i].p_data = (uint8_t*)&sw_data1[i].goal_position;
    sw_infos1.xel_count++;
  }
  sw_infos1.is_info_changed = true;

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos2.packet.p_buf = user_pkt_buf2;
  sr_infos2.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos2.packet.is_completed = false;
  sr_infos2.addr = SR_START_ADDR;
  sr_infos2.addr_length = SR_ADDR_LEN;
  sr_infos2.p_xels = info_xels_sr2;
  sr_infos2.xel_count = 0;

  for (int i = 0; i < DXL_ID_CNT; i++) {
    info_xels_sr2[i].id = DXL_ID_LIST_L[i];
    info_xels_sr2[i].p_recv_buf = (uint8_t*)&sr_data2[i];
    sr_infos2.xel_count++;
  }
  sr_infos2.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos2.packet.p_buf = nullptr;
  sw_infos2.packet.is_completed = false;
  sw_infos2.addr = SW_START_ADDR;
  sw_infos2.addr_length = SW_ADDR_LEN;
  sw_infos2.p_xels = info_xels_sw2;
  sw_infos2.xel_count = 0;

  for (int i = 0; i < DXL_ID_CNT; i++) {
    info_xels_sw2[i].id = DXL_ID_LIST_L[i];
    info_xels_sw2[i].p_data = (uint8_t*)&sw_data2[i].goal_position;
    sw_infos2.xel_count++;
  }
  sw_infos2.is_info_changed = true;

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos3.packet.p_buf = user_pkt_buf3;
  sr_infos3.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos3.packet.is_completed = false;
  sr_infos3.addr = SR_START_ADDR;
  sr_infos3.addr_length = SR_ADDR_LEN;
  sr_infos3.p_xels = info_xels_sr3;
  sr_infos3.xel_count = 0;

  for (int i = 0; i < DXL_ID_CNT_ARM_R; i++) {
    info_xels_sr3[i].id = DXL_ID_LIST_ARM_R[i];
    info_xels_sr3[i].p_recv_buf = (uint8_t*)&sr_data3[i];
    sr_infos3.xel_count++;
  }
  sr_infos3.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos3.packet.p_buf = nullptr;
  sw_infos3.packet.is_completed = false;
  sw_infos3.addr = SW_START_ADDR;
  sw_infos3.addr_length = SW_ADDR_LEN;
  sw_infos3.p_xels = info_xels_sw3;
  sw_infos3.xel_count = 0;

  for (int i = 0; i < DXL_ID_CNT_ARM_R; i++) {
    info_xels_sw3[i].id = DXL_ID_LIST_ARM_R[i];
    info_xels_sw3[i].p_data = (uint8_t*)&sw_data3[i].goal_position;
    sw_infos3.xel_count++;
  }
  sw_infos3.is_info_changed = true;

  // Fill the members of structure to syncRead using external user packet buffer
  sr_infos5.packet.p_buf = user_pkt_buf5;
  sr_infos5.packet.buf_capacity = user_pkt_buf_cap;
  sr_infos5.packet.is_completed = false;
  sr_infos5.addr = SR_START_ADDR;
  sr_infos5.addr_length = SR_ADDR_LEN;
  sr_infos5.p_xels = info_xels_sr5;
  sr_infos5.xel_count = 0;

  for (int i = 0; i < DXL_ID_CNT_ARM_L_HEAD; i++) {
    info_xels_sr5[i].id = DXL_ID_LIST_ARM_L_HEAD[i];
    info_xels_sr5[i].p_recv_buf = (uint8_t*)&sr_data5[i];
    sr_infos5.xel_count++;
  }
  sr_infos5.is_info_changed = true;

  // Fill the members of structure to syncWrite using internal packet buffer
  sw_infos5.packet.p_buf = nullptr;
  sw_infos5.packet.is_completed = false;
  sw_infos5.addr = SW_START_ADDR;
  sw_infos5.addr_length = SW_ADDR_LEN;
  sw_infos5.p_xels = info_xels_sw5;
  sw_infos5.xel_count = 0;

  for (int i = 0; i < DXL_ID_CNT_ARM_L_HEAD; i++) {
    info_xels_sw5[i].id = DXL_ID_LIST_ARM_L_HEAD[i];
    info_xels_sw5[i].p_data = (uint8_t*)&sw_data5[i].goal_position;
    sw_infos5.xel_count++;
  }
  sw_infos5.is_info_changed = true;
}

void bno_setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Wire.begin();
  digitalWrite(LED_BUILTIN, LOW);
  if (myIMU.begin() == false)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
    digitalWrite(LED_BUILTIN, HIGH);
    while (1)
      ;
  }
  delay(10);
  Wire.setClock(400000); // Increase I2C data rate to 400kHz
  delay(10);
  // myIMU.enableLinearAccelerometer(8); // m/s^2 no gravity
  myIMU.enableAccelerometer(8);
  myIMU.enableRotationVector(4);      // quat
  myIMU.enableGyro(12);               // rad/s

  nh.advertise(pub_imu);

  digitalWrite(LED_BUILTIN, LOW);
}

void bno_loop() {
  inertial.header.frame_id = in;
  inertial.header.stamp = nh.now();
  if (myIMU.dataAvailable() == true)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    // internal copies of the IMU data
    byte linAccuracy = 0;
    byte gyroAccuracy = 0;
    float quatRadianAccuracy = 0;
    byte quatAccuracy = 0;

    // get IMU data in one go for each sensor type
    //myIMU.getLinAccel(lin.x, lin.y, lin.z, linAccuracy);
    myIMU.getAccel(lin.x, lin.y, lin.z, linAccuracy);
    myIMU.getGyro(ang.x, ang.y, ang.z, gyroAccuracy);
    myIMU.getQuat(inertial.orientation.x, inertial.orientation.y, inertial.orientation.z, inertial.orientation.w, quatRadianAccuracy, quatAccuracy);
    inertial.angular_velocity = ang;
    inertial.linear_acceleration = lin;
    pub_imu.publish(&inertial);
    //        digitalWrite(LED_BUILTIN, LOW);
  }
  pub_imu.publish(&inertial);
  delay(1);
}
