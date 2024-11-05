#include <Dynamixel2Arduino.h>
#include <ros.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle nh;

#define DEBUG_SERIAL Serial
#define DXL_SERIAL1 Serial1 // right leg
#define DXL_SERIAL2 Serial5 // left lef
#define DXL_SERIAL3 Serial2 // right arm
#define DXL_SERIAL5 Serial3 // left arm and head

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
const uint8_t DXL_ID_LIST_ARM_L_HEAD[DXL_ID_CNT_ARM_L_HEAD] = {16,17,18,19,20};// 3 primeiros são do braço esquerdo e os dois últimos são da cabeça
const int8_t Signal_left[DXL_ID_CNT] = {-1,-1,1,1,-1,1};
const int8_t Signal_right[DXL_ID_CNT] = {-1,-1,-1,-1,1,1};
const int8_t Signal_arm_r[DXL_ID_CNT_ARM_R] = {-1,-1,-1}; //ainda precisa ser definido
const int8_t Signal_arm_l_head[DXL_ID_CNT_ARM_L_HEAD] = {-1,1,1,1,1}; //ainda precisa ser definido
const uint16_t user_pkt_buf_cap = 128;
uint8_t user_pkt_buf1[user_pkt_buf_cap];
uint8_t user_pkt_buf2[user_pkt_buf_cap];
uint8_t user_pkt_buf3[user_pkt_buf_cap];
uint8_t user_pkt_buf5[user_pkt_buf_cap];

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

sw_data_t sw_data5[DXL_ID_CNT_ARM_R];
DYNAMIXEL::InfoSyncWriteInst_t sw_infos5;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw5[DXL_ID_CNT_ARM_L_HEAD];

Dynamixel2Arduino dxl1(DXL_SERIAL1, DXL_DIR_PIN1);
Dynamixel2Arduino dxl2(DXL_SERIAL2, DXL_DIR_PIN2);
Dynamixel2Arduino dxl3(DXL_SERIAL3, DXL_DIR_PIN3);
Dynamixel2Arduino dxl5(DXL_SERIAL5, DXL_DIR_PIN5);

// This namespace is required to use DYNAMIXEL Control table item name definitions
using namespace ControlTableItem;

int32_t goal_position_initial_R[DXL_ID_CNT] = {2048,2048,2048,2048,2048,2048};
int32_t goal_position_R[DXL_ID_CNT] = {2048,2048,2048,2048,2048,2048};

int32_t goal_position_initial_L[DXL_ID_CNT] = {2048,2048,2048,2048,2048,2048};
int32_t goal_position_L[DXL_ID_CNT] = {2048,2048,2048,2048,2048,2048};

int32_t goal_position_initial_arm_r[DXL_ID_CNT_ARM_R] = {2048,2048,2048};
int32_t goal_position_arm_r[DXL_ID_CNT_ARM_R] = {2048,2048,2048};

int32_t goal_position_initial_arm_l_head[DXL_ID_CNT_ARM_L_HEAD] = {2048,2048,2048,2048,2048};
int32_t goal_position_arm_l_head[DXL_ID_CNT_ARM_L_HEAD] = {2048,2048,2048,2048,2048};

int convert_command(int ang_command /*, int32_t marta_joints_initial, bool joint_reversed*/) {
  //Converte os valores de 16 bits recebidos pelo rosserial para os valores de 12 bits
  int position_command = map(ang_command, -1800, 1800, -2048, 2048);
  
  return position_command;
}

int convert_state(int ang_state /*, int32_t marta_joints_initial, bool joint_reversed*/) {
  //Converte os valores de 16 bits recebidos pelo rosserial para os valores de 12 bits
  int position_state = map(ang_state, -2048, 2048, -1800, 1800);
  
  return position_state;
}

void right_leg_cb(const std_msgs::Int16MultiArray& cmd_msg) {
   for (int i = 0; i < DXL_ID_CNT; i++) {
      int32_t d_theta = convert_command(cmd_msg.data[i]);
      goal_position_R[i] = goal_position_initial_R[i] + d_theta*Signal_right[i];
   }
}

void left_leg_cb(const std_msgs::Int16MultiArray& cmd_msg) {
   for (int i = 0; i < DXL_ID_CNT; i++) {
      int32_t d_theta = convert_command(cmd_msg.data[i]);
      goal_position_L[i] = goal_position_initial_L[i] + d_theta*Signal_left[i];
   }
}

void arm_r_cb(const std_msgs::Int16MultiArray& cmd_msg) {
   for (int i = 0; i < DXL_ID_CNT_ARM_R; i++) {
      int32_t d_theta = convert_command(cmd_msg.data[i]);
      goal_position_arm_r[i] = goal_position_arm_r[i] + d_theta*Signal_arm_r[i];
   }
}

void arm_l_head_cb(const std_msgs::Int16MultiArray& cmd_msg) {
   for (int i = 0; i < DXL_ID_CNT_ARM_L_HEAD; i++) {
      int32_t d_theta = convert_command(cmd_msg.data[i]);
      goal_position_arm_l_head[i] = goal_position_arm_l_head[i] + d_theta*Signal_arm_l_head[i];
   }
}

int delay1 = 1; // Ajuste de delay

std_msgs::Int16MultiArray msg_R;
std_msgs::Int16MultiArray msg_L;
std_msgs::Int16MultiArray msg_arm_R;
std_msgs::Int16MultiArray msg_arm_L_head;

ros::Publisher right_leg_pub("/marta/right_leg/state", &msg_R);
ros::Publisher left_leg_pub("/marta/left_leg/state", &msg_L);
ros::Publisher arm_r_pub("/marta/arm_r/state", &msg_arm_R);
ros::Publisher arm_l_head_pub("/marta/arm_l_head/state", &msg_arm_L_head);
ros::Subscriber<std_msgs::Int16MultiArray> right_leg_sub("/marta/right_leg/command", right_leg_cb);
ros::Subscriber<std_msgs::Int16MultiArray> left_leg_sub("/marta/left_leg/command", left_leg_cb);
ros::Subscriber<std_msgs::Int16MultiArray> arm_r_sub("/marta/arm_r/command", arm_r_cb);
ros::Subscriber<std_msgs::Int16MultiArray> arm_l_head_sub("/marta/arm_l_head/command", arm_l_head_cb);

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

}

void loop() {
  static uint32_t try_count = 0;
  uint8_t i, recv_cnt;

  // Insert a new Goal Position to the SyncWrite Packet
  for (i = 0; i < DXL_ID_CNT; i++) {
    sw_data1[i].goal_position = goal_position_R[i];
    sw_data2[i].goal_position = goal_position_L[i];
    sw_data3[i].goal_position = goal_position_arm_r[i];
    sw_data5[i].goal_position = goal_position_arm_l_head[i];
    
  }
  unsigned long start = micros();

  // Update the SyncWrite packet status
  sw_infos1.is_info_changed = true;
  sw_infos2.is_info_changed = true;
  sw_infos3.is_info_changed = true;
  sw_infos5.is_info_changed = true;
  
  dxl1.syncWrite(&sw_infos1);
  dxl2.syncWrite(&sw_infos2);
  dxl3.syncWrite(&sw_infos3);
  dxl5.syncWrite(&sw_infos5);

  delay(delay1);

  // Transmit predefined SyncRead instruction packet
  // and receive a status packet from each DYNAMIXEL
  uint8_t recv_cnt1 = dxl1.syncRead(&sr_infos1);
  uint8_t recv_cnt2 = dxl2.syncRead(&sr_infos2);
  uint8_t recv_cnt3 = dxl3.syncRead(&sr_infos3);
  uint8_t recv_cnt5 = dxl5.syncRead(&sr_infos5);
  
  if (recv_cnt1 > 0) {
    // Prepare msg data
    msg_R.data_length = recv_cnt1;
    msg_R.data = (int16_t*) malloc(recv_cnt1 * sizeof(int16_t));

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
    msg_L.data = (int16_t*) malloc(recv_cnt2 * sizeof(int16_t));
    
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
    msg_arm_R.data = (int16_t*) malloc(recv_cnt3 * sizeof(int16_t));
    
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
    msg_arm_L_head.data = (int16_t*) malloc(recv_cnt5 * sizeof(int16_t));
    
    for (i = 0; i < recv_cnt5; i++) {
      msg_arm_L_head.data[i] = convert_state(sr_data5[i].present_position - goal_position_initial_arm_l_head[i])*Signal_arm_l_head[i];
      //Serial.println(msg_L.data[i]); // Debug print to Serial
    }
    
    arm_l_head_pub.publish(&msg_arm_L_head);
    free(msg_arm_L_head.data); // Free the allocated memory
  }

  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  unsigned long end = micros();
  unsigned long elapsed_microseconds = end - start;

  nh.spinOnce();
  delay(3 * delay1);
}

void dynamixel_setup(){
  int i;
  pinMode(LED_BUILTIN, OUTPUT);
  DEBUG_SERIAL.begin(1000000);
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
