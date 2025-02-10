#include "config.h"

// ------ DYNAMIXEL IDs e sinais ------
const uint8_t DXL_ID_LIST_R[DXL_ID_CNT] = {1,2,3,4,5,6};
const uint8_t DXL_ID_LIST_L[DXL_ID_CNT] = {7,8,9,10,11,12};
const uint8_t DXL_ID_LIST_ARM_R[DXL_ID_CNT_ARM_R] = {13,14,15};
const uint8_t DXL_ID_LIST_ARM_L_HEAD[DXL_ID_CNT_ARM_L_HEAD] = {19,20,16,17,18};

const int8_t Signal_left[DXL_ID_CNT]      = {-1,-1, 1, 1,-1, 1};
const int8_t Signal_right[DXL_ID_CNT]     = {-1,-1,-1,-1, 1, 1};
const int8_t Signal_arm_r[DXL_ID_CNT_ARM_R] = {-1, 1,-1};
const int8_t Signal_arm_l_head[DXL_ID_CNT_ARM_L_HEAD] = {1, 1, 1,-1, 1};

// ------ Buffers para Sync Read/Write ------
uint8_t user_pkt_buf1[user_pkt_buf_cap];
uint8_t user_pkt_buf2[user_pkt_buf_cap];
uint8_t user_pkt_buf3[user_pkt_buf_cap];

// ------ Juntas (posições de referência) ------
// Perna direita
const int32_t rHipYaw   = 2048;
const int32_t rHipRoll  = 1980;
const int32_t rHipPitch = 2020;
const int32_t rKneePitch= 2048;
const int32_t rAnklePitch=2048;
const int32_t rAnkleRoll= 2048;

// Perna esquerda
const int32_t lHipYaw   = 2048;
const int32_t lHipRoll  = 2010;
const int32_t lHipPitch = 1100;
const int32_t lKneePitch= 2048;
const int32_t lAnklePitch=2020;
const int32_t lAnkleRoll= 2048;

unsigned int minPosR[DXL_ID_CNT] = {rHipYaw-341, rHipRoll-148, rHipPitch-455, rKneePitch-1024, rAnklePitch-455, rAnkleRoll-341};
unsigned int minPosL[DXL_ID_CNT] = {lHipYaw-341, lHipRoll-455, lHipPitch-1024, lKneePitch-1024, lAnklePitch-455, lAnkleRoll-341};
unsigned int maxPosR[DXL_ID_CNT] = {rHipYaw+341, rHipRoll+455, rHipPitch+1024, rKneePitch+1024, rAnklePitch+455, rAnkleRoll+341};
unsigned int maxPosL[DXL_ID_CNT] = {lHipYaw+341, lHipRoll+148, lHipPitch+455, lKneePitch+1024, lAnklePitch+455, lAnkleRoll+341};

int32_t goal_position_initial_R[DXL_ID_CNT] = {rHipYaw, rHipRoll, rHipPitch, rKneePitch, rAnklePitch, rAnkleRoll};
int32_t goal_position_R[DXL_ID_CNT]           = {rHipYaw, rHipRoll, rHipPitch, rKneePitch, rAnklePitch, rAnkleRoll};

int32_t goal_position_initial_L[DXL_ID_CNT] = {lHipYaw, lHipRoll, lHipPitch, lKneePitch, lAnklePitch, lAnkleRoll};
int32_t goal_position_L[DXL_ID_CNT]           = {lHipYaw, lHipRoll, lHipPitch, lKneePitch, lAnklePitch, lAnkleRoll};

int32_t goal_position_initial_arm_r[DXL_ID_CNT_ARM_R]     = {2048,2048,2048};
int32_t goal_position_arm_r[DXL_ID_CNT_ARM_R]             = {2048,2048,2048};

int32_t goal_position_initial_arm_l_head[DXL_ID_CNT_ARM_L_HEAD] = {2048,2048,2048,2048,2048};
int32_t goal_position_arm_l_head[DXL_ID_CNT_ARM_L_HEAD]         = {2048,2048,2048,2048,2048};

// ------ Endereços para Sync Read/Write ------
const uint16_t SR_START_ADDR  = 132;
const uint16_t SR_ADDR_LEN    = 4;
const uint16_t SW_START_ADDR  = 116;
const uint16_t SW_ADDR_LEN    = 4;

const uint16_t SR_START_ADDR1 = 38;
const uint16_t SR_ADDR_LEN1   = 2;
const uint16_t SW_START_ADDR1 = 30;
const uint16_t SW_ADDR_LEN1   = 2;

// ------ Funções de Conversão ------
int convert_command(float ang_command) {
  // Converte de radianos para valor de 12 bits
  return (int)((ang_command / M_PI) * 2048);
}

int convert_command_degree(int ang_command) {
  // Converte um comando em graus (escala de 16 bits) para 12 bits
  return map(ang_command, -1800, 1800, -2048, 2048);
}

float convert_state(int ang_state) {
  // Converte de 12 bits para radianos
  return (float)ang_state * M_PI / 2048.0;
}
