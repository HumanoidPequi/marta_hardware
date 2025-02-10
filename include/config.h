#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <Dynamixel2Arduino.h>
#include <math.h>

// ==================== SERIAL E PINOS ====================
#define DXL_SERIAL1 Serial1  // perna direita
#define DXL_SERIAL2 Serial3  // perna esquerda
#define DXL_SERIAL3 Serial2  // braço direito
#define DXL_SERIAL5 Serial5  // braço esquerdo e cabeça

const int DXL_DIR_PIN1 = 2;
const int DXL_DIR_PIN2 = 22;
const int DXL_DIR_PIN3 = 9;
const int DXL_DIR_PIN5 = 23;

// ==================== DYNAMIXEL ====================
const uint8_t BROADCAST_ID = 254;
const float DYNAMIXEL_PROTOCOL_VERSION   = 2.0;
const float DYNAMIXEL_PROTOCOL_VERSION1  = 1.0;

const uint8_t DXL_ID_CNT = 6;
const uint8_t DXL_ID_CNT_ARM_R = DXL_ID_CNT / 2;
const uint8_t DXL_ID_CNT_ARM_L_HEAD = DXL_ID_CNT - 1;

extern const uint8_t DXL_ID_LIST_R[DXL_ID_CNT];
extern const uint8_t DXL_ID_LIST_L[DXL_ID_CNT];
extern const uint8_t DXL_ID_LIST_ARM_R[DXL_ID_CNT_ARM_R];
extern const uint8_t DXL_ID_LIST_ARM_L_HEAD[DXL_ID_CNT_ARM_L_HEAD];

extern const int8_t Signal_left[DXL_ID_CNT];
extern const int8_t Signal_right[DXL_ID_CNT];
extern const int8_t Signal_arm_r[DXL_ID_CNT_ARM_R];
extern const int8_t Signal_arm_l_head[DXL_ID_CNT_ARM_L_HEAD];

// ==================== PACOTES PARA SYNC READ/WRITE ====================
const uint16_t user_pkt_buf_cap = 128;
extern uint8_t user_pkt_buf1[user_pkt_buf_cap];
extern uint8_t user_pkt_buf2[user_pkt_buf_cap];
extern uint8_t user_pkt_buf3[user_pkt_buf_cap];

// Endereços para leitura e escrita (DYNAMIXEL)
extern const uint16_t SR_START_ADDR;
extern const uint16_t SR_ADDR_LEN;
extern const uint16_t SW_START_ADDR;
extern const uint16_t SW_ADDR_LEN;

extern const uint16_t SR_START_ADDR1;
extern const uint16_t SR_ADDR_LEN1;
extern const uint16_t SW_START_ADDR1;
extern const uint16_t SW_ADDR_LEN1;

// ==================== TIPOS DE DADOS PARA SYNC ====================
typedef struct sr_data {
  int32_t present_position;
} __attribute__((packed)) sr_data_t;

typedef struct sw_data {
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

// ==================== DEFINIÇÕES DE JUNTAS ====================
// Perna direita
extern const int32_t rHipYaw;
extern const int32_t rHipRoll;
extern const int32_t rHipPitch;
extern const int32_t rKneePitch;
extern const int32_t rAnklePitch;
extern const int32_t rAnkleRoll;

// Perna esquerda
extern const int32_t lHipYaw;
extern const int32_t lHipRoll;
extern const int32_t lHipPitch;
extern const int32_t lKneePitch;
extern const int32_t lAnklePitch;
extern const int32_t lAnkleRoll;

// Limites e posições iniciais
extern unsigned int minPosR[DXL_ID_CNT];
extern unsigned int minPosL[DXL_ID_CNT];
extern unsigned int maxPosR[DXL_ID_CNT];
extern unsigned int maxPosL[DXL_ID_CNT];

extern int32_t goal_position_initial_R[DXL_ID_CNT];
extern int32_t goal_position_R[DXL_ID_CNT];

extern int32_t goal_position_initial_L[DXL_ID_CNT];
extern int32_t goal_position_L[DXL_ID_CNT];

extern int32_t goal_position_initial_arm_r[DXL_ID_CNT_ARM_R];
extern int32_t goal_position_arm_r[DXL_ID_CNT_ARM_R];

extern int32_t goal_position_initial_arm_l_head[DXL_ID_CNT_ARM_L_HEAD];
extern int32_t goal_position_arm_l_head[DXL_ID_CNT_ARM_L_HEAD];

// ==================== FUNÇÕES DE CONVERSÃO ====================
int convert_command(float ang_command);
int convert_command_degree(int ang_command);
float convert_state(int ang_state);

#endif  // CONFIG_H
