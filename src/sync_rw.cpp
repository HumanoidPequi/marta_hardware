#include "sync_rw.h"
#include "dynamixel_manager.h"
#include "config.h"
#include <Arduino.h>

// ------ Grupo 1: Perna Direita ------
DYNAMIXEL::InfoSyncReadInst_t sr_infos1;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr1[DXL_ID_CNT];
sr_data_t sr_data1[DXL_ID_CNT];

DYNAMIXEL::InfoSyncWriteInst_t sw_infos1;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw1[DXL_ID_CNT];
sw_data_t sw_data1[DXL_ID_CNT];

// ------ Grupo 2: Perna Esquerda ------
DYNAMIXEL::InfoSyncReadInst_t sr_infos2;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr2[DXL_ID_CNT];
sr_data_t sr_data2[DXL_ID_CNT];

DYNAMIXEL::InfoSyncWriteInst_t sw_infos2;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw2[DXL_ID_CNT];
sw_data_t sw_data2[DXL_ID_CNT];

// ------ Grupo 3: Braço Direito ------
DYNAMIXEL::InfoSyncReadInst_t sr_infos3;
DYNAMIXEL::XELInfoSyncRead_t info_xels_sr3[DXL_ID_CNT_ARM_R];
sr_data_t sr_data3[DXL_ID_CNT_ARM_R];

DYNAMIXEL::InfoSyncWriteInst_t sw_infos3;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw3[DXL_ID_CNT_ARM_R];
sw_data_t sw_data3[DXL_ID_CNT_ARM_R];

// ------ Grupo 4: Braço Esquerdo/ Cabeça ------
DYNAMIXEL::InfoSyncWriteInst_t sw_infos5;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw5[DXL_ID_CNT_ARM_L_HEAD];
sw_data_t sw_data5[DXL_ID_CNT_ARM_L_HEAD];

void setupSyncReadWrite() {
  // Configurações para o grupo 1 (perna direita)
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
    sr_infos1.xel_count++;
  }
  sr_infos1.is_info_changed = true;
  
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
  
  // Configurações para o grupo 2 (perna esquerda)
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
  
  // Configurações para o grupo 3 (braço direito)
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
  
  // Configurações para o grupo 4 (braço esquerdo/cabeça)
  sw_infos5.packet.p_buf = nullptr;
  sw_infos5.packet.is_completed = false;
  sw_infos5.addr = SW_START_ADDR1;
  sw_infos5.addr_length = SW_ADDR_LEN1;
  sw_infos5.p_xels = info_xels_sw5;
  sw_infos5.xel_count = 0;
  for (int i = 0; i < DXL_ID_CNT_ARM_L_HEAD; i++) {
    info_xels_sw5[i].id = DXL_ID_LIST_ARM_L_HEAD[i];
    info_xels_sw5[i].p_data = (uint8_t*)&sw_data5[i].goal_position;
    sw_infos5.xel_count++;
  }
  sw_infos5.is_info_changed = true;
}

void performSyncReadWrite() {
  // Insere as posições desejadas nos buffers de sync write
  for (int i = 0; i < DXL_ID_CNT; i++) {
    sw_data1[i].goal_position = goal_position_R[i];
    sw_data2[i].goal_position = goal_position_L[i];
  }
  sw_infos1.is_info_changed = true;
  sw_infos2.is_info_changed = true;
  
  for (int i = 0; i < DXL_ID_CNT_ARM_R; i++) {
    sw_data3[i].goal_position = goal_position_arm_r[i];
  }
  sw_infos3.is_info_changed = true;
  
  for (int i = 0; i < DXL_ID_CNT_ARM_L_HEAD; i++) {
    sw_data5[i].goal_position = goal_position_arm_l_head[i];
  }
  sw_infos5.is_info_changed = true;
  
  // Envia os comandos de Sync Write para cada grupo
  dxl1.syncWrite(&sw_infos1);
  dxl2.syncWrite(&sw_infos2);
  dxl3.syncWrite(&sw_infos3);
  dxl5.syncWrite(&sw_infos5);
  
  // Em seguida, realiza as Sync Read para obter os estados dos motores.
  dxl1.syncRead(&sr_infos1);
  dxl2.syncRead(&sr_infos2);
  dxl3.syncRead(&sr_infos3);
}
