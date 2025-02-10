#include "dynamixel_manager.h"
#include "config.h"
#include <Arduino.h>

// Criação dos objetos para cada porta
Dynamixel2Arduino dxl1(DXL_SERIAL1, DXL_DIR_PIN1);
Dynamixel2Arduino dxl2(DXL_SERIAL2, DXL_DIR_PIN2);
Dynamixel2Arduino dxl3(DXL_SERIAL3, DXL_DIR_PIN3);
Dynamixel2Arduino dxl5(DXL_SERIAL5, DXL_DIR_PIN5);

void setupDynamixels() {
  Serial.begin(1000000);
  
  dxl1.begin(1000000);
  dxl1.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);
  
  dxl2.begin(1000000);
  dxl2.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);
  
  dxl3.begin(1000000);
  dxl3.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION);
  
  dxl5.begin(1000000);
  dxl5.setPortProtocolVersion(DYNAMIXEL_PROTOCOL_VERSION1);
  
  // Configuração para perna direita
  for (int i = 0; i < DXL_ID_CNT; i++) {
    dxl1.torqueOff(DXL_ID_LIST_R[i]);
    dxl1.setOperatingMode(DXL_ID_LIST_R[i], OP_POSITION);
  }
  dxl1.torqueOn(BROADCAST_ID);
  
  // Configuração para perna esquerda
  for (int i = 0; i < DXL_ID_CNT; i++) {
    dxl2.torqueOff(DXL_ID_LIST_L[i]);
    dxl2.setOperatingMode(DXL_ID_LIST_L[i], OP_POSITION);
  }
  dxl2.torqueOn(BROADCAST_ID);
  
  // Configuração para braço direito
  for (int i = 0; i < DXL_ID_CNT_ARM_R; i++) {
    dxl3.torqueOff(DXL_ID_LIST_ARM_R[i]);
    dxl3.setOperatingMode(DXL_ID_LIST_ARM_R[i], OP_POSITION);
  }
  dxl3.torqueOn(BROADCAST_ID);
  
  // Configuração para braço esquerdo e cabeça
  for (int i = 0; i < DXL_ID_CNT_ARM_L_HEAD; i++) {
    dxl5.torqueOff(DXL_ID_LIST_ARM_L_HEAD[i]);
    dxl5.setOperatingMode(DXL_ID_LIST_ARM_L_HEAD[i], OP_POSITION);
  }
  dxl5.torqueOn(BROADCAST_ID);
}
