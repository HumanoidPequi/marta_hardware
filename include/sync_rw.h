#ifndef SYNC_RW_H
#define SYNC_RW_H

#include <Dynamixel2Arduino.h>
#include "config.h"

// Declaração dos objetos que serão usados em main.cpp para acesso aos dados dos motores:
extern DYNAMIXEL::InfoSyncReadInst_t sr_infos1;
extern sr_data_t sr_data1[DXL_ID_CNT];

extern DYNAMIXEL::InfoSyncReadInst_t sr_infos2;
extern sr_data_t sr_data2[DXL_ID_CNT];

extern DYNAMIXEL::InfoSyncReadInst_t sr_infos3;
extern sr_data_t sr_data3[DXL_ID_CNT_ARM_R];

void setupSyncReadWrite();
void performSyncReadWrite();

#endif  // SYNC_RW_H
