#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include "configuration.h"

typedef struct {
  float omega_dps[3];
  float acc_dps[3];
  float mag_gauss[3];
} IMU_TypeDef;

void imu_init(void * param);
IMU_TypeDef * imu_read();

#endif
