#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include "configuration.h"
#include <stdint.h>

typedef struct {
  float omega_dps[3];
  float acc_g[3];
  float mag_gauss[3];
  float height;
} IMU_TypeDef;

uint8_t imu_init(void * param);
IMU_TypeDef * imu_read();

#endif
