#ifndef IMU_INTERFACE_H
#define IMU_INTERFACE_H

#include "configuration.h"
#include <stdint.h>

//V array nastavis katere akse senzorji dajejo po vrsti
//ce senzor najprej poslje Y, potem X, otem Z
//potem naj bo szr_axes[Y_INDEX, X_INDEX, Z_INDEX]
static const uint8_t gyro_axes [3] = {X_INDEX, Y_INDEX, Z_INDEX};
static const uint8_t acc_axes  [3] = {X_INDEX, Y_INDEX, Z_INDEX};
static const uint8_t mag_axes  [3] = {Y_INDEX, Z_INDEX, X_INDEX};


typedef struct {
  float omega_dps[3];
  float acc_g[3];
  float mag_gauss[3];
  float height;
} IMU_TypeDef;

uint8_t imu_init(void * param);
IMU_TypeDef * imu_read();

#endif
