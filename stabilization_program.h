#ifndef STABILIZATION_PROGRAM_H
#define STABILIZATION_PROGRAM_H

#include <stdint.h>
#include "imu_interface.h"

typedef struct {
  float pitch_angle;
  float roll_angle;

  float heading_angle;
} World_data;

typedef struct {
  World_data * world_data;
  float omega_pid_input[3];
} Calculated_IMU_Data;

Calculated_IMU_Data * calulate_imu_data(IMU_TypeDef * imu_data, float delta_time);


#endif
