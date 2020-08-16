#ifndef STABILIZATION_PROGRAM_H
#define STABILIZATION_PROGRAM_H

#include <stdint.h>
#include "configuration.h"
#include "imu_interface.h"
#include "radio_interface.h"
#include "motor_interface.h"
#include "pid.h"

typedef struct {
  float pitch_angle;
  float roll_angle;
  //float yaw_angle;//test

  float heading_angle;
} World_data;

typedef struct {
  World_data * world_data;
  float omega_pid_input[3];
} Calculated_IMU_Data;

Calculated_IMU_Data * calulate_imu_data(IMU_TypeDef * imu_data, float delta_time);
uint16_t * calculate_motor_powers(Calculated_IMU_Data * imu_data, Radio_pkg radio_data, PIDProfile ** pid_profiles, float delta_time);


#endif
