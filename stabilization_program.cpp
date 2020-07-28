#include "stabilization_program.h"
#include <math.h>

// PI / 180
#define DEG_TO_RAD  (0.0174532925f)

// 180 / PI
#define RAD_TO_DEG  (57.295779513f)

World_data world_data;
Calculated_IMU_Data calc_data = {&world_data};

Calculated_IMU_Data * calulate_imu_data(IMU_TypeDef * imu_data, float delta_time) {

  //PID INPUT CALC
  for(uint8_t i = 0; i < 3; i++) {
    calc_data.omega_pid_input[i] = (calc_data.omega_pid_input[i] * 0.8) + (imu_data->omega_dps[i] * 0.2);
  }

  //ANGLE CALC
  world_data.pitch_angle += imu_data->omega_dps[PITCH_INDEX] * delta_time;
  world_data.roll_angle  += imu_data->omega_dps[ROLL_INDEX]  * delta_time;

  float koeficient = sin(imu_data->omega_dps[YAW_INDEX] * delta_time * DEG_TO_RAD);
  world_data.pitch_angle -= world_data.roll_angle  * koeficient;
  world_data.roll_angle  += world_data.pitch_angle * koeficient;

  float angle_pitch_acc = -atan2(imu_data->acc_g[ROLL_INDEX],  imu_data->acc_g[YAW_INDEX]) * RAD_TO_DEG; //in deg
  float angle_roll_acc  =  atan2(imu_data->acc_g[PITCH_INDEX], imu_data->acc_g[YAW_INDEX]) * RAD_TO_DEG;

  world_data.pitch_angle = (world_data.pitch_angle * 0.995) + (angle_pitch_acc * 0.005);
  world_data.roll_angle  = (world_data.roll_angle  * 0.995) + (angle_roll_acc  * 0.005);


  //HEADING CALC

  return &calc_data;
}
