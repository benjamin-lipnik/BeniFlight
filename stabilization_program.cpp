#include "stabilization_program.h"
#include <math.h>
#include <Arduino.h>

// PI / 180
#define DEG_TO_RAD  (0.0174532925f)

// 180 / PI
#define RAD_TO_DEG  (57.295779513f)

static World_data world_data;
static Calculated_IMU_Data calc_data = {&world_data};

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

  world_data.pitch_angle = (world_data.pitch_angle * 0.9992) + (angle_pitch_acc * 0.0008);
  world_data.roll_angle  = (world_data.roll_angle  * 0.9992) + (angle_roll_acc  * 0.0008);

  //HEADING CALC

  //YEP NEKAK GLOBALNO ZA AKSE BI BLO TREBA ZRIHTAT
  //VLEK TODO
  #define MAG_X 1
  #define MAG_Y 0
  #define MAG_Z 2

  //TOLE KONCN DELA!!! :) :D
  imu_data->mag_gauss[MAG_X] *= -1;
  //imu_data->mag_gauss[MAG_Y] *= -1;
  imu_data->mag_gauss[MAG_Z] *= -1;

  float compass_x = imu_data->mag_gauss[MAG_X] * cos(-world_data.pitch_angle * DEG_TO_RAD)
                  + imu_data->mag_gauss[MAG_Y] * sin(world_data.roll_angle   * DEG_TO_RAD) * sin(-world_data.pitch_angle * DEG_TO_RAD)
                  - imu_data->mag_gauss[MAG_Z] * cos(world_data.roll_angle   * DEG_TO_RAD) * sin(-world_data.pitch_angle * DEG_TO_RAD);

  float compass_y = imu_data->mag_gauss[MAG_Y] * cos(world_data.roll_angle   * DEG_TO_RAD)
                  + imu_data->mag_gauss[MAG_Z] * sin(world_data.roll_angle   * DEG_TO_RAD);


  //tu bi se blo treba dodat magnetno deklinacijo  v ljubljani mislim da je okol 4 stopinje,
  float tmp_heading = atan2(compass_y, compass_x) * RAD_TO_DEG;

  //tole je fajn za ugotavlat kire so akse magnetometra
  //float tmp_heading = atan2(imu_data->mag_gauss[MAG_Y], imu_data->mag_gauss[MAG_X]) * RAD_TO_DEG;

  //Transformacija iz ()-180 do 180) v (0 do 360)
  if(tmp_heading < 0)
    tmp_heading+= 360;
  else if(tmp_heading > 360)
    tmp_heading -= 360;

  world_data.heading_angle = tmp_heading;

  return &calc_data;
}

static uint16_t motor_powers[4];
uint16_t * calculate_motor_powers(Calculated_IMU_Data * imu_data, Radio_pkg * radio_data, PIDProfile ** pid_profiles, float delta_time) {

  if(!radio_data)
    radio_data = (Radio_pkg *)&flushed_pkg;

  if(!(radio_data->buttons & (1<<ARM_BIT))) {
    //UNARMED
    for(uint8_t i = 0; i < 4; i++) {
      motor_powers[i] = 1000;
    }

    pid_clear(pid_profiles[ROLL_INDEX]);
    pid_clear(pid_profiles[PITCH_INDEX]);
    pid_clear(pid_profiles[YAW_INDEX]);

    return motor_powers;
  }

  float pid_roll_setpoint  = map(radio_data->roll,  1000, 2000, -165,  165);
  float pid_pitch_setpoint = map(radio_data->pitch, 1000, 2000,  165, -165);
  float pid_yaw_setpoint   = map(radio_data->yaw,   1000, 2000, -165,  165);


#ifdef ENABLE_HEADLESS
  //roll je x aksa
  //pitch je y aksa

  float x_tmp = pid_roll_setpoint;
  float y_tmp = pid_pitch_setpoint;

  float fi = imu_data->world_data->heading_angle * DEG_TO_RAD;

  float sin_fi = sin(fi);
  float cos_fi = cos(fi);

  //TRANSFORMACIJA
  pid_roll_setpoint  = (x_tmp * cos_fi) - (y_tmp * sin_fi);
  pid_pitch_setpoint = (x_tmp * sin_fi) + (y_tmp * cos_fi);

#endif

  //testni heading_lock da vijio ce dejla
  //treba je naredit tak, da poisce v kero smer se je blizje obrnat (v CW ali CCW)
  //in potem tisto vstavimo kot premik, ker nemoremo vstavit direkt kota, ker
  //je nezvezno -> to pomeni, da je kot 0 stopinj isto kot kot 360 stopinj in se za isti v drugo smer obraca
  //pa take pa to

#ifdef ENABLE_HEADLOCK

  float yaw_fix = 0;
  if(imu_data->world_data->heading_angle > 180) {
    //obracaj v eno smer
    yaw_fix = (360 - imu_data->world_data->heading_angle);
  }
  else {
    //obracaj v drugo smer
    yaw_fix = -imu_data->world_data->heading_angle; //minus da obrne smer
  }

  //AUTO YAW
  pid_yaw_setpoint -= yaw_fix * HEADLOCK_STRENGTH;

#endif

#ifdef ENABLE_AUTOLEVEL
  pid_roll_setpoint  -= imu_data->world_data->roll_angle  * AUTOLEVEL_STRENGTH;
  pid_pitch_setpoint -= imu_data->world_data->pitch_angle * AUTOLEVEL_STRENGTH;
#endif

  float roll_correction  = pid_calculate(pid_profiles[ROLL_INDEX],  imu_data->omega_pid_input[ROLL_INDEX],  pid_roll_setpoint,  delta_time);
  float pitch_correction = pid_calculate(pid_profiles[PITCH_INDEX], imu_data->omega_pid_input[PITCH_INDEX], pid_pitch_setpoint, delta_time);
  float yaw_correction   = pid_calculate(pid_profiles[YAW_INDEX],   imu_data->omega_pid_input[YAW_INDEX],   pid_yaw_setpoint,   delta_time);

  if(radio_data->power > 2000)
    radio_data->power = 2000;
  if(radio_data->power < IDLE_POWER)
    radio_data->power = IDLE_POWER;

  radio_data->power *= MAX_USER_POWER;

  motor_powers[MOTOR_A_INDEX] = radio_data->power + roll_correction + pitch_correction + yaw_correction;
  motor_powers[MOTOR_B_INDEX] = radio_data->power - roll_correction + pitch_correction - yaw_correction;
  motor_powers[MOTOR_C_INDEX] = radio_data->power + roll_correction - pitch_correction - yaw_correction;
  motor_powers[MOTOR_D_INDEX] = radio_data->power - roll_correction - pitch_correction + yaw_correction;

  for(uint8_t i = 0; i < 4; i++) {
    if(motor_powers[i] > 2000)
      motor_powers[i] = 2000;
    if(motor_powers[i] < IDLE_POWER)
      motor_powers[i] = IDLE_POWER;
  }
  return motor_powers;
}
