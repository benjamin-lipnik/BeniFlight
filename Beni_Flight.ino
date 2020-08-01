//cim bolj standardizirano podajanje podatkov
//interfaci za motorje, imu, radio,...
#include "configuration.h"
#include "comm.h"
#include "imu_interface.h"
#include "radio_interface.h"
#include "motor_interface.h"
#include "failsafe.h"
#include "stabilization_program.h"

char str[100]; //buffer za izpisovanje
unsigned long loop_micros = 0;
unsigned long loop_millis = 0;
unsigned long delta_micros = 0;
unsigned long last_radio_update = 0;

//PIDProfile roll_pid_profile  = {1.3f, 0.005f, 16.5f, 400.0f};
PIDProfile roll_pid_profile  = {1.2f, 0.0f, 16.5f, 400.0f};
PIDProfile pitch_pid_profile = {1.2f, 0.0f, 16.5f, 400.0f};
//PIDProfile yaw_pid_profile   = {2.0f, 0.002f, 0.0f,  300.0f};
PIDProfile yaw_pid_profile   = {4.0f, 0.002f, 0.0f,  300.0f};

Radio_pkg radio_data;

void setup() {
  pinMode(MCU_LED_PIN,  OUTPUT);
  pinMode(RED_LED_PIN,  OUTPUT);
  pinMode(BLUE_LED_PIN, OUTPUT);

  digitalWrite(BLUE_LED_PIN, HIGH);

  uint8_t init_info;
  init_info  = print_init (NULL);
  init_info &= imu_init   (NULL);
  init_info &= radio_init (NULL);
  init_info &= motors_init(NULL);

  if(init_info != INIT_OK) {
    error_handler_id(0);
  }

  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN,  HIGH);
  println((char *)"OK!");

  Radio_pkg * radio_rx = radio_read();
  if(radio_rx && radio_rx->buttons & (1<<ARM_BIT)) {
    //ARMED ON START
    error_handler_id(1);
  }

  loop_millis = millis();
  loop_micros = micros();
}

void loop() {

  /*TIMING*/
  delta_micros = loop_micros;//last_micros
  loop_micros  = micros();
  delta_micros = loop_micros - delta_micros; //last - new micros
  loop_millis  = loop_micros * 0.001;

  if(delta_micros > 4000) { //too long
    error_handler_id(2);
    //error
  }
  /*IMU*/

  //Preberemo podatke z IMU in jih preracunamo v uporabne vrednosti
  IMU_TypeDef * imu_data = imu_read();
  //sprintf(str, "dt: %lu, MAG X: %.2f, Y: %.2f, Z:%.2f\n\r", delta_micros, imu_data->mag_gauss[X_INDEX], imu_data->mag_gauss[Y_INDEX], imu_data->mag_gauss[Z_INDEX]);
  //sprintf(str, "dt: %lu, GYRO X: %.2f, Y: %.2f, Z:%.2f\n\r", delta_micros, imu_data->omega_dps[X_INDEX], imu_data->omega_dps[Y_INDEX], imu_data->omega_dps[Z_INDEX]);
  //sprintf(str, "dt: %lu, ACC X: %.2f, Y: %.2f, Z:%.2f\n\r", delta_micros, imu_data->acc_g[X_INDEX], imu_data->acc_g[Y_INDEX], imu_data->acc_g[Z_INDEX]);
  //print(str);

  /*RADIO*/

  //Pollamo za podatke od dalinca in gledamo, ce nam je zanjkal signala
  //Gledamo tudi ce smo signal dobili nazaj.
  Radio_pkg * radio_rx = radio_read();

  if(radio_rx) {
    radio_data = *radio_rx;
    last_radio_update = loop_millis;
  }

  //FAILSAFE
  if(!radio_rx && radio_data.buttons & (1<<ARM_BIT)) {
    #ifdef ENABLE_FAILSAFE
      if(loop_millis - last_radio_update > LOST_SIGNAL_FAILSAFE_TIMEOUT) {
        error_handler_id(3);
      }
      else if(loop_millis - last_radio_update > 150) {
        //flush_rc_paket, set power to failsafe power
        radio_data.power = LOST_SIGNAL_POWER;
        radio_data.roll  = 1500;
        radio_data.pitch = 1500;
        radio_data.yaw   = 1500;
      }
    #else
      if(loop_millis - last_radio_update > 100) {
        error_handler_id(3);
      }
    #endif
  }

  if(radio_rx) {
    sprintf(str, "p: %d, y: %d, t: %d, r: %d", radio_data.power, radio_data.yaw, radio_data.pitch, radio_data.roll);
    //println(str);
    Serial.println(str);
  }
  //test
  radio_data.buttons = 0; // disarmamo z vsak slucaj

  /*STABILIZACIJA*/

  //ce je vseeee ok pol stabiliziramo dron, in poslusamo dalinc
  //ce ni signala pol stabiliziramo dron in padamo proti tlem
  //ce je vse narobe ugasnemo motorje in vklopimo padalo ce ga mamo in ustavimo program

  Calculated_IMU_Data * calc_data = calulate_imu_data(imu_data, delta_micros * 0.000001);

  //FAILSAFE
  if(abs(calc_data->world_data->roll_angle) > 70 || abs(calc_data->world_data->pitch_angle) > 70) {
    error_handler_id(4);
  }

  //neko funkcijo bi blo fajn met, ki bi vzela imu podatke, vhod daljinca in bi zracunala moci motorjev
  //sprintf(str, "heading: %.3f ", calc_data->world_data->heading_angle);
  //print(str);


  //sprintf(str, "dt: %lu, p: %.2f, r: %.2f", delta_micros, calc_data->world_data->pitch_angle, calc_data->world_data->roll_angle);
  //println(str);

  PIDProfile * pid_profiles[3]; // TODO tole bi blo fajn nekak popravit da nebo tak cudn
  pid_profiles[ROLL_INDEX]  = &roll_pid_profile;
  pid_profiles[PITCH_INDEX] = &pitch_pid_profile;
  pid_profiles[YAW_INDEX]   = &yaw_pid_profile;

  uint16_t * motor_powers = calculate_motor_powers(calc_data, &radio_data, pid_profiles, delta_micros * 0.001);

  //sprintf(str, "A: %d, B: %d, C: %d, D: %d", motor_powers[MOTOR_A_INDEX],motor_powers[MOTOR_B_INDEX],motor_powers[MOTOR_C_INDEX],motor_powers[MOTOR_D_INDEX]);
  //println(str);

  for(uint8_t i = 0; i < 4; i++) {
    motor_assign_power(i, motor_powers[i]);
  }
  motors_apply();


  /*SERIAL READER*/
  //char * cmd = serial_reader();
  //if(cmd != NULL) {
  //  println(cmd);
  //}
}
