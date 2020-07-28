//cim bolj standardizirano podajanje podatkov
//interfaci za motorje, imu, radio,...
#include "configuration.h"
#include "comm.h"
#include "imu_interface.h"
#include "radio_interface.h"
#include "motor_interface.h"
#include "failsafe.h"
#include "stabilization_program.h"

static char str[100]; //buffer za izpisovanje
static unsigned long loop_micros = 0;
static unsigned long loop_millis = 0;
static unsigned long delta_micros = 0;

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

  loop_micros = micros();

  digitalWrite(BLUE_LED_PIN, LOW);
  digitalWrite(RED_LED_PIN,  HIGH);
  println((char *)"OK!");
}

void loop() {

  /*TIMING*/
  delta_micros = loop_micros;//last_micros
  loop_micros  = micros();
  delta_micros = loop_micros - delta_micros; //last - new micros
  loop_millis  = loop_micros * 0.001;

  if(delta_micros > 4000) { //too long
    //error_handler_id(1);
    //error
  }
  /*IMU*/

  //Preberemo podatke z IMU in jih preracunamo v uporabne vrednosti
  IMU_TypeDef * imu_data = imu_read();

  //sprintf(str, "dt: %lu, ACC X: %.2f, Y: %.2f, Z:%.2f\n\r", delta_micros, imu_data->omega_dps[X_INDEX], imu_data->omega_dps[Y_INDEX], imu_data->omega_dps[Z_INDEX]);
  //print(str);

  /*RADIO*/

  //Pollamo za podatke od dalinca in gledamo, ce nam je zanjkal signala
  //Gledamo tudi ce smo signal dobili nazaj.
  Radio_pkg * radio_data = radio_read();


  /*STABILIZACIJA*/

  //ce je vseeee ok pol stabiliziramo dron, in poslusamo dalinc
  //ce ni signala pol stabiliziramo dron in padamo proti tlem
  //ce je vse narobe ugasnemo motorje in vklopimo padalo ce ga mamo in ustavimo program

  Calculated_IMU_Data * calc_data = calulate_imu_data(imu_data, delta_micros * 0.000001);
  sprintf(str, "dt: %lu, p: %.2f, r: %.2f\n\r", delta_micros, calc_data->world_data->pitch_angle, calc_data->world_data->roll_angle);
  print(str);


  /*SERIAL READER*/
  char * cmd = serial_reader();
  if(cmd != NULL) {
    println(cmd);
  }
}
