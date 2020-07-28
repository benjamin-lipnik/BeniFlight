//cim bolj standardizirano podajanje podatkov
//interfaci za motorje, imu, radio,...
#include "comm.h"
#include "imu_interface.h"
#include "radio_interface.h"
#include "motor_interface.h"

static char str[100];

void setup() {
  pinMode(PC13, OUTPUT);

  print_init(NULL);
  imu_init(NULL);
  radio_init(NULL);
  motors_init(NULL);

  println((char *)"OK!");
}

void loop() {

  /*TIMING*/
  static unsigned long loop_micros = 0;
  static unsigned long loop_millis = 0;
  static unsigned long delta_micros = 0;

  delta_micros = loop_micros;//last_micros
  loop_micros  = micros();
  delta_micros = loop_micros - delta_micros; //last - new micros
  loop_millis  = loop_micros * 0.001;

  if(delta_micros > 4000) { //too long
    //error
  }
  /*IMU*/

  //Preberemo podatke z IMU in jih preracunamo v uporabne vrednosti
  IMU_TypeDef * imu_data = imu_read();
  

  /*RADIO*/

  //Pollamo za podatke od dalinca in gledamo, ce nam je zanjkal signala
  //Gledamo tudi ce smo signal dobili nazaj.
  Radio_pkg * radio_data = radio_read();

  /*STABILIZACIJA*/

  //ce je vseeee ok pol stabiliziramo dron, in poslusamo dalinc
  //ce ni signala pol stabiliziramo dron in padamo proti tlem
  //ce je vse narobe ugasnemo motorje in vklopimo padalo ce ga mamo in ustavimo program

}
