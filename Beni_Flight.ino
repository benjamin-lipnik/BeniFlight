//cim bolj standardizirano podajanje podatkov
//interfaci za motorje, imu, radio,...
#include "comm.h"
#include "imu_interface.h"


static char str[100];

void setup() {
  pinMode(PC13, OUTPUT);
  print_init(NULL);
  imu_init(NULL);


  println((char *)"OK!");
}

void loop() {
  IMU_TypeDef * imu_data = imu_read();
  sprintf(str, "X: %.2f, Y: %.2f, Z: %.2f\n\r", imu_data->omega_dps[0], imu_data->omega_dps[1], imu_data->omega_dps[2]);
  print(str);

  delay(50);
}
