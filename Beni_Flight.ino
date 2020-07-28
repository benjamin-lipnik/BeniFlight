//cim bolj standardizirano podajanje podatkov
//interfaci za motorje, imu, radio,...
#include "comm.h"
#include "imu_interface.h"
#include "RF_RX.h"

static char str[100];
typedef struct {
  uint8_t power;

  uint8_t roll;
  uint8_t pitch;
  uint8_t yaw;

  uint8_t arm;
}Radio_pkg;

Radio_pkg rc_pkg;
uint8_t rx_address[] = { 1, 2, 3, 4, 0 };

void setup() {
  pinMode(PC13, OUTPUT);
  print_init(NULL);
  imu_init(NULL);

  preInit(NULL);
  if(!initRadio(rx_address, BITRATE2MBPS, 100)) {
    while(1) {
      print((char *)"Radio not working.\n\r");
      delay(100);
    }
  }

  println((char *)"OK!");
}

void loop() {

  readData(&rc_pkg, sizeof(rc_pkg));
  sprintf(str, "p: %u, r: %d, t: %u, y: %u\n\r", rc_pkg.power, rc_pkg.roll, rc_pkg.pitch, rc_pkg.yaw);
  print(str);

  /*
  IMU_TypeDef * imu_data = imu_read();
  sprintf(str, "X: %d, Y: %d, Z: %d\n\r", imu_data->mag_gauss[0], imu_data->mag_gauss[1], imu_data->mag_gauss[2]);
  print(str);
  */


  delay(50);
}
