#include "imu_interface.h"
#include "configuration.h"
#include <Arduino.h>
#include <Wire.h>

#define GYRO_ADDRESS (uint8_t)(0b1101011)
#define ACC_ADDRESS  (uint8_t)(0b00000000)
#define MAG_ADDRESS  (uint8_t)(0b00011110)

TwoWire I2C(2, I2C_FAST_MODE); // I2C2
static IMU_TypeDef imu_data;

static void write_reg(uint8_t address, uint8_t reg, uint8_t data) {
  I2C.beginTransmission(address);
  I2C.write(reg);
  I2C.write(data);
  I2C.endTransmission();
}
static uint8_t read_reg(uint8_t address, uint8_t reg) {
  I2C.beginTransmission(address);
  I2C.write(reg);
  I2C.endTransmission();
  I2C.requestFrom(address, 1);
  return I2C.read();
}

void imu_init(void * param) {

  I2C.begin();

  /*MAG INIT*/
  write_reg(MAG_ADDRESS, 0x00, 0b111 << 2);
  write_reg(MAG_ADDRESS, 0x01, 0b111 << 5);
  write_reg(MAG_ADDRESS, 0x02, 0);

  /*GYRO INIT*/

  uint8_t test = read_reg(GYRO_ADDRESS, 0x0f);
  while(1) {
    Serial.println(test, HEX);
    delay(100);
    test = read_reg(GYRO_ADDRESS, 0x0f);
    //sprintf(str, "test: %02x\n\r", test);
    //print(str);
    //delay(100);
  }

}
IMU_TypeDef * imu_read() {

  int16_t mag_buffer[3];
  for(int i = 0; i < 3; i++) {
    mag_buffer[i] = read_reg(MAG_ADDRESS, 0x03 + 2*i) << 8 | read_reg(MAG_ADDRESS, 0x03 + 2*i + 1);
    imu_data.mag_gauss[i] = (float)mag_buffer[i];
  }

  int16_t gyro_buffer[3];
  for(int i = 0; i < 3; i++) {
    gyro_buffer[i] = read_reg(GYRO_ADDRESS, 0x03 + 2*i) << 8 | read_reg(GYRO_ADDRESS, 0x03 + 2*i + 1);
    imu_data.omega_dps[i] = (float)gyro_buffer[i];
  }


  return &imu_data;
}
