#include "imu_interface.h"
#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ADDRESS (uint8_t)(0x68)
#define LSM303_MAG_ADDRESS  (uint8_t)(0b00011110)

#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_PWR_MGMT_1   0x6B

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

uint8_t imu_init(void * param) {

  I2C.begin();
  delay(10);

  /*LSM303 MAG INIT*/
  write_reg(LSM303_MAG_ADDRESS, 0x00, 0b111 << 2);
  write_reg(LSM303_MAG_ADDRESS, 0x01, 0b111 << 5);
  write_reg(LSM303_MAG_ADDRESS, 0x02, 0);

  /*MPU GYRO & ACC INIT*/

  write_reg(MPU6050_ADDRESS, MPU_REG_PWR_MGMT_1,   0x00);
  write_reg(MPU6050_ADDRESS, MPU_REG_GYRO_CONFIG,  0x08); //FS_SEL
  write_reg(MPU6050_ADDRESS, MPU_REG_ACCEL_CONFIG, 0x10); //AFS_SEL
  write_reg(MPU6050_ADDRESS, MPU_REG_CONFIG,       0x03); //LOW PASS FILTER (DLPF_CFG)

  uint8_t check = read_reg(MPU6050_ADDRESS, MPU_REG_GYRO_CONFIG) == 0x08;
  check &= read_reg(MPU6050_ADDRESS, MPU_REG_ACCEL_CONFIG) == 0x10;
  check &= read_reg(LSM303_MAG_ADDRESS, 0x01) == (0b111 << 5);

  if(check)
    return INIT_OK;
  return INIT_ERROR;
}
IMU_TypeDef * imu_read() { // fajn bi blo continuous branje podatkov ampak se mi zdej pac ne ljubi, tak da TODO :)

  int16_t value = 0;
  for(int i = 0; i < 3; i++) {
    //Mag
    value = 0;
    value = read_reg(LSM303_MAG_ADDRESS, 0x03 + 2*i) << 8 | read_reg(LSM303_MAG_ADDRESS, 0x03 + 2*i + 1);
    imu_data.mag_gauss[i] = (float)value; // tu je se treba normalizirat vrednosti, ko bo cajt

    //acc
    value = 0;
    value = read_reg(MPU6050_ADDRESS, 0x3B + 2*i) << 8 | read_reg(MPU6050_ADDRESS, 0x3B + 2*i + 1);
    imu_data.acc_g[i] = (float)value / 4096.0f;

    //gyro
    value = 0;
    value = read_reg(MPU6050_ADDRESS, 0x43 + 2*i) << 8 | read_reg(MPU6050_ADDRESS, 0x43 + 2*i + 1);
    imu_data.omega_dps[i] = (float)value / 65.5f;
  }

  return &imu_data;
}
