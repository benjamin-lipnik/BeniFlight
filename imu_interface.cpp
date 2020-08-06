#include "imu_interface.h"
#include <Arduino.h>
#include <Wire.h>
#include "comm.h"

#define MPU6050_ADDRESS     (uint8_t)(0x68)
#define LSM303_MAG_ADDRESS  (uint8_t)(0b00011110)

#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_PWR_MGMT_1   0x6B

TwoWire I2C(2, I2C_FAST_MODE); // I2C2
static IMU_TypeDef imu_data;



//callibration
static const int16_t gyro_callib[3] = {247, 23, 18};
static const int16_t acc_callib [3] = {-203, 30, -113};

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
    //imu_data.mag_gauss[mag_axes[i]] = (float)value / 230.0f;

    //FILTRIRANJE
    static int16_t gauss_filter[3];
    gauss_filter[i] = gauss_filter[i] * 0.7f + value * 0.3f;
    imu_data.mag_gauss[mag_axes[i]] = (float)gauss_filter[i] / 230.0f;

    //acc
    value = 0;
    value = read_reg(MPU6050_ADDRESS, 0x3B + 2*i) << 8 | read_reg(MPU6050_ADDRESS, 0x3B + 2*i + 1);
    value += acc_callib[i];
    imu_data.acc_g[acc_axes[i]] = (float)value / 4096.0f;

    //////CALLIBRATION
    //static char str[100];
    //sprintf(str, "%d,", value);
    //print(str);

    //gyro
    value = 0;
    value = read_reg(MPU6050_ADDRESS, 0x43 + 2*i) << 8 | read_reg(MPU6050_ADDRESS, 0x43 + 2*i + 1);
    value += gyro_callib[i];
    imu_data.omega_dps[gyro_axes[i]] = (float)value / 65.5f;
  }
  //print((char *)"\n");
  //println(NULL);

  /*GYRO INVERTERS*/

#ifdef GYRO_INVERT_X
  imu_data.omega_dps[X_INDEX] *= -1;
#endif

#ifdef GYRO_INVERT_Y
  imu_data.omega_dps[Y_INDEX] *= -1;
#endif

#ifdef GYRO_INVERT_Z
  imu_data.omega_dps[Z_INDEX] *= -1;
#endif

  /*ACC INVERTERS*/

#ifdef ACC_INVERT_X
  imu_data.acc_g[X_INDEX] *= -1;
#endif

#ifdef ACC_INVERT_Y
  imu_data.acc_g[Y_INDEX] *= -1;
#endif

#ifdef ACC_INVERT_Z
  imu_data.acc_g[Z_INDEX] *= -1;
#endif

/*MAG INVERTERS*/

#ifdef MAG_INVERT_X
  imu_data.mag_gauss[X_INDEX] *= -1;
#endif

#ifdef MAG_INVERT_Y
  imu_data.mag_gauss[Y_INDEX] *= -1;
#endif

#ifdef MAG_INVERT_Z
  imu_data.mag_gauss[Z_INDEX] *= -1;
#endif

  return &imu_data;
}
