/* The register map is provided at
 * https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
 *
 * Mapping of the different gyro and accelero configurations:
 *
 * GYRO_CONFIG_[0,1,2,3]  range = +- [250, 500,1000,2000] deg/s
 *                        sensi =    [131,65.5,32.8,16.4] bit/(deg/s)
 *
 * ACC_CONFIG_[0,1,2,3]   range = +- [    2,   4,   8,  16] g (9.81 m/s^2)
 *                        sensi =    [16384,8192,4096,2048] bit/gravity
 */

#pragma once
#include "hardware/i2c.h"
#include "hardware/timer.h"
#include "pico/stdlib.h"
#include <cmath>

const uint8_t MPU6050_ADDR = 0x68;
const uint8_t MPU6050_SMPLRT_DIV_REGISTER = 0x19;
const uint8_t MPU6050_CONFIG_REGISTER = 0x1a;
const uint8_t MPU6050_GYRO_CONFIG_REGISTER = 0x1b;
const uint8_t MPU6050_ACCEL_CONFIG_REGISTER = 0x1c;
const uint8_t MPU6050_PWR_MGMT_1_REGISTER = 0x6b;

const uint8_t MPU6050_GYRO_OUT_REGISTER = 0x43;
const uint8_t MPU6050_ACCEL_OUT_REGISTER = 0x3B;

const float RAD_2_DEG = 57.29578; // [deg/rad]
const float CALIB_OFFSET_NB_MES = 500;
const float TEMP_LSB_2_DEGREE = 340.0; // [bit/celsius]
const float TEMP_LSB_OFFSET = 12412.0;

const float DEFAULT_GYRO_COEFF = 0.98;

class MPU6050 {
public:
  // INIT and BASIC FUNCTIONS
  MPU6050(i2c_inst_t *i2c = i2c0, uint8_t addr = MPU6050_ADDR);
  uint8_t begin(int gyro_config_num = 1, int acc_config_num = 0);

  int writeData(uint8_t reg, uint8_t data);
  uint8_t readData(uint8_t reg);
  void readBuffer(uint8_t reg, uint8_t *buffer, uint len);

  void calcOffsets(bool is_calc_gyro = true, bool is_calc_acc = true);
  void calcGyroOffsets() {
    calcOffsets(true, false);
  }; // retro-compatibility with v1.0.0
  void calcAccOffsets() {
    calcOffsets(false, true);
  }; // retro-compatibility with v1.0.0

  void setAddress(uint8_t addr) { addr = addr; };
  uint8_t getAddress() { return addr; };

  // MPU CONFIG SETTER
  uint8_t setGyroConfig(int config_num);
  uint8_t setAccConfig(int config_num);

  void setGyroOffsets(float x, float y, float z);
  void setAccOffsets(float x, float y, float z);

  void setFilterGyroCoef(float gyro_coeff);
  void setFilterAccCoef(float acc_coeff);

  // MPU CONFIG GETTER
  float getGyroXoffset() { return gyroXoffset; };
  float getGyroYoffset() { return gyroYoffset; };
  float getGyroZoffset() { return gyroZoffset; };

  float getAccXoffset() { return accXoffset; };
  float getAccYoffset() { return accYoffset; };
  float getAccZoffset() { return accZoffset; };

  float getFilterGyroCoef() { return filterGyroCoef; };
  float getFilterAccCoef() { return 1.0 - filterGyroCoef; };

  // DATA GETTER
  float getTemp() { return temp; };

  float getAccX() { return accX; };
  float getAccY() { return accY; };
  float getAccZ() { return accZ; };

  float getGyroX() { return gyroX; };
  float getGyroY() { return gyroY; };
  float getGyroZ() { return gyroZ; };

  float getAccAngleX() { return angleAccX; };
  float getAccAngleY() { return angleAccY; };

  float getAngleX() { return angleX; };
  float getAngleY() { return angleY; };
  float getAngleZ() { return angleZ; };

  // INLOOP UPDATE
  void fetchData();
  void update();

  // UPSIDE DOWN MOUNTING
  bool upsideDownMounting = false;

private:
  i2c_inst_t *i2c = nullptr;
  uint8_t addr = MPU6050_ADDR; // 0x68 or 0x69
  float gyro_lsb_to_degsec, acc_lsb_to_g;
  float gyroXoffset, gyroYoffset, gyroZoffset;
  float accXoffset, accYoffset, accZoffset;
  float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
  float angleAccX, angleAccY;
  float angleX, angleY, angleZ;
  uint64_t Told;
  float filterGyroCoef; // complementary filter coefficient to balance gyro vs
                        // accelero data to get angle
};
