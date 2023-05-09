/* MPU6050_light library for Raspberry Pi Pico
 *
 * Authors: Lyubomyr Kryshtanovskyi (github.com/lyubomyrk) 2023
 *              ported to pico using C++ SDK
 *          Romain JL. Fétick (github.com/rfetick)
 *              simplifications and corrections
 *          Tockn (github.com/tockn)
 *              initial author (v1.5.2)
 */

#include "MPU6050_light.hpp"

/* Wrap an angle in the range [-limit,+limit] (special thanks to Edgar Bonet!)
 */
float wrap(float angle, float limit) {
  angle = fmodf(angle, 2 * limit);
  if (angle < 0) {
    angle += 2 * limit;
  }
  return angle - limit;
}

/* INIT and BASIC FUNCTIONS */

MPU6050::MPU6050(i2c_inst_t *i2c_instance, uint8_t addr) {
  i2c = i2c_instance;
  addr = addr;
  setFilterGyroCoef(DEFAULT_GYRO_COEFF);
  setGyroOffsets(0, 0, 0);
  setAccOffsets(0, 0, 0);
}

uint8_t MPU6050::begin(int gyro_config_num, int acc_config_num) {
  // changed calling register sequence
  // [https://github.com/rfetick/MPU6050_light/issues/1] -> thanks to augustosc
  uint8_t status =
      writeData(MPU6050_PWR_MGMT_1_REGISTER,
                0x01); // check only the first connection with status
  writeData(MPU6050_SMPLRT_DIV_REGISTER, 0x00);
  writeData(MPU6050_CONFIG_REGISTER, 0x00);
  setGyroConfig(gyro_config_num);
  setAccConfig(acc_config_num);

  this->update();
  /* angleX = this->getAccAngleX();
  angleY = this->getAccAngleY(); */
  angleX = 0;
  angleY = 0;
  angleZ = 0;
  Told = time_us_64();
  return status;
}

int MPU6050::writeData(uint8_t reg, uint8_t data) {
  uint8_t buf[] = {reg, data};
  int status = i2c_write_blocking(i2c, addr, buf, 2, false);
  return status; // 0 if success
}

// This method is not used internaly, maybe by user...
uint8_t MPU6050::readData(uint8_t reg) {
  uint8_t data;
  i2c_write_blocking(i2c, addr, &reg, 1, true);
  i2c_read_blocking(i2c, addr, &data, 1, false);
  return data;
}

void MPU6050::readBuffer(uint8_t reg, uint8_t *buffer, uint len) {
  i2c_write_blocking(i2c, addr, &reg, 1, true);
  i2c_read_blocking(i2c, addr, buffer, len, false);
}

/* SETTER */

uint8_t MPU6050::setGyroConfig(int config_num) {
  uint8_t status;
  switch (config_num) {
  case 0: // range = +- 250 deg/s
    gyro_lsb_to_degsec = 131.0;
    status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x00);
    break;
  case 1: // range = +- 500 deg/s
    gyro_lsb_to_degsec = 65.5;
    status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x08);
    break;
  case 2: // range = +- 1000 deg/s
    gyro_lsb_to_degsec = 32.8;
    status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x10);
    break;
  case 3: // range = +- 2000 deg/s
    gyro_lsb_to_degsec = 16.4;
    status = writeData(MPU6050_GYRO_CONFIG_REGISTER, 0x18);
    break;
  default: // error
    status = 1;
    break;
  }
  return status;
}

uint8_t MPU6050::setAccConfig(int config_num) {
  uint8_t status;
  switch (config_num) {
  case 0: // range = +- 2 g
    acc_lsb_to_g = 16384.0;
    status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x00);
    break;
  case 1: // range = +- 4 g
    acc_lsb_to_g = 8192.0;
    status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x08);
    break;
  case 2: // range = +- 8 g
    acc_lsb_to_g = 4096.0;
    status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x10);
    break;
  case 3: // range = +- 16 g
    acc_lsb_to_g = 2048.0;
    status = writeData(MPU6050_ACCEL_CONFIG_REGISTER, 0x18);
    break;
  default: // error
    status = 1;
    break;
  }
  return status;
}

void MPU6050::setGyroOffsets(float x, float y, float z) {
  gyroXoffset = x;
  gyroYoffset = y;
  gyroZoffset = z;
}

void MPU6050::setAccOffsets(float x, float y, float z) {
  accXoffset = x;
  accYoffset = y;
  accZoffset = z;
}

void MPU6050::setFilterGyroCoef(float gyro_coeff) {
  if ((gyro_coeff < 0) or (gyro_coeff > 1)) {
    gyro_coeff = DEFAULT_GYRO_COEFF;
  } // prevent bad gyro coeff, should throw an error...
  filterGyroCoef = gyro_coeff;
}

void MPU6050::setFilterAccCoef(float acc_coeff) {
  setFilterGyroCoef(1.0 - acc_coeff);
}

/* CALC OFFSET */

void MPU6050::calcOffsets(bool is_calc_gyro, bool is_calc_acc) {
  if (is_calc_gyro) {
    setGyroOffsets(0, 0, 0);
  }
  if (is_calc_acc) {
    setAccOffsets(0, 0, 0);
  }
  float ag[6] = {0, 0, 0, 0, 0, 0}; // 3*acc, 3*gyro

  for (int i = 0; i < CALIB_OFFSET_NB_MES; i++) {
    this->fetchData();
    ag[0] += accX;
    ag[1] += accY;
    ag[2] += (accZ - 1.0);
    ag[3] += gyroX;
    ag[4] += gyroY;
    ag[5] += gyroZ;
    sleep_ms(1); // wait a little bit between 2 measurements
  }

  if (is_calc_acc) {
    accXoffset = ag[0] / CALIB_OFFSET_NB_MES;
    accYoffset = ag[1] / CALIB_OFFSET_NB_MES;
    accZoffset = ag[2] / CALIB_OFFSET_NB_MES;
  }

  if (is_calc_gyro) {
    gyroXoffset = ag[3] / CALIB_OFFSET_NB_MES;
    gyroYoffset = ag[4] / CALIB_OFFSET_NB_MES;
    gyroZoffset = ag[5] / CALIB_OFFSET_NB_MES;
  }
}

/* UPDATE */

void MPU6050::fetchData() {
  uint8_t rawData8[14];
  readBuffer(MPU6050_ACCEL_OUT_REGISTER, rawData8, 14);

  int16_t rawData[7]; // [ax,ay,az,temp,gx,gy,gz]
  for (int i = 0; i < 7; i++) {
    rawData[i] = rawData8[2 * i] << 8;
    rawData[i] |= rawData8[2 * i + 1];
  }

  accX = ((float)rawData[0]) / acc_lsb_to_g - accXoffset;
  accY = ((float)rawData[1]) / acc_lsb_to_g - accYoffset;
  accZ = (!upsideDownMounting - upsideDownMounting) * ((float)rawData[2]) /
             acc_lsb_to_g -
         accZoffset;
  temp = (rawData[3] + TEMP_LSB_OFFSET) / TEMP_LSB_2_DEGREE;
  gyroX = ((float)rawData[4]) / gyro_lsb_to_degsec - gyroXoffset;
  gyroY = ((float)rawData[5]) / gyro_lsb_to_degsec - gyroYoffset;
  gyroZ = ((float)rawData[6]) / gyro_lsb_to_degsec - gyroZoffset;
}

void MPU6050::update() {
  // retrieve raw data
  this->fetchData();

  // estimate tilt angles: this is an approximation for small angles!
  float sgZ =
      accZ < 0 ? -1 : 1; // allow one angle to go from -180 to +180 degrees
  angleAccX = atan2(accY, sgZ * sqrt(accZ * accZ + accX * accX)) *
              RAD_2_DEG; // [-180,+180] deg
  angleAccY = -atan2(accX, sqrt(accZ * accZ + accY * accY)) *
              RAD_2_DEG; // [- 90,+ 90] deg

  // Correctly wrap X and Y angles (special thanks to Edgar Bonet!)
  // https://github.com/gabriel-milan/TinyMPU6050/issues/6
  uint64_t Tnew = time_us_64();
  float dt = (Tnew - Told) / 1'000'000.0;
  Told = Tnew;
  angleX = wrap(filterGyroCoef * (angleAccX +
                                  wrap(angleX + gyroX * dt - angleAccX, 180)) +
                    (1.0 - filterGyroCoef) * angleAccX,
                180);
  angleY =
      wrap(filterGyroCoef * (angleAccY +
                             wrap(angleY + sgZ * gyroY * dt - angleAccY, 90)) +
               (1.0 - filterGyroCoef) * angleAccY,
           90);
  angleZ += gyroZ * dt; // not wrapped
}
