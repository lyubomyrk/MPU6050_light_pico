#include "MPU6050_light.hpp"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdio.h>

const uint I2C_SDA = 16;
const uint I2C_SCL = 17;
const uint baud = 400'000; // 400kHz

int main() {
  stdio_init_all();

  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(I2C_SDA);
  gpio_pull_up(I2C_SCL);

  i2c_init(i2c0, baud);
  MPU6050 imu;
  imu.begin();
  imu.calcOffsets();

  while (true) {
    imu.update();
    printf("Gyro: %f %f %f\n", imu.getAngleX(), imu.getAngleY(),
           imu.getAngleZ());
  }

  return 0;
}
