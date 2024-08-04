/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico-i2c-common.h"
#include "pico/binary_info.h"
#include "pico/stdlib.h"
#include "pico/time.h"
#include "pico/types.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define BUF_LEN 3

const uint LED_PIN = 25;

/* Example code to talk to a MPU6050 MEMS accelerometer and gyroscope

   This is taking to simple approach of simply reading registers. It's perfectly
   possible to link up an interrupt line and set things up to read from the
   inbuilt FIFO to make it more useful.

   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor I2C) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board, other boards may vary.

   GPIO PICO_DEFAULT_I2C_SDA_PIN (On Pico this is GP4 (pin 6)) -> SDA on MPU6050
   board GPIO PICO_DEFAULT_I2C_SCL_PIN (On Pico this is GP5 (pin 7)) -> SCL on
   MPU6050 board 3.3v (pin 36) -> VCC on MPU6050 board GND (pin 38)  -> GND on
   MPU6050 board
*/

// By default these devices  are on bus addr 0x68
// static int addr = 0x68;
static int addr = 0x69;

static void mpu6050_reset(i2c_inst_t *i2c) {
  int rval;
  printf("Resetting MPU6050\n");

  // Two byte reset. First byte register, second byte data
  // There are a load more options to set up the device in different ways that
  // could be added here
  uint8_t buf[] = {0x6B, 0x80};
  rval = i2c_write_blocking(i2c, addr, buf, 2, false);
  printf("rval: %d\n", rval);
  // from Hunter Adam's code:
  // Set gyro sample rate (set to 1KHz, same as accel)
  // uint8_t gyro_rate[] = {0x19, 0b00000111} ;
  // i2c_write_blocking(i2c, addr, gyro_rate, 2, false);

  // Configure the Gyro range (+/- 250 deg/s)
  // uint8_t gyro_settings[] = {0x1b, 0b00000000} ;
  // i2c_write_blocking(i2c, addr, gyro_settings, 2, false);

  // Configure the Accel range (+/- 2g's)
  // uint8_t accel_settings[] = {0x1c, 0b00000000} ;
  // i2c_write_blocking(i2c, addr, accel_settings, 2, false);

#if 0
    // Configure interrupt pin
    uint8_t pin_settings[] = {0x37, 0b00010000} ;
    i2c_write_blocking(i2c, addr, pin_settings, 2, false);

    // Configure data ready interrupt
    uint8_t int_config[] = {0x38, 0x01} ;
    i2c_write_blocking(i2c, addr, int_config, 2, false);
#endif
}

void mpu6050_read_raw(i2c_inst_t *i2c, int16_t accel[3], int16_t gyro[3],
                      int16_t *temp) {
  // For this particular device, we send the device the register we want to read
  // first, then subsequently read from the device. The register is auto
  // incrementing so we don't need to keep sending the register we want, just
  // the first.

  uint8_t buffer[6];
  int16_t temp_accel, temp_gyro;
  int rval;

  printf("Reading MPU6050\n");

  // Start reading acceleration registers from register 0x3B for 6 bytes
  uint8_t val = 0x3B;
  i2c_write_blocking(i2c, addr, &val, 1,
                     true); // true to keep master control of bus
  i2c_read_blocking(i2c, addr, buffer, 6, false);

  for (int i = 0; i < 3; i++) {
    temp_accel = (buffer[i << 1] << 8 | buffer[(i << 1) + 1]);
    accel[i] = temp_accel;
    accel[i] <<= 2; // convert to g's (fixed point)
  }

  // Now gyro data from reg 0x43 for 6 bytes
  // The register is auto incrementing on each read
  val = 0x43;
  i2c_write_blocking(i2c, addr, &val, 1, true);
  i2c_read_blocking(i2c, addr, buffer, 6, false); // False - finished with bus

  for (int i = 0; i < 3; i++) {
    temp_gyro = (buffer[i << 1] << 8 | buffer[(i << 1) + 1]);
    gyro[i] = temp_gyro;
    // gyro[i] = (gyro[i], 500<<16) ; // deg/sec
  }
  // Now temperature from reg 0x41 for 2 bytes
  // The register is auto incrementing on each read
  val = 0x41;
  i2c_write_blocking(i2c, addr, &val, 1, true);
  rval = i2c_read_blocking(i2c, addr, buffer, 2,
                           false); // False - finished with bus
  printf("rval: %d\n", rval);
  *temp = buffer[0] << 8 | buffer[1];

  gpio_put(LED_PIN, 1);
  sleep_ms(250);
  gpio_put(LED_PIN, 0);
}
static void mpu6050_read_raw_sdk(i2c_inst_t *i2c, int16_t accel[3],
                                 int16_t gyro[3], int16_t *temp) {
  // For this particular device, we send the device the register we want to read
  // first, then subsequently read from the device. The register is auto
  // incrementing so we don't need to keep sending the register we want, just
  // the first.

  uint8_t buffer[6];
  int rval;

  // Start reading acceleration registers from register 0x3B for 6 bytes
  uint8_t val = 0x3B;
  i2c_write_blocking(i2c, addr, &val, 1,
                     true); // true to keep master control of bus
  rval = i2c_read_blocking(i2c, addr, buffer, 6, false);
  printf("rval: %d\n", rval);

  printf("%x %x %x %x %x %x\n", buffer[0], buffer[1], buffer[2], buffer[3],
         buffer[4], buffer[5]);

  for (int i = 0; i < 3; i++) {
    accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    printf("%d", accel[i]);
  }
  printf("\n");

  // Now gyro data from reg 0x43 for 6 bytes
  // The register is auto incrementing on each read
  val = 0x43;
  i2c_write_blocking(i2c, addr, &val, 1, true);
  rval = i2c_read_blocking(i2c, addr, buffer, 6,
                           false); // False - finished with bus
  printf("rval: %d\n", rval);

  for (int i = 0; i < 3; i++) {
    gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    ;
  }

  // Now temperature from reg 0x41 for 2 bytes
  // The register is auto incrementing on each read
  val = 0x41;
  i2c_write_blocking(i2c, addr, &val, 1, true);
  rval = i2c_read_blocking(i2c, addr, buffer, 2,
                           false); // False - finished with bus
  printf("rval: %d\n", rval);
  *temp = buffer[0] << 8 | buffer[1];

  gpio_put(LED_PIN, 1);
  sleep_ms(250);
  gpio_put(LED_PIN, 0);
  // puts("Hello World\n");
}

#define NORMAL
#ifdef NORMAL
int main() {
  i2c_inst_t *i2c = i2c0;
  const uint sda_pin = 8;
  const uint scl_pin = 9;

  stdio_init_all(); // Pins

  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);

  sleep_ms(5000);
  printf("Hello, MPU6050! Reading raw data from registers using SCL %d and SDA "
         "%d\n",
         sda_pin, scl_pin);
  sleep_ms(2000);

#define RUN_COLLECTION
#ifdef RUN_COLLECTION
  i2c_init(i2c, 400 * 1000);
  gpio_set_function(sda_pin, GPIO_FUNC_I2C);
  gpio_set_function(scl_pin, GPIO_FUNC_I2C);
  gpio_pull_up(sda_pin);
  gpio_pull_up(scl_pin);
  // bi_decl(bi_2pins_with_func(I2C_AUDIO_SDA_PIN, I2C_AUDIO_SCL_PIN,
  // GPIO_FUNC_I2C));

  // mpu6050_reset(i2c);

  //    while (1) {
        scan_i2c(i2c);
  //        sleep_ms(5000);
  //    }

  int16_t acceleration[BUF_LEN], gyro[BUF_LEN], temp;

  while (1) {
    mpu6050_read_raw(i2c, acceleration, gyro, &temp);

    absolute_time_t timestamp = get_absolute_time();
    printf("Time: %lld\n", timestamp);

    // These are the raw numbers from the chip, so will need tweaking to be
    // really useful. See the datasheet for more information
    printf("Acc. X = %d, Y = %d, Z = %d\n", acceleration[0], acceleration[1],
           acceleration[2]);
    printf("Gyro. X = %d, Y = %d, Z = %d\n", gyro[0], gyro[1], gyro[2]);
    // Temperature is simple so use the datasheet calculation to get deg C.
    // Note this is chip temperature.
    printf("Temp. = %f\n", (temp / 340.0) + 36.53);
    printf("*******************************************************************"
           "*************************\n");

    memset(acceleration, 0, BUF_LEN);
    memset(gyro, 0, BUF_LEN);
    temp = 0;
    sleep_ms(1000);
  }
#endif
}
#endif
