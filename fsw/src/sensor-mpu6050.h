/**
 * @file
 *
 * @brief Accelerometer Sensor MPU6050 Driver API
 *
 * @ingroup I2CSensorMPU6050
 */


#ifndef _DEV_I2C_SENSOR_MPU6050_H
#define _DEV_I2C_SENSOR_MPU6050_H

#include <dev/i2c/i2c.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// Device address when ADO = 0
#define MPU6050_ADDRESS 0x68

// Registers to read the accelerometer and gyroscope
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define GYRO_XOUT_H        0x43
#define GYRO_XOUT_L        0x44
#define GYRO_YOUT_H        0x45
#define GYRO_YOUT_L        0x46
#define GYRO_ZOUT_H        0x47
#define GYRO_ZOUT_L        0x48

// Control registers
#define WOM_EN	           0x06
#define GYRO_CONFIG        0x1B
#define ACCEL_CONFIG       0x1C
#define WOM_THR            0x1F
#define MOT_DUR            0x20
#define INT_ENABLE         0x38
#define SIGNAL_PATH_RESET  0x68
#define PWR_MGT_1          0x6B
#define PWR_MGT_2          0x6C

/**
 * @defgroup I2CSensorMPU6050 Accelerometer Sensor MPU6050 Driver
 *
 * @ingroup I2CDevice
 *
 * @brief Driver for MPU6050 accelerometer sensor.
 *
 * @{
 */
#define gyroscope_read
#define accelerometer_read

typedef enum {
  SENSOR_MPU6050_SET_CONF,
  SENSOR_MPU6050_SET_REG
} sensor_mpu6050_command;

typedef enum {
  X,
  Y,
  Z
} sensor_mpu6050_axis;

static int sensor_mpu6050_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg);

int i2c_dev_register_sensor_mpu6050(const char *bus_path, const char *dev_path);
int sensor_mpu6050_set_conf(int fd);
int sensor_mpu6050_set_register(int fd);


// I2C functions
static int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff);

static int sensor_mpu6050_set_reg_8(i2c_dev *dev, int ptr, uint8_t val);
static int sensor_mpu6050_get_reg_8(uint8_t register_add, uint8_t **buff);

#ifdef gyroscope_read
// Accelerometer functions
int sensor_mpu6050_get_gyro(int16_t **buff);
static int sensor_mpu6050_get_gyro_axis(uint8_t **buff, sensor_mpu6050_axis axis);

#endif

#ifdef accelerometer_read
// Accelerometer functions
int sensor_mpu6050_get_accel(int16_t **buff);
static int sensor_mpu6050_get_accel_axis(uint8_t **buff, sensor_mpu6050_axis axis);

#endif

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DEV_I2C_SENSOR_MPU6050_H */
