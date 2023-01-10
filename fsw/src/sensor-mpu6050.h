/**
 * @file
 *
 * @brief IMU Sensor MPU6050 Driver API
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
#include <math.h>


#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

// Device address when ADO = 0
#define MPU6050_ADDRESS 0x68

// Registers to read the accelerometer, gyroscope and temperature
#define ACCEL_XOUT_H       0x3B
#define ACCEL_XOUT_L       0x3C
#define ACCEL_YOUT_H       0x3D
#define ACCEL_YOUT_L       0x3E
#define ACCEL_ZOUT_H       0x3F
#define ACCEL_ZOUT_L       0x40
#define TEMP_OUT_H         0x41
#define TEMP_OUT_L         0x42
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
#define MPU6050_WHOAMI     0x75

// Definitions used for angle calculation
#define RAD_2_DEG             180.0/3.14159 // [deg/rad]
#define CALIB_OFFSET_NB_MES   500

#define DEFAULT_GYRO_COEFF    0.98

/**
 * @defgroup I2CSensorMPU6050 IMU Sensor MPU6050 Driver
 *
 * @ingroup I2CDevice
 *
 * @brief Driver for MPU6050 IMU sensor.
 *
 * @{
 */

#define runtime_ms()    rtems_clock_get_uptime_nanoseconds()/1000000

typedef enum {
  SENSOR_MPU6050_BEGIN,
  SENSOR_MPU6050_SET_REG
} sensor_mpu6050_command;

typedef enum {
  X,
  Y,
  Z
} sensor_mpu6050_axis;

typedef struct {
  float gyroXoffset, gyroYoffset, gyroZoffset;
  float accXoffset, accYoffset, accZoffset;
  float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
  float angleAccX, angleAccY;
  float angleX, angleY, angleZ;
  unsigned long preInterval;
  float filterGyroCoef; // complementary filter coefficient to balance gyro vs accelero data to get angle
}SENSOR_MPU6050_Priv_Data_t;

typedef struct {
  float temp, accX, accY, accZ, gyroX, gyroY, gyroZ;
  float angleX, angleY, angleZ;
  float angleAccX, angleAccY;
}SENSOR_MPU6050_Data_t;

int i2c_dev_register_sensor_mpu6050(const char *bus_path, const char *dev_path);
int sensor_mpu6050_begin(int fd);
int sensor_mpu6050_set_register(int fd);

// Data functions
SENSOR_MPU6050_Data_t sensor_mpu6050_get_data(void);
void sensor_mpu6050_read_data(void);
void sensor_mpu6050_calcOffsets(void);
float sensor_mpu6050_get_temp(void);

/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DEV_I2C_SENSOR_MPU6050_H */
