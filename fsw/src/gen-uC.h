/**
 * @file
 *
 * @brief Generic uC Driver API
 *
 * @ingroup I2CMicroController
 */

#ifndef _DEV_I2C_uC_H
#define _DEV_I2C_uC_H

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

// Device address
#define UC_ADDRESS 0x36

/**
 * @defgroup I2CMicroController Driver
 *
 * @ingroup I2CDevice
 *
 * @brief Driver for a generic uC.
 *
 * @{
 */

typedef enum {
  UC_SEND_TEST
} uC_command;

int i2c_dev_register_uC(const char *bus_path, const char *dev_path);
int uC_send_test(int fd);


// I2C functions

#ifdef uC_reading

int uC_get_bytes(uint16_t chip_address, uint8_t register_add, uint8_t **buff);

#endif

int uC_set_bytes(uint16_t chip_address, uint8_t **val, int numBytes);


/** @} */

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* _DEV_I2C_uC_H */
