/**
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef PLATFORM_H_
#define PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>

/**
 * @brief Structure VL53L5CX_Platform needs to be filled by the customer,
 * depending on his platform. At least, it contains the VL53L5CX I2C address.
 * Some additional fields can be added, as descriptors, or platform
 * dependencies. Anything added into this structure is visible into the platform
 * layer.
 */

typedef struct
{
  /* To be filled with customer's platform. At least an I2C address/descriptor
   * needs to be added */
  /* Example for most standard platform : I2C address of sensor */
  uint16_t address;

  /* For Linux implementation, file descriptor */
  int fd;

  const char * i2c_device;

} VL53L5CX_Platform;

/*
 * @brief The macro below is used to define the number of target per zone sent
 * through I2C. This value can be changed by user, in order to tune I2C
 * transaction, and also the total memory size (a lower number of target per
 * zone means a lower RAM). The value must be between 1 and 4.
 */

#define VL53L5CX_NB_TARGET_PER_ZONE 1U

/*
 * @brief The macro below can be used to avoid data conversion into the driver.
 * By default there is a conversion between firmware and user data. Using this macro
 * allows to use the firmware format instead of user format. The firmware format allows
 * an increased precision.
 */

#define VL53L5CX_USE_RAW_FORMAT

/*
 * @brief All macro below are used to configure the sensor output. User can
 * define some macros if he wants to disable selected output, in order to reduce
 * I2C access.
 */

#define VL53L5CX_DISABLE_AMBIENT_PER_SPAD
#define VL53L5CX_DISABLE_NB_SPADS_ENABLED
#define VL53L5CX_DISABLE_NB_TARGET_DETECTED
#define VL53L5CX_DISABLE_SIGNAL_PER_SPAD
#define VL53L5CX_DISABLE_RANGE_SIGMA_MM
#define VL53L5CX_DISABLE_DISTANCE_MM
#define VL53L5CX_DISABLE_REFLECTANCE_PERCENT
#define VL53L5CX_DISABLE_TARGET_STATUS
#define VL53L5CX_DISABLE_MOTION_INDICATOR

// Undefine DISABLE macros above if ENABLE macro is defined
#include "platform_enables.h"

/**
 * @brief Mandatory function used to read one single byte.
 * @param (VL53L5CX_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint16_t) Address : I2C location of value to read.
 * @param (uint8_t) *p_values : Pointer of value to read.
 * @return (uint8_t) status : 0 if OK
 */

uint8_t RdByte(VL53L5CX_Platform * p_platform, uint16_t reg_address, uint8_t * p_value);

/**
 * @brief Mandatory function used to write one single byte.
 * @param (VL53L5CX_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint16_t) reg_address : I2C location of value to read.
 * @param (uint8_t) value : Pointer of value to write.
 * @return (uint8_t) status : 0 if OK
 */

uint8_t WrByte(VL53L5CX_Platform * p_platform, uint16_t reg_address, uint8_t value);

/**
 * @param (VL53L5CX_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint16_t) reg_address : I2C location of values to read.
 * @param (uint8_t) *p_values : Buffer of bytes to read.
 * @param (uint32_t) size : Size of *p_values buffer.
 * @return (uint8_t) status : 0 if OK
 */

uint8_t RdMulti(
  VL53L5CX_Platform * p_platform, uint16_t reg_address, uint8_t * p_values, uint32_t size);

/**
 * @brief Mandatory function used to write multiples bytes.
 * @param (VL53L5CX_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint16_t) reg_address : I2C location of values to write.
 * @param (uint8_t) *p_values : Buffer of bytes to write.
 * @param (uint32_t) size : Size of *p_values buffer.
 * @return (uint8_t) status : 0 if OK
 */

uint8_t WrMulti(
  VL53L5CX_Platform * p_platform, uint16_t reg_address, uint8_t * p_values, uint32_t size);

/**
 * @brief Optional function, only used to perform an hardware reset of the
 * sensor. This function is not used in the API, but it can be used by the host.
 * This function is not mandatory to fill if user don't want to reset the
 * sensor.
 * @param (VL53L5CX_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @return (uint8_t) status : 0 if OK
 */

uint8_t Reset_Sensor(VL53L5CX_Platform * p_platform);

/**
 * @brief Mandatory function, used to swap a buffer. The buffer size is always a
 * multiple of 4 (4, 8, 12, 16, ...).
 * @param (uint8_t*) buffer : Buffer to swap, generally uint32_t
 * @param (uint16_t) size : Buffer size to swap
 */

void SwapBuffer(uint8_t * buffer, uint16_t size);
/**
 * @brief Mandatory function, used to wait during an amount of time. It must be
 * filled as it's used into the API.
 * @param (VL53L5CX_Platform*) p_platform : Pointer of VL53L5CX platform
 * structure.
 * @param (uint32_t) TimeMs : Time to wait in ms.
 * @return (uint8_t) status : 0 if wait is finished.
 */

uint8_t WaitMs(VL53L5CX_Platform * p_platform, uint32_t TimeMs);

/**
 * @brief I2C communication channel initialization
 * @param (int) *fd : pointer on a I2C channel descriptor.
 * @return (uint8_t) status : 0 if OK
 */
int32_t vl53l5cx_comms_init(VL53L5CX_Platform * p_platform);

/**
 * @brief I2C communication channel deletion
 * @param (int) fd : I2C channel descriptor.
 * @return (uint8_t) status : 0 if OK
 */
int32_t vl53l5cx_comms_close(VL53L5CX_Platform * p_platform);

#endif  // _PLATFORM_H_
