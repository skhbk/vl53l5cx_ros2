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

#ifndef VL53L5CX_TYPES_H_
#define VL53L5CX_TYPES_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#ifndef NULL
#define NULL 0
#endif

#if !defined(STDINT_H) && !defined(_STDINT_H) && !defined(__STDINT_H) && \
  !defined(_GCC_STDINT_H) && !defined(__STDINT_DECLS) && \
  !defined(_GCC_WRAP_STDINT_H)  && !defined(_STDINT) && \
  !defined(_LINUX_TYPES_H)

#pragma message("Please STDINT type definitions and define for your platform")

typedef unsigned long long uint64_t;
typedef unsigned int uint32_t;
typedef int int32_t;
typedef unsigned short uint16_t;
typedef short int16_t;
typedef unsigned char uint8_t;
typedef signed char int8_t;

#endif

#ifdef __cplusplus
}
#endif

#endif /* VL53L5CX_TYPES_H_ */
