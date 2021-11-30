/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-19     RealThread   first version
 */

#ifndef __BOARD_H__
#define __BOARD_H__

/* USER INCLUDE */
#include <stm32f4xx.h>
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal.h"
#include <drv_common.h>
#include "led.h"
#include "dcmi.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "motor.h"
#include "servo.h"
#include "encoder.h"
#include "ov2640.h"
#include "delay.h"
#include "usart.h"
#include "brd_cfg.h"
#include "M8266HostIf.h"
#include "M8266WIFI_ops.h"
#include "M8266WIFIDrv.h"
#include "usr_cfg.h"
#include "pid.h"
#include <gy52z.h>
#include "dht11.h"
#include "mq_sensor.h"
#include "wifi.h"
/* USER INCLUDE */

#ifdef __cplusplus
extern "C"
{
#endif

/*-------------------------- CHIP CONFIG BEGIN --------------------------*/

#define CHIP_FAMILY_STM32
#define CHIP_SERIES_STM32F4
#define CHIP_NAME_STM32F407ZE

/*-------------------------- CHIP CONFIG END --------------------------*/

/*-------------------------- ROM/RAM CONFIG BEGIN --------------------------*/

#define ROM_START              ((uint32_t)0x08000000)
#define ROM_SIZE               (512 * 1024)
#define ROM_END                ((uint32_t)(ROM_START + ROM_SIZE))

#define RAM_START              (0x20000000)
#define RAM_SIZE               (128 * 1024)
#define RAM_END                (RAM_START + RAM_SIZE)

/*-------------------------- ROM/RAM CONFIG END --------------------------*/

/*-------------------------- CLOCK CONFIG BEGIN --------------------------*/

#define BSP_CLOCK_SOURCE                  ("HSI")
#define BSP_CLOCK_SOURCE_FREQ_MHZ         ((int32_t)0)
#define BSP_CLOCK_SYSTEM_FREQ_MHZ         ((int32_t)168)

/*-------------------------- CLOCK CONFIG END --------------------------*/

/*-------------------------- UART CONFIG BEGIN --------------------------*/

/** After configuring corresponding UART or UART DMA, you can use it.
 *
 * STEP 1, define macro define related to the serial port opening based on the serial port number
 *                 such as     #define BSP_USING_UART1
 *
 * STEP 2, according to the corresponding pin of serial port, define the related serial port information macro
 *                 such as     #define BSP_UART1_TX_PIN       "PA9"
 *                             #define BSP_UART1_RX_PIN       "PA10"
 *
 */

#define BSP_USING_UART1
#define BSP_UART1_TX_PIN       "PA9"
#define BSP_UART1_RX_PIN       "PA10"

/*-------------------------- UART CONFIG END --------------------------*/

#ifdef __cplusplus
}
#endif

#endif /* __BOARD_H__ */
