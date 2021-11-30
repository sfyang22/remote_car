/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-12     10426       the first version
 */
#ifndef APPLICATIONS_DHT11_H_
#define APPLICATIONS_DHT11_H_

#include "main.h"

/** DHT11信号线接口
PG2     ------> DHT11_DATA
*/

//设定标志(static unsigned char status)的宏值
#define OK      1
#define ERROR   0

#define NUMBER  1000

#define SIZE    5

#define DHT11_GPIO_CLK      __HAL_RCC_GPIOG_CLK_ENABLE
#define DHT11_GPIO_PORT     GPIOG
#define DHT11_GPIO_PIN      GPIO_PIN_2

#define DHT11_GPIO_HIGH     HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_SET)
#define DHT11_GPIO_LOW      HAL_GPIO_WritePin(DHT11_GPIO_PORT, DHT11_GPIO_PIN, GPIO_PIN_RESET)

#define DHT11_GPIO_READ     HAL_GPIO_ReadPin(DHT11_GPIO_PORT, DHT11_GPIO_PIN)

extern int temp_value, humi_value;

void DHT11_Init(void);
uint8_t DHT11_ReadTempAndHumi(void);


#endif /* APPLICATIONS_DHT11_H_ */
