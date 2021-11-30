/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-20     10426       the first version
 */
#ifndef APPLICATIONS_LED_H_
#define APPLICATIONS_LED_H_

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"


#define LED_CLK_ENABLE             __GPIOF_CLK_ENABLE
#define LED_PORT                   GPIOF
#define LED_PIN                    GPIO_PIN_9

#define CAR_LED_CLK_ENABLE             __GPIOC_CLK_ENABLE
#define CAR_LED_PORT                   GPIOC
#define CAR_LED_PIN                    GPIO_PIN_4

#define LED_ON()                   HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET)
#define LED_OFF()                  HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET)

#define CAR_LED_ON()                   HAL_GPIO_WritePin(CAR_LED_PORT, CAR_LED_PIN, GPIO_PIN_SET)
#define CAR_LED_OFF()                  HAL_GPIO_WritePin(CAR_LED_PORT, CAR_LED_PIN, GPIO_PIN_RESET)


void LED_Init(void);
void CAR_LED_Init(void);

#endif /* APPLICATIONS_LED_H_ */
