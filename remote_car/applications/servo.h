/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-24     10426       the first version
 */
#ifndef APPLICATIONS_SERVO_H_
#define APPLICATIONS_SERVO_H_

#include "tim.h"

/** 舵机信号线接口
PG3     ------> SERVO1_PWM
PG4     ------> SERVO2_PWM
*/

/** 舵机周期 20(ms) **/
#define SERVO_PWM_TIME          (20000 / 10)

#define SERVO_F_HIGH            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET)
#define SERVO_F_LOW             HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET)

#define SERVO_B_HIGH            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET)
#define SERVO_B_LOW             HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_RESET)


void SERVO_Init(void);
void Servo_B_Control(float angle);
void Servo_F_Control(float angle);

#endif /* APPLICATIONS_SERVO_H_ */
