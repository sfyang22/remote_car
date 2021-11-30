/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-24     10426       the first version
 */
#ifndef DRIVERS_MOTOR_H_
#define DRIVERS_MOTOR_H_

#include "tim.h"

/** 电机信号线接口
PA0     ------> MOTOR_LF_PWM
PA2     ------> MOTOR_RF_PWM
PA1     ------> MOTOR_LB_PWM
PA3     ------> MOTOR_RB_PWM
*/

/** 电机控制线接口
PF0     ------> MOTOR_LF1
PF1     ------> MOTOR_LF2

PF2     ------> MOTOR_RF1
PF3     ------> MOTOR_RF2

PF4     ------> MOTOR_LB1
PF5     ------> MOTOR_LB2

PF6     ------> MOTOR_RB1
PF7     ------> MOTOR_RB2
*/

//控制线输出高电平
#define MOTOR_LF1_HIGH                      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET)
#define MOTOR_LF2_HIGH                      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET)

#define MOTOR_RF1_HIGH                      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET)
#define MOTOR_RF2_HIGH                      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET)

#define MOTOR_LB1_HIGH                      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_SET)
#define MOTOR_LB2_HIGH                      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET)

#define MOTOR_RB1_HIGH                      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)
#define MOTOR_RB2_HIGH                      HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_SET)
//控制线输出低电平
#define MOTOR_LF1_LOW                       HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET)
#define MOTOR_LF2_LOW                       HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET)

#define MOTOR_RF1_LOW                       HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET)
#define MOTOR_RF2_LOW                       HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET)

#define MOTOR_LB1_LOW                       HAL_GPIO_WritePin(GPIOF, GPIO_PIN_5, GPIO_PIN_RESET)
#define MOTOR_LB2_LOW                       HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET)

#define MOTOR_RB1_LOW                       HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MOTOR_RB2_LOW                       HAL_GPIO_WritePin(GPIOF, GPIO_PIN_7, GPIO_PIN_RESET)

//左前
#define MOTOR_LF_SPEED(speed)                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, speed)
//右前
#define MOTOR_RF_SPEED(speed)                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, speed)
//左后
#define MOTOR_LB_SPEED(speed)                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, speed)
//右后
#define MOTOR_RB_SPEED(speed)                __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, speed)

//获取马达编码器计数值
#define MOTOR_LF_GetCount()                  __HAL_TIM_GET_COUNTER(&htim1)
#define MOTOR_RF_GetCount()                  __HAL_TIM_GET_COUNTER(&htim2)
#define MOTOR_LB_GetCount()                  __HAL_TIM_GET_COUNTER(&htim3)
#define MOTOR_RB_GetCount()                  __HAL_TIM_GET_COUNTER(&htim4)

void MOTOR_Init(void);
void MOTOR_LF_Control(int16_t speed);
void MOTOR_RF_Control(int16_t speed);
void MOTOR_LB_Control(int16_t speed);
void MOTOR_RB_Control(int16_t speed);

#endif /* DRIVERS_MOTOR_H_ */
