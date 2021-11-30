/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-31     10426       the first version
 */
#ifndef APPLICATIONS_ENCODER_H_
#define APPLICATIONS_ENCODER_H_

#include "tim.h"

/** 编码器接口
PE9     ------> ENCODER_LF1
PE11     ------> ENCODER_LF2

PA5     ------> ENCODER_RF1
PB3     ------> ENCODER_RF2

PA7     ------> ENCODER_LB1
PB4     ------> ENCODER_LB2

PD12     ------> ENCODER_RB1
PD13     ------> ENCODER_RB2
*/

//电机满速脉冲数(4倍频)
#define ENCODER_FULL_PULSE               1660
//计数值缺省值
#define ENCODER_COUNT                   5000

//定时器结构体
#define LF_TIM                          htim1
#define RF_TIM                          htim2
#define LB_TIM                          htim3
#define RB_TIM                          htim4

//编码器设置计数值
#define LF_SET_COUNT(x)                  __HAL_TIM_SET_COUNTER(&LF_TIM, x);
#define RF_SET_COUNT(x)                  __HAL_TIM_SET_COUNTER(&RF_TIM, x);
#define LB_SET_COUNT(x)                  __HAL_TIM_SET_COUNTER(&LB_TIM, x);
#define RB_SET_COUNT(x)                  __HAL_TIM_SET_COUNTER(&RB_TIM, x);

//编码器获取计数值
#define LF_GET_COUNT()                  __HAL_TIM_GET_COUNTER(&LF_TIM)
#define RF_GET_COUNT()                  __HAL_TIM_GET_COUNTER(&RF_TIM)
#define LB_GET_COUNT()                  __HAL_TIM_GET_COUNTER(&LB_TIM)
#define RB_GET_COUNT()                  __HAL_TIM_GET_COUNTER(&RB_TIM)


/* 编码器结构体 */
typedef struct
{
    int16_t speed_count;        //差值
    uint16_t now_count;         //这次计数器值
}ENCODER;

void ENCODER_Init(void);
void ENCODER_ResetCount(void);
void ENCODER_CaculateSpeed(float *LF_speed, float *RF_speed, float *LB_speed, float *RB_speed);

#endif /* APPLICATIONS_ENCODER_H_ */
