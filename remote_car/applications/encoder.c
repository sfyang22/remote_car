/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-31     10426       the first version
 */
/* INCLUDE */
#include "encoder.h"

/*
 * 编码器初始化函数
 */
void ENCODER_Init(void)
{
    /** 定时器初始化函数 **/
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_TIM3_Init();
    MX_TIM4_Init();

    /** 开启编码器TIM1\2\3\4 **/
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_3);
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_4);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_4);

    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_4);

    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_3);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_4);

    /** 编码器复位 **/
    ENCODER_ResetCount();
}

/*
 * 编码器计数值复位函数
 * 功能： 将编码器数值复位成初始中值
 */
void ENCODER_ResetCount(void)
{
    /** 设置编码器初始值在中间 **/
    LF_SET_COUNT(ENCODER_COUNT);
    RF_SET_COUNT(ENCODER_COUNT);
    LB_SET_COUNT(ENCODER_COUNT);
    RB_SET_COUNT(ENCODER_COUNT);
}

/*
 * 编码器计算速度函数
 * 功能：计算出电机转速
 * 输入参数：LF_speed 左前电机速度 RF_speed 右前电机速度
 *LB_speed 左后电机速度 RB_speed 右后电机速度
 */
void ENCODER_CaculateSpeed(float *LF_speed, float *RF_speed, float *LB_speed, float *RB_speed)
{
    /** 编码器结构体定义 **/
    static ENCODER LF_Encoder = {0}, RF_Encoder = {0}, LB_Encoder = {0}, RB_Encoder = {0};

    LF_Encoder.now_count = LF_GET_COUNT();   //获取当前计数值
    RF_Encoder.now_count = RF_GET_COUNT();
    LB_Encoder.now_count = LB_GET_COUNT();
    RB_Encoder.now_count = RB_GET_COUNT();

    ENCODER_ResetCount();  //复位计数值

    LF_Encoder.speed_count = (int16_t)(LF_Encoder.now_count - ENCODER_COUNT);        //计算上一次与这一次差值
    RF_Encoder.speed_count = (int16_t)(RF_Encoder.now_count - ENCODER_COUNT);
    LB_Encoder.speed_count = (int16_t)(LB_Encoder.now_count - ENCODER_COUNT);
    RB_Encoder.speed_count = (int16_t)(RB_Encoder.now_count - ENCODER_COUNT);

    *LF_speed = (float)LF_Encoder.speed_count / ENCODER_FULL_PULSE * 100;            //求出当前速度百分比
    *RF_speed = (float)RF_Encoder.speed_count / ENCODER_FULL_PULSE * 100;
    *LB_speed = (float)LB_Encoder.speed_count / ENCODER_FULL_PULSE * 100;
    *RB_speed = (float)RB_Encoder.speed_count / ENCODER_FULL_PULSE * 100;

}





