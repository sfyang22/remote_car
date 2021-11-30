/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-24     10426       the first version
 */
#include "servo.h"
#include <rtthread.h>

void SERVO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruture;

    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitStruture.Pin = GPIO_PIN_3 | GPIO_PIN_4;
    GPIO_InitStruture.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruture.Pull = GPIO_NOPULL;
    GPIO_InitStruture.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOG, &GPIO_InitStruture);

}

//控制舵机角度
//输入：角度值0~180°
void Servo_F_Control(float angle)
{
    uint16_t pulse;

    pulse = (500 + 2000 * ((angle + 90) / 180.0)) / 10;          //通过角度求出PWM信号值

    SERVO_F_HIGH;
    rt_thread_delay(pulse);
    SERVO_F_LOW;
    rt_thread_delay(SERVO_PWM_TIME - pulse);
}

//控制舵机角度
//输入：角度值0~180°
void Servo_B_Control(float angle)
{
    uint16_t pulse;

    pulse = (500 + 2000 * ((angle + 90) / 180.0)) / 10;          //通过角度求出PWM信号值

    SERVO_B_HIGH;
    rt_thread_delay(pulse);
    SERVO_B_LOW;
    rt_thread_delay(SERVO_PWM_TIME - pulse);

}
