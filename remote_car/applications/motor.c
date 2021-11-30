/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-24     10426       the first version
 */
#include "motor.h"

/* 控制线引脚初始化 */
static void MOTOR_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruture;

    __HAL_RCC_GPIOF_CLK_ENABLE();

    GPIO_InitStruture.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 |
                            GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruture.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruture.Pull = GPIO_PULLUP;
    GPIO_InitStruture.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    HAL_GPIO_Init(GPIOF, &GPIO_InitStruture);

    MOTOR_LF1_HIGH;
    MOTOR_LF2_HIGH;
    MOTOR_RF1_HIGH;
    MOTOR_RF2_HIGH;

}

/*
 * LF电机速度控制函数
 * 输入： speed 电机速度
*/
void MOTOR_LF_Control(int16_t speed)
{
    uint16_t compare;

    if(speed > 0)
    {
        MOTOR_LF1_HIGH;
        MOTOR_LF2_LOW;
        compare = (uint16_t) speed;
    }
    else if(speed < 0)
    {
        MOTOR_LF1_LOW;
        MOTOR_LF2_HIGH;
        compare = (uint16_t) (-speed);
    }
    else
    {
        MOTOR_LF1_HIGH;
        MOTOR_LF2_HIGH;
        compare = 0;
    }
    MOTOR_LF_SPEED(compare);
}

/*
 * RF电机速度控制函数
 * 输入： speed 电机速度
*/
void MOTOR_RF_Control(int16_t speed)
{
    uint16_t compare;

    if(speed > 0)
    {
        MOTOR_RF1_HIGH;
        MOTOR_RF2_LOW;
        compare = (uint16_t) speed;
    }
    else if(speed < 0)
    {
        MOTOR_RF1_LOW;
        MOTOR_RF2_HIGH;
        compare = (uint16_t) (-speed);
    }
    else
    {
        MOTOR_RF1_HIGH;
        MOTOR_RF2_HIGH;
        compare = 0;
    }
    MOTOR_RF_SPEED(compare);
}

/*
 * LB电机速度控制函数
 * 输入： speed 电机速度
*/
void MOTOR_LB_Control(int16_t speed)
{
    uint16_t compare;

    if(speed > 0)
    {
        MOTOR_LB1_HIGH;
        MOTOR_LB2_LOW;
        compare = (uint16_t) speed;
    }
    else if(speed < 0)
    {
        MOTOR_LB1_LOW;
        MOTOR_LB2_HIGH;
        compare = (uint16_t) (-speed);
    }
    else
    {
        MOTOR_LB1_HIGH;
        MOTOR_LB2_HIGH;
        compare = 0;
    }
    MOTOR_LB_SPEED(compare);
}

/*
 * RB电机速度控制函数
 * 输入： speed 电机速度
*/
void MOTOR_RB_Control(int16_t speed)
{
    uint16_t compare;

    if(speed > 0)
    {
        MOTOR_RB1_HIGH;
        MOTOR_RB2_LOW;
        compare = (uint16_t) speed;
    }
    else if(speed < 0)
    {
        MOTOR_RB1_LOW;
        MOTOR_RB2_HIGH;
        compare = (uint16_t) (-speed);
    }
    else
    {
        MOTOR_RB1_HIGH;
        MOTOR_RB2_HIGH;
        compare = 0;
    }
    MOTOR_RB_SPEED(compare);
}

/* 电机初始化函数 */
void MOTOR_Init(void)
{
    /** 定时器初始化 **/
    MX_TIM5_Init();
    /** GPIO初始化 **/
    MOTOR_GPIO_Init();
    /** 使能PWM **/
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);


    /** 主输出使能，高级定时器开启PWM需要 **/
    //htim1.Instance->BDTR |= 1 << 15;

}


