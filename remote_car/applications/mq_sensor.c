/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-13     10426       the first version
 */
#include "mq_sensor.h"

uint16_t adc_value[3];

/*
 * 气体传感器初始化
 */
void MQ_Init(void)
{
    /** DMA通道初始化 **/
    MX_DMA_Init();
    /** ADC初始化 **/
    MX_ADC1_Init();
}

/*
 * 开启传感器采样DMA通道
 */
void MQ_StartSample(void)
{
    HAL_ADC_Start_DMA(&hadc1, adc_value, 3);
}



