/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-13     10426       the first version
 */
#ifndef APPLICATIONS_MQ_SENSOR_H_
#define APPLICATIONS_MQ_SENSOR_H_

#include "main.h"
#include "adc.h"

/** MQ传感器与锂电池电压接口
PC0     ------> MQ-7
PC1     ------> MQ-135
PC2     ------> LIPO_VOL
*/

extern uint16_t adc_value[];

void MQ_Init(void);
void MQ_StartSample(void);

#endif /* APPLICATIONS_MQ_SENSOR_H_ */
