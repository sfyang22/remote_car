/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-28     10426       the first version
 */
#ifndef APPLICATIONS_WIFI_H_
#define APPLICATIONS_WIFI_H_

#include "main.h"
#include "usart.h"

extern uint8_t ctl_cmd[];
extern uint8_t ctl_flag;

void WIFI_Init(void);
void WIFI_AUX_Waiting(void);
void WIFI_SendData(uint8_t *dat, uint16_t len, uint8_t end);


#endif /* APPLICATIONS_WIFI_H_ */
