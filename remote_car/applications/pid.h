/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-31     10426       the first version
 */
#ifndef APPLICATIONS_PID_H_
#define APPLICATIONS_PID_H_

#include "main.h"
#include "math.h"

typedef struct{
    float P;
    float I;
    float D;
    float now_error;
    float ierror;
    float last_error;
    float previous_error;
    int16_t output;
}PID_InitType;

int16_t MOTOR_PID(PID_InitType *pid, float now, float expect);
int16_t MOTOR_IncreasePID(PID_InitType *pid, float now, float expect);

#endif /* APPLICATIONS_PID_H_ */
