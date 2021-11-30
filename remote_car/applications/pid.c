/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-31     10426       the first version
 */
#include "pid.h"

/*
 * 位置式PID
 */
int16_t MOTOR_PID(PID_InitType *pid, float now, float expect)
{
    float temp, I_sta = 1.0F;
    int16_t output;

    pid->now_error = expect - now;               //当前误差

    if(fabsf(pid->now_error) < 0.2F)               //允许误差
    {
        pid->now_error = 0.0F;
    }

    pid->ierror += pid->now_error;

    if(pid->ierror < -1000.0F)
    {
        pid->ierror = -1000.0F;
    }
    else if(pid->ierror > 1000.0F)
    {
        pid->ierror = 1000.0F;
    }

    if(fabsf(expect) <= 30)
    {
        I_sta = 0.3;
    }


    temp = pid->P * pid->now_error + I_sta * pid->I * pid->ierror + pid->D * (pid->now_error - pid->last_error);

    pid->last_error = pid->now_error;            //保留作为上一次误差

    if(temp > 1000.0F)
    {
        temp = 1000.0F;
    }
    else if(temp < -1000.0F)
    {
        temp = -1000.0F;
    }
    else if(fabsf(temp) < 0.001F)        //float型与0比较
    {
        temp = 0.0;
    }

    output = (int16_t) temp;

    return output;
}

/*
 * 增量式PID
 */
int16_t MOTOR_IncreasePID(PID_InitType *pid, float now, float expect)
{
    float temp;

    pid->now_error = expect - now;               //当前误差

    if(fabsf(pid->now_error) < 0.2F)               //允许误差
    {
        pid->now_error = 0.0F;
    }

    pid->ierror += pid->now_error;


    if(pid->ierror < -300.0F)                  //积分限幅
    {
        pid->ierror = -300.0F;
    }
    else if(pid->ierror > 300.0F)
    {
        pid->ierror = 300.0F;
    }


    temp = pid->P * (pid->now_error - pid->last_error)
            + pid->I * pid->now_error
            + pid->D * (pid->now_error - 2 * pid->last_error + pid->previous_error);

    pid->previous_error = pid->last_error;
    pid->last_error = pid->now_error;            //保留作为上一次误差

    pid->output += (int16_t) temp;

    if(pid->output > 1000.0F)
    {
        pid->output = 1000.0F;
    }
    else if(pid->output < -1000.0F)
    {
        pid->output = -1000.0F;
    }
    else if(fabsf(pid->output) < 0.001F)        //float型与0比较
    {
        pid->output = 0.0;
    }

    return pid->output;
}

