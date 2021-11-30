/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-28     10426       the first version
 */
#include "wifi.h"
#include <rtthread.h>
#include <stdio.h>

uint8_t ctl_cmd[6];
uint8_t ctl_flag = 0;

static void AUX_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruture;

    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitStruture.Pin = GPIO_PIN_0;
    GPIO_InitStruture.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruture.Pull = GPIO_PULLUP;
    GPIO_InitStruture.Speed = GPIO_SPEED_FREQ_HIGH;

    HAL_GPIO_Init(GPIOE, &GPIO_InitStruture);

}

void WIFI_Init(void)
{
    /** 串口3初始化 **/
    MX_USART3_UART_Init();
    /** 开启串口3接收中断 **/
   __HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
}

void WIFI_AUX_Waiting(void)
{
    while(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0) == RESET)     //等待数传WIIF空闲
    {
        rt_thread_mdelay(3);
    }
}

//数传WIFI发送数据
//输入： *dat 需要发送数据数组 len 数据长度 end 结束位
//返回： 无
void WIFI_SendData(uint8_t *dat, uint16_t len, uint8_t end)
{
    uint16_t i;
    uint8_t send_dat[len + 2];

    send_dat[0] = 0xAA;     //0xAA 起始位

    for(i=0;i<len;i++)      //拼接数据
    {
        send_dat[i+1] = *dat++;
    }

    send_dat[len+1] = end; //添加结束位

    HAL_UART_Transmit(&huart3, send_dat, len+2, 100);       //通过串口发送
}

//串口3中断服务子程序
void USART3_IRQHandler(void)
{
    static uint8_t rec_co = 0;
    static uint8_t rec[8];

    rt_interrupt_enter();       //进入中断保护

    if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_RXNE) != RESET)
    {
        __HAL_UART_CLEAR_FLAG(&huart3, UART_FLAG_RXNE);
        rec[rec_co] = huart3.Instance->DR;

        if(rec[0] == 0xAA)      //若接收到起始位0xAA
        {
            rec_co++;
        }

        if(rec_co == 8)         //接收满6个字节
        {
            rec_co = 0;
            if(rec[7] == 0xBB)      //若接收到结束位，则数据正确
            {
                memcpy(ctl_cmd, rec+1, 6);        //拷贝进存储区待用户取出
                ctl_flag = 1;                   //数据收到标志位
            }
        }

    }

    rt_interrupt_leave();       //退出中断保护
}

