/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-11-12     10426       the first version
 */
#include "dht11.h"
#include <rtthread.h>

static uint8_t status;
static uint8_t value_array[SIZE];
/* 可在其他的文件引用温湿度值,实际是温度的整数的 10 倍 如 dht11 读回的温度是 26,则 temp_value = 260, 湿度同理 */
int temp_value, humi_value;

static unsigned char ReadValue(void);

extern void Delay_1ms(unsigned int ms)
{
    rt_thread_mdelay(ms);
}
static void DHT11_Delay_10us(void)
{
    rt_thread_delay(1);     //10us
}

static void DHT11_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStruture;

    DHT11_GPIO_CLK();

    GPIO_InitStruture.Pin = DHT11_GPIO_PIN;
    GPIO_InitStruture.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruture.Pull = GPIO_NOPULL;
    GPIO_InitStruture.Speed = GPIO_SPEED_FREQ_MEDIUM;

    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruture);
}

__inline__ static void DHT11_GPIO_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStruture;

    GPIO_InitStruture.Pin = DHT11_GPIO_PIN;
    GPIO_InitStruture.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruture.Pull = GPIO_PULLUP;
    GPIO_InitStruture.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruture);
}

__inline__ static void DHT11_GPIO_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStruture;

    GPIO_InitStruture.Pin = DHT11_GPIO_PIN;
    GPIO_InitStruture.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruture.Pull = GPIO_NOPULL;
    GPIO_InitStruture.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(DHT11_GPIO_PORT, &GPIO_InitStruture);
}

void DHT11_Init(void)
{
    DHT11_GPIO_Config();
    DHT11_GPIO_OUT();
}



/*读一个字节的数据*/
static unsigned char DHT11_ReadValue(void)
{
    unsigned char value = 0, i;
    uint32_t count = 0;
    status = OK; //设定标志为正常状态
    for(i = 8; i > 0; i--)
    {
         //高位在先
         value <<= 1;
         count = 0;
         //每一位数据前会有一个 50us 的低电平时间.等待 50us 低电平结束
         DHT11_GPIO_IN();
         while(DHT11_GPIO_READ == 0 && count++ < NUMBER);
         if(count >= NUMBER)
         {
             status = ERROR; //设定错误标志
             return 0; //函数执行过程发生错误就退出函数
         }
         //26-28us 的高电平表示该位是 0,为 70us 高电平表该位 1
         DHT11_Delay_10us();
         DHT11_Delay_10us();
         DHT11_Delay_10us();
         //延时 30us 后检测数据线是否还是高电平
         if(DHT11_GPIO_READ != 0)
         {
             //进入这里表示该位是 1
             value++;
             //等待剩余(约 40us)的高电平结束
             DHT11_GPIO_IN();
             while(DHT11_GPIO_READ != 0 && count++ < NUMBER);
             if(count >= NUMBER)
             {
                 status = ERROR; //设定错误标志
                 return 0;
             }
         }
    }
    return (value);
}
//读温度和湿度函数，读一次的数据,共五字节，读出成功函数返回 OK, 错误返回 ERROR
uint8_t DHT11_ReadTempAndHumi(void)
{
    unsigned char i = 0, check_value = 0;
    uint32_t count = 0;
    //EA = 0;
    DHT11_GPIO_OUT();
    DHT11_GPIO_LOW; //拉低数据线大于 18ms 发送开始信号
    Delay_1ms(20); //需大于 18 毫秒
    DHT11_GPIO_HIGH; //释放数据线,用于检测低电平的应答信号
    //延时 20-40us,等待一段时间后检测应答信号,应答信号是从机拉低数据线 80us
    DHT11_Delay_10us();
    DHT11_Delay_10us();
    DHT11_Delay_10us();
    DHT11_Delay_10us();
    DHT11_GPIO_IN();
    if(DHT11_GPIO_READ != 0) //检测应答信号,应答信号是低电平
    {
    //没应答信号
        //EA = 1;
        return ERROR;
    }
    else
    {
        //有应答信号
        while(DHT11_GPIO_READ == 0 && count++ < NUMBER); //等待应答信号结束
        if(count >= NUMBER) //检测计数器是否超过了设定的范围
        {
            DHT11_GPIO_OUT();
            DHT11_GPIO_HIGH;
            //EA = 1;
            return ERROR; //读数据出错,退出函数
        }
        count = 0;
        //DHT11_GPIO_HIGH;//释放数据线
        DHT11_GPIO_IN();
        //应答信号后会有一个 80us 的高电平，等待高电平结束
        while(DHT11_GPIO_READ != 0 && count++ < NUMBER);
        if(count >= NUMBER)
        {
            DHT11_GPIO_OUT();
            DHT11_GPIO_HIGH;
            //EA = 1;
            return ERROR; //退出函数
        }
        //读出湿.温度值
        for(i = 0; i < SIZE; i++)
        {
            value_array[i] = DHT11_ReadValue();
            if(status == ERROR)//调用 ReadValue()读数据出错会设定 status 为 ERROR
            {

                DHT11_GPIO_OUT();
                DHT11_GPIO_HIGH;
                //EA = 1;
                return ERROR;
            }
            //读出的最后一个值是校验值不需加上去
            if(i != SIZE - 1)
            {
                //读出的五字节数据中的前四字节数据和等于第五字节数据表示成功
                check_value += value_array[i];
            }
        }//end for

        //在没用发生函数调用失败时进行校验
        if(check_value == value_array[SIZE - 1])
        {
            //将温湿度扩大 10 倍方便分离出每一位
            humi_value = value_array[0] * 10;
            temp_value = value_array[2] * 10;
            DHT11_GPIO_OUT();
            DHT11_GPIO_HIGH;
            //EA = 1;
            return OK; //正确的读出 dht11 输出的数据
        }
        else
        {
            //校验数据出错
            //EA = 1;
            return ERROR;
        }
    }
}

