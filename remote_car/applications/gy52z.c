#include "string.h"
#include <rtthread.h>
#include <board.h>
#include <gy52z.h>
#include "sys.h"
#include <stdio.h>


 /**
  * @brief  USART GPIO 配置,工作参数配置
  * @param  无
  * @retval 无
  */
static void gy52z_Config(void)
{
    /** 串口2初始化 **/
    MX_USART2_UART_Init();
    /** 开启串口2接收中断 **/
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);
}

//发送一个字节到陀螺仪
static void gy52z_SendByte(uint8_t ch)
{
    HAL_UART_Transmit(&huart2, ch, 1, 100);
}

void gy52z_Init(void)
{
    /** 配置串口 **/
    gy52z_Config();
    //设置刷新频率
    /*
    * 10Hz 输出设置指令-----A5 59 01 FF
    *   50Hz 输出设置指令-----A5 59 02 00
    *   100Hz 输出设置指令----A5 59 03 01
    *   200Hz 输出设置指令----A5 59 04 02
    */
    gy52z_SendByte(0xA5);
    delay_us(350);
    gy52z_SendByte(0x59);
    delay_us(350);
    gy52z_SendByte(0x03);
    delay_us(350);
    gy52z_SendByte(0x01);  //以上是设置刷新频率

    /*
    *
    *   115200设置指令:A5 58 01 FE
    *   9600 设置指令:A5 58 02 FF
    */
    gy52z_SendByte(0xA5);
    delay_us(350);
    gy52z_SendByte(0x58);
    delay_us(350);
    gy52z_SendByte(0x01);
    delay_us(350);
    gy52z_SendByte(0xFE);  //以上是设置波特率

    /*
    *   加陀校准指令  A5 57 01 FD
    *   磁力计校准指令：A5 57 02 Fe
    */
    gy52z_SendByte(0xA5);
    delay_us(350);
    gy52z_SendByte(0x57);
    delay_us(350);
    gy52z_SendByte(0x01);
    delay_us(350);
    gy52z_SendByte(0xFD);  //以上是加陀校准
    delay_ms(1000);

}

void GY25Z_Calculate(void)  //解算陀螺仪数据
{
    uint8_t count=0;
    if(GY25Z_Data.re_state == 1)
    {
        if(GY25Z_Data.rebuf [2]&0x01)
        {
            GY25Z_Data.ACC_X_ori =((GY25Z_Data.rebuf [4]<<8)|GY25Z_Data.rebuf [5]);
            GY25Z_Data.ACC_Y_ori =(GY25Z_Data.rebuf [6]<<8)|GY25Z_Data.rebuf [7];
            GY25Z_Data.ACC_Z_ori =(GY25Z_Data.rebuf [8]<<8)|GY25Z_Data.rebuf [9];
            count=6;
        }

        if(GY25Z_Data.rebuf [2]&0x02)
        {
            GY25Z_Data.GYRO_X_ori =(GY25Z_Data.rebuf [4+count]<<8)|GY25Z_Data.rebuf [5+count];
            GY25Z_Data.GYRO_Y_ori =(GY25Z_Data.rebuf [6+count]<<8)|GY25Z_Data.rebuf [7+count];
            GY25Z_Data.GYRO_Z_ori =(GY25Z_Data.rebuf [8+count]<<8)|GY25Z_Data.rebuf [9+count];
            count+=6;
        }

        if(GY25Z_Data.rebuf [2]&0x10)
        {
            GY25Z_Data.ROLL_ori  =((GY25Z_Data.rebuf [4+count]<<8)|GY25Z_Data.rebuf [5+count]);
            GY25Z_Data.PITCH_ori =((GY25Z_Data.rebuf [6+count]<<8)|GY25Z_Data.rebuf [7+count]);
            GY25Z_Data.YAW_ori   =((GY25Z_Data.rebuf [8+count]<<8)|GY25Z_Data.rebuf [9+count]);
            count+=6;
        }

        if(GY25Z_Data.rebuf [2]&0x40)
        {
            GY25Z_Data.Temp =((GY25Z_Data.rebuf [4+count]<<8)|GY25Z_Data.rebuf [5+count])/100;
            count+=2;
        }

        GY25Z_Data.ACC_X=GY25Z_Data.ACC_X_ori/16383.5;
        GY25Z_Data.ACC_Y=GY25Z_Data.ACC_Y_ori/16383.5;
        GY25Z_Data.ACC_Z=GY25Z_Data.ACC_Z_ori/16383.5;

        GY25Z_Data.GYRO_X = GY25Z_Data.GYRO_X_ori/16.3835;
        GY25Z_Data.GYRO_Y = GY25Z_Data.GYRO_Y_ori/16.3835;
        GY25Z_Data.GYRO_Z = GY25Z_Data.GYRO_Z_ori/16.3835;

        GY25Z_Data.ROLL = GY25Z_Data.ROLL_ori/100;
        GY25Z_Data.PITCH = GY25Z_Data.PITCH_ori/100;
        GY25Z_Data.YAW = GY25Z_Data.YAW_ori/100;

        GY25Z_Data.re_state =0;
    }
}



//串口中断服务子程序
void USART2_IRQHandler(void)
{
    static uint8_t rebuf[30]={0},i=0;

    rt_interrupt_enter();       //进入中断保护
    if(__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE) != RESET)
    {
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);

        rebuf[i++] = huart2.Instance->DR;
        if(rebuf[0]!=0x5a)
        {
            i=0;
        }
        if((i==2)&&(rebuf[1]!=0x5a))
        {
            i=0;
        }
        if(i==(rebuf[3]+5))
        {
            memcpy(GY25Z_Data.rebuf, rebuf, i);
            GY25Z_Data.re_state =1;
            i=0;
        }
    }
    rt_interrupt_leave();       //退出中断保护

}



