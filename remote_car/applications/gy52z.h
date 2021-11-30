#ifndef _JY61P_H_
#define _JY61P_H_


#include "main.h"
#include "usart.h"

/** 编码器接口
PD5     ------> GY52Z_RXD
PD6     ------> GY52Z_TXD

*/
struct GY25Z_Data{
    uint8_t rebuf[30];  //接收缓冲区
    uint8_t re_state;   //标志位
    /** 原始数据 **/
    int16_t Temp;
    int16_t ACC_X_ori;
    int16_t ACC_Y_ori;
    int16_t ACC_Z_ori;
    int16_t GYRO_X_ori;
    int16_t GYRO_Y_ori;
    int16_t GYRO_Z_ori;
    int16_t ROLL_ori;
    int16_t PITCH_ori;
    int16_t YAW_ori;
    /** 解算后数据 **/
    float ACC_X;
    float ACC_Y;
    float ACC_Z;
    float GYRO_X;
    float GYRO_Y;
    float GYRO_Z;
    float ROLL;
    float PITCH;
    float YAW;
}GY25Z_Data;


void gy52z_Init(void);
void GY25Z_Calculate(void);

#endif /* __USART_H */
