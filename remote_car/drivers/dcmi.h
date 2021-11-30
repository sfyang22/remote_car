#ifndef _DCMI_H
#define _DCMI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//DCMI 驱动代码
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/14
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved
/////////////////////////////////////////////////////////////////////////////////
/**DCMI GPIO Configuration
PE5     ------> DCMI_D6
PE6     ------> DCMI_D7
PA4     ------> DCMI_HSYNC
PA6     ------> DCMI_PIXCLK
PC7     ------> DCMI_D1
PC8     ------> DCMI_D2
PC9     ------> DCMI_D3
PC6     ------> DCMI_D0
PC11     ------> DCMI_D4
PB6     ------> DCMI_D5
PB7     ------> DCMI_VSYNC
*/

extern DCMI_HandleTypeDef DCMI_Handler;        //DCMI句柄
extern DMA_HandleTypeDef  DMADMCI_Handler;     //DMA句柄

void DCMI_Init(void);
void DCMI_DMA_Init(u32 memaddr,u16 memsize,u32 memblen,u32 meminc);
void DCMI_Start(void);
void DCMI_Stop(void);
void DCMI_Set_Window(u16 sx,u16 sy,u16 width,u16 height);
void DCMI_CR_Set(u8 pclk,u8 hsync,u8 vsync);
#endif





















