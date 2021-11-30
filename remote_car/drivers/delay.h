#ifndef _DELAY_H
#define _DELAY_H
#include "sys.h"

#define SYSTEM_SUPPORT_OS                           1

extern uint8_t OSRunning;                          //在board.c文件的rt_hw_board_init()里面将其置为1

void delay_init(u16 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);
void delay_ostimedly(u32 ticks);


#endif

