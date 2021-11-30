/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-19     RealThread   first version
 */

#include <rtthread.h>
#include <board.h>
#include <drv_common.h>

RT_WEAK void rt_hw_board_init()
{
    extern void hw_board_init(char *clock_src, int32_t clock_src_freq, int32_t clock_target_freq);

    /* Heap initialization */
#if defined(RT_USING_HEAP)
    rt_system_heap_init((void *) HEAP_BEGIN, (void *) HEAP_END);
#endif

    hw_board_init(BSP_CLOCK_SOURCE, BSP_CLOCK_SOURCE_FREQ_MHZ, BSP_CLOCK_SYSTEM_FREQ_MHZ);

    /* Set the shell console output device */
#if defined(RT_USING_DEVICE) && defined(RT_USING_CONSOLE)
    rt_console_set_device(RT_CONSOLE_DEVICE_NAME);
#endif

    /* Board underlying hardware initialization */
#ifdef RT_USING_COMPONENTS_INIT
    rt_components_board_init();
#endif


/*******************用户初始化********************/
/** 延时函数初始化 **/
    delay_init(168);
/** LED初始化 **/
    LED_Init();
/** 编码器初始化 **/
    ENCODER_Init();
/** 马达初始化 **/
    MOTOR_Init();
/** 舵机初始化 **/
    SERVO_Init();
/** 陀螺仪初始化 **/
    gy52z_Init();
/** 温湿度传感器初始化 **/
    DHT11_Init();
/** 气体传感器初始化 **/
    MQ_Init();
/** 车辆前照灯初始化 **/
    CAR_LED_Init();
/** 数传WIFI初始化 **/
    WIFI_Init();

#if 1
/** MT8266WIFI初始化 **/
    M8266HostIf_Init();
    while(!M8266WIFI_Module_Init_Via_SPI());
    rt_kprintf("MT8266WIFI init success!\r\n");

    MX_DMA_Init();
/** OV2640初始化 **/
    while(OV2640_Init());
    rt_kprintf("\r\nov2640 init success!\r\n");
/** DCMI初始化 **/
    OV2640_JPEG_Mode();     //JPEG模式
    DCMI_Init();
#endif

    OSRunning = 1;
}
