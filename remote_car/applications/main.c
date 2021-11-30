/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2021-10-19     RT-Thread    first version
 */

#include <rtthread.h>
#include <board.h>

__ALIGNED(4)

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

extern u8 ov_frame;                              //帧率

#define SEND_PACK_SIZE  1024                //包字节
#define JPEG_BUF_SIZE (25*1024)           //定义JPEG数据缓存jpeg_buf的大小(字节)
u8 jpeg_buf[JPEG_BUF_SIZE];            //JPEG数据缓存buf
//volatile u32 jpeg_data_len=0;           //buf中的JPEG有效数据长度
volatile u8 jpeg_data_ok=0;             //JPEG数据采集完成标志
                                        //0,数据没有采集完;
                                        //1,数据采集完了,但是还没处理;
                                        //2,数据已经处理完成了,可以开始下一帧接收

/* 指示灯线程 */
rt_thread_t led_thread = RT_NULL;
/* 摄像头线程 */
rt_thread_t camera_thread = RT_NULL;
/* 陀螺仪接收线程 */
rt_thread_t gy52z_thread = RT_NULL;
/* 马达线程 */
rt_thread_t motor_thread = RT_NULL;
/* 编码器线程 */
rt_thread_t encoder_thread = RT_NULL;
/* 气体传感器线程 */
rt_thread_t mq_sensor_thread = RT_NULL;
/* 温湿度传感器线程 */
rt_thread_t dht11_thread = RT_NULL;
/* 前照灯线程 */
rt_thread_t car_led_thread = RT_NULL;
/* 前舵机线程 */
rt_thread_t servo_f_thread = RT_NULL;
/* 后舵机线程 */
rt_thread_t servo_b_thread = RT_NULL;
/* 数传WIFI线程 */
rt_thread_t wifi_thread = RT_NULL;
/* 数传回传WIFI线程 */
rt_thread_t wifi_send_thread = RT_NULL;


/* 定义信号量 */
rt_sem_t jpeg_send_sem = RT_NULL;
rt_sem_t speed_sem = RT_NULL;
rt_sem_t wifi_sem = RT_NULL;

/* 定义消息队列 */
rt_mq_t motor_mq = RT_NULL;
rt_mq_t servo_f_mq = RT_NULL;
rt_mq_t servo_b_mq = RT_NULL;
rt_mq_t motor_L_mq = RT_NULL;
rt_mq_t motor_R_mq = RT_NULL;
rt_mq_t car_led_mq = RT_NULL;
rt_mq_t gy52z_mq = RT_NULL;

/* 定义邮箱 */
rt_mailbox_t air_mb = RT_NULL;
rt_mailbox_t co_mb = RT_NULL;
rt_mailbox_t humi_mb = RT_NULL;
rt_mailbox_t temp_mb = RT_NULL;
rt_mailbox_t lipo_mb = RT_NULL;

/* 指示灯线程函数 */
static void led_thread_entry(void* par);
/* 摄像头线程函数 */
static void camera_thread_entry(void* par);
/* 摄像头接收线程函数 */
static void gy52z_thread_entry(void* par);
/* 马达线程函数 */
static void motor_thread_entry(void* par);
/* 编码器线程函数 */
static void encoder_thread_entry(void* par);
/* 气体线程函数 */
static void mq_sensor_thread_entry(void* par);
/* 温湿度线程函数 */
static void dht11_thread_entry(void* par);
/* 前照灯线程函数 */
static void car_led_thread_entry(void* par);
/* 前舵机线程函数 */
static void servo_f_thread_entry(void* par);
/* 后舵机线程函数 */
static void servo_b_thread_entry(void* par);
/* 数传WIFI线程函数 */
static void wifi_thread_entry(void* par);
/* 数传回传WIFI线程函数 */
static void wifi_send_thread_entry(void* par);

/* 电机PID结构体 (P,I,D)**/
PID_InitType LF_PID = {14.0F, 2.0F, 0.1F};
PID_InitType RF_PID = {12.0F, 2.8F, 0.1F};
PID_InitType LB_PID = {14.0F, 2.5F, 0.1F};
PID_InitType RB_PID = {12.0F, 2.8F, 0.1F};
/* 小车当前速度 */
float now_speed[4];     //LF RF LB RB

int main(void)
{
    /**************** 线程 ***********************/
    led_thread = rt_thread_create("led", (void*) led_thread_entry, RT_NULL, 256, 30, 10);
    if(RT_NULL != led_thread)
    {
        rt_thread_startup(led_thread);
    }

    camera_thread = rt_thread_create("camera", (void*) camera_thread_entry, RT_NULL, JPEG_BUF_SIZE, 29, 100);
    if(RT_NULL != camera_thread)
    {
        rt_thread_startup(camera_thread);
    }

    gy52z_thread = rt_thread_create("gy52z_thread", (void*) gy52z_thread_entry, RT_NULL, 1024, 23, 10);
    if(RT_NULL != gy52z_thread)
    {
        rt_thread_startup(gy52z_thread);
    }

    motor_thread = rt_thread_create("motor", (void*) motor_thread_entry, RT_NULL, 1024, 16, 10);
    if(RT_NULL != motor_thread)
    {
        rt_thread_startup(motor_thread);
    }

    encoder_thread = rt_thread_create("encoder", (void*) encoder_thread_entry, RT_NULL, 1024, 15, 10);
    if(RT_NULL != encoder_thread)
    {
        rt_thread_startup(encoder_thread);
    }

    mq_sensor_thread = rt_thread_create("mq_sensor_thread", (void*) mq_sensor_thread_entry, RT_NULL, 512, 28, 10);
    if(RT_NULL != mq_sensor_thread)
    {
        rt_thread_startup(mq_sensor_thread);
    }

    dht11_thread = rt_thread_create("dht11_thread", (void*) dht11_thread_entry, RT_NULL, 512, 26, 10);
    if(RT_NULL != dht11_thread)
    {
        rt_thread_startup(dht11_thread);
    }

    car_led_thread = rt_thread_create("car_led_thread", (void*) car_led_thread_entry, RT_NULL, 256, 19, 10);
    if(RT_NULL != car_led_thread)
    {
       rt_thread_startup(car_led_thread);
    }

    servo_f_thread = rt_thread_create("servo_f_thread", (void*) servo_f_thread_entry, RT_NULL, 512, 17, 10);
    if(RT_NULL != servo_f_thread)
    {
       rt_thread_startup(servo_f_thread);
    }

    servo_b_thread = rt_thread_create("servo_b_thread", (void*) servo_b_thread_entry, RT_NULL, 512, 17, 10);
    if(RT_NULL != servo_b_thread)
    {
       rt_thread_startup(servo_b_thread);
    }

    wifi_thread = rt_thread_create("wifi_thread", (void*) wifi_thread_entry, RT_NULL, 512, 18, 10);
    if(RT_NULL != wifi_thread)
    {
       rt_thread_startup(wifi_thread);
    }

    wifi_send_thread = rt_thread_create("wifi_send_thread", (void*) wifi_send_thread_entry, RT_NULL, 512, 25, 10);
    if(RT_NULL != wifi_send_thread)
    {
       rt_thread_startup(wifi_send_thread);
    }

    /**************** 邮箱 ***********************/
    air_mb = rt_mb_create("air_mb", 4, RT_IPC_FLAG_FIFO);
    if(RT_NULL == air_mb)
    {
        return -1;
    }

    co_mb = rt_mb_create("co_mb", 4, RT_IPC_FLAG_FIFO);
    if(RT_NULL == co_mb)
    {
        return -1;
    }

    humi_mb = rt_mb_create("humi_mb", 4, RT_IPC_FLAG_FIFO);
    if(RT_NULL == humi_mb)
    {
        return -1;
    }

    temp_mb = rt_mb_create("temp_mb", 4, RT_IPC_FLAG_FIFO);
    if(RT_NULL == temp_mb)
    {
        return -1;
    }

    lipo_mb = rt_mb_create("lipo_mb", 4, RT_IPC_FLAG_FIFO);
    if(RT_NULL == lipo_mb)
    {
       return -1;
    }

    /**************** 信号量 ***********************/
    jpeg_send_sem = rt_sem_create("jpeg_send_sem", 0, RT_IPC_FLAG_FIFO);
    if(RT_NULL == jpeg_send_sem)
    {
        return -1;
    }

    speed_sem = rt_sem_create("speed_sem", 0, RT_IPC_FLAG_FIFO);
    if(RT_NULL == speed_sem)
    {
      return -1;
    }

    wifi_sem = rt_sem_create("wifi_sem", 1, RT_IPC_FLAG_FIFO);
    if(RT_NULL == wifi_sem)
    {
      return -1;
    }

    /**************** 消息 ***********************/
    servo_f_mq = rt_mq_create("servo_f_mq", 1, 1, RT_IPC_FLAG_FIFO);
    if(RT_NULL == servo_f_mq)
    {
        return -1;
    }

    servo_b_mq = rt_mq_create("servo_b_mq", 1, 1, RT_IPC_FLAG_FIFO);
    if(RT_NULL == servo_b_mq)
    {
        return -1;
    }

    motor_L_mq = rt_mq_create("motor_L_mq", 1, 1, RT_IPC_FLAG_FIFO);
    if(RT_NULL == motor_L_mq)
    {
        return -1;
    }

    motor_R_mq = rt_mq_create("motor_L_mq", 1, 1, RT_IPC_FLAG_FIFO);
    if(RT_NULL == motor_L_mq)
    {
        return -1;
    }

    car_led_mq = rt_mq_create("car_led_mq", 2, 1, RT_IPC_FLAG_FIFO);
    if(RT_NULL == car_led_mq)
    {
        return -1;
    }

    gy52z_mq = rt_mq_create("gy52z_mq", 3, 1, RT_IPC_FLAG_FIFO);
    if(RT_NULL == gy52z_mq)
    {
      return -1;
    }

}

static void wifi_send_thread_entry(void* par)
{
    rt_uint16_t *air, *co;         //空气质量 一氧化碳浓度百分比
    rt_uint16_t *humi;            //湿度
    rt_int16_t *temp;             //温度
    rt_uint8_t *lipo_power;      //锂电池电量
    static rt_uint8_t send_info[9];

    while(1)
    {
        rt_mb_recv(air_mb, (rt_ubase_t *)&air, RT_WAITING_FOREVER);
        rt_mb_recv(co_mb, (rt_ubase_t *)&co, RT_WAITING_FOREVER);
        rt_mb_recv(lipo_mb, (rt_ubase_t *)&lipo_power, RT_WAITING_FOREVER);
        rt_mb_recv(humi_mb, (rt_ubase_t *)&humi, RT_WAITING_FOREVER);
        rt_mb_recv(temp_mb, (rt_ubase_t *)&temp, RT_WAITING_FOREVER);

        send_info[0] = (*air) >> 8;
        send_info[1] = (*air);
        send_info[2] = (*co) >> 8;
        send_info[3] = (*co);
        send_info[4] = *lipo_power;
        send_info[5] = (*humi) >> 8;
        send_info[6] = (*humi);
        send_info[7] = (*temp) >> 8;
        send_info[8] = (*temp);

        WIFI_AUX_Waiting();     //等待WIFI空闲
        rt_sem_take(wifi_sem, RT_WAITING_FOREVER);      //防止抢占WIIF发送资源
        WIFI_SendData(send_info, sizeof(send_info), 0xBB);    //发送数据
        rt_sem_release(wifi_sem);

//        for(int i=0;i<sizeof(send_info);i++)
//        {
//            rt_kprintf("%d  ", send_info[i]);
//        }
//        rt_kprintf("\r\n");
    }
}

static void wifi_thread_entry(void* par)
{
    rt_int8_t servo_f_value, servo_b_value;
    rt_int16_t motor_L_value, motor_R_value;
    rt_int8_t L_value, R_value;
    rt_int8_t motor_dif, motor_speed;       //左右差速 前后速度
    rt_uint8_t car_led[2];      //前照灯状态

    while(1)
    {
        if(ctl_flag)
        {
            ctl_flag = 0;
            servo_f_value = (rt_int8_t)ctl_cmd[0];
            servo_b_value = (rt_int8_t)ctl_cmd[1];
            motor_speed = (rt_int8_t)ctl_cmd[2];
            motor_dif = (rt_int8_t)ctl_cmd[3];
            car_led[0] = ctl_cmd[4];
            car_led[1] = ctl_cmd[5];
            motor_L_value = motor_speed + motor_dif;
            motor_R_value = motor_speed - motor_dif;

            if(motor_L_value < -100)
            {
                motor_L_value = -100;
            }
            else if(motor_L_value > 100)
            {
                motor_L_value = 100;
            }

            if(motor_R_value < -100)
            {
                motor_R_value = -100;
            }
            else if(motor_R_value > 100)
            {
                motor_R_value = 100;
            }

            L_value = (rt_int8_t)motor_L_value;
            R_value = (rt_int8_t)motor_R_value;

            rt_mq_send(servo_f_mq, &servo_f_value, 1);
            rt_mq_send(servo_b_mq, &servo_b_value, 1);
            rt_mq_send(motor_L_mq, &L_value, 1);
            rt_mq_send(motor_R_mq, &R_value, 1);
            rt_mq_send(car_led_mq, car_led, 2);
        }



        rt_thread_mdelay(30);
    }
}

static void servo_f_thread_entry(void* par)
{
    rt_int8_t f_angle = 90;

    while(1)
    {
        rt_mq_recv(servo_f_mq, &f_angle, 1, RT_WAITING_NO);
        Servo_F_Control((float)f_angle);      //舵机角度
    }
}

static void servo_b_thread_entry(void* par)
{
    rt_int8_t b_angle = 90;
    while(1)
    {
        rt_mq_recv(servo_b_mq, &b_angle, 1, RT_WAITING_NO);
        Servo_B_Control((float)b_angle);      //舵机角度
    }
}

static void car_led_thread_entry(void* par)
{
    static uint8_t car_led[2] = {0};  //开关状态 亮度

    while(1)
    {
        rt_mq_recv(car_led_mq, &car_led, 2, RT_WAITING_NO);

        if(car_led[0] == 1)     //开灯状态
        {
            CAR_LED_ON();
            rt_thread_delay(car_led[1]);
            CAR_LED_OFF();
            rt_thread_delay(100-car_led[1]);
        }
        else
        {
            rt_thread_mdelay(200);
        }

    }
}

static void dht11_thread_entry(void* par)
{
    rt_uint16_t humi_v;
    rt_uint16_t temp_v;
    while(1)
    {
        DHT11_ReadTempAndHumi();        //读DHT11传感器数据

        humi_v = (rt_uint16_t)(humi_value);     //转成float型
        temp_v = (rt_uint16_t)(temp_value);

        rt_mb_send(humi_mb, (rt_ubase_t)&humi_v);
        rt_mb_send(temp_mb, (rt_ubase_t)&temp_v);

        rt_thread_mdelay(500);
    }
}

#define LIPO_COUNT      10

static void mq_sensor_thread_entry(void* par)
{
    static uint16_t lipo_co = 0;
    static float lipo_value[LIPO_COUNT];
    float co1_potency = 0.0F, air_quality = 0.0F, lipo_power = 0.0F;    //CO1浓度, 空气质量， 锂电池电量
    rt_uint16_t air, co;
    rt_uint8_t lipo;

    MQ_StartSample();       //气体传感器开始采样
    while(1)
    {
        /** 换算成锂电池电量百分比  **/
        if(lipo_co == LIPO_COUNT)       //求平均值
        {
            lipo_co = 0;
            lipo_power = 0;
            for(uint8_t i=0;i<LIPO_COUNT;i++)
            {
               lipo_power += lipo_value[i];
            }
            lipo_power /= LIPO_COUNT;
            lipo_power *= 100;
            /** 换算成百分比 **/
            co1_potency = (float)adc_value[0] / 4095.0 * 1000;
            air_quality = (float)adc_value[1] / 4095.0 * 1000;

            air = (rt_uint16_t)air_quality;
            co = (rt_uint16_t)co1_potency;
            lipo = (rt_uint8_t)lipo_power;

            rt_mb_send(air_mb, (rt_ubase_t)&air);
            rt_mb_send(co_mb, (rt_ubase_t)&co);
            rt_mb_send(lipo_mb, (rt_ubase_t)&lipo);
//            rt_kprintf("%d %d %d\r\n", send[0], send[1], send[2]);
        }
        else
        {
            lipo_value[lipo_co++] = ((((float)adc_value[2] / 4095.0) * 3.3) -  2.92041) / (3.3 - 2.92041);
        }

        rt_thread_mdelay(50);
    }
}

static void encoder_thread_entry(void* par)
{
    while(1)
    {
        ENCODER_CaculateSpeed(&now_speed[0], &now_speed[1], &now_speed[2], &now_speed[3]);
        rt_sem_release(speed_sem);
        rt_thread_mdelay(100);
    }
}

static void motor_thread_entry(void* par)
{
//    int32_t temp[4];
    static int8_t L_speed = 0, R_speed = 0;

    while(1)
    {
        rt_sem_take(speed_sem, RT_WAITING_FOREVER);
        rt_mq_recv(motor_L_mq, &L_speed, 1, RT_WAITING_NO);
        rt_mq_recv(motor_R_mq, &R_speed, 1, RT_WAITING_NO);
        MOTOR_IncreasePID(&LF_PID, now_speed[0], L_speed);
        MOTOR_IncreasePID(&RF_PID, now_speed[1], R_speed);
        MOTOR_IncreasePID(&LB_PID, now_speed[2], L_speed);
        MOTOR_IncreasePID(&RB_PID, now_speed[3], R_speed);

        if(L_speed == 0)
        {
            MOTOR_LF_Control(0);
            MOTOR_LB_Control(0);
        }
        else
        {
            MOTOR_LF_Control(LF_PID.output);
            MOTOR_LB_Control(LB_PID.output);
        }

        if(R_speed == 0)
        {
            MOTOR_RF_Control(0);
            MOTOR_RB_Control(0);
        }
        else
        {
            MOTOR_RF_Control(RF_PID.output);
            MOTOR_RB_Control(RB_PID.output);
        }




//        rt_kprintf("L = %d   R = %d\r\n", L_speed, R_speed);
//        rt_kprintf("speed = ");
//
//        for(uint8_t i=0;i<4;i++)
//        {
//            temp[i] = (int32_t)(now_speed[i]);
//            rt_kprintf("%d  ", temp[i]);
//        }
//
//        rt_kprintf("\r\n");

    }
}

static void led_thread_entry(void* par)
{
    while(1)
    {
        LED_ON();
        rt_thread_mdelay(300);
        LED_OFF();
        rt_thread_mdelay(300);

    }
}

static void gy52z_thread_entry(void* par)
{
    int16_t x, y, z;
    int8_t x_e, y_e, z_e;
    static int16_t last_x = 0, last_y = 0, last_z = 0;
    static rt_uint8_t gy52z_send[3];        //WIFI发送数组

    while(1)
    {
        GY25Z_Calculate();
        x = (int16_t)(GY25Z_Data.PITCH);
        y = (int16_t)(GY25Z_Data.ROLL);
        z = (int16_t)(GY25Z_Data.YAW);

        x_e = last_x - x;
        y_e = last_y - y;
        z_e = last_z - z;

        gy52z_send[0] = x_e;
        gy52z_send[1] = y_e;
        gy52z_send[2] = z_e;

        last_x = x;
        last_y = y;
        last_z = z;

        WIFI_AUX_Waiting();
        rt_sem_take(wifi_sem, RT_WAITING_FOREVER);
        WIFI_SendData(gy52z_send, 3, 0xCC);
        rt_sem_release(wifi_sem);
//        rt_kprintf("x = %d, y = %d, z = %d\r\n", gy52z_send[0], gy52z_send[1], gy52z_send[2]);
        rt_thread_mdelay(50);
    }
}

static void camera_thread_entry(void* par)
{
    rt_uint8_t rec_ok = 0;
    rt_uint16_t status = 0;
    rt_uint16_t jpeg_co = 0;
    rt_uint8_t connect_flag = 0;
    rt_uint16_t jpeg_len = 0;
    rt_uint8_t jpeg_send_len[4] = {0};
    DCMI_DMA_Init((uint32_t) &jpeg_buf, JPEG_BUF_SIZE, DMA_MDATAALIGN_BYTE, DMA_MINC_ENABLE);//DCMI DMA配置
    OV2640_OutSize_Set(320, 220);//设置输出尺寸
    DCMI_Start();           //开始获取一帧数据

    M8266WIFI_SPI_Setup_Connection(CONNECTION_TYPE, LOCAL_PORT, REMOTE_ADDR, REMOTE_PORT, LINK_NO, 0, RT_NULL);

    while(1)
    {
        M8266WIFI_SPI_Query_Connection(LINK_NO, CONNECTION_TYPE, &connect_flag, RT_NULL, RT_NULL, RT_NULL, RT_NULL);
        if(connect_flag != 3 && connect_flag != 4)
        {
            M8266WIFI_SPI_Disconnect_Connection(LINK_NO, RT_NULL);
            M8266WIFI_SPI_Setup_Connection(CONNECTION_TYPE, LOCAL_PORT, REMOTE_ADDR, REMOTE_PORT, LINK_NO, 0, RT_NULL);
            rt_thread_mdelay(1000);
        }

        /* 等待一帧图片传输完成 */
        rt_sem_take(jpeg_send_sem, RT_WAITING_FOREVER);

        /** 检测到帧开头,jpeg_co为帧开头数组偏移量 **/
        jpeg_co = 0;
        while(jpeg_buf[jpeg_co] != 0xFF || jpeg_buf[jpeg_co+1] != 0xD8)
        {
            jpeg_co++;
        }

        /** 计算图片字节大小 **/
//        jpeg_len = jpeg_co;
//        while((jpeg_buf[jpeg_len] != 0xD9) || (jpeg_buf[jpeg_len-1] != 0xFF))
//        {
//            jpeg_len++;
//        }
//
//        jpeg_len++;
//        jpeg_len -= jpeg_co;
        jpeg_len = 1024 * 8;

        jpeg_send_len[0] = 0xFF;
        jpeg_send_len[1] = 0xFF;
        jpeg_send_len[2] = (rt_uint8_t)(jpeg_len >> 8);
        jpeg_send_len[3] = (rt_uint8_t)jpeg_len;

        M8266WIFI_SPI_Send_Data(jpeg_send_len, 4, LINK_NO, &status);
        M8266WIFI_SPI_Send_BlockData(jpeg_buf+jpeg_co, jpeg_len, 5, LINK_NO, RT_NULL, RT_NULL, RT_NULL);

        while(rec_ok != 0xAB)
        {
            rt_thread_mdelay(20);
            M8266WIFI_SPI_RecvData(&rec_ok, 1, 5, RT_NULL, RT_NULL);
        }

        jpeg_data_ok = 2;           //标记数据处理完成
//        rt_thread_mdelay(100);

    }


}


void jpeg_data_process(void)
{
    if(jpeg_data_ok==0) //jpeg数据还未采集完?
    {
        __HAL_DMA_DISABLE(&DMADMCI_Handler);//关闭DMA
        while(DMA2_Stream1->CR&0X01);   //等待DMA2_Stream1可配置
        //jpeg_data_len=JPEG_BUF_SIZE-__HAL_DMA_GET_COUNTER(&DMADMCI_Handler);//得到剩余数据长度
        jpeg_data_ok=1;                 //标记JPEG数据采集完按成,等待其他函数处理
        rt_sem_release(jpeg_send_sem);      //释放信号量
    }
    if(jpeg_data_ok==2) //上一次的jpeg数据已经被处理了
    {
        __HAL_DMA_SET_COUNTER(&DMADMCI_Handler,JPEG_BUF_SIZE);//传输长度为jpeg_buf_size字节
        __HAL_DMA_ENABLE(&DMADMCI_Handler); //打开DMA
        jpeg_data_ok=0;                     //标记数据未采集
    }
}




MSH_CMD_EXPORT(led_thread_entry, "led_thread");
MSH_CMD_EXPORT(camera_thread_entry, "camera_thread");
MSH_CMD_EXPORT(motor_thread_entry, "motor_thread");
MSH_CMD_EXPORT(gy52z_thread_entry, "gy52z_thread");
MSH_CMD_EXPORT(mq_sensor_thread_entry, "mq_sensor_thread");
MSH_CMD_EXPORT(dht11_thread_entry, "dht11_thread");
MSH_CMD_EXPORT(servo_f_thread_entry, "servo_f_thread");
MSH_CMD_EXPORT(servo_b_thread_entry, "servo_b_thread");
MSH_CMD_EXPORT(wifi_thread_entry, "wifi_thread");
MSH_CMD_EXPORT(wifi_send_thread_entry, "wifi_send_thread");
