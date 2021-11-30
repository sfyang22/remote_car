/********************************************************************
 * M8266WIFI_ops.h
 * .Description
 *     header file of M8266WIFI HAL operations
 * .Copyright(c) Anylinkin Technology 2015.5-
 *     IoT@anylinkin.com
 *     http://www.anylinkin.com
 *     http://anylinkin.taobao.com
 *  Author
 *     wzuo
 *  Date
 *  Version
 ********************************************************************/
 
#ifndef _M8266WIFI_OPS_H_ 
#define _M8266WIFI_OPS_H_

void M8266WIFI_Module_delay_ms(u16 nms);

u8 M8266WIFI_Module_Init_Via_SPI(void);
u8 M8266WIFI_SPI_wait_sta_connecting_to_ap_and_get_ip(char* sta_ip, u8 max_wait_time_in_s);
u8 M8266WIFI_Config_Connection_via_SPI(u8 tcp_udp, u16 local_port, char* remote_addr, u16 remote_port, u8 link_no);

u8 M8266WIFI_Sleep_module_for_some_time_and_then_wakeup_automatically(u16 time_to_wakeup_in_ms);
u8 M8266WIFI_Sleep_Module(void);
u8 M8266WIFI_Wakeup_Module(void);

#endif
