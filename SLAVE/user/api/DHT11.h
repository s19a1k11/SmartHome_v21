#ifndef _DHT11_H
#define _DHT11_H

#include "common.h"

/*
	需要统一具体使用哪个引脚做 DHT11 传感器 IO 引脚
	负责从板部分的自行商榷
*/

//初始化函数
void GPIO_E4_Init(void);

//控制 GPIO 引脚
void GPIO_E4_Output(void);

//控制 GPIO 引脚
void GPIO_E4_Input(void);

//发送准备接收信号
void DHT11_Start_Signal(void);

//接收 DHT11 返回的信号
u8 DHT11_Response(void);

//读取
u8 DHT11_Read_Bit(void);

u8 DHT11_Read_Byte(void);

u8 DHT11_Read_Data(u8 *Integer,u8 *fractional,u8 *Integer_f,u8 *fractional_f);

#endif

