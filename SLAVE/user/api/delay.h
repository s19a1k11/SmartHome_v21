#ifndef _DELAY_H
#define _DELAY_H

#include "common.h"

/**
	延时控制函数,控制时间单位为 us 微秒
	u32 us 为用户提供的延时控制微秒数
*/
void delay_us(u32 us);

/**
	延时控制函数,控制时间单位为 ms 毫秒
	u32 用户提供的控制延时的毫秒数
*/
void delay_ms(u32 ms);

#endif

