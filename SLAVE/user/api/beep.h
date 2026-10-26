#ifndef _BEEP_H
#define _BEEP_H

#include "common.h"

//控制 Beep 工作状态反转
#define Beep_Toggle GPIOB->ODR ^= (0x01<<8)

/**
	Beep 蜂鸣器模块初始化， 配置引脚为 PB8 引脚，
			 工作模式为 [通用推挽输出模式,速率 2MHz]
*/
void Beep_Init(void);

void Beep_Ctrl(u8 flag);

#endif

