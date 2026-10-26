#ifndef GPIO_INIT_ONE_H
#define GPIO_INIT_ONE_H

#include "common.h"

/*
	根据传入的 GPIO 组和 GPIO 引脚号 进行 初始化
	uint32_t RCC_GPIOx 使能哪个 APB2 的 GPIO 组
	GPIO_TypeDef* GPIOX GPIO 组
	uint16_t GPIO_Pin 对应几号引脚
	GPIOMode_TypeDef GPIO_Mode 工作模式
	GPIOSpeed_TypeDef GPIO_Speed 输出速率
*/
void GPIO_Init_One(uint32_t RCC_GPIOx,GPIO_TypeDef* GPIOX,uint16_t GPIO_Pin,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed);

/*
	对应 GPIO 的控制
	GPIO_TypeDef* GPIOx GPIO 组
	uint16_t GPIO_Pin 对应 GPIO 引脚
*/
void GPIO_Ctrl_One(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,u8 flag);


#endif
