#ifndef _TIM_H
#define _TIM_H

#include "common.h"

/**
*	@brief TIM6 基础定时器初始化函数，要求用户提供
*				 1. PSC 预分频器倍数
*				 2. ARR 自动重装载寄存器数据
*				 psc : 预分频器倍数，数据范围在 u16 范围以内 (0~65535)
*				 arr : 自动重装寄存器数据，数据范围同上
*/
 void TIM6_Init(u16 psc,u16 arr);

/* 【卡6】毫秒计数器, 定义在 TIM.c, 上电后每 1ms +1 */
extern volatile u32 g_ms;

#endif


