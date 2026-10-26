#ifndef _SYSTICK_H
#define _SYSTICK_H

#include "common.h"

/**
 *  @brief 初始化 SysTick 函数
 *
 *  内部转发到 Delay_Init，系统会以 1ms 节拍运行
 */
void SysTick_Init(void);

/**
 *  @brief 利用 SysTick 精准延时函数
 *
 *  @param ms 要延时的毫秒数值
 */
void Delay_ms(u32 ms);

#endif
