#ifndef _DELAY_H
#define _DELAY_H

#include "common.h"

/**
 *  @brief 延时模块初始化，配置 SysTick 为 1ms 节拍并开启中断
 *
 *  必须在 main 函数开头调用一次。delay_ms / delay_us 都依赖这个时基，
 *  没有初始化就调用会导致程序死等。
 */
void Delay_Init(void);

/**
 *  @brief 获取系统上电后的毫秒计时
 *
 *  @return 自 Delay_Init 完成起的毫秒数，32 位约 49.7 天回绕一次，
 *          外部用【当前值 - 上次值】的方式做差即可兼容回绕
 */
u32 Delay_GetMs(void);

/**
 *  @brief 毫秒级延时
 *
 *  @param ms 要延时的毫秒数值
 */
void delay_ms(u32 ms);

/**
 *  @brief 微秒级延时
 *
 *  @param us 要延时的微秒数值
 */
void delay_us(u32 us);

#endif
