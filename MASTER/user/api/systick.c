#include "systick.h"
#include "delay.h"

/**
 *  @brief 初始化 SysTick 函数
 *
 *  说明：SysTick 资源已经由 delay.c 统一管理（1ms 节拍 + 系统毫秒计时），
 *        这里不再自己配置寄存器，直接转发到 Delay_Init，避免两处同时
 *        配置 SysTick 导致节拍被改掉。Delay_ms 也统一走 Delay_GetMs。
 */
void SysTick_Init(void)
{
    Delay_Init();
}

/**
 *  @brief 毫秒级阻塞延时
 *
 *  @param ms 要延时的毫秒数值
 */
void Delay_ms(u32 ms)
{
    u32 start = Delay_GetMs();

    while ((Delay_GetMs() - start) < ms)
    {
        // 等待系统节拍累加到位
    }
}
