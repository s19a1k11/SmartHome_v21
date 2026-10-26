#include "delay.h"

// 系统毫秒计时，由 SysTick 中断每 1ms 累加一次
static volatile u32 delay_sys_ms = 0;

/**
 *  @brief 延时模块初始化
 *
 *  之前 delay_us 里写死 72 个 NOP 且没有用到参数，导致 delay_ms(2000)
 *  实际只有 2ms 左右，ESP8266 的 AT 流程全部收不到应答。这里改成以
 *  SysTick 为时基的实现，SysTick 计数仍然可以在中断之外读取。
 */
void Delay_Init(void)
{
    // 1. 先停掉 SysTick，避免配置过程中产生额外中断
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    // 2. 选择 SysTick 时钟源为 HCLK（72 MHz），这样 1ms = 72000 个周期
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

    // 3. 配置重装载值并开启计数与中断
    SysTick_Config(SystemCoreClock / 1000);

    // 4. 计数从 0 开始，保证第一次延时不至于偏短
    SysTick->VAL = 0;

    delay_sys_ms = 0;
}

/**
 *  @brief 获取系统毫秒计时
 */
u32 Delay_GetMs(void)
{
    return delay_sys_ms;
}

/**
 *  @brief 毫秒级延时
 */
void delay_ms(u32 ms)
{
    u32 start = delay_sys_ms;

    // 用差值判断，天然兼容 32 位计时回绕
    while ((delay_sys_ms - start) < ms)
    {
        // 等待节拍累加到位
    }
}

/**
 *  @brief 微秒级延时
 *
 *  直接读 SysTick 的当前计数值做差，不依赖中断，精度到微秒级。
 *  SysTick 是向下计数器，取值要用重装值把跨回绕的一段补回来。
 */
void delay_us(u32 us)
{
    u32 ticks = 0;
    u32 told = 0;
    u32 tnow = 0;
    u32 reload = 0;
    u32 cnt = 0;

    if (us == 0)
    {
        return;
    }

    ticks = us * (SystemCoreClock / 1000000UL);
    if (ticks > 0xFFFFFFUL)
    {
        ticks = 0xFFFFFFUL;
    }

    reload = SysTick->LOAD;
    told = SysTick->VAL;

    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                cnt += told - tnow;
            }
            else
            {
                // 发生了重装载，补上跨过去的那一段
                cnt += reload - tnow + told;
            }
            told = tnow;

            if (cnt >= ticks)
            {
                break;
            }
        }
    }
}

/**
 *  @brief SysTick 中断处理函数
 *
 *  每 1ms 触发一次，只做毫秒计数，不做任何耗时操作。
 *  stm32f10x_it.c 里原来的空 SysTick_Handler 已删除，避免重复定义。
 */
void SysTick_Handler(void)
{
    delay_sys_ms += 1;
}
