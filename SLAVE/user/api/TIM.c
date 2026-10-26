#include "all.h"

/* 【卡6】毫秒计数器: TIM6 每 1ms 中断一次就 +1
   volatile 是告诉编译器"这变量会被中断偷偷改", 每次用都必须去内存重新取值,
   不许优化进寄存器缓存, 否则主循环会永远读到 0 */
volatile u32 g_ms = 0;

/**
*   @brief TIM6 基础定时器初始化函数, 要求用户提供
*          1. PSC 预分频器倍数
*          2. ARR 自动重装载寄存器数据
*          psc : 预分频器倍数, 数据范围在 u16 范围以内 (0~65535)
*          arr : 自动重装寄存器数据, 数据范围同上
*
*   原来这里是直接写 TIM6->PSC / ARR / CR1 / DIER,
*   现在换成 TIM_TimeBaseInit + TIM_ITConfig + TIM_Cmd。
*   分频和周期一个没变: PSC = psc-1, ARR = arr-1, 向上计数, 不缓冲。
*   调用 TIM6_Init(72, 1000) 时: 72MHz/72 = 1MHz -> 数 1000 次 = 1ms
*/
void TIM6_Init(u16 psc, u16 arr)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

	/* 1. 开 TIM6 时钟 (TIM6 挂在 APB1 上) */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);

	/* 2. 时基: 预分频 + 周期 + 计数方向
		  TIM_CKD_DIV1    : 时钟不分频, 直接进计数器(和原来 CR1 复位值一致)
		  TIM_CounterMode_Up : 向上计数 */
	TIM_TimeBaseStructure.TIM_Prescaler     = psc - 1;
	TIM_TimeBaseStructure.TIM_Period        = arr - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	/* 3. 开更新中断(计满一圈产生一次) */
	TIM_ITConfig(TIM6, TIM_IT_Update, ENABLE);

	/* 4. NVIC: 优先级 2, 并允许 TIM6 中断进 CPU */
	NVIC_SetPriority(TIM6_IRQn, 2);
	NVIC_EnableIRQ(TIM6_IRQn);

	/* 5. 启动定时器 */
	TIM_Cmd(TIM6, ENABLE);
}

void TIM6_IRQHandler(void)
{
	/* 先确认确实是"更新"中断, 再清标志, 最后干活
	   清标志必须在干活之前: 否则 g_ms++ 期间又来一次中断会被吃掉 */
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)
	{
		TIM_ClearITPendingBit(TIM6, TIM_IT_Update);

		g_ms++;     /* 1ms 到, 毫秒计数 +1 (中断里只干最短的事) */
	}
}
