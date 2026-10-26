#include "tick.h"

/*
	主机轮询节拍定时器 (TIM4) —— 1 秒周期
	==================================================================
	中断里【只做三件常数级的事】:
	    1) 清更新中断标志
	    2) tick_cnt++          累计秒数
	    3) 置 poll_due 标志    告诉主循环"该轮询了"

	★ 为什么不在这里直接轮询:
	  轮询三台从机 = 9 笔事务, 正常约 90ms, 最坏(三台全离线)约 600ms。
	  放在中断里会让主循环彻底停摆: MQTT 心跳发不出去(链路被踢),
	  云端下发的指令也处理不了。中断只负责"打拍子", 干活留给主循环。

	★ missed 计数:
	  poll_due 是"有没有到点"的布尔量, 如果主循环被阻塞了 30 秒,
	  这 30 个节拍会合并成 1 个 poll_due=1, 信息就丢了。
	  所以额外用 missed 记录"上一拍处理完到现在又过了几拍",
	  主循环据此判断自己是否错过了轮询窗口。
*/

static volatile u32 s_tick_cnt = 0;     /* 上电以来的节拍数(秒) */
static volatile u8  s_poll_due = 0;     /* 1 = 有节拍到点未处理 */
static volatile u16 s_missed   = 0;     /* 到点后又过了几拍(说明主循环慢) */

void Tick_Init(void)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure = {0};
	NVIC_InitTypeDef NVIC_InitStructure = {0};

	/* 1. 开 TIM4 时钟 (TIM4 挂在 APB1 上) */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	/* 2. 先关掉再配置(有些参数运行时不能改) */
	TIM_Cmd(TIM4, DISABLE);

	/*
		3. 时基:
		   APB1 = 36MHz, 但 APB1 预分频 != 1 时定时器时钟 = 2 x PCLK1 = 72MHz
		   PSC = 7200-1  -> 72MHz / 7200 = 10kHz, 即 1 格 = 100us
		   ARR = 10000-1 -> 数 10000 格 = 1 秒
	*/
	TIM_TimeBaseStructure.TIM_Prescaler     = 7200 - 1;
	TIM_TimeBaseStructure.TIM_Period        = (TICK_PERIOD_MS * 10) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/* 4. 开更新中断 */
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

	/*
		5. NVIC: 抢占 3 / 子 0 (最低)

		★ 为什么定时器节拍放最低:
		  它只是"打拍子", 晚几毫秒、甚至晚几十毫秒都毫无影响 ——
		  主循环看到 poll_due=1 之后才去轮询, 节拍本身的抖动
		  不会影响任何时序。而 USART2/USART3 的接收中断晚一步就丢字节,
		  SR602 的边沿晚一点就漏掉"有人来了"。
		  所以它排在所有中断的最后。

		完整的主机优先级表(见 main.c 顶部注释):
		  USART3 MQTT 接收  抢占 0  最高
		  USART2 Modbus 接收 抢占 1
		  EXTI2  SR602      抢占 2
		  TIM4   轮询节拍    抢占 3  最低
		  USART1 调试回显    抢占 3  最低(同级, 靠子优先级排后面)
	*/
	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* 6. 清零并启动 */
	s_tick_cnt = 0;
	s_poll_due = 0;
	s_missed   = 0;
	TIM_SetCounter(TIM4, 0);
	TIM_Cmd(TIM4, ENABLE);
}

void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		/* 先清标志再干活: 否则干活期间又溢出一次会被吃掉 */
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);

		s_tick_cnt += 1;

		if(s_poll_due == 0)
		{
			/* 上一拍已经被主循环取走了, 这一拍是正常的 */
			s_poll_due = 1;
		}
		else
		{
			/*
				上一拍还没被取走, 说明主循环被阻塞了超过一个周期。
				这里只记"错过了几拍", 不再重复置标志。
				★ 计数上限: 主循环最长阻塞是 MQTT 建链的几十秒,
				  0xFFFF 足够, 到顶就不再涨(防回绕)。
			*/
			if(s_missed < 0xFFFF)
			{
				s_missed += 1;
			}
		}
	}
}

u8 Tick_Take(void)
{
	u8 due = 0;

	/* 关中断读改写: 这组变量在 ISR 里被改, 不加保护可能读到中间态 */
	__disable_irq();
	due = s_poll_due;
	s_poll_due = 0;
	__enable_irq();

	return due;
}

u16 Tick_Take_Missed(void)
{
	u16 m = 0;

	__disable_irq();
	m = s_missed;
	s_missed = 0;
	__enable_irq();

	return m;
}

u32 Tick_Count(void)
{
	return s_tick_cnt;
}
