#include "sr602.h"
#include "delay.h"

/*
	SR602 人体红外模块驱动 (主机侧, EXTI 双边沿中断)
	==================================================================
	引脚和极性全部由 sr602.h 的宏决定, 本文件不出现任何具体引脚名。
	这样换板子/换引脚只改头文件那几行, 逻辑一行不动。

	★ 中断里只做两件事: 记电平 + 记时刻 + 置标志。
	  所有判断(夜晚判定/延时关灯/该开哪盏灯)都在主循环里做。
	  这是"快进快出"的直接体现 —— ISR 执行时间是常数级(几微秒)。
*/

/* 中断里写、主循环读的共享状态, 必须 volatile */
static volatile u8  s_level    = 0;     /* 当前"有没有人"(已归一化极性) */
static volatile u8  s_changed  = 0;     /* 1 = 自上次被取走后变过 */
static volatile u32 s_edge_ms  = 0;     /* 最近一次变化的毫秒时刻 */

void SR602_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	EXTI_InitTypeDef EXTI_InitStructure = {0};
	NVIC_InitTypeDef NVIC_InitStructure = {0};

	/* 1. 使能 GPIO 组时钟 + AFIO 时钟(EXTI 线映射必须开 AFIO) */
	RCC_APB2PeriphClockCmd(SR602_RCC_GPIO_CLK | RCC_APB2Periph_AFIO, ENABLE);

	/* 2. 引脚配成上拉输入
	      上拉的作用: 模块没插/线断时引脚被内部电阻拉到高电平,
	      读数是确定的高, 而不是随机跳变 —— 故障可复现才好查 */
	/*
		★ 必须用【下拉输入】而不是上拉(v21 修)。

		踩过的坑: 原来配的是 GPIO_Mode_IPU(上拉)。
		战神V1 的 PE2 是 KEY2 按键脚, 板载本身就有上拉电阻。
		当 SR602 模块【没插】时, 引脚被上拉到高电平 ->
		SR602_Read() 返回 1 -> 上层误判"一直有人" ->
		夜晚时光照一低于阈值就会开灯, 而且永远不关
		(因为"人"一直在), 灯就这么亮一整夜。

		改下拉之后: 模块没插/线断 -> 读到低电平 -> "无人" ->
		不会误开灯。这是安全的失败方向。
		SR602 的 OUT 是推挽输出, 内部下拉电阻(约40k)不影响它驱动高电平。
	*/
	GPIO_InitStructure.GPIO_Pin  = SR602_GPIO_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(SR602_GPIO_PORT, &GPIO_InitStructure);

	/* 3. 把该引脚映射到 EXTI 线
	      注意: 同一个 EXTI 线号在所有端口里只能选一个。
	      PE2 -> EXTI2, 所以其它端口的 Pin_2 就不能再用 EXTI 了。 */
	GPIO_EXTILineConfig(SR602_EXTI_PORT_SRC, SR602_EXTI_PIN_SRC);

	/* 4. EXTI 配双边沿触发
	      上升沿 = 有人来了, 下降沿 = 人走了, 两个都要 */
	EXTI_InitStructure.EXTI_Line    = SR602_EXTI_LINE;
	EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/*
		5. NVIC: 抢占 2 / 子 0

		★ 为什么排在"接收"之后、"发送"之前:
		  按需求: 收 > 发, SR602 次之。
		  更具体的理由: USART3/USART2 的接收晚一步就丢字节(硬实时),
		  而 SR602 的边沿晚几十微秒完全无所谓(人体信号持续几百毫秒以上)。
		  但 SR602 又要比"打拍子"和"调试打印"重要 —— 漏掉一次边沿
		  可能就漏掉一次"有人来了", 而节拍晚一点、日志晚一点毫无影响。
	*/
	NVIC_InitStructure.NVIC_IRQChannel = SR602_EXTI_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* 6. 初始化状态: 读一次当前电平, 别让上电瞬间误判成"有人来了" */
	s_level   = SR602_Read();
	s_changed = 0;
	s_edge_ms = Delay_GetMs();
}

u8 SR602_Read(void)
{
	u8 level = 0;

	level = GPIO_ReadInputDataBit(SR602_GPIO_PORT, SR602_GPIO_PIN) ? 1 : 0;

	/*
		极性归一: 把"引脚电平"翻成"有没有人"。
		这样上层看到的永远是 1 = 有人,
		换模块极性时只改宏, 云端物模型和自动开灯逻辑都不用动。
	*/
	/*
		★ 用 #if 而不是 if: SR602_ACTIVE_LEVEL 是编译期常量,
		  写 if 会让"极性翻转"那半句变成死代码, 编译器报
		  warning #111-D: statement is unreachable。
		  用 #if 之后, 编译出来的代码只有一条 return, 没有死分支。
	*/
#if (SR602_ACTIVE_LEVEL)
	return level;
#else
	return (u8)(level ? 0 : 1);
#endif
}

/*
	EXTI2 中断 —— 快进快出
	==================================================================
	只做: 清标志 -> 读电平 -> 记时刻 -> 置变化标志。
	★ 这里【绝对不许】调用 ModBus 写寄存器 / 打日志 / 做判断:
	  ModBus 写寄存器要发 8 字节 @9600 = 8.3ms, 在中断里做会把
	  485 接收彻底卡死; 打日志是阻塞的, 更不行。
	  所以中断只"报告事实", 决策交给主循环。
*/
void EXTI2_IRQHandler(void)
{
	if(EXTI_GetITStatus(SR602_EXTI_LINE) != RESET)
	{
		/* 先清标志, 避免处理期间同一个边沿又被挂起 */
		EXTI_ClearITPendingBit(SR602_EXTI_LINE);

		/* 读当前电平(双边沿都要读, 因为不知道这次是上升还是下降) */
		s_level   = SR602_Read();
		s_edge_ms = Delay_GetMs();
		s_changed = 1;
	}
}

u8 SR602_Take_Changed(void)
{
	u8 c = 0;

	/* 关中断读改写: s_changed 在 ISR 里被置位 */
	__disable_irq();
	c = s_changed;
	s_changed = 0;
	__enable_irq();

	return c;
}

u32 SR602_Last_Edge_Ms(void)
{
	return s_edge_ms;
}

/*
	主循环每轮读一次引脚, 更新缓存
	★ 为什么要有缓存:
	  上报云端、自动开灯判断都在同一个循环里跑。如果每次判断都重新读引脚,
	  人正好在这几十毫秒里走动的话, 前后两次判断会读到不同值,
	  逻辑就自相矛盾(比如"判定有人要开灯"和"上报无人"同时发生)。
	  所以主循环开头读一次, 之后全用缓存值。
*/
void SR602_Sample(void)
{
	u8 now = SR602_Read();

	/* 缓存值跟着引脚走(不依赖中断, 防止中断被关掉时状态不更新) */
	if(now != s_level)
	{
		s_level   = now;
		s_edge_ms = Delay_GetMs();
		s_changed = 1;
	}
}

u8 SR602_Current(void)
{
	return s_level;
}
