#include "all.h"

/*
	舵机 PWM 初始化
	---------------------------------------------------------------------
	为什么不复用"PB5 那套 PWM 调光"写法:
		TIM3 只有一组重映射位(MAPR 的 bit11:10), 同一时刻只能选一种:
			部分重映射 -> CH2 落到 PB5;  不重映射 -> CH2 留在 PA7。
		PB5 是板载 LED0, 谁占了重映射, 另一个就用不了。
		本项目舵机在 PA7, 所以必须是"不重映射";
		灯只做纯开关(0=灭 1=亮), 不再调光, 那套调光代码已从工程移除。

	时基计算:
		72MHz / 72 = 1MHz  -> 计数器每 1us 加 1
		ARR = 20000 - 1    -> 计 20000 次 = 20000us = 20ms = 50Hz
		所以 CCR2 填多少, 高电平就是多少微秒, 不用再换算

	本文件已由寄存器写法改为标准外设库:
		TIM_TimeBaseInit  <- PSC / ARR / 计数方向
		TIM_OC2Init       <- CCMR1 的 OC2M / OC2PE / OC2FE 和 CCER 的 CC2E / CC2P
		TIM_Cmd           <- CR1 的 CEN
*/
void Servo_Init(void)
{
	GPIO_InitTypeDef        GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_OCInitTypeDef       TIM_OCInitStructure;

	/* 1. 开时钟: GPIOA 在 APB2, TIM3 在 APB1, 改重映射还要 AFIO */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	/* 2. PA7 配 [复用推挽输出, 2MHz]
		  复用 = 引脚交给片上外设(TIM3)驱动, 而不是由 ODR 决定电平 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 3. TIM3 不重映射: TIM3_REMAP[1:0] = 00
		  00 -> CH1=PA6  CH2=PA7  CH3=PB0  CH4=PB1
		  用 FullRemap 这个参数 + DISABLE, 等于"把重映射位清成 00" */
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM3, DISABLE);

	/* 4. 时基: 1MHz 计数, 20ms 一圈 */
	TIM_TimeBaseStructure.TIM_Prescaler     = 72 - 1;
	TIM_TimeBaseStructure.TIM_Period        = 20000 - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	/* 5. CH2 做成 PWM 输出
		  TIM_OCMode_PWM1      : CNT < CCR2 时输出有效电平(对应 OC2M = 110)
		  TIM_OutputState_Enable: 打开通道 2 输出(对应 CC2E = 1)
		  TIM_Pulse = 1500     : 上电先给中位 1500us, 免得舵机乱甩
		  TIM_OCPolarity_High  : 高电平有效(对应 CC2P = 0), 所以 CCR 就是脉宽
								 (模板里是 Low, 给 CCR 得反着算)

		  注: 原寄存器版还设了 OC2FE = 1(快速输出使能)。它只在单脉冲模式下
			  影响触发响应, 连续 PWM 输出时用不到, 库函数默认 0 即可, 行为一致。 */
	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse       = 1500;
	TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
	TIM_OC2Init(TIM3, &TIM_OCInitStructure);

	/* 6. 启动 TIM3 */
	TIM_Cmd(TIM3, ENABLE);
}

/*
	设置档位
	---------------------------------------------------------------------
	档位表: 0 -> 500us(0 度)   1 -> 1500us(90 度)   2 -> 2500us(180 度)

	如果舵机到两端还在"吱吱"响(顶到机械限位), 说明这颗 SG90 的实际
	行程没那么宽, 把 500 / 2500 改成 800 / 2200 就行。
*/
void Servo_Set_Angle(u8 idx)
{
	static const u16 pulse[3] = {500, 1500, 2500};

	if(idx > 2) return;

	TIM_SetCompare2(TIM3, pulse[idx]);
}
