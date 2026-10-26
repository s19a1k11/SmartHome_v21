#include "beep.h"

/*
	蜂鸣器: PB8, 高电平响
	-----------------------------------------------------------------
	原写法是直接改 GPIOB->CRH / ODR, 现在改成标准外设库函数。
	硬件参数一个没变: 通用推挽输出 / 2MHz / 上电默认低电平(不响)。
	beep 在 PB8, CRH 管的是 PB8~PB15; 原代码写 GPIOB->CRH &= ~(0x0F)
	恰好就是 PB8 那 4 位(没移位是因为 PB8 正好在最低位)。
*/
void Beep_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 1. 开 GPIOB 时钟 (APB2) */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	/* 2. PB8 配成推挽输出, 2MHz */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 3. 默认低电平 -> 蜂鸣器停止工作 */
	GPIO_ResetBits(GPIOB, GPIO_Pin_8);
}

void Beep_Ctrl(u8 flag)
{
	if(flag)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_8);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_8);
	}
}
