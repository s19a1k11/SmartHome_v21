#include "Led.h"

/*
	板载 LED: LED0 -> PB5, LED1 -> PE5
	-----------------------------------------------------------------
	硬件参数: 推挽输出 / 2MHz / 上电输出高电平 = 熄灭
	(本板 LED 是低电平点亮, 所以"高电平"对应"灭")。
*/
void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* 1. 开 GPIOB + GPIOE 时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOE, ENABLE);

	/* 2. PB5 和 PE5 都是推挽输出, 2MHz, 所以共用同一个结构体 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	/* 3. 上电先输出高电平 -> 两盏灯都灭 */
	GPIO_SetBits(GPIOB, GPIO_Pin_5);
	GPIO_SetBits(GPIOE, GPIO_Pin_5);
}

/*
	按语义控制 LED0: 1 = 亮 / 0 = 灭
	★ 极性转换只在这里做一次:
	    亮 -> 引脚拉低(GPIO_ResetBits)
	    灭 -> 引脚拉高(GPIO_SetBits)
	  这样 ModBus_Set_Reg / 本地直控 / 以后加的按键 都只需要说"亮/灭",
	  谁也不用记住"低电平点亮"这件事。
*/
void Led_Set_OnOff(u8 on)
{
	if(on)
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);   /* 拉低 = 亮 */
	}
	else
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_5);     /* 拉高 = 灭 */
	}
}

/* flag = 1 -> 输出高电平, flag = 0 -> 输出低电平
   (注意: 这里的 1 是"引脚高", 不是"灯亮"; 本板灯是低电平亮) */
void Led0_Ctrl(u8 flag)
{
	if(flag)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_5);
	}
	else
	{
		GPIO_ResetBits(GPIOB, GPIO_Pin_5);
	}
}

void Led1_Ctrl(u8 flag)
{
	if(flag)
	{
		GPIO_SetBits(GPIOE, GPIO_Pin_5);
	}
	else
	{
		GPIO_ResetBits(GPIOE, GPIO_Pin_5);
	}
}
