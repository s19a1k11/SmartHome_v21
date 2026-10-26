#include "all.h"

void GPIO_Init_One(uint32_t RCC_GPIOx,GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin,GPIOMode_TypeDef GPIO_Mode,GPIOSpeed_TypeDef GPIO_Speed)
{
	RCC_APB2PeriphClockCmd(RCC_GPIOx,ENABLE);
	
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;
	//设置模式
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed;
	//选择设置哪个 GPIO 组
	GPIO_Init(GPIOx, &GPIO_InitStructure);
}

void GPIO_Ctrl_One(GPIO_TypeDef* GPIOx,uint16_t GPIO_Pin,u8 flag)
{
		if (flag)
    {
			//拉高
        GPIO_SetBits(GPIOx, GPIO_Pin);
    }
    else
    {
			//拉低
        GPIO_ResetBits(GPIOx, GPIO_Pin);
    }
}

