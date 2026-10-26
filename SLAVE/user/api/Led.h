#ifndef _LED_H
#define _LED_H

#include "common.h"

/**
	Led 模块初始化，需要两个 IO 引脚， 对应
	PB5 引脚控制 DSO/LED0
	PE5 引脚控制 DSO/LED1
	两个引脚都为 [通用推挽输出模式,速率 2 MHz]
							 [默认电平为高电平模式]
*/
void Led_Init(void);

/**
	控制 LED0 的亮灭 —— 按【语义】而不是按电平

	★ 这是本工程唯一推荐给外部调用的 LED 接口。
	  为什么不用 Led0_Ctrl(1) 表示"亮":
	    本板 LED 是低电平点亮, 引脚高 = 灯灭, 引脚低 = 灯亮。
	    调用方写 Led0_Ctrl(1) 时脑子里想的往往是"开灯", 实际却关灯,
	    这个极性反了几乎每个项目都要踩一次。
	  所以这里把极性彻底封在本函数内部, 外部只说"亮/灭"。

	on = 1 -> 灯亮(内部拉低 PB5)
	on = 0 -> 灯灭(内部拉高 PB5)
*/
void Led_Set_OnOff(u8 on);

/**
	控制灯的亮灭
	flag 为 1 -> 引脚输出高电平 (本板 = 灯灭)
	flag 为 0 -> 引脚输出低电平 (本板 = 灯亮)

	★ 这是底层"引脚电平"接口, 极性容易搞反, 业务代码请用 Led_Set_OnOff
*/
void Led0_Ctrl(u8 flag);

/**
	控制 LED1
	flag 为 1 -> 引脚输出高电平 (本板 = 灯灭)
	flag 为 0 -> 引脚输出低电平 (本板 = 灯亮)
*/
void Led1_Ctrl(u8 flag);

#endif
