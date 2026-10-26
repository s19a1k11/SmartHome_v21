#ifndef _KEY_H
#define _KEY_H

#include "common.h"
#include "delay.h"

/*
利用枚举类型声明告示当前按键对应的数据编号。
*/
typedef enum
{
	KEY_UP_PRESSED,
	KEY_2_PRESSED,
	KEY_1_PRESSED,
	KEY_0_PRESSED,
	NO_KEY_PRESSED
} Key_Value;

/**
 * @brief 按键模块初始化函数，需要配置的内容有
 *		KEY_UP ==> PA0 对应工作模式以为【下拉输入模式】
 *	    KEY2 ==> PE2 对应工作模式以为【上拉输入模式】
 *	    KEY1 ==> PE3 对应工作模式以为【上拉输入模式】 
 *	    KEY0 ==> PE4 对应工作模式以为【上拉输入模式】
 */
void Key_Init(void);

/**
 * @brief 用于反馈当前按键模块哪一个按键被按下。
 *
 * @return 返回类型对应 Key_Value 声明的枚举内容
 */
Key_Value Key_Press(void);
	

#endif


