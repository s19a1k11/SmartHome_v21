#ifndef _USART3_H
#define _USART3_H

#include "common.h"
#include "type.h"

/*
	USART3 模块初始化函数，要求用户提供对应工作波特率
	Tx ==> PB10 [复用推挽输出模式]
	Rx ==> PB11 [浮空输入模式]
	★ 不要照着 PA9/PA10 接: 那是 USART1(串口助手那个口)。
	NRZ 数据格式为 8N1 模式

	【本版说明: 从机固件不再用 USART3 上云】
	  上云(ESP8266 + MQTT)是主机的事, 从机只做 485 从站。
	  这里保留 USART3 有两个用途:
	    1) 现场调试: 接 USB-TTL 到 PB11, 用 SLAVE_USART3_LOCAL_CMD 打开
	       本地直控, 不接主机也能验证灯/蜂鸣器/舵机/传感器;
	    2) 以后要加"本地屏"或第二路通信时, 驱动是现成的。

	  ★ 中断已按快进快出改写: 只收字节 + 置标志, 解析全部放到主循环。
	    改造前是在中断里直接 strstr + 调 ModBus_Set_Reg,
	    9600/115200 下解析几十字节要几十微秒到几百微秒,
	    期间 USART2(485) 的字节会丢 —— 那是"偶发 CRC 错"的真正来源。
*/
void USART3_Init(u32 brr);

/*
	USART3 模块发送一个字节数据到接收端
	byte 需要进行发送的一个字节数据
*/
void USART3_SendByte(u8 byte);

/*
	USART3 模块指定字节个数数据内容到接收端
	buffer 发送数据对应内存空间首地址
	len 	 发送目标数据字节数
*/
void USART3_SendBuffer(u8 *buffer, u16 len);

/*
	USART3 模块发送字符串数据到接收端
	str 	 发送的字符串数据内容
*/
void USART3_SendString(const char *str);

/*
	主循环里调用: 处理 USART3 收到的一帧
	★ 只有打开了 SLAVE_USART3_LOCAL_CMD 才会真正解析并执行,
	  关闭时本函数立刻返回, 不占 CPU
	能认的格式(和主机那边云端下发保持一致的键名, 方便同一套工具):
	    {"Led":1}     {"Beep":1}    {"servo":2}
	解析后统一走 ModBus_Set_Reg, 保证"变量 + 引脚"一起更新
*/
void USART3_Poll(void);

#endif
