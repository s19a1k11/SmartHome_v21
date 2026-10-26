#ifndef _USART1_H
#define _USART1_H

#include "common.h"
#include "type.h"

/*
	串口1 在编写过程中需要做回显，调试功能，不做真正的功能要求
	[注意] 因只做调试功能所以需要在保证数据接收或上传完毕后在进行工作
				 如 Modbus 一帧接收或发送完毕后拉低 PD7 后顺手将数据发送给 串口1 一份
				 进行回显，看究竟收到了什么 发送了什么
				 或打印 There is XXX 确定程序执行到哪里
				 如果没有打印说明 程序未进入该函数
*/


/*
	USART1 模块初始化函数，要求用户提供对应工作波特率
	Tx ==> PA9 [复用推挽输出模式]
	Rx ==> PA10 [浮空输入模式]
	NRZ 数据格式为 8N1 模式
	brr 用户提供的波特率参数， 需要根据当前 MCU 时钟频率
	和进行合理选择
*/
void USART1_Init(u32 brr);

/*
	USART1 模块发送一个字节数据到接收端
	byte 需要进行发送的一个字节数据
*/
void USART1_SendByte(u8 byte);

/*
	USART1 模块指定字节个数数据内容到接收端
	buffer 发送数据对应内存空间首地址
	len 	 发送目标数据字节数
*/
void USART1_SendBuffer(u8 *buffer,u16 len);

/*
	USART1 模块发送字符串数据到接收端
	str 	 发送的字符串数据内容
*/
void USART1_SendString(const char *str);

void USART1_IRQHandler(void);

#endif
