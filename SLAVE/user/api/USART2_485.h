#ifndef _USART2_485_H
#define _USART2_485_H

#include "common.h"
#include "type.h"


/*
	USART2 模块初始化函数 对应 GPIO 引脚为
	PA2 >> USART2_TX
	PA3 >> USART2_RX

	brr 用户提供的参数当前 USART2 模块波特率
*/

void USART2_485_Init(u32 brr);

/*
	 USART2 中断实现
	 ★ 快进快出: 只做"存字节 + 复位断帧定时器", 不打印、不解析、不动 PD7
*/
void USART2_IRQHandler(void);

/*
	清空 USART2 接收缓冲
	★ 只允许在主循环上下文调用(它会写 count/flag, 与中断有竞争),
	  中断里绝对不要调它
*/
void USART2_Recv_Clear(void);

/*
	开关 USART2 的接收中断
	★ 从机发应答时也必须用: 485 收发器在发送态会把自己发出的字节
	  回送到自己的 RX 脚, 如果接收中断还开着, 这一帧会被当成
	  "总线上来的请求"收进缓冲区 —— 结果是自己刚发完应答,
	  缓冲区里就躺着一份自己的应答, 下一轮被当成请求去解析,
	  地址/功能码全对不上, 表现是"偶尔回一帧莫名其妙的异常码"。
*/
void USART2_Recv_IT_Cmd(FunctionalState state);

void USART2_SendByte(u8 byte);

void USART2_SendBuffer(u8 *buffer,u16 count);

void USART2_SendString(const char *str);

#endif
