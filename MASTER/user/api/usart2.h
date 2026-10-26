#ifndef _USART2_H
#define _USART2_H

#include "common.h"

#define USART2_BUFFER_SIZE (128)

typedef struct usart2_value
{
    // USART2 缓冲区数组，用于存放数据
    u8 data[USART2_BUFFER_SIZE];
    // USART2 有效字节个数
    u16 count;
    // USART2 一帧数据接收完成标识符
    u8 flag;
} USART2_Value;

// USART2 全局接收数据管理结构体，RS485 总线上收到的应答帧就放在这里
extern USART2_Value usart2_value;

/**
 *  @brief 初始化 USART2
 *
 *  硬件连接：Tx ==> PA2【复用推挽输出】 Rx ==> PA3【浮空输入】
 *  USART2 挂在 APB1 上，最高只能到 36MHz，波特率上限比 USART1 低，
 *  但 RS485 总线跑 9600 完全够用。
 *
 *  ★ 中断已按快进快出改写: RXNE 里只搬字节, IDLE 里只置 flag,
 *    校验和解析全部在主循环(ModBus_Transaction)里做。
 *
 *  @param brr USART2的波特率
 */
void USART2_Init(u32 brr);

// USART2 接收中断处理函数
void USART2_IRQHandler(void);

/**
 *  @brief 清空 USART2 接收缓冲区，准备接收下一帧
 */
void USART2_Recv_Clear(void);

/**
 *  @brief 开关 USART2 的接收中断
 *
 *  RS485 是半双工，自己发出去的字节会被自己收回来，
 *  所以在发送期间必须把接收中断关掉，发完再打开。
 *
 *  @param state ENABLE 打开接收中断 / DISABLE 关闭接收中断
 */
void USART2_Recv_IT_Cmd(FunctionalState state);

/**
 *  @brief 发送字符串
 */
void USART2_SendString(const char *str);

/**
 *  @brief 发送单个字符
 */
void USART2_SendByte(u8 byte);

/**
 *  @brief 发送指定个数的字符
 */
void USART2_SendBuffer(u8 *buffer, u16 len);

/**
 *  @brief 发送一个十进制整数，方便调试打印寄存器值
 */
void USART2_SendNumber(u32 num);

#endif
