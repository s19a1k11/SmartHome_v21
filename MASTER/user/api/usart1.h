#ifndef _USART1_H
#define _USART1_H

#include "common.h"

#define USART1_BUFFER_SIZE (256)

typedef struct usart1_value
{
    // USART1 缓冲区数组，用于存放数据
    u8 data[USART1_BUFFER_SIZE];
    // USART1 有效字节个数
    u16 count;
    // USART1 一帧数据接收完成标识符
    u8 flag;
} USART1_Value;

// USART1 全局接收数据管理结构体，仅用于调试打印，不参与总线通信
extern USART1_Value usart1_value;

/**
 *  @brief 初始化 USART1
 *
 *  @param brr USART1的波特率
 */
void USART1_Init(u32 brr);

// USART1 接收中断处理函数
void USART1_IRQHandler(void);

/**
 *  @brief 清空 USART1 接收缓冲区，准备接收下一帧
 */
void USART1_Recv_Clear(void);

/**
 *  @brief 发送字符串
 *
 *  @param str 要发送的字符串地址
 */
void USART1_SendString(const char *str);

/**
 *  @brief 发送单个字符
 *
 *  @param byte 要发送的字符
 */
void USART1_SendByte(u8 byte);

/**
 *  @brief 发送指定个数的字符
 *
 *  @param buffer 要发送的字符串
 *  @param len 发送字符串的有效长度
 */
void USART1_SendBuffer(u8 *buffer, u16 len);

/**
 *  @brief 发送一个十进制整数，方便调试打印寄存器值
 *
 *  @param num 要发送的无符号整数
 */
void USART1_SendNumber(u32 num);

#endif
