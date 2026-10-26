#ifndef _USART3_H
#define _USART3_H

#include "common.h"
#include "usart1.h"

#define USART3_BUFFER_SIZE (512)

typedef struct usart3_value
{
    // USART3 缓冲区数组，用于存放数据
    u8 data[USART3_BUFFER_SIZE];
    // USART3 有效字节个数
    u16 count;
    // USART3 一帧数据接收完成标识符
    u8 flag;
} USART3_Value;

// USART3 全局接收数据管理结构体，连接 ESP8266 / MQTT 云平台
extern USART3_Value usart3_value;

/**
 *  @brief USART3 初始化
 *
 *  @param brr USART3的波特率
 */
void USART3_Init(u32 brr);

// USART3 接收中断处理函数
void USART3_IRQHandler(void);

/**
 *  @brief 清空 USART3 接收缓冲区，准备接收下一帧
 */
void USART3_Recv_Clear(void);

/**
 *  @brief 发送字符串
 *
 *  @param str 要发送的字符串地址
 */
void USART3_SendString(const char *str);

/**
 *  @brief 发送单个字符
 *  @param byte 要发送的字符
 */
void USART3_SendByte(u8 byte);

/**
 *  @brief 发送指定个数的字节
 *
 *  @param buffer 要发送的字符串
 *  @param len 发送字符串的有效长度
 */
void USART3_SendBuffer(u8 *buffer, u16 len);

/**
 *  @brief 取 USART3 当前收到的原始字节(供 CLI 的 @esp 诊断用)
 *
 *  ★ 为什么需要: ESP8266 出问题时最想知道"模块到底回了什么"。
 *    没有这个接口只能看到上层打的 "module no response",
 *    分不清是没收到(接线/供电) 还是 收到乱码(波特率错)。
 *
 *  @param buf 输出缓冲区
 *  @param max 缓冲区容量
 *  @return 实际拷贝的字节数
 */
u16 USART3_Get_Raw(u8 *buf, u16 max);

#endif
