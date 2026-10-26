#ifndef _RS485_H
#define _RS485_H

#include "common.h"
#include "usart2.h"

// RS485 一帧的长度上限就是 USART2 的接收缓冲大小，
// 从机应答帧最长 5 + 2 * 寄存器个数，128 字节对本项目绰绰有余
#define RS485_BUFFER_SIZE (USART2_BUFFER_SIZE)

/*
RS485 半双工方向控制引脚 ==> PD7
    PD7 输出 1 【发送使能，占用总线】
    PD7 输出 0 【接收使能，释放总线】

★ 设计约束(本项目硬要求): 全工程【只有 rs485.c】可以碰 PD7。
  其它文件一律通过 RS485_SendBuffer() 发数据, 不再自己切方向。

为什么把 RS485_DIR_TX/RX 这两个宏从本头文件里删掉:
  宏定义在头文件里, 任何 include 了它的文件都能写 RS485_DIR_TX(),
  等于"人人都能操作 PD7", 和"PD7 只在实现函数内部改"这条要求直接冲突。
  改成 rs485.c 里的 static 函数之后, 编译器保证别的文件根本调不到,
  这是把约束从"约定"变成"编译期强制"。

本层只负责【方向切换 + 发送时序】，字节收发由 usart2.c 完成，
总线上收到的应答帧存放在 usart2_value 里。
*/

/**
 *  @brief 初始化 RS485 模块，内部会初始化方向脚与 USART2
 *
 *  @param brr RS485 总线波特率，从机侧固定 9600
 */
void RS485_Init(u32 brr);

/**
 *  @brief 清空 RS485 接收缓冲区，准备接收下一帧
 */
void RS485_Recv_Clear(void);

/**
 *  @brief 发送指定个数的字节到 RS485 总线
 *
 *  内部会完成【关接收中断 -> 切发送态 -> 发完等 TC -> 切接收态 -> 开中断】，
 *  调用方不用关心方向脚。
 *
 *  @param buffer 要发送的数据首地址
 *  @param len 发送的字节个数
 */
void RS485_SendBuffer(u8 *buffer, u16 len);

/**
 *  @brief 发送单个字节到 RS485 总线
 */
void RS485_SendByte(u8 byte);

/**
 *  @brief 发送字符串到 RS485 总线
 */
void RS485_SendString(const char *str);

#endif
