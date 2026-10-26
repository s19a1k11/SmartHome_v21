#ifndef _LOG_H
#define _LOG_H

#include "common.h"

/*
	调试日志缓冲 (USART1 = 调试回显口) —— 主机侧
	==================================================================
	★ 本工程的硬要求:
	  "所有的主机和从机的串口1 需要作为调试使用的数据回显,
	   他应该在所有的需要完成的操作结束后再进行发送"

	为什么必须延后:
	  USART1 是 9600 阻塞发送, 1 个字符约 1.04ms。
	  主机的关键路径是 485 轮询(每台 3 笔事务, 每笔最多 200ms 超时)
	  和 MQTT 心跳。如果边轮询边打印, 打印时间会直接叠加进轮询周期;
	  更糟的是打印期间 485 上从机的应答可能已经到达并被下一帧覆盖。

	做法:
	  所有日志先写进 RAM 缓冲 (LOG_Str / LOG_Num / LOG_Hex 都只是 memcpy 级操作),
	  主循环把该做的事(轮询 / 上报 / 下发 / 心跳)全部做完之后,
	  最后一步才调 LOG_Flush() 真正往 USART1 发。

	★ 使用约束:
	  1) 只允许在主循环上下文调用, 绝对不许在中断里调
	  2) LOG_Flush() 是阻塞的, 一个循环里只调一次, 放在最后
	  3) 缓冲满了会丢新内容并打截断标记, 不会越界、也不会卡住
*/

/* 缓冲大小: 一次循环里最多攒多少字节日志。2048 约等于 2 秒的串口发送量 */
#define LOG_BUF_SIZE (2048)

void LOG_Init(void);
void LOG_Str(const char *s);
void LOG_Chr(char c);
void LOG_Num(u16 n);
void LOG_HexNum(u16 v);
void LOG_HexDump(const char *tag, u8 *buf, u16 len);

/*
	把缓冲里的内容真正发到 USART1, 然后清空
	★ 这是唯一会产生阻塞的地方, 必须在"所有需要完成的操作结束后"调用
	缓冲为空时立刻返回, 不做任何 IO
*/
void LOG_Flush(void);

/* 缓冲里还有没有待发内容(1 = 有) */
u8 LOG_Pending(void);

#endif
