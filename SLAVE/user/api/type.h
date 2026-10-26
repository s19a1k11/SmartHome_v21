#ifndef _TYPE_H
#define _TYPE_H

#include "common.h"

//串口 结构体 大小
#define USART1_BUFFER_SIZE (256)
#define USART2_BUFFER_SIZE (256)
#define USART3_BUFFER_SIZE (256)


// 串口1 结构体
typedef struct usart1_data
{
	// u8 类型缓冲区数组，用于存放数据内容
	u8 data[USART1_BUFFER_SIZE];
	// data 数组中，有效字节个数r
	u16 count;
	/*
	case 1: 当前数据尚未接收完成， flag=0;
	case 2; 当前数据接收完毕，触发 IDLE 中断 ,flag = 1
	case 3; data 数据收集缓冲区已满 ,flag = 1
	*/
	u8 flag;
} USART1_Data;

// 该关键字使外部可以读取该数据
extern USART1_Data usart1_value;

// 串口2 结构体
typedef struct usart2_data
{
	// u8 类型缓冲区数组，用于存放数据内容
	u8 data[USART2_BUFFER_SIZE];
	// data 数组中，有效字节个数
	u16 count;
	/*
	case 1: 当前数据尚未接收完成， flag=0;
	case 2; 当前数据接收完毕，触发 IDLE 中断 ,flag = 1
	case 3; data 数据收集缓冲区已满 ,flag = 1
	*/
	u8 flag;
} USART2_Data;

// 该关键字使外部可以读取该数据
extern USART2_Data usart2_value;

// 串口3 结构体
typedef struct usart3_data
{
	// u8 类型缓冲区数组，用于存放数据内容
	u8 data[USART3_BUFFER_SIZE];
	// data 数组中，有效字节个数
	u16 count;
	/*
	case 1: 当前数据尚未接收完成， flag=0;
	case 2; 当前数据接收完毕，触发 IDLE 中断 ,flag = 1
	case 3; data 数据收集缓冲区已满 ,flag = 1
	*/
	u8 flag;
} USART3_Data;

// 该关键字使外部可以读取该数据
extern USART3_Data usart3_value;

#endif
