#include "all.h"

/*
	RS485 方向脚 PD7 —— 全工程唯一操作它的地方
	==================================================================
	上电顺序很关键: 必须先把 PD7 配成输出并拉低(接收态), 再使能 USART2。
	否则 USART2 使能的那一刻 PD7 还是复位默认的浮空输入,
	若总线上此刻正好有别的从机在发数据, 本机会误收几个字节进来,
	表现为"上电第一帧总是 CRC 失败"。
	main.c 里的调用顺序已经按这个要求排好。
*/

void My_485_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure = {0};

	/* 1. 使能 GPIOD 时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

	/* 2. PD7 配成通用推挽输出
	      推挽而不是开漏: 485 收发器的 DE/RE 是高阻输入, 需要真正的推挽驱动
	      2MHz 足够: 方向翻转每秒最多几百次, 不需要 50MHz 的边沿速度
	          (高频反而会往 485 线上辐射干扰) */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* 3. 立刻置为接收态, 绝不能让本机在上电瞬间占住总线 */
	My_485_Rx();
}

void My_485_Tx(void)
{
	/* 拉高 = 发送使能。写完这一句 485 收发器就把 A/B 线交给本机驱动了,
	   所以调用方必须紧接着把数据写进 USART2, 中间不要夹任何打印/延时。 */
	GPIO_SetBits(GPIOD, GPIO_Pin_7);
}

void My_485_Rx(void)
{
	/* 拉低 = 释放总线, 回到接收态 */
	GPIO_ResetBits(GPIOD, GPIO_Pin_7);
}

void My_485_SendBuffer(u8 *buf, u16 len)
{
	if(buf == 0 || len == 0)
	{
		return;
	}

	/*
		★ 1. 先关接收中断, 避免收到自己的回声
		485 收发器发送时会把 A/B 线上的电平同时送回 RO 脚,
		如果不关接收中断, 本机发出去的应答会被自己完整收进缓冲区,
		主循环下一轮把它当成新请求去解析 —— 地址是自己、功能码也对,
		于是回一帧内容错乱的应答, 主机看到的就是"CRC 失败/帧长不对"。
		这种故障只在"刚发完的瞬间"出现, 极难定位, 所以必须在这里堵死。
	*/
	USART2_Recv_IT_Cmd(DISABLE);

	My_485_Tx();                        /* 2. 占用总线 */

	USART2_SendBuffer(buf, len);        /* 3. 逐字节发送(每个字节内部等 TC) */

	/*
		4. 等最后一个字节真正移出移位寄存器再释放总线。
		USART2_SendBuffer 里每个字节都等了 TC, 所以正常情况这里 TC 已经是 1,
		这一句只是兜底: 万一上层换了发送实现, 也不会把最后一字节切断。
	*/
	while(USART_GetFlagStatus(USART2, USART_FLAG_TC) == 0);

	My_485_Rx();                        /* 5. 释放总线 */

	/*
		6. 丢掉发送期间可能残留的回声字节, 再打开接收中断。
		★ 顺序不能反: 先清再开。反过来会在清和开之间留一个窗口,
		  正好有回声进来就又被收下了。
	*/
	USART2_Recv_Clear();
	USART2_Recv_IT_Cmd(ENABLE);
}
