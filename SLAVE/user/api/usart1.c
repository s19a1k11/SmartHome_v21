#include "usart1.h"

/*
	USART1 模块初始化函数，要求用户提供对应工作波特率
	Tx ==> PA9 [复用推挽输出模式, 10MHz]
	Rx ==> PA10 [浮空输入模式]
	NRZ 数据格式为 8N1 模式

	本串口只做调试日志输出与回显, 不接 485 总线。
	★ 日志全部用英文: 汉字按 UTF-8 送到串口助手(GBK)会显示成 ??????

	★ 本工程的硬要求: "串口1 作为调试回显, 必须在所有需要完成的操作
	  结束后再进行发送"。
	  这个要求的落实分两层:
	    1) 日志内容先进 RAM 缓冲 (见 log.c), 不在业务代码里直接发;
	    2) main 循环把采集/应答/上报全部做完, 最后一步才 LOG_Flush()。
	  所以本文件里的发送函数只被 log.c 的 LOG_Flush 调用。
*/
USART1_Data usart1_value = {0};

void USART1_Init(u32 brr)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* 1. 开时钟: GPIOA 和 USART1 都挂在 APB2 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1, ENABLE);

	/* 2. PA9  = TX -> 复用推挽输出 10MHz
	      PA10 = RX -> 浮空输入(电平完全由外部决定) */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 3. 串口参数: 8 位数据 / 1 位停止 / 无校验 / 无硬件流控 / 收发都要 */
	USART_InitStructure.USART_BaudRate            = brr;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);

	/* 4. 开 RXNE + IDLE:
	      RXNE = 收到一个字节
	      IDLE = 总线空闲(一帧结束了)
	   本串口是调试口, 收到的多半是人在串口助手里敲的一行命令,
	   用 IDLE 判"一行敲完了"最省事, 而且它不参与 485 时序。 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

	/*
		★ 优先级: 抢占 3 / 子 1 (最低)
		调试口晚几毫秒没有任何影响, 但它是【发送口】——
		发送是阻塞轮询 TC 的, 一旦开始发, 只有更高优先级的中断能打断它。
		把它放到最低, 意味着 485 收字节(抢占 0) 可以随时打断日志发送,
		这正是"收 > 发"原则: 日志可以等, 总线上的字节不能等。
	*/
	NVIC_InitTypeDef NVIC_InitStructure = {0};
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	/* 5. 使能串口 */
	USART_Cmd(USART1, ENABLE);
}

void USART1_SendByte(u8 byte)
{
	/*
		发送前等 TC(发送完成), 再写 DR。
		TC 上电默认是 1, 所以第一个字节不会白等。
	*/
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);

	USART_SendData(USART1, byte);
}

void USART1_SendBuffer(u8 *buffer, u16 len)
{
	while(len--)
	{
		USART1_SendByte(*buffer);
		buffer += 1;
	}
}

void USART1_SendString(const char *str)
{
	while(*str)
	{
		USART1_SendByte(*str);
		str += 1;
	}
}

/*
	USART1 接收中断 —— 快进快出
	==================================================================
	★ 中断里只做: 读 DR -> 存一格 -> 置标志。清缓冲交给主循环。
	  这里【不许】打印(会递归: 在调试口的 ISR 里往调试口写)。
*/
void USART1_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
	{
		u8 byte = (u8)USART_ReceiveData(USART1);

		/*
			只有"上一帧已经被主循环取走"(flag == 0) 时才继续往缓冲里追加;
			否则说明主循环还没处理完上一帧, 这一帧直接丢掉,
			而不是在中断里把上一帧抹掉 —— 丢掉新字节比毁掉旧数据安全。
		*/
		if(usart1_value.flag == 0 && usart1_value.count < USART1_BUFFER_SIZE)
		{
			usart1_value.data[usart1_value.count++] = byte;
		}
	}

	/* 总线空闲: 一帧结束。清 IDLE 的方法是"先读 SR 再读 DR" */
	if(USART_GetFlagStatus(USART1, USART_FLAG_IDLE) != RESET)
	{
		(void)USART_ReceiveData(USART1);
		if(usart1_value.count > 0)
		{
			usart1_value.flag = 1;
		}
	}
}
