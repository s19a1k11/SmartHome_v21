#include "USART2_485.h"

/*
	USART2 = RS485 物理层 (PA2 = TX, PA3 = RX), 9600 8N1
	==================================================================
	本文件只负责"把总线上的字节搬进缓冲区", 以及"把缓冲区里的字节搬上总线"。
	帧的起始/结束判定交给 TIM2 的 T3.5 定时(见 ModBus.c 的 TIM2_IRQHandler),
	方向脚 PD7 的操作交给 485_PD7.c。
	三层各管一件事, 任何一层出问题都能单独测。
*/

void USART2_485_Init(u32 brr)
{
	/*
		RCC 时钟使能模块 GPIOA 和 USART2
		void RCC_APB2PeriphClockCmd(uint32_t RCC_APB2Periph, FunctionalState NewState);
		void RCC_APB1PeriphClockCmd(uint32_t RCC_APB1Periph, FunctionalState NewState);
	*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

	// GPIO 模块配置
	/*
		通过分析得出 USART2 Tx > PA2 对应模式为 复用推挽输出模式
		             USART2 Rx > PA3 对应模式为 浮空输入模式
		每个模式或引脚都有相应的 枚举 或结构体
	*/
	GPIO_InitTypeDef GPIO_InitStructure = {0};
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

	// USART2 Tx
	GPIO_Init(GPIOA,&GPIO_InitStructure);

	// USART2 Rx
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//USART2 必要配置
	USART_InitTypeDef USART_InitStructure = {0};

	// 8 N 1 配置 (注意虽然平时只说 8 N 1 但实际还有起始位 1 )
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	//波特率配置
	USART_InitStructure.USART_BaudRate = brr;

	//使能 USART Rx Tx
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	// 设置当前 USART 为软件触发模式
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Init(USART2, &USART_InitStructure);

	// 中断配置 需要注册中断有 RXNE
	/*
		RXNE 接收数据缓冲区非空中断，对应数据接收过程
		void USART_ITConfig(USART_TypeDef *USARTx, uint16_t USART_IT, FunctionalState NewState)
	*/
	USART_ITConfig(USART2,USART_IT_RXNE,ENABLE);
	/*
		* IDLE 中断已关闭 —— 判帧完全交给 TIM2 的 t3.5 定时器

		为什么关:
		  ModBus RTU 规范要求"帧间空闲 >= 3.5 字符时间(t3.5)才算帧结束"。
		  IDLE 硬件判据只有 1 个字符时间(约 1.04ms), 远小于 3.65ms,
		  用它判帧不符合规范。规范明文: 波特率<=19200bps 时 t1.5/t3.5 必须严格遵守。

		历史教训:
		  原来这里 ENABLE 了 IDLE, 但下面的 IDLE 处理代码被注释掉了。
		  IDLE 标志只有"读 DR"才能清除, 空闲期间没有新字节就没人读 DR,
		  => 标志永远清不掉 => 中断返回立刻再进 => 无限空转
		  => CPU 被占死 => TIM2 中断永远轮不到 => t3.5 判帧失效 => 多帧粘连。
		  -> "使能"和"处理"必须成对出现, 绝不能只使能不处理。
	*/
	//USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);   <-- 已关闭 IDLE

	NVIC_InitTypeDef NVIC_InitStructure = {0};

	//设置当前中断请求编号
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//设置抢占优先级 占先优先级
	/*
		★ USART2 必须是最高优先级(0):
		  它是唯一"晚一步就丢字节"的中断。其它中断(TIM2 判帧、TIM6 计时)
		  晚几微秒没有任何影响, 所以都排在它后面。
	*/
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

	//初始化当前 NVIC 中断管理控制器，告知当前 USART2 中断请求开启
	NVIC_Init(&NVIC_InitStructure);

	// 开启 USART2 模块
	/*
		void USART_Cmd(USART_TypeDef *USARTx, FunctionalState NewState)
		开启 OR  关闭指定 USART 模块函数
	*/
	USART_Cmd(USART2,ENABLE);
}

USART2_Data usart2_value = {0};

/*
	USART2 接收中断
	==================================================================
	★ 快进快出原则(本工程硬要求):
	  这个中断每 1.04ms 就可能来一次, 里面任何一次阻塞都会直接丢字节。
	  所以它只做三件常数级的事:
	      1) 把 DR 里的字节搬进缓冲区(或丢弃)
	      2) 复位 TIM2 计数器(告诉它"总线还没安静")
	      3) 缓冲区满时置 flag

	这里【绝对不许】出现:
	    USART1_SendString / ModBus_PrintHex / memset 整块缓冲 / 任何 delay
	  历史教训: 改造前这里虽然只打 2 个字符, 但那 2 个字符在 9600 下是 2.1ms,
	  而一个字节只占 1.04ms, 等于中断里停了两个字节的时间, 后面两个字节必丢。

	打印和解析全部搬到 main 循环, 中断只负责"记下来"。
*/
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2,USART_IT_RXNE))
	{
		/*
			★ 边界保护: 缓冲区写满后不再写入, 防止 data[256] 越界
			★ 上一帧还没被主循环取走(flag==1)时, 新来的字节直接丢弃:
			  否则新帧会拼在旧帧屁股后面, 长度和 CRC 全乱, 主循环拿到的是
			  一个"半截旧帧 + 半截新帧"的怪物, 两个都废掉。
			  丢新字节至少保住已经完整的那一帧。
		*/
		if(usart2_value.flag)
		{
			(void)USART_ReceiveData(USART2);
		}
		else if(usart2_value.count < USART2_BUFFER_SIZE)
		{
			usart2_value.data[usart2_value.count++] = (u8)USART_ReceiveData(USART2);
		}
		else
		{
			/* 读走但不保存, 避免 RXNE 一直挂起 */
			(void)USART_ReceiveData(USART2);
		}

		/*
			每收到一个字节就把 T3.5 定时器重新数一遍:
			只要还有字节在来, 它就永远数不满 -> 不会误判帧结束;
			总线安静超过 3.5 个字符时间 -> 它溢出 -> 判帧完成。
		*/
		TIM_SetCounter(TIM2, 0);
		TIM_Cmd(TIM2,ENABLE);

		if(usart2_value.count >= USART2_BUFFER_SIZE)
		{
			//如果超出当前缓冲区大小则直接结束
			usart2_value.flag = 1;
		}
	}
	/*
		* IDLE 中断处理 —— 已删除

		本工程用 TIM2 的 t3.5 定时判帧(符合 ModBus RTU 规范),
		IDLE 已在上面关闭, 这里不再需要处理。

		! 如果哪天想把 IDLE 打开, 必须【同时】把处理代码加回来,
		   否则 IDLE 标志永远清不掉 -> 中断无限空转 -> TIM2 被饿死。
	*/
}

/*
	清空接收缓冲
	★ 只能在主循环调用。它改 count/flag 的同时中断也在改,
	  所以要先关中断再搬, 搬完再开 —— 否则可能清到一半又被写进一个字节,
	  下一帧就变成"残帧 + 新帧"粘在一起, CRC 必败。
*/
void USART2_Recv_Clear(void)
{
	__disable_irq();
	usart2_value.count = 0;
	usart2_value.flag  = 0;
	memset(usart2_value.data, 0, USART2_BUFFER_SIZE);
	__enable_irq();
}

/*
	开关 USART2 的接收中断
	★ 从机发送应答前必须关掉, 发完再打开。
	  原因: 485 收发器在发送态(DE=1)时, A/B 线上的电平由本机驱动,
	  同时 RO 脚也会把自己发出的数据回送给自己 —— 如果接收中断开着,
	  这一帧应答会被自己收进 usart2_value, 主循环下一轮把它当"请求"
	  去解析, 地址对得上(就是自己), 功能码也对得上, 于是回一帧
	  谁也解释不了的应答, 主机那边就收到"帧长不对/CRC 失败"。
	  这类故障只在"刚发完应答的瞬间"出现, 非常难查。
*/
void USART2_Recv_IT_Cmd(FunctionalState state)
{
	USART_ITConfig(USART2, USART_IT_RXNE, state);
}

void USART2_SendByte(u8 byte){
	//进行数据发送
	//先判断上一帧数据是否发送完毕
	while(!USART_GetFlagStatus(USART2,USART_FLAG_TC));

	/*
		void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
	*/
	USART_SendData(USART2,byte);
}

void USART2_SendBuffer(u8 *buffer,u16 count)
{
	while(count--)
	{
		USART2_SendByte(*buffer);
		buffer +=1;
	}
}

void USART2_SendString(const char *str)
{
	while(*str)
	{
		USART2_SendByte(*str);
		str += 1;
	}
}
