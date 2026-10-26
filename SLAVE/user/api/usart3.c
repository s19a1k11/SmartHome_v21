#include "all.h"

/*
	USART3: PB10(TX) / PB11(RX), 115200 8N1
	==================================================================
	从机固件里 USART3 是【可选的本地直控口】, 不是上云口。
	上云由主机负责, 从机只做 485 从站。

	SLAVE_USART3_LOCAL_CMD = 0 (默认):
	    只初始化串口 + 收字节, 不解析。留着以后接本地屏/第二路通信。
	SLAVE_USART3_LOCAL_CMD = 1:
	    主循环调用 USART3_Poll(), 解析 {"Led":1} / {"Beep":1} / {"servo":2},
	    不接主机也能单独验证执行器。适合出厂前单板测试。
*/
#define SLAVE_USART3_LOCAL_CMD  0

#if (SLAVE_USART3_LOCAL_CMD)
/* 前置声明: 定义在本文件末尾, 主循环的 Poll 要用 */
u8 Usart3_Get_U16(const char *text, const char *key, u16 *out);
#endif

void USART3_Init(u32 brr)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	/* 1. 开时钟: GPIOB 在 APB2, USART3 在 APB1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	/* 2. PB10 = TX -> 复用推挽输出 2MHz
	      PB11 = RX -> 浮空输入 */
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* 3. 8N1, 收发都要 */
	USART_InitStructure.USART_BaudRate            = brr;
	USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits            = USART_StopBits_1;
	USART_InitStructure.USART_Parity              = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);

	/*
		4. 只开 RXNE, 不开 IDLE
		   ★ 理由和 USART2 一样: 一帧的结束由主循环用"超时/满包"判,
		     中断里只负责收字节。开 IDLE 就必须处理 IDLE, 漏处理会中断风暴。
		   ★ 优先级 2: 本地直控口不是硬实时, 排在 485(0) 和 TIM2 判帧(1) 之后,
		     485 有字节要收时可以打断它。
	*/
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	/*
		★ 优先级 3 (从机侧最低):
		  本串口是"可选的本地直控口", 不参与 485 时序, 也不上云。
		  排在 TIM6(2, 1ms 时基) 之后 —— 时基被打断会让所有基于 g_ms 的
		  节拍产生抖动, 而本地直控晚几毫秒毫无影响。

		从机完整优先级表:
		  USART2 485 接收  抢占 0  最高(晚一步丢字节)
		  TIM2   判帧结束  抢占 1
		  TIM6   1ms 时基  抢占 2
		  USART3 本地直控  抢占 3
		  USART1 调试回显  抢占 3 子 1(最低)
	*/
	NVIC_SetPriority(USART3_IRQn, 3);
	NVIC_EnableIRQ(USART3_IRQn);

	/* 5. 使能串口 */
	USART_Cmd(USART3, ENABLE);
}

USART3_Data usart3_value = {0};

/*
	USART3 接收中断 —— 快进快出
	==================================================================
	只做: 读 DR -> 存缓冲 -> 满了置 flag。
	★ 改造前这里在中断里做 JSON 解析 + 直接驱动 GPIO,
	  115200 下一个字节 86us, 而 strstr 扫 100 字节 + sprintf 要上百微秒,
	  这段时间 USART2 的 485 字节可能到两个 —— 直接丢。
	  现在解析全部在 USART3_Poll() 里做, 中断只负责搬字节。
*/
void USART3_IRQHandler(void)
{
	if(USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET)
	{
		/* 缓冲区还有位置才存; 满了就丢掉这个字节。
		   不管存不存, DR 都必须读走, 否则 RXNE 一直挂着, 中断会反复进 */
		if(usart3_value.count < USART3_BUFFER_SIZE)
		{
			usart3_value.data[usart3_value.count++] = (u8)USART_ReceiveData(USART3);
		}
		else
		{
			(void)USART_ReceiveData(USART3);
			usart3_value.flag = 1;      /* 满了交给主循环处理 */
		}
	}
}

/*
	主循环里处理 USART3 收到的一帧
	-----------------------------------------------------------------
	为什么在这里而不是中断里:
		解析要扫字符串、要算数字、要动 PWM, 都是耗时活。
		中断里做这些, 485 那边来的字节就会丢。

	帧结束判据: 主循环发现"有数据 且 已经 50ms 没新字节", 认为一帧收完。
	这个判据不依赖 IDLE 中断, 也不要求对方发换行。
*/
void USART3_Poll(void)
{
#if (SLAVE_USART3_LOCAL_CMD)
	static u32 s_last_rx_ms = 0;
	static u16 s_last_count = 0;
	u16 len = 0;
	u16 val = 0;
	const char *p = 0;

	/* 1. 没有新字节就记一下时间, 直接返回 */
	if(usart3_value.count == 0)
	{
		s_last_count = 0;
		return;
	}

	/* 2. 字节数还在变 -> 说明一帧没发完, 记时间等下一轮 */
	if(usart3_value.count != s_last_count)
	{
		s_last_count = usart3_value.count;
		s_last_rx_ms = g_ms;
		return;
	}

	/* 3. 字节数稳定但还没安静够 50ms -> 继续等 */
	if((u32)(g_ms - s_last_rx_ms) < 50)
	{
		return;
	}

	/* 4. 一帧收完: 补字符串结尾后解析 */
	len = usart3_value.count;
	if(len >= USART3_BUFFER_SIZE) len = USART3_BUFFER_SIZE - 1;
	usart3_value.data[len] = 0;

	p = (const char *)usart3_value.data;

	/*
		★ 用 ModBus_Set_Reg 而不是直接驱动 GPIO:
		  只动引脚不更新 g_led/g_beep/g_servo, 主机下一轮回读会读到旧值,
		  云端就会以为命令没生效而无限重发。
	*/
	if(Usart3_Get_U16(p, "Led", &val))
	{
		(void)ModBus_Set_Reg(REG_OUTPUT_BASE + 0, val);
	}
	else if(Usart3_Get_U16(p, "Beep", &val))
	{
		(void)ModBus_Set_Reg(REG_OUTPUT_BASE + 1, val);
	}
	else if(Usart3_Get_U16(p, "servo", &val))
	{
		(void)ModBus_Set_Reg(REG_OUTPUT_BASE + 2, val);
	}

	/* 5. 处理完清缓冲, 准备下一帧 */
	usart3_value.count = 0;
	usart3_value.flag = 0;
	s_last_count = 0;
	memset(usart3_value.data, 0, USART3_BUFFER_SIZE);
#else
	/* 本地直控关闭: 收到的字节直接丢掉, 不让它无限堆积 */
	if(usart3_value.flag || usart3_value.count >= USART3_BUFFER_SIZE)
	{
		usart3_value.count = 0;
		usart3_value.flag = 0;
	}
#endif
}

#if (SLAVE_USART3_LOCAL_CMD)
/*
	从 JSON 文本里取一个整数键值
	-----------------------------------------------------------------
	为什么不用 strstr 直接找 "Led":1:
	  strstr 是子串包含, 发 {"Led":12} 会命中 "Led":1 被当成 1,
	  越界保护形同虚设。必须把冒号后面的数字完整读出来再交给 Set_Reg 判。

	能认: "Led":0   "Led": 1   "Led":12
	不认: "Led" : 1  (键和冒号之间有空格, 云端不会这么发)
*/
u8 Usart3_Get_U16(const char *text, const char *key, u16 *out)
{
	const char *p = text;
	const char *q = 0;
	u32 v = 0;

	if(text == 0 || key == 0 || out == 0) return 0;

	/* 一处处找 key, 直到找到一个「真正的键名」(前面紧挨着双引号) */
	while((p = strstr(p, key)) != 0)
	{
		/* 前面必须是引号, 否则这是别的词的尾巴, 不是键名 */
		if(p != text && p[-1] == '"')
		{
			q = p + strlen(key);

			/* 键名后面必须紧跟 " 和 : */
			if(*q == '"' && q[1] == ':')
			{
				q += 2;
				while(*q == ' ' || *q == '\t') q += 1;

				if(*q >= '0' && *q <= '9')
				{
					v = 0;
					while(*q >= '0' && *q <= '9')
					{
						v = v * 10 + (u32)(*q - '0');
						if(v > 65535) return 0;		/* 超出 u16, 当非法 */
						q += 1;
					}
					*out = (u16)v;
					return 1;
				}
			}
		}
		p += 1;			/* 这个位置不合适, 从下一个字符继续 */
	}

	return 0;
}
#endif

void USART3_SendByte(u8 byte)
{
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);

	USART_SendData(USART3, byte);
}

void USART3_SendBuffer(u8 *buffer, u16 len)
{
	while(len--)
	{
		USART3_SendByte(*buffer);
		buffer += 1;
	}
}

void USART3_SendString(const char *str)
{
	while(*str)
	{
		USART3_SendByte(*str);
		str += 1;
	}
}
