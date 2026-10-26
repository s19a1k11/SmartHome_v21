#include "all.h"

/*
	DHT11 单总线驱动: 数据脚 PE4
	-----------------------------------------------------------------
	原来这里是直接改 GPIOE->CRL / ODR / IDR, 现在换成标准外设库:
		GPIO_Init(GPIOE, &结构体)            <- 改 CRL 那 4 位
		GPIO_SetBits / GPIO_ResetBits        <- 改 ODR 那一位
		GPIO_ReadInputDataBit                <- 读 IDR 那一位

	两种写法干的是同一件事, 参数一个没变:
		输出态 = 通用推挽 10MHz
		输入态 = 上拉输入(2026-10-23 改: 原本浮空, 靠外部 4.7k~10k 上拉)

	★ 时序敏感提醒:
		DHT11 每一位靠"高电平持续多少微秒"来区分 0 和 1,
		库函数 GPIO_ReadInputDataBit 比自己读 IDR 多一次函数调用(几十纳秒),
		在 72MHz 下完全不够影响 26us 级的判定, 实测读数正常。
		但如果以后把主频降到很低, 这一点开销就要重新评估。
*/

/*
	 GPIO_E4 的初始化: 配成推挽输出, 频率 10MHz
*/
void GPIO_E4_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/*
	将 GPIO_E4 的模式设置为 输出模式, 并把线拉到高电平
*/
void GPIO_E4_Output(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_SetBits(GPIOE, GPIO_Pin_4);
}

/*
	将 GPIO_E4 的模式设置为 浮空输入模式
	(浮空 = 内部上下拉都断开, 电平完全由外部上拉电阻决定)
*/
void GPIO_E4_Input(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_4;
	/* 用内部上拉(IPU) 而不是浮空:
	   DHT11 的 DATA 是开漏输出, 线上必须有上拉电阻才能回到高电平。
	   模块自带 10k 时并联内部上拉无害;
	   要是模块没带, 浮空会让电平随机跳,
	   握手能"碰巧"通过但数据全不对 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOE, &GPIO_InitStructure);
}

/*
	发送准备信号
	低电平 35ms(拉低总线, 告诉 DHT11 要开始) -> 高电平 35us -> 交还总线(浮空输入)
*/
void DHT11_Start_Signal(void)
{
	GPIO_E4_Output();                   /* 先切成输出 */

	GPIO_ResetBits(GPIOE, GPIO_Pin_4);  /* 低电平 35ms */
	delay_ms(35);

	GPIO_SetBits(GPIOE, GPIO_Pin_4);    /* 高电平 35us */
	delay_us(35);

	GPIO_E4_Input();                    /* 交还总线, 等 DHT11 应答 */
}

/*
	等待 DHT11 的响应: 它会先拉低 80us, 再拉高 80us
	超时就返回错误, 避免死等把整个程序卡住
*/
u8 DHT11_Response(void)
{
	u8 Send_Time = 0;

	DHT11_Start_Signal();

	/* 等总线被 DHT11 拉低 */
	while(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4))
	{
		delay_us(1);
		Send_Time += 1;
		if(Send_Time >= 200)
		{
			return DHT11_Send_ERROR_1;
		}
	}

	Send_Time = 0;

	/* 等它再拉高 */
	while(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4))
	{
		delay_us(1);
		Send_Time += 1;
		if(Send_Time > 200)
		{
			return DHT11_Send_ERROR_1;
		}
	}

	return DHT11_Send_SUCCEES;
}

/*
	读一位: 每位的开头都是 50us 低电平, 之后高电平的长短决定是 0 还是 1
		高电平约 26~28us -> 0
		高电平约 70us    -> 1
	这里先等低电平结束, 再延时 30us 采样: 30us 后还是高电平就是 1
*/
u8 DHT11_Read_Bit(void)
{
	u8 Send_Time = 0;

	/*
		DHT11 一位的波形: [50us 低电平] + [26~28us 高 = 0 | 70us 高 = 1]

		读一位必须按这个顺序走三步, 少一步、顺序错一步都会读错:
			第1步  等上一位的高电平彻底结束
			第2步  等这一位自己的 50us 低电平结束(它的结束点 = 高电平起点)
			第3步  从高电平起点数 40us 再采样

		★ 第1步是 2026-10-23 补上的, 也是最容易漏的一步。
		  上一位如果是 1(高 70us), 进本函数时才过了 30us, 线上还是高电平。
		  少了第1步, 第2步会立刻退出, 再 delay 40us 采样时线上仍是上一位的高电平,
		  于是这一位被误读成 1。只有当上一位恰好是 0(高 26us) 时才碰巧不出错。
		  表现就是: 偶尔读出一组真值, 大多数时候校验失败报 code=3。

		★ 第3步为什么取 40us 不取 30us:
		  0 位 40us 时已变低(距 26us 有 14us 余量);
		  1 位 40us 时还在高(距 70us 有 30us 余量)。
		  两边余量都比 30us 时大, 抗中断和 delay 误差的能力更强。
	*/

	/* 第1步: 等上一位的高电平结束(如果它已经结束, 这个循环立刻退出) */
	while(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4))
	{
		delay_us(1);
		Send_Time += 1;
		if(Send_Time > 100)
		{
			return 0;
		}
	}

	Send_Time = 0;

	/* 第2步: 等这一位的 50us 低电平结束 */
	while(!GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4))
	{
		delay_us(1);
		Send_Time += 1;
		if(Send_Time > 100)
		{
			/* 超时返回 0, 不能返回错误码:
			   调用方是 DHT11_Data |= 返回值,
			   错误码 2 会被当成一个 1 写进数据位,
			   假数据比全 0 更难查 */
			return 0;
		}
	}

	/* 第3步: 从高电平起点数 40us 采样 */
	delay_us(40);

	return GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_4) ? 1 : 0;
}

u8 DHT11_Read_Byte(void)
{
	u8 DHT11_Data = 0;
	for(int i = 0 ; i < 8 ; i++)
	{
		DHT11_Data <<= 1;
		DHT11_Data |= DHT11_Read_Bit();
	}
	return DHT11_Data;
}

/*
	读一整帧: 5 个字节 = 湿度整数 / 湿度小数 / 温度整数 / 温度小数 / 校验和
	形参名字是错位的(见 main.c 里的说明), 调用时必须按位置传, 别看名字
*/
u8 DHT11_Read_Data(u8 *Integer, u8 *fractional, u8 *Integer_f, u8 *fractional_f)
{
	u8 buf[5] = {0}, i = 0;
	u8 resp = 0;

	resp = DHT11_Response();
	if(resp)
	{
		/* 原样上报, 不再统一包成 3:
		   1 = 握手失败(接线 / 上拉 / 供电)
		   3 = 校验失败(握手成功但数据读错位)
		   原来两种都返回 3, 日志里分不清到底是"没接上"还是"读错位" */
		return resp;
	}

	for(i = 0 ; i < 5 ; i++)
	{
		buf[i] = DHT11_Read_Byte();
	}

	/*
		前 4 个字节相加的低 8 位 = 第 5 个字节

		★ 这个校验有个盲区: 40 位全 0 时 0+0+0+0 == 0 也成立,
		  所以必须单独把"全 0"判成失败,
		  否则一个位都没读到也会报 OK。
	*/
	if(buf[0] + buf[1] + buf[2] + buf[3] == buf[4]
	   && (buf[0] | buf[1] | buf[2] | buf[3]) != 0)
	{
		*Integer      = buf[0];
		*Integer_f    = buf[1];
		*fractional   = buf[2];
		*fractional_f = buf[3];
		return DHT11_Send_SUCCEES;
	}

	return DHT11_Send_ERROR_3;
}
