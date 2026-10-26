#include "all.h"

#define USART1_BRR (9600)
#define USART2_BRR (9600)
#define USART3_BRR (115200)

#ifdef MODBUS_SLAVE

/*
	【卡7】把 0~255 打成一串十进制字符 —— 现在写进日志缓冲, 不直接发串口
*/
static void Print_U8(u8 v)
{
	LOG_Num(v);
}

/*
	【卡10】把 0~65535 打成一串十进制字符
	为什么不能直接用 Print_U8: 光照值是 0~4095, 早就超过 u8 的 255,
	                             Print_U8 会把高位截掉, 1420 会显示成 92
*/
static void Print_U16(u16 v)
{
	LOG_Num(v);
}

#endif	/* MODBUS_SLAVE */

/*
	采集节拍: 多久读一次 DHT11 / 光照, 单位 ms
	为什么是 2000:
	  DHT11 采样率上限 1Hz, 读太勤会返回上一帧或直接失败;
	  一次读取约 35ms(起始信号 delay_ms(35) + 位判定), 2 秒一拍占用 1.75%;
	  这个节拍对室内温湿度/光照完全够 —— 它们不会在 1 秒内剧变。
*/
#define SLAVE_ACQ_INTERVAL  2000

#ifdef MODBUS_SLAVE

int main(void)
{
	/*
		★ 先设置 NVIC 优先级分组
		2 位抢占 + 2 位子优先级。
		不设置时默认是 Group0(0 位抢占)，导致所有中断同级、无法互相抢占。
	*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	USART1_Init(USART1_BRR);
	LOG_Init();                         /* 日志缓冲必须在 USART1 之后初始化 */

	ModBus_TIM_Config();

	/*
		★ 顺序调整: 先把 485 方向脚 PD7 配成输出并拉低(接收态),
		再使能 USART2。否则 USART2 使能时 PD7 还是复位默认的浮空输入,
		若总线上此刻有数据, 可能误收几个字节。
		★ PD7 的初始化封装在 My_485_Init() 内部(485_PD7.c),
		  它自己会先配推挽输出再拉低, 调用方不用再手动拉低。
	*/
	My_485_Init();
	USART2_485_Init(USART2_BRR);

	Beep_Init();
	Led_Init();

	/*
		【卡6】启动 TIM6 时基: 72MHz / 72 = 1MHz 计数, 数 1000 次 = 1ms
		从此 g_ms 每 1ms +1, ModBus 的 0x0001 寄存器 = g_ms/1000 = 上电秒数
	*/
	TIM6_Init(72, 1000);

	/* 【卡9】舵机: TIM3_CH2 -> PA7, 50Hz, 上电先回 90 度中位 */
	Servo_Init();

	/* DHT11 数据脚 PE4: 先只开时钟并置成上拉输入(总线空闲态) */
	GPIO_E4_Init();
	GPIO_E4_Input();

	/* 光照: PA1(ADC1_IN1), 只初始化一次 */
	Light_Init();

	/*
		USART3: PB10(TX) / PB11(RX), 115200 8N1
		从机固件里它是"可选的本地直控口", 详见 usart3.c 顶部说明。
		保留初始化是为了: 以后接本地屏 / 出厂前单板测试。
	*/
	USART3_Init(USART3_BRR);

	/*
		★ 版本标记: 开机就打印固件版本 + 主从身份 + 本机地址
		用途: 一眼确认板子上烧的到底是哪一版固件、是哪一号从机
		(三块板烧同一份源码, 只改 MODBUS_SLAVE_ADDR 宏)
	*/
	LOG_Str("Init OK\r\n");
	/*
		★ 版本串必须每次改动都更新, 否则无法确认板上跑的是哪一版。
	*/
	LOG_Str("[VER] 2026-10-24-v21 SLAVE addr=");
	Print_U8(RS485_IP);
	LOG_Str(" max=");
	Print_U8(MODBUS_SLAVE_MAX);
	LOG_Str("\r\n");

	/*
		★ [BUILD] 行: 编译时刻, 由编译器自动填。
		  用途: 区分"改了没重编"和"编了没烧进去"。
	*/
	LOG_Str("[BUILD] ");
	LOG_Str(__DATE__);
	LOG_Str(" ");
	LOG_Str(__TIME__);
	LOG_Str("\r\n");
	LOG_Flush();                        /* 开机信息: 此刻还没进入循环, 可以直接发 */

	/*
		【设计说明: 总线只由主机发起】
		从机不再主动往 485 上发任何东西(改造前每 2 秒推 21 字节上报帧,
		靠 (地址-1)*700ms 错相避让)。原因:
		  1) 485 是半双工单总线, 主动上报和主机问询应答会撞车。
		     错相只能降低概率, 主机轮询周期一旦漂移就迟早撞上。
		  2) 主机本来就在轮询, 数据新鲜度够, 再叠一层上报是重复通道。
		  3) 少一条通道, 离线判定才有唯一依据。
		现在: 总线只由主机发起, 从机只应答。总线冲突从"概率问题"变成"不存在"。

		★ 采集仍然是本机自己做, 2 秒一拍, 结果写进寄存器缓存;
		  主机来问时直接念缓存, 不在应答路径上读传感器
		  (DHT11 一次 22~35ms, 会让主机 200ms 超时)。
	*/
	u8  dht_hi = 0, dht_hf = 0, dht_ti = 0, dht_tf = 0, dht_ret = 0;
	u32 acq_last = 0;
	u8  acq_first = 1;      /* 1 = 还没采过, 上电立刻采一次 */
	u8  acq_just_done = 0;  /* 1 = 本轮采过, 用于决定要不要记日志 */

	while(1)
	{
		acq_just_done = 0;

		/*
			★ 采集节拍
			用 (u32)(g_ms - acq_last) >= 间隔 而不是 if(g_ms > acq_last+间隔):
			g_ms 是 u32, 49 天回绕一次, 直接比大小会在回绕那一瞬间卡死。
		*/
		if(acq_first || (u32)(g_ms - acq_last) >= SLAVE_ACQ_INTERVAL)
		{
			acq_first = 0;
			acq_last  = g_ms;
			acq_just_done = 1;

			/* ---------- 1. DHT11 温湿度 ---------- */
			/*
				DHT11_Read_Data 的四个形参名字是错位的, 必须按【位置】传, 别看名字:
				  第1个 Integer      <- 湿度整数
				  第2个 fractional   <- 温度整数   (名字叫"小数", 实际装的是温度整数!)
				  第3个 Integer_f    <- 湿度小数
				  第4个 fractional_f <- 温度小数
			*/
			dht_ret = DHT11_Read_Data(&dht_hi, &dht_ti, &dht_hf, &dht_tf);

			if(dht_ret == DHT11_Send_SUCCEES)
			{
				/* 读成功: 写进变量, 主机发 03 查询输入区就能拿到 */
				g_humi     = dht_hi;
				g_humi_dec = dht_hf;
				g_temp     = dht_ti;
				g_temp_dec = dht_tf;

				ModBus_Sensor_Result(1);        /* 成功一次, 连续失败计数清零 */

				LOG_Str("[DHT11 OK] H=");
				Print_U8(dht_hi);  LOG_Str(".");
				Print_U8(dht_hf);
				LOG_Str(" T=");
				Print_U8(dht_ti);  LOG_Str(".");
				Print_U8(dht_tf);
				LOG_Str("\r\n");
			}
			else
			{
				/*
					读失败(没接线 / 上拉丢失 / 时序被打断):
					★ 保留旧值, 不覆盖 —— 用 0 覆盖会让云端曲线出现假的"骤降到 0"
					★ 交给 ModBus_Sensor_Result 记连续失败次数,
					  连续 2 次失败就置系统区 0x0002 的故障位
				*/
				ModBus_Sensor_Result(0);

				LOG_Str("[DHT11 ERR] code=");
				Print_U8(dht_ret);
				LOG_Str(" fail_cnt=");
				Print_U8(ModBus_Sensor_Fail_Count());
				if(ModBus_Sensor_Health() & SENSOR_FAULT_DHT11)
				{
					LOG_Str(" FAULT keep old\r\n");
				}
				else
				{
					LOG_Str(" keep old\r\n");
				}
			}

			/* ---------- 2. 光照 ---------- */
			g_light = Light_Read();
			LOG_Str("[LIGHT] L=");
			Print_U16(g_light);
			LOG_Str("\r\n");
		}

		/*
			---------- 3. 处理 485 上收到的一整帧 ----------
			★ 帧的判定在 TIM2 中断里完成(总线安静够 T3.5 -> 置 flag),
			  这里只负责"取走 + 应答 + 清缓冲"。
			★ 这一段【必须排在采集之后、日志发送之前】:
			  主机等应答只给 200ms, 采集最坏占 35ms, 先应答再打日志
			  才能保证应答不被日志拖慢。
		*/
		if(usart2_value.flag)
		{
			usart2_value.flag = 0;

			/*
				ModBus_Read_03_Cmd 内部自己完成: 校验 -> 组帧 -> 切 PD7 -> 发送 -> 切回
				PD7 的操作全部收敛在 485_PD7.c
			*/
			(void)ModBus_Read_03_Cmd();

			/*
				★ 必须清缓冲, 否则下一帧会拼在上一帧屁股后面, 长度和 CRC 全乱。
				  用 USART2_Recv_Clear() 而不是直接写 count=0:
				  它内部关中断再清, 避免清到一半又被中断写进一个字节。
			*/
			USART2_Recv_Clear();
		}

		/* ---------- 4. 可选的本地直控口(默认关闭, 见 usart3.c) ---------- */
		USART3_Poll();

		/*
			---------- 5. ★ 最后一步才发日志 ----------
			这是"串口1 必须在所有需要完成的操作结束后再发送"这条要求的落地点:
			采集做完了、485 应答发完了、本地直控处理完了, 才把这一轮攒下的
			日志一次性打出去。

			为什么顺序这么重要:
			  USART1 是 9600 阻塞发送, 1 个字符 1.04ms。如果边采集边打印,
			  打印时间会直接叠加到 485 应答延迟上; 而应答窗口只有 200ms,
			  打几十个字符就吃掉一大半。
			  现在无论这一轮攒了多少日志, 都不会挡在采集和应答前面。

			代价: 日志会晚一轮出现(最多 2 秒)。对调试完全够 ——
			      看的是"这台从机在干什么", 不是微秒级时序。
		*/
		(void)acq_just_done;            /* 保留变量便于以后按需细分日志 */
		LOG_Flush();
	}
}
#endif
