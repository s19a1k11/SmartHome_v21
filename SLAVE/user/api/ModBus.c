#include "ModBus.h"
#include "485_PD7.h"	/* My_485_SendBuffer: PD7 只在这里面被操作 */
#include "TIM.h"		/* g_ms 时基 */
#include "servo.h"		/* Servo_Set_Angle */
#include "Led.h"		/* Led_Set_OnOff */
#include "beep.h"		/* Beep_Ctrl */

/* ==========================================================================
	本机地址: 全工程唯一的地址来源, 初值来自 ModBus.h 的 MODBUS_SLAVE_ADDR
	改从机号只改那一个宏, 这里不用动
   ========================================================================== */
u8 RS485_IP = MODBUS_SLAVE_ADDR;

/*
	寄存器缓存 ("柜子里的数")
	-----------------------------------------------------------------
	为什么要有这些变量:
		主机只等 250ms, 而读一次 DHT11 要 22~35ms。所以不能在应答
		路径上现场读传感器, 必须提前采好放在这里, 被问时直接念。
	谁能写: main.c 的采集节拍 (DHT11 / 光照)
	谁能读: ModBus_Get_Reg, 只允许 return, 不许做耗时动作
	下面的初值是"还没接传感器时的占位值", 接上后会被真实值覆盖
*/
u16 g_humi     = 0;      /* 湿度整数 %          */
u16 g_humi_dec = 0;      /* 湿度小数            */
u16 g_temp     = 0;      /* 温度整数 (摄氏度)    */
u16 g_temp_dec = 0;      /* 温度小数            */
u16 g_light    = 0;      /* 光照值 0~4095, 由 main.c 每 2 秒调用 Light_Read() 更新 */
u16 g_led      = 0;      /* 灯 0=灭 1=亮        */
u16 g_beep     = 0;      /* 蜂鸣器 0/1          */
u16 g_servo    = 1;      /* 舵机 0/1/2, 初值 1 = 和上电中位对齐 */

/*
	传感器健康状态
	-----------------------------------------------------------------
	fail_cnt: DHT11 连续失败次数, 成功一次就清零
	health  : 位图, bit0 = DHT11 故障
	为什么单独放系统区而不占输入区:
		输入区 6 个寄存器的含义(湿度/温度/光照/保留)已经被主机和云端
		固化成物模型了, 往里塞诊断位会破坏既有协议;
		系统区本来就是"固件信息", 扩两个寄存器不影响别人。
*/
static u8  s_dht_fail_cnt = 0;
static u16 s_sensor_health = 0;

void ModBus_TIM_Config(void)
{
	//使能对应的 APB 时钟组
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);

	//明确关闭定时器，如果不关闭则有些参数无法进行修改
	TIM_Cmd(TIM2,DISABLE);

	//定时器配置初始化
	TIM_TimeBaseInitTypeDef TIM_TimBaseInitStructure = {0};
	TIM_TimBaseInitStructure.TIM_Prescaler = 72 - 1;
	TIM_TimBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimBaseInitStructure.TIM_Period = DATA_FRAME_TIME_INTERVAL - 1;
	TIM_TimBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;

	TIM_TimeBaseInit(TIM2,&TIM_TimBaseInitStructure);

	/*
		设置当前定时为一次性定时,
		因为每次只有收取数据时需要判断是否断开或接收完毕
	*/
	TIM_SelectOnePulseMode(TIM2,TIM_OPMode_Single);

	//配置当前定时器中断
	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	NVIC_InitTypeDef NVIC_InitStructure = {0};
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	/*
		★ 优先级: 抢占 1 / 子 0 (USART2 是 抢占 0 / 子 1)
		含义: TIM2 可以被 USART2 抢占, 但 TIM2 不能抢占 USART2。
		USART2 收字节是硬实时(晚一步就丢字节);
		TIM2 判帧结束是软实时(晚几微秒无影响)。
		取值必须 0~3, 因为 NVIC_PriorityGroup_2 只给 2 位抢占。
	*/
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;

	NVIC_Init(&NVIC_InitStructure);

	//当前定时器不开启,只在接收数据时启用定时器
	TIM_SetCounter(TIM2,0);
}

/*
	TIM2 溢出中断 = 总线已经安静够 T3.5 = 一帧收完
	==================================================================
	★ 快进快出: 这里只做"置标志", 一个字符都不打印, 一个传感器都不读。
	  日志统一由 main 循环在最后一步 LOG_Flush() 发。
	  现在中断里的工作量是常数级: 清标志 -> 关定时器 -> 置 flag -> 退出。
*/
void TIM2_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM2,TIM_IT_Update))
	{
		//清除更新中断标志(只清 UIF, 不用 SR=0 整体清零)
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
		//明确定时器关闭，同时计数器数据清理
		TIM_Cmd(TIM2,DISABLE);
		TIM_SetCounter(TIM2,0);

		/*
			帧太短(不足 5 字节)直接丢弃并清空计数;
			够长则置标志, 由主循环取走。
			★ 这里不打印任何东西 —— 短帧在 main 里由 ModBus_Read_03_Cmd
			  统一记 "SLAVE FRAME TOO SHORT", 信息一样全, 但不占中断时间。
		*/
		if(usart2_value.count < 5)
		{
			usart2_value.count = 0;
			usart2_value.flag  = 0;
		}
		else
		{
			//表示数据接收完整
			usart2_value.flag = 1;
		}
	}
}

/*
	回异常帧: 功能码最高位置 1, 后跟异常码
	errcode: 0x01 非法功能  0x02 非法数据地址  0x03 非法数据值
*/
void ModBus_Send_Exception(u8 addr, u8 func, u8 errcode)
{
	u8 buf[5] = {0};
	u8 pos = 0;

	buf[pos++] = addr;
	buf[pos++] = func | 0x80;
	buf[pos++] = errcode;

	u16 crc = Modbus_CRC16(buf, pos);
	buf[pos++] = crc & 0x00FF;
	buf[pos++] = crc >> 8;

	My_485_SendBuffer(buf, pos);

	/* 异常帧: 一行文字足够, 不打整帧(异常本身就是低频事件, 但也没必要刷屏) */
	LOG_Str("[SLAVE] EXC sent code=0x");
	LOG_HexNum(errcode);
	LOG_Str("\r\n");
	/* 异常帧也是低频事件, 无条件打出来便于现场核对 */
	LOG_HexDump("TX", buf, pos);
}

/*
	寄存器范围检查: 这一段是否完整落在某一个区段内
	-----------------------------------------------------------------
	地址编码: 高字节 = 区段号, 低字节 = 段内偏移
		0x00xx 系统区(4格)   0x01xx 输入区(6格)   0x02xx 输出区(3格)
	为什么不用 start+count>REG_MAX:
		那个式子假设地址从 0 连续编号。地址变成 0x0100 后
		0x0100+6=262 恒大于 6, 每一帧合法请求都会被判越界, 只回 83 02
	为什么用 u32 算:
		addr+count 在 u16 上会回绕, addr=0xFFFA 时会算成 0 而误判通过
*/
static u8 ModBus_RegRange_Ok(u16 addr, u16 count)
{
	u32 s;
	u32 e;

	if(count == 0 || count > REG_MAX) return 0;

	s = (u32)addr;
	e = s + count;          /* 右端点(不含) */

	/*
		系统区起点就是 0, 所以这里不需要判断下界(s >= 0 恒成立),
		只判上界; 输入/输出区起点非 0, 两边都要判。
	*/
	if(e <= (u32)REG_SYS_BASE + REG_SYS_NUM)                                return 1;  /* 系统区 0x0000~0x0003 */
	if(s >= REG_INPUT_BASE  && e <= (u32)REG_INPUT_BASE  + REG_INPUT_NUM)  return 1;  /* 输入区 0x0100~0x0105 */
	if(s >= REG_OUTPUT_BASE && e <= (u32)REG_OUTPUT_BASE + REG_OUTPUT_NUM) return 1;  /* 输出区 0x0200~0x0202 */

	return 0;
}

/*
	读寄存器: 从"柜子"里念数
	-----------------------------------------------------------------
	★ 这个函数在应答路径上, 主机只给 250ms, 所以这里只允许 return,
	  绝不允许读传感器 (DHT11 一次 22~35ms, 会把应答拖到超时)
*/
u16 ModBus_Get_Reg(u16 addr)
{
	/* ---- 系统区 ---- */
	switch(addr)
	{
		case REG_SYS_FW_VERSION: return 0x0103;                 /* v1.3 = 固件 v21 */
		case REG_SYS_RUN_SEC:    return (u16)(g_ms / 1000);      /* 上电运行秒数 */
		case REG_SYS_SENSOR:     return s_sensor_health;         /* 传感器健康位图 */
		case REG_SYS_DHT_FAIL:   return (u16)s_dht_fail_cnt;     /* DHT11 连续失败次数 */
	}

	/* ---- 输入区: 温湿度 / 光照 / 保留 ---- */
	switch(addr)
	{
		case REG_IN_HUMI:     return g_humi;      /* 湿度整数 */
		case REG_IN_HUMI_DEC: return g_humi_dec;  /* 湿度小数 */
		case REG_IN_TEMP:     return g_temp;      /* 温度整数 */
		case REG_IN_TEMP_DEC: return g_temp_dec;  /* 温度小数 */
		case REG_IN_LIGHT:    return g_light;     /* 光照 */
		case REG_IN_RESERVED: return 0;           /* 保留(SR602 已移到主机侧) */
	}

	/* ---- 输出区: 灯 / 蜂鸣器 / 舵机 ---- */
	switch(addr)
	{
		case REG_OUT_LED:   return g_led;         /* 灯 0=灭 1=亮 */
		case REG_OUT_BEEP:  return g_beep;        /* 蜂鸣器 */
		case REG_OUT_SERVO: return g_servo;       /* 舵机 */
	}

	return 0xFFFF;   /* 没有这个寄存器 (和"值真的是0"区分开) */
}

/*
	写寄存器: 校验 -> 记录 -> 执行
	-----------------------------------------------------------------
	为什么先记录再执行:
		只动引脚不记录, 主机下一轮读回来永远是 0, 云端会以为命令没生效
		从而不停重发同一条命令
	为什么越界要 return 0 而不是截断成上限:
		截断会让错误无声地穿过整条链路; 拒绝会让主机收到异常帧, 能立刻发现
*/
u8 ModBus_Set_Reg(u16 addr, u16 val)
{
	switch(addr)
	{
		case REG_OUT_LED:                    /* 灯 0=灭 1=亮 */
			/* 本项目不做 PWM 调光, 所以这个寄存器是纯开关:
			   只接受 0 和 1, 写 2 及以上按越界处理, 不写入。
			   ★ 极性只在 Led_Set_OnOff 内部处理, 这里只传"亮/灭"的语义值 */
			if(val > 1) return 0;
			g_led = val;
			Led_Set_OnOff((u8)val);
			return 1;

		case REG_OUT_BEEP:                   /* 蜂鸣器 0/1 */
			if(val > 1) return 0;
			g_beep = val;
			Beep_Ctrl((u8)val);
			return 1;

		case REG_OUT_SERVO:                  /* 舵机 0/1/2 */
			if(val > 2) return 0;
			g_servo = val;
			Servo_Set_Angle((u8)val);        /* 0/1/2 -> 0/90/180 度 */
			return 1;

		default: return 0;
	}
}

/*
	采集层上报一次 DHT11 采集结果
	==================================================================
	★ 需求: "某个从机如果连续两次数据采集都没有成功则反馈云端该从机离线"
	  这句话在这里落地成两层:
	    1) 传感器层(本函数): 连续失败达到 SENSOR_FAIL_LIMIT(2) 就置故障位
	       0x0002 的 bit0, 并把连续失败次数写到 0x0003 方便现场定位。
	       数据寄存器保留上一次的有效值, 不用 0 覆盖 —— 0 会让云端曲线出现
	       一个假的"骤降到 0", 比留着旧值更难判读。
	    2) 通信层(主机侧): 连续两次问询都没有应答 -> 判离线 -> 上报 Online=false
	  两层分开的好处: "从机在线但传感器坏了" 和 "从机整个掉线了" 是两种
	  完全不同的故障, 云端能一眼分清该派人去换传感器还是查供电/接线。
*/
void ModBus_Sensor_Result(u8 ok)
{
	if(ok)
	{
		s_dht_fail_cnt = 0;
		s_sensor_health &= (u16)(~SENSOR_FAULT_DHT11);
	}
	else
	{
		/* 防溢出: 只需要知道"到没到门限", 不必真的累加到 65535 */
		if(s_dht_fail_cnt < 0xFF) s_dht_fail_cnt += 1;

		if(s_dht_fail_cnt >= SENSOR_FAIL_LIMIT)
		{
			s_sensor_health |= SENSOR_FAULT_DHT11;
		}
	}
}

u8 ModBus_Sensor_Fail_Count(void)
{
	return s_dht_fail_cnt;
}

u16 ModBus_Sensor_Health(void)
{
	return s_sensor_health;
}

/*
	从机应答: 收到主机的请求帧, 按功能码分发处理
	==================================================================
	谁调它: main.c 从机分支里, 每次 usart2_value.flag == 1 时调一次。
	        而 flag 是 TIM2 溢出中断置的, 含义是"总线已经安静够 3.5 个字符时间"。

	支持功能码:
		0x03 / 0x04  读保持/输入寄存器  (本工程里两者处理完全一样)
		0x06         写单个寄存器        (应答 = 把请求帧原样发回去)
		其余功能码   回异常码 0x01 = 功能码不认识

	★ 本函数里【只记日志不发送】, 所有日志走 LOG_* 进缓冲,
	  由 main 循环在最后一步统一 LOG_Flush()。
	  这样应答帧的发送时刻完全不受日志量影响 —— 这是"调试回显不能
	  挡在时序关键路径上"这条要求的核心实现。
*/
u8 ModBus_Read_03_Cmd(void)
{
	/* req 只是 usart2_value.data 的别名, 不是拷贝。 */
	u8 *req = usart2_value.data;

	/* 功能码固定在第 1 个字节(第 0 个字节是从机地址) */
	u8 func  = req[1];

	/* 这一帧实际收到几个字节, 由 USART2 接收中断一个个累加出来 */
	u16 len  = usart2_value.count;


	/*
		★ RX 原始帧只在调试模式下打(MODBUS_SLAVE_LOG)。
		  为什么不在正常路径无条件打:
		    从机日志是在应答【之后】才 Flush 的(不拖慢应答),
		    但它会拖慢采集节拍和下一帧的响应时机。
		    实测代价: 每帧 RX+TX 约 95 字符 = 99ms @9600;
		    主机每秒问 3 次就是 297ms/秒 花在发日志上,
		    而采集一拍只要 35ms —— 节拍会被彻底打乱。
		  需要看正常帧时把 MODBUS_SLAVE_LOG 置 1(慢速调试用)。
	*/
#if MODBUS_SLAVE_LOG
	LOG_HexDump("RX", req, len);
#endif

	// ---------- 第 1 道检查: 长度 ----------
	// 最短的请求帧是 8 字节: 地址1 + 功能码1 + 数据4 + CRC2
	if(len < 8)
	{
		LOG_Str("[SLAVE] FRAME TOO SHORT len=");
		LOG_Num(len);
		LOG_Str(" (need >=8)\r\n");
		return MODBUS_ERR_SHORT;
	}

	// ---------- 第 2 道检查: CRC ----------
	// ModBus 传 CRC 是"低字节在前", 拼回 16 位数时顺序要反过来
	u16 recv_crc = (req[len-1] << 8) | req[len-2];
	u16 calc_crc = Modbus_CRC16(req, len-2);
	if(recv_crc != calc_crc)
	{
		/*
			校验不过, 说明线上传的东西已经不可信了, 直接丢, 连异常帧都不回。
			★ 但日志必须打全: 收到的 CRC / 算出的 CRC / 帧长。
			  只打一句 "CRC FAIL" 等于没说 —— 现场最想知道的就是这三个数。
		*/
		LOG_Str("[SLAVE] CRC FAIL len=");
		LOG_Num(len);
		LOG_Str(" recv=0x");
		LOG_HexNum(recv_crc);
		LOG_Str(" calc=0x");
		LOG_HexNum(calc_crc);
		LOG_Str("\r\n");
		/*
			★ 出错时必须打原始帧 —— 这是唯一能回答"到底收到了什么"的证据。
			  正常帧才受 MODBUS_SLAVE_LOG 开关控制(为了不拖慢节拍),
			  出错是低频事件, 几行日志换一个可定位的线索, 值得。
		*/
		LOG_HexDump("RX", req, len);
		return MODBUS_ERR_CRC;
	}

	// ---------- 第 3 道检查: 地址 ----------
	// 靠"不是我就不吭声"实现多机共享总线
	if(req[0] != RS485_IP)
	{
		/*
			不是叫自己的帧, 静默丢弃(不回帧)。
			★ 但日志要留一行: 三台从机接在同一条线上, 每台都会看到
			  别人的帧。如果现场发现"某台板子日志里全是别人的地址",
			  就说明它收到了帧但地址配错了 —— 这是排查地址冲突的关键线索。
			  只打地址不打断整帧, 避免刷爆缓冲。
		*/
		/*
			★ 地址不匹配的日志要保留(排查地址冲突的关键线索),
			  但必须限流: 主机每秒问 3 次、每次问 3 台, 每台从机
			  都会看到另外两台的帧 —— 不限流的话这条日志每秒刷 6 次,
			  而且永远打的是同样两个地址。
			  做法: 只在"第一次"或"地址变了"时打。
		*/
		{
			static u8 s_last_skip_addr = 0xFF;
			if(req[0] != s_last_skip_addr)
			{
				s_last_skip_addr = req[0];
				LOG_Str("[SLAVE] ADDR SKIP me=");
				LOG_Num(RS485_IP);
				LOG_Str(" got=");
				LOG_Num(req[0]);
				LOG_Str("\r\n");
			}
		}
		return MODBUS_ERR_ADDR;
	}

	// ---------- 应答缓冲区 ----------
	u8 resp[5 + REG_MAX*2 + 2] = {0};
	u8 pos = 0;

	switch(func)
	{
		// -------- 读寄存器 (03 和 04 走同一段代码) --------
		case 0x03:
		case 0x04:
		{
			u16 start = (req[2] << 8) | req[3];
			u16 count = (req[4] << 8) | req[5];

			// 范围检查: 这一段必须完整落在某一个区段内
			if(!ModBus_RegRange_Ok(start, count))
			{
				LOG_Str("[SLAVE] REG RANGE BAD start=0x");
				LOG_HexNum(start);
				LOG_Str(" count=");
				LOG_Num(count);
				LOG_Str("\r\n");
				// 0x02 = 非法数据地址。回的是异常帧(功能码 | 0x80)
				ModBus_Send_Exception(req[0], func, 0x02);
				return 0;
			}

			// 应答帧头: 地址 + 功能码 + 后面数据一共占几个字节
			resp[pos++] = RS485_IP;
			resp[pos++] = func;
			resp[pos++] = (u8)(count * 2);

			// 逐个把寄存器值填进数据区(高字节在前)
			for(u8 i = 0 ; i < count ; i++)
			{
				u16 v = ModBus_Get_Reg(start + i);
				resp[pos++] = (u8)(v >> 8);
				resp[pos++] = (u8)(v & 0x00FF);
			}

			// 最后算 CRC(低字节先发 —— 这是 ModBus RTU 唯一的顺序例外)
			u16 crc = Modbus_CRC16(resp, pos);
			resp[pos++] = (u8)(crc & 0x00FF);
			resp[pos++] = (u8)(crc >> 8);

			// 交给 485 发出去(内部自己切 PD7 并等 TC)
			My_485_SendBuffer(resp, pos);
#if MODBUS_SLAVE_LOG
			LOG_HexDump("TX", resp, pos);
#endif
			break;
		}

		// -------- 写单个寄存器 --------
		case 0x06:
		{
			u16 addr = (req[2] << 8) | req[3];
			u16 val  = (req[4] << 8) | req[5];

			if(!ModBus_RegRange_Ok(addr, 1))
			{
				LOG_Str("[SLAVE] WR REG RANGE BAD addr=0x");
				LOG_HexNum(addr);
				LOG_Str("\r\n");
				ModBus_Send_Exception(req[0], func, 0x02);
				return 0;
			}

			// 真正干活的地方: 内部按地址决定改哪个全局变量 + 驱动哪个外设。
			// ★ 返回值必须接: 它在"值越界"时返回 0。
			//   不接就变成"假装成功": 寄存器没变、引脚没动,
			//   但下面照样把请求帧原样回显, 主机以为写成了。
			if(!ModBus_Set_Reg(addr, val))
			{
				LOG_Str("[SLAVE] WR VALUE REJECT addr=0x");
				LOG_HexNum(addr);
				LOG_Str(" val=");
				LOG_Num(val);
				LOG_Str("\r\n");
				ModBus_Send_Exception(req[0], func, 0x03);
				return 0;
			}

			// 06 的标准应答就是"把请求帧原样发回去"
			My_485_SendBuffer(req, 8);
#if MODBUS_SLAVE_LOG
			LOG_HexDump("TX", req, 8);
#endif
			break;
		}

		// -------- 不支持的功能码 --------
		default:
		{
			// 0x01 = 非法功能码。最高位在 Send_Exception 里置 1
			LOG_Str("[SLAVE] FUNC UNSUPPORTED 0x");
			LOG_HexNum(func);
			LOG_Str("\r\n");
			ModBus_Send_Exception(req[0], func, 0x01);
			break;
		}
	}

	return 1;
}

/*
	ModBus 的 CRC-16 (多项式 0xA001 是 0x8005 位反转后的写法)
	==================================================================
	初始 crc = 0xFFFF
	每来一个字节: 先异或进 crc 的低 8 位, 再循环 8 次
	每次看最低位: 是 1 就右移后再异或 0xA001, 是 0 就只右移
*/
uint16_t Modbus_CRC16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    size_t i, j;

    for (i = 0; i < length; i++)
    {
        crc ^= data[i];

        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

/*
	调试辅助: 全部改成写日志缓冲, 不再直接发串口
	★ 改造前这几个函数直接 USART1_SendString, 会插在应答路径里阻塞。
	  现在它们只做 RAM 拷贝, 真正的发送在 main 循环最后一步。
*/
void ModBus_PrintHex(const char *tag, u8 *buffer, u16 len)
{
	LOG_HexDump(tag, buffer, len);
}

void ModBus_PrintNum(u16 n)
{
	LOG_Num(n);
}

void ModBus_PrintHexNum(u16 v)
{
	LOG_HexNum(v);
}
