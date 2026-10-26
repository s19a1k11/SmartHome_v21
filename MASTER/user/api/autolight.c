#include "autolight.h"
#include "delay.h"
#include "log.h"

/*
	夜晚回家自动开灯实现
	==================================================================
	所有判断都在这里(主循环上下文), 中断只负责把 SR602 的边沿记下来。
	本文件里【没有任何日志直接发串口】—— 全部走 LOG_* 进缓冲,
	由 main 循环最后一步统一 LOG_Flush()。
*/

/* 门口那盏灯 = 1 号从机的灯 */
#define AUTOLIGHT_SLAVE_INDEX   (0)         /* g_slave[0] 就是 1 号 */
#define AUTOLIGHT_LED_REG       (OUT_REG_LED)

static u32 s_last_action_ms = 0;    /* 最近一次开/关灯的时刻, 用于防抖 */
static u16 s_avg_light      = 0;    /* 最近一次算出的平均光照 */
static u8  s_is_night       = 0;    /* 最近一次夜晚判定结果 */
static u8  s_leave_ms_set   = 0;    /* 1 = 人已离开, 关灯计时已启动 */
static u32 s_leave_ms       = 0;    /* 人离开的起始时刻 */

/*
	动作最小间隔
	为什么要它: 485 写一笔事务约 8~30ms, 加上从机应答最多 200ms。
	如果判定逻辑每轮(1 秒)都动作一次, 总线会被写命令占满。
	3 秒足够让"开灯 -> 人走近 -> 灯保持亮"这个过程稳定, 也避免
	光照在阈值附近抖动时灯反复开关(现象是灯闪)。
*/
#define AUTOLIGHT_MIN_GAP_MS    (3000)

void AutoLight_Init(void)
{
	s_last_action_ms = 0;
	s_avg_light      = 0xFFFF;
	s_is_night       = 0;
	s_leave_ms_set   = 0;
	s_leave_ms       = 0;
}

/*
	算三台在线从机的光照平均值
	-----------------------------------------------------------------
	★ 只统计 online=1 且数据可信(未达采集失败门限)的从机:
	  离线从机的 light 是"最后一次读到的旧值", 可能已经几小时了,
	  拿它参与平均会把"现在是不是晚上"判断错。
	★ 一台可用都没有时返回 0xFFFF, 上层据此【不做任何动作】:
	  宁可不开灯, 也不能凭 0 或旧值乱开灯(半夜灯自己亮起来更糟)。
*/
static u16 autolight_calc_avg(void)
{
	u32 sum = 0;
	u16 n   = 0;
	u8  i   = 0;
	u16 v   = 0;

	for(i = 0 ; i < SLAVE_NUM ; i++)
	{
		if(g_slave[i].online == 0)          continue;   /* 离线, 数据不可信 */
		if(ModBus_Data_Failed(i))           continue;   /* 在线但采集坏了 */

		v = g_slave[i].light;
		if(v > 4095) v = 4095;                          /* 防御: ADC 只有 12 位 */

#if LIGHT_INVERT
		v = (u16)(4095 - v);
#endif

		sum += v;
		n   += 1;
	}

	if(n == 0) return 0xFFFF;
	return (u16)(sum / n);
}

u16 AutoLight_Avg_Light(void)
{
	return s_avg_light;
}

u8 AutoLight_Is_Night(void)
{
	return s_is_night;
}

/*
	按当前状态写 1 号从机的灯
	返回 1 = 写成功 / 0 = 没写或写失败
*/
static u8 autolight_write_led(u16 on)
{
	u8 stat = 0;

	/* 从机离线时不要发: 发了也是总线空等一个超时, 白白拖慢轮询 */
	if(g_slave[AUTOLIGHT_SLAVE_INDEX].online == 0)
	{
		LOG_Str("[AUTO] slave1 offline, skip led\r\n");
		return 0;
	}

	stat = ModBus_Write_Single_Reg(g_slave[AUTOLIGHT_SLAVE_INDEX].addr,
	                               AUTOLIGHT_LED_REG, on);
	if(stat == MODBUS_OK)
	{
		/* 本地缓存也同步, 免得下一轮又判断成"灯还是旧状态" */
		g_slave[AUTOLIGHT_SLAVE_INDEX].led = on;
		LOG_Str(on ? "[AUTO] light ON\r\n" : "[AUTO] light OFF\r\n");
		return 1;
	}

	LOG_Str("[AUTO] led write fail stat=");
	LOG_Num(stat);
	LOG_Str("\r\n");
	return 0;
}

u8 AutoLight_Poll(u8 sr602_human)
{
	u32 now = Delay_GetMs();
	u8  acted = 0;
	u16 avg = 0;

	/* ---------- 1. 先刷新平均光照与夜晚判定 ---------- */
	avg = autolight_calc_avg();
	s_avg_light = avg;

	if(avg == 0xFFFF)
	{
		/*
			没有任何一台从机的光照可用 -> 不动作。
			s_is_night 保持上一次的值(不更新), 但不参与开灯判断 ——
			下面的判断里会先看 avg 是否有效。
		*/
		LOG_Str("[AUTO] no valid light, hold\r\n");
		return 0;
	}

	/*
		夜晚判定: 光照低于阈值 = 晚上
		为什么用"低于"而不是"高于": 光敏模块的 AO 电压在光照强时
		通常更高(光敏电阻阻值变小), 所以 ADC 读数越大 = 越亮。
		如果你的模块相反, 把 autolight.h 的 LIGHT_INVERT 改成 1,
		这里不用动 —— 翻转已经在 calc_avg 里做完了。
	*/
	s_is_night = (u8)((avg < NIGHT_LIGHT_THRESHOLD) ? 1 : 0);

	/*
		★ LUX 日志降频(v21 加)。
		  原来每秒打一条, 一小时 3600 行, 内容几乎不变(3710~3716 波动),
		  把真正有用的信息(云端下发 / ESP 报错 / 从机恢复)全冲走了。

		  而且 @9600 一条 38 字符 = 40ms, 每秒白烧 40ms 的串口带宽,
		  在"2 台从机离线"的场景下会把这个已经吃紧的轮询周期顶爆。

		  策略: 每 30 秒打一条; 但【状态变化时立刻打】
		        (夜晚判定翻转 / 有人无人翻转 / 灯状态翻转),
		        这样该看到的还是能看到。
	*/
	{
		static u16 s_lux_log_cnt = 0;
		static u8  s_last_night = 0xFF;
		static u8  s_last_human = 0xFF;
		static u8  s_last_led   = 0xFF;
		u8 changed = 0;

		if(s_is_night != s_last_night) { changed = 1; s_last_night = s_is_night; }
		if(sr602_human != s_last_human) { changed = 1; s_last_human = sr602_human; }
		if(g_slave[AUTOLIGHT_SLAVE_INDEX].led != s_last_led)
		{
			changed = 1;
			s_last_led = (u8)g_slave[AUTOLIGHT_SLAVE_INDEX].led;
		}

		s_lux_log_cnt += 1;

		if(changed || s_lux_log_cnt >= 30)
		{
			s_lux_log_cnt = 0;
			LOG_Str("[LUX] avg=");
			LOG_Num(avg);
			LOG_Str(" night=");
			LOG_Num(s_is_night);
			LOG_Str(" human=");
			LOG_Num(sr602_human);
			LOG_Str(" led=");
			LOG_Num(g_slave[AUTOLIGHT_SLAVE_INDEX].led);
			LOG_Str(changed ? " *\r\n" : "\r\n");   /* * 表示状态变化 */
		}
	}

	/* ---------- 2. 开灯判定: 有人 + 夜晚 + 灯灭 ---------- */
	if(sr602_human && s_is_night && g_slave[AUTOLIGHT_SLAVE_INDEX].led == 0)
	{
		/* 人回来了, 取消关灯计时 */
		s_leave_ms_set = 0;

		/* 防抖: 距上次动作太近就跳过, 避免灯闪 */
		if((u32)(now - s_last_action_ms) >= AUTOLIGHT_MIN_GAP_MS)
		{
			if(autolight_write_led(1))
			{
				s_last_action_ms = now;
				acted = 1;
			}
		}
	}

	/* ---------- 3. 关灯判定: 无人 + 超时 + 灯亮 ---------- */
	else if((sr602_human == 0) && (g_slave[AUTOLIGHT_SLAVE_INDEX].led != 0))
	{
		/*
			人走了: 第一次进来时记下时刻, 之后一直等。
			★ 为什么用"离开时刻"而不是"最后看到人的时刻":
			  两者等价(人一消失就记), 但用离开时刻更直观,
			  而且 SR602 有保持时间, 用它更贴合"人确实走了"这个事件。
		*/
		if(s_leave_ms_set == 0)
		{
			s_leave_ms_set = 1;
			s_leave_ms = now;
			LOG_Str("[AUTO] human left, start timer\r\n");
		}

		if((u32)(now - s_leave_ms) >= AUTOLIGHT_OFF_DELAY_MS)
		{
			if((u32)(now - s_last_action_ms) >= AUTOLIGHT_MIN_GAP_MS)
			{
				if(autolight_write_led(0))
				{
					s_last_action_ms = now;
					s_leave_ms_set = 0;
					acted = 1;
				}
			}
		}
	}
	else
	{
		/* 人还在 / 不需要关灯: 清掉关灯计时, 免得下次误判 */
		if(sr602_human)
		{
			s_leave_ms_set = 0;
		}
	}

	return acted;
}
