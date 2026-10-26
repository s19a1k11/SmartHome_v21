#include "cli.h"
#include "usart1.h"
#include "log.h"
#include "cloud.h"
#include "modbus.h"
#include "autolight.h"
#include "sr602.h"
#include "usart3.h"
#include "esp8266.h"
#include "lcd.h"
#include "lcd_init.h"
#include "lcd_ui.h"
#include "delay.h"

/*
	串口1 模拟 MQTT 指令实现
	==================================================================
	所有解析都在主循环(CLI_Poll), 中断只负责把字节搬进 usart1_value。
	所有输出走 LOG_*, 由 main 循环最后一步统一 Flush。
*/

/* 逐帧日志开关(默认关: 打开会吃掉大量串口带宽) */
static u8 s_modbus_log = 0;

/* 上一轮已经处理过的字节数, 用来判断"有没有新内容" */
static u16 s_last_len = 0;

void CLI_Init(void)
{
	s_modbus_log = 0;
	s_last_len = 0;
}

void CLI_Set_ModBus_Log(u8 on)
{
	s_modbus_log = (u8)(on ? 1 : 0);
}

u8 CLI_Get_ModBus_Log(void)
{
	return s_modbus_log;
}

/*
	去掉行尾的 \r \n 和首尾空格
	返回处理后的有效长度
*/
static u16 cli_trim(u8 *buf, u16 len)
{
	u16 s = 0;
	u16 e = len;

	/* 去尾部 */
	while(e > 0)
	{
		u8 c = buf[e - 1];
		if(c == '\r' || c == '\n' || c == ' ' || c == '\t') e -= 1;
		else break;
	}
	/* 去头部 */
	while(s < e)
	{
		u8 c = buf[s];
		if(c == ' ' || c == '\t') s += 1;
		else break;
	}

	/* 前移到缓冲区开头(原地压缩) */
	if(s > 0)
	{
		u16 i;
		for(i = 0; i < (u16)(e - s); i++) buf[i] = buf[s + i];
		e = (u16)(e - s);
	}

	buf[e] = 0;     /* 补字符串结尾, 方便后面用字符串函数 */
	return e;
}

/* 比较两个字符串是否相等(不区分大小写, 用于命令关键字) */
static u8 cli_streq(const char *a, const char *b)
{
	while(*a && *b)
	{
		char ca = *a;
		char cb = *b;
		if(ca >= 'A' && ca <= 'Z') ca = (char)(ca - 'A' + 'a');
		if(cb >= 'A' && cb <= 'Z') cb = (char)(cb - 'A' + 'a');
		if(ca != cb) return 0;
		a += 1;
		b += 1;
	}
	return (u8)((*a == 0 && *b == 0) ? 1 : 0);
}

/* 判断字符串是否以 prefix 开头(不区分大小写) */
static u8 cli_startswith(const char *s, const char *prefix)
{
	while(*prefix)
	{
		char cs = *s;
		char cp = *prefix;
		if(cs >= 'A' && cs <= 'Z') cs = (char)(cs - 'A' + 'a');
		if(cp >= 'A' && cp <= 'Z') cp = (char)(cp - 'A' + 'a');
		if(cs != cp) return 0;
		s += 1;
		prefix += 1;
	}
	return 1;
}

/* 打印当前版本与关键配置 */
static void cli_print_version(void)
{
	LOG_Str("[CLI] fw=2026-10-24-v21 MASTER slaves=");
	LOG_Num(SLAVE_NUM);
	LOG_Str("\r\n");
	LOG_Str("[CLI] reply_timeout=");
	LOG_Num(MODBUS_REPLY_TIMEOUT);
	LOG_Str("ms upload=");
	LOG_Num(CLOUD_UPLOAD_INTERVAL / 1000);
	LOG_Str("s modbus_log=");
	LOG_Num(s_modbus_log);
	LOG_Str("\r\n");
}

/* 打印三台从机的完整状态 */
static void cli_print_stat(void)
{
	u8 i = 0;

	LOG_Str("[CLI] ===== slaves =====\r\n");
	for(i = 0; i < SLAVE_NUM; i++)
	{
		LOG_Str("[CLI] #");
		LOG_Num(i + 1);
		LOG_Str(" addr=");
		LOG_Num(g_slave[i].addr);
		LOG_Str(g_slave[i].online ? " ONLINE" : " OFFLINE");
		LOG_Str(" err=");
		LOG_Num(g_slave[i].err_cnt);
		LOG_Str(" dErr=");
		LOG_Num(g_slave[i].data_err_cnt);
		LOG_Str("\r\n");

		if(g_slave[i].online)
		{
			LOG_Str("[CLI]    T=");
			LOG_Num(g_slave[i].temp_int);
			LOG_Str(".");
			LOG_Num(g_slave[i].temp_dec);
			LOG_Str(" H=");
			LOG_Num(g_slave[i].humi_int);
			LOG_Str(".");
			LOG_Num(g_slave[i].humi_dec);
			LOG_Str(" Lux=");
			LOG_Num(g_slave[i].light);
			LOG_Str("\r\n");
			LOG_Str("[CLI]    LED=");
			LOG_Num(g_slave[i].led);
			LOG_Str(" BEEP=");
			LOG_Num(g_slave[i].beep);
			LOG_Str(" SERVO=");
			LOG_Num(g_slave[i].servo);
			LOG_Str(" run=");
			LOG_Num(g_slave[i].run_seconds);
			LOG_Str("s\r\n");
		}
	}

	LOG_Str("[CLI] SR602=");
	LOG_Num(SR602_Current());
	LOG_Str(" LuxAvg=");
	LOG_Num(AutoLight_Avg_Light());
	LOG_Str(" Night=");
	LOG_Num(AutoLight_Is_Night());
	LOG_Str(" led1=");
	LOG_Num(g_slave[0].led);
	LOG_Str("\r\n");
}

static void cli_print_help(void)
{
	LOG_Str("[CLI] ===== commands =====\r\n");
	LOG_Str("[CLI] JSON (same as cloud downlink):\r\n");
	LOG_Str("[CLI]   {\"Led1\":true}            led1 on\r\n");
	LOG_Str("[CLI]   {\"Led1\":false}           led1 off\r\n");
	LOG_Str("[CLI]   {\"Beep1\":true}           beep1 on\r\n");
	LOG_Str("[CLI]   {\"SG90_1\":\"clockwise\"}  servo1 180deg\r\n");
	LOG_Str("[CLI]   {\"SG90_1\":\"counter_clockwise\"} servo1 0deg\r\n");
	LOG_Str("[CLI]   {\"stm32\":false,...}      ignore whole cmd\r\n");
	LOG_Str("[CLI] local:\r\n");
	LOG_Str("[CLI]   @help  @stat  @poll  @upload\r\n");
	LOG_Str("[CLI]   @autolight on|off  @sr602  @version  @esp  @lcd\r\n");
	LOG_Str("[CLI]   @log on|off   (per-frame modbus log)\r\n");
}

/*
	执行一条本地命令(@ 开头)
	返回 1 = 已处理
*/
static u8 cli_run_local(u8 *line)
{
	/* 跳过 @ */
	const char *cmd = (const char *)line + 1;

	if(cli_streq(cmd, "help"))
	{
		cli_print_help();
		return 1;
	}

	if(cli_streq(cmd, "version"))
	{
		cli_print_version();
		return 1;
	}

	if(cli_streq(cmd, "stat"))
	{
		cli_print_stat();
		return 1;
	}

	if(cli_streq(cmd, "lcd"))
	{
		/*
			★ @lcd: 屏幕自检
			  依次画: 纯红 -> 纯绿 -> 纯蓝 -> 正常界面
			  用来快速判断是"SPI 完全不通"还是"数据通了但显示错"。
			    三个颜色都能看到 -> 屏幕和接线都正常, 问题在界面逻辑
			    只有白色/黑色    -> SPI 没通, 查 PA5/PA7/CS/DC
		*/
		LOG_Str("[CLI] lcd test: RED\r\n");
		LCD_Fill(0, 0, LCD_W - 1, LCD_H - 1, RED);
		delay_ms(600);
		LOG_Str("[CLI] lcd test: GREEN\r\n");
		LCD_Fill(0, 0, LCD_W - 1, LCD_H - 1, GREEN);
		delay_ms(600);
		LOG_Str("[CLI] lcd test: BLUE\r\n");
		LCD_Fill(0, 0, LCD_W - 1, LCD_H - 1, BLUE);
		delay_ms(600);
		LOG_Str("[CLI] lcd test: restore UI\r\n");
		LCD_UI_Init();
		LCD_UI_Refresh_All();
		LOG_Str("[CLI] lcd test done\r\n");
		return 1;
	}

	if(cli_streq(cmd, "esp"))
	{
		/*
			★ @esp: 诊断 ESP8266 链路
			  依次做: 发 AT 看回不回 OK -> 如果回了, 说明模块活着,
			          问题在热点/TCP/MQTT 层; 如果不回, 说明是模块级故障。
			  同时把"收到了什么"原样打出来, 用来区分:
			    没收到  -> 接线/供电/模块波特率
			    乱码    -> 主机波特率与模块不一致
			    别的内容-> 模块停在透传态
		*/
		LOG_Str("[CLI] esp probe...\r\n");
		{
			u8 chpd = 0, gpio0 = 0;
			ESP8266_Get_Ctrl_Pins(&chpd, &gpio0);
			LOG_Str("[CLI] CH_PD(PA4)=");
			LOG_Num(chpd);
			LOG_Str(" GPIO0(PA15)=");
			LOG_Num(gpio0);
			LOG_Str("\r\n");
			if(chpd == 0)
			{
				LOG_Str("[CLI] !! CH_PD is LOW: module is DISABLED (powered off)\r\n");
			}
			if(gpio0 == 0)
			{
				LOG_Str("[CLI] !! GPIO0 is LOW: module in DOWNLOAD mode, AT firmware not running\r\n");
			}
		}
		USART3_Recv_Clear();
		USART3_SendString("AT\r\n");
		{
			u16 w = 0;
			while(w < 2000)
			{
				if(usart3_value.count > 0) break;
				delay_ms(10);
				w += 10;
			}
		}
		{
			static u8 raw[128];
			u16 n = USART3_Get_Raw(raw, sizeof(raw));
			u16 i = 0;

			LOG_Str("[CLI] esp got=");
			LOG_Num(n);
			LOG_Str("B raw=[");
			for(i = 0; i < n && i < 60; i++)
			{
				u8 c = raw[i];
				if(c >= 0x20 && c < 0x7F) LOG_Chr((char)c);
				else if(c == '\r') LOG_Str("\\r");
				else if(c == '\n') LOG_Str("\\n");
				else LOG_Chr('.');
			}
			LOG_Str("]\r\n");

			if(n == 0)
			{
				LOG_Str("[CLI] no bytes from ESP: check PB10->RX, PB11->TX, 3.3V power, module baud\r\n");
			}
			else
			{
				/* 简单判断有没有 OK */
				u8 has_ok = 0;
				for(i = 0; i + 1 < n; i++)
				{
					if(raw[i] == 'O' && raw[i+1] == 'K') { has_ok = 1; break; }
				}
				LOG_Str(has_ok ? "[CLI] ESP alive (got OK)\r\n"
				               : "[CLI] ESP replied but no OK: baudrate mismatch or wrong mode\r\n");
			}
		}
		return 1;
	}

	if(cli_streq(cmd, "sr602"))
	{
		LOG_Str("[CLI] SR602=");
		LOG_Num(SR602_Current());
		LOG_Str(" (1=human) LuxAvg=");
		LOG_Num(AutoLight_Avg_Light());
		LOG_Str(" Night=");
		LOG_Num(AutoLight_Is_Night());
		LOG_Str("\r\n");
		return 1;
	}

	if(cli_streq(cmd, "poll"))
	{
		LOG_Str("[CLI] poll all...\r\n");
		ModBus_Poll_All();
		ModBus_Update_Online();
		cli_print_stat();
		return 1;
	}

	if(cli_streq(cmd, "upload"))
	{
		LOG_Str("[CLI] upload now\r\n");
		if(!Cloud_Upload_All())
		{
			LOG_Str("[CLI] upload skipped (gap or too long)\r\n");
		}
		return 1;
	}

	if(cli_startswith(cmd, "autolight"))
	{
		const char *arg = cmd + 9;      /* 跳过 "autolight" */
		while(*arg == ' ') arg += 1;

		if(cli_startswith(arg, "on"))
		{
			/* 直接写 1 号从机的灯, 绕过夜晚判定(测试用) */
			if(ModBus_Write_Single_Reg(g_slave[0].addr, OUT_REG_LED, 1) == MODBUS_OK)
			{
				g_slave[0].led = 1;
				LOG_Str("[CLI] led1 forced ON\r\n");
			}
			else
			{
				LOG_Str("[CLI] led1 write FAIL\r\n");
			}
			return 1;
		}
		if(cli_startswith(arg, "off"))
		{
			if(ModBus_Write_Single_Reg(g_slave[0].addr, OUT_REG_LED, 0) == MODBUS_OK)
			{
				g_slave[0].led = 0;
				LOG_Str("[CLI] led1 forced OFF\r\n");
			}
			else
			{
				LOG_Str("[CLI] led1 write FAIL\r\n");
			}
			return 1;
		}
		LOG_Str("[CLI] usage: @autolight on|off\r\n");
		return 1;
	}

	if(cli_startswith(cmd, "log"))
	{
		const char *arg = cmd + 3;
		while(*arg == ' ') arg += 1;

		if(cli_startswith(arg, "on"))
		{
			s_modbus_log = 1;
			LOG_Str("[CLI] modbus log ON (poll will be slow)\r\n");
			return 1;
		}
		if(cli_startswith(arg, "off"))
		{
			s_modbus_log = 0;
			LOG_Str("[CLI] modbus log OFF\r\n");
			return 1;
		}
		LOG_Str("[CLI] usage: @log on|off\r\n");
		return 1;
	}

	LOG_Str("[CLI] unknown cmd: @");
	LOG_Str(cmd);
	LOG_Str("  (try @help)\r\n");
	return 1;
}

u8 CLI_Poll(void)
{
	u16 len = 0;
	u16 i = 0;
	u8 *buf = 0;

	/*
		★ 判帧方式: 用 IDLE 中断置的 flag(总线安静 = 一行收完了)。
		  为什么不用"等换行符": 串口助手不一定勾"加回车换行",
		  而 IDLE 判帧不依赖对方发什么, 更稳。
	*/
	if(usart1_value.flag == 0)
	{
		/* 没有完整一帧, 但要注意: 上一帧被处理过之后 count 会清零 */
		if(usart1_value.count == 0) s_last_len = 0;
		return 0;
	}

	len = usart1_value.count;
	buf = usart1_value.data;

	/*
		★ 去重: 同一条命令可能因为 IDLE 多次触发而被看到两次。
		  用"字节数和内容长度都没变"判断"这条已经处理过了"。
		  为什么不用时间戳: 更简单, 而且命令不会在几毫秒内重复发两次。
	*/
	if(len == s_last_len && len > 0)
	{
		usart1_value.flag = 0;
		return 0;
	}
	s_last_len = len;

	/* 拷一份出来处理, 避免边处理边被中断改写 */
	{
		static u8 s_line[CLI_LINE_MAX];
		u16 n = (len < (u16)(CLI_LINE_MAX - 1)) ? len : (u16)(CLI_LINE_MAX - 1);

		for(i = 0; i < n; i++) s_line[i] = buf[i];
		s_line[n] = 0;

		/* 先释放接收缓冲, 让下一行可以进来(处理期间不阻塞接收) */
		usart1_value.flag = 0;
		USART1_Recv_Clear();
		s_last_len = 0;

		n = cli_trim(s_line, n);

		if(n == 0)
		{
			return 0;       /* 空行, 忽略 */
		}

		/* 回显收到的内容 —— 现场能看到"我发的东西板子确实收到了" */
		LOG_Str("[CLI] rx: ");
		LOG_Str((const char *)s_line);
		LOG_Str("\r\n");

		if(s_line[0] == '@')
		{
			return cli_run_local(s_line);
		}

		if(s_line[0] == '{')
		{
			/*
				★ 关键: 走的是云端下发【完全相同】的函数。
				  所以这里能通, 接上云平台就一定能通。
			*/
			LOG_Str("[CLI] -> Cloud_Handle_Downlink\r\n");
			Cloud_Handle_Downlink((const char *)s_line);

			/* 下发后立刻回读一次, 让人马上看到执行结果 */
			ModBus_Poll_All();
			ModBus_Update_Online();
			cli_print_stat();
			return 1;
		}

		LOG_Str("[CLI] not a command: need '{' (json) or '@' (local)\r\n");
		return 1;
	}
}
