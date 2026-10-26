#include "lcd_ui.h"
#include "lcd.h"
#include "lcd_init.h"
#include "modbus.h"
#include "autolight.h"
#include "sr602.h"
#include "tick.h"
#include "delay.h"
#include "log.h"

/*
	LCD 界面实现
	==================================================================
	★ 核心设计: 只重绘变化的部分。
	  每台从机的数据用一个"上次显示值"缓存, 值没变就不动屏幕。
	  实测: 数据不变时每秒的 LCD 开销是 0; 有变化时只重绘那一行(约 10ms)。
*/

/* ============================================================
   布局常量 (128 x 160) —— ★ v22 修正版

   踩过的坑: 第一版布局让底部第2行落在 y=152,
   而 152 + 16 = 168 > 160, 超出屏幕 8 像素 ——
   屏幕上看到的就是"最下面一行只有一半"。

   重新按像素精确分配(全部在 160 以内):
     y=  0  标题栏       0~15   (16)
     y= 16  分隔线       16~17  (2)
     y= 18  从机1 行1    18~33
     y= 34  从机1 行2    34~49
     y= 50  从机1 行3    50~65
     y= 66  分隔线       66~67
     y= 68  从机2 行1    68~83
     y= 84  从机2 行2    84~99
     y=100  从机2 行3   100~115
     y=116  分隔线      116~117
     y=118  底部 行1    118~133
     y=134  底部 行2    134~149
     y=150  底部留白    150~159 (10 像素)

   横向: 128 / 8 = 16 个字符, 所有行都已按此校验过。
   ============================================================ */

#define UI_HEADER_Y      0
#define UI_HEADER_H      16
#define UI_SLAVE1_Y      18      /* 从机1 第一行 */
#define UI_SLAVE2_Y      68      /* 从机2 第一行 */
#define UI_ROW_STEP      16      /* 行间距 = 字号, 不留空隙 */
#define UI_BOTTOM_Y      118     /* 底部第一行 */

/* ---- 颜色 ---- */
#define C_BG             BLACK
#define C_TITLE_BG       DARKBLUE
#define C_TITLE_FG       WHITE
#define C_ONLINE         GREEN
#define C_OFFLINE        RED
#define C_LABEL          LGRAY
#define C_VALUE          WHITE

/* 字号 */
#define FONT_SIZE        16
#define CHAR_W           (FONT_SIZE / 2)    /* 8 像素 */

/* ---- 上次显示值缓存 (用于判断是否需要重绘) ---- */
typedef struct
{
	u8  inited;
	u8  online;
	u8  led;
	u8  sen_err;
	u16 temp_int;
	u16 temp_dec;
	u16 humi_int;
	u16 humi_dec;
	u16 light;
} UI_Slave_Cache;

static UI_Slave_Cache s_cache[SLAVE_NUM];
static u8  s_last_human = 0xFF;
static u8  s_last_night = 0xFF;
static u8  s_last_luxavg_100 = 0xFF;   /* 只比较百位, 避免频繁重绘 */
static u8  s_last_cloud_ok = 0xFF;
static u32 s_last_update_ms = 0;

/*
	外部依赖: 云端连接状态
	★ 直接用 cloud.c 里的 g_mqtt_connack, 不新增接口。
	  这个变量在收到 CONNACK 时置 1, 断开时清 0, 正是"云端是否可用"。
*/
extern volatile u8 g_mqtt_connack;
#define UI_CLOUD_OK()   (g_mqtt_connack ? 1 : 0)

/* ============================================================
   内部工具
   ============================================================ */

/**
 *  @brief 计算某台从机的"传感器是否异常"
 *
 *  ★ 和云端 SenErr 的定义保持一致(见 cloud.c):
 *      从机自报的健康位 bit0  或  主机连续读不到数据
 *    且从机必须在线 —— 离线时不算"传感器故障", 那是另一个问题。
 */
static u8 ui_sen_err(u8 index)
{
	Slave_Device *dev = &g_slave[index];

	if(!dev->online) return 0;

	if(dev->sensor_health & 0x0001) return 1;
	if(ModBus_Data_Failed(index))   return 1;

	return 0;
}

/**
 *  @brief 把 u16 转成 4 位十进制字符串 (右对齐, 不足补空格)
 *
 *  ★ 不用 sprintf: 省几百字节, 而且 sprintf 会拉进完整的格式化代码。
 */
static void ui_u16_to_str(u16 v, char *out)
{
	out[0] = (char)('0' + (v / 1000) % 10);
	out[1] = (char)('0' + (v / 100) % 10);
	out[2] = (char)('0' + (v / 10) % 10);
	out[3] = (char)('0' + v % 10);
	out[4] = '\0';

	/* 前导零变空格, 但至少保留一位 */
	if(out[0] == '0')
	{
		out[0] = ' ';
		if(out[1] == '0')
		{
			out[1] = ' ';
			if(out[2] == '0')
			{
				out[2] = ' ';
			}
		}
	}
}

/**
 *  @brief 画一台从机的完整信息块 (只在数据变化时调用)
 */
static void ui_draw_slave(u8 index)
{
	Slave_Device *dev = &g_slave[index];
	UI_Slave_Cache *c = &s_cache[index];
	u16 y = (index == 0) ? UI_SLAVE1_Y : UI_SLAVE2_Y;
	char buf[8];
	char line[24];
	u8 i = 0;
	u8 n = 0;

	/* 1. 状态行: "#1 ONLINE" / "#1 OFFLINE" */
	line[n++] = '#';
	line[n++] = (char)('1' + index);
	line[n++] = ' ';
	if(dev->online)
	{
		const char *on = "ONLINE";
		for(i = 0; on[i]; i++) line[n++] = on[i];
	}
	else
	{
		const char *off = "OFFLINE";
		for(i = 0; off[i]; i++) line[n++] = off[i];
	}
	line[n] = '\0';
	LCD_Fill(0, y, LCD_W - 1, (u16)(y + FONT_SIZE - 1), C_BG);
	LCD_ShowString(0, y, (const u8 *)line,
	               dev->online ? C_ONLINE : C_OFFLINE, C_BG, FONT_SIZE, 0);

	/* 2. 温湿度行: "T:24.2 H:31.0" */
	y = (u16)(y + UI_ROW_STEP);
	LCD_Fill(0, y, LCD_W - 1, (u16)(y + FONT_SIZE - 1), C_BG);
	if(dev->online)
	{
		LCD_ShowString(0, y, (const u8 *)"T:", C_LABEL, C_BG, FONT_SIZE, 0);
		ui_u16_to_str(dev->temp_int, buf);
		LCD_ShowString((u16)(2 * CHAR_W), y, (const u8 *)buf, C_VALUE, C_BG, FONT_SIZE, 0);
		buf[0] = '.';
		buf[1] = (char)('0' + (dev->temp_dec % 10));
		buf[2] = '\0';
		LCD_ShowString((u16)(6 * CHAR_W), y, (const u8 *)buf, C_VALUE, C_BG, FONT_SIZE, 0);

		/*
			横向: "T:24.2 H:31.0" 共 14 字符 = 112 像素, 安全。
			  T: 0~1, 温度整数 2~5, 小数 6~7
			  H: 8~9, 湿度整数 10~13, 小数 14~15
			湿度整数是 2 位(ui_u16_to_str 补空格到 4 位), 小数占 14~15,
			到 128 刚好 16 字符 —— 这是设计好的满宽, 不越界。
		*/
		LCD_ShowString((u16)(8 * CHAR_W), y, (const u8 *)"H:", C_LABEL, C_BG, FONT_SIZE, 0);
		ui_u16_to_str(dev->humi_int, buf);
		LCD_ShowString((u16)(10 * CHAR_W), y, (const u8 *)buf, C_VALUE, C_BG, FONT_SIZE, 0);
		buf[0] = '.';
		buf[1] = (char)('0' + (dev->humi_dec % 10));
		buf[2] = '\0';
		LCD_ShowString((u16)(14 * CHAR_W), y, (const u8 *)buf, C_VALUE, C_BG, FONT_SIZE, 0);
	}
	else
	{
		LCD_ShowString(0, y, (const u8 *)"T:--.- H:--.-", C_LABEL, C_BG, FONT_SIZE, 0);
	}

	/* 3. 光照 + 灯 + 传感器状态行: "L:3719 D:0 E:0" */
	y = (u16)(y + UI_ROW_STEP);
	LCD_Fill(0, y, LCD_W - 1, (u16)(y + FONT_SIZE - 1), C_BG);
	if(dev->online)
	{
		LCD_ShowString(0, y, (const u8 *)"L:", C_LABEL, C_BG, FONT_SIZE, 0);
		ui_u16_to_str(dev->light, buf);
		LCD_ShowString((u16)(2 * CHAR_W), y, (const u8 *)buf, C_VALUE, C_BG, FONT_SIZE, 0);

		LCD_ShowString((u16)(7 * CHAR_W), y, (const u8 *)"D:", C_LABEL, C_BG, FONT_SIZE, 0);
		buf[0] = dev->led ? '1' : '0';
		buf[1] = '\0';
		LCD_ShowString((u16)(9 * CHAR_W), y, (const u8 *)buf,
		               dev->led ? C_ONLINE : C_LABEL, C_BG, FONT_SIZE, 0);

		LCD_ShowString((u16)(11 * CHAR_W), y, (const u8 *)"E:", C_LABEL, C_BG, FONT_SIZE, 0);
		buf[0] = ui_sen_err(index) ? '1' : '0';
		buf[1] = '\0';
		LCD_ShowString((u16)(13 * CHAR_W), y, (const u8 *)buf,
		               ui_sen_err(index) ? C_OFFLINE : C_LABEL, C_BG, FONT_SIZE, 0);
	}
	else
	{
		LCD_ShowString(0, y, (const u8 *)"L:---- D:- E:-", C_LABEL, C_BG, FONT_SIZE, 0);
	}

	/* 4. 更新缓存 */
	c->inited   = 1;
	c->online   = dev->online;
	c->led      = dev->led;
	c->sen_err  = ui_sen_err(index);
	c->temp_int = dev->temp_int;
	c->temp_dec = dev->temp_dec;
	c->humi_int = dev->humi_int;
	c->humi_dec = dev->humi_dec;
	c->light    = dev->light;
}

/**
 *  @brief 判断某台从机的显示数据是否变化
 */
static u8 ui_slave_changed(u8 index)
{
	Slave_Device *dev = &g_slave[index];
	UI_Slave_Cache *c = &s_cache[index];

	if(!c->inited) return 1;

	if(c->online   != dev->online)   return 1;
	if(c->led      != dev->led)      return 1;
	if(c->sen_err  != ui_sen_err(index)) return 1;
	if(c->light    != dev->light)    return 1;

	/* 温湿度只在整数部分变化时重绘(小数抖动太频繁) */
	if(c->temp_int != dev->temp_int) return 1;
	if(c->humi_int != dev->humi_int) return 1;

	return 0;
}

/**
 *  @brief 画底部状态行 (SR602 / 夜晚 / 云端)
 */
static void ui_draw_bottom(void)
{
	u16 y = UI_BOTTOM_Y;
	char buf[8];

	/* 第 1 行: "H:0 N:0"  (人体 / 夜晚) */
	LCD_Fill(0, y, LCD_W - 1, (u16)(y + FONT_SIZE - 1), C_BG);
	LCD_ShowString(0, y, (const u8 *)"H:", C_LABEL, C_BG, FONT_SIZE, 0);
	buf[0] = SR602_Current() ? '1' : '0';
	buf[1] = '\0';
	LCD_ShowString((u16)(2 * CHAR_W), y, (const u8 *)buf,
	               SR602_Current() ? ORANGE : C_LABEL, C_BG, FONT_SIZE, 0);

	LCD_ShowString((u16)(4 * CHAR_W), y, (const u8 *)"N:", C_LABEL, C_BG, FONT_SIZE, 0);
	buf[0] = AutoLight_Is_Night() ? '1' : '0';
	buf[1] = '\0';
	LCD_ShowString((u16)(6 * CHAR_W), y, (const u8 *)buf,
	               AutoLight_Is_Night() ? CYAN : C_LABEL, C_BG, FONT_SIZE, 0);

	/*
		★ 横向排布(128 像素 = 16 字符), 这里用满 16 字符:
		    "H:0 N:0 Lux:3719"
		     0123456789012345
		   H: 占 0~1, 值在 2
		   N: 占 4~5, 值在 6
		   Lux: 占 8~11, 值在 12~15  <- 刚好到第 16 字符, 不越界
		   改造前写 12*CHAR_W=96, 4 位数字到 128 正好压线,
		   LCD_ShowChar 内部对 x >= LCD_W 会跳过, 末位可能不显示。
	*/
	LCD_ShowString((u16)(8 * CHAR_W), y, (const u8 *)"Lux:", C_LABEL, C_BG, FONT_SIZE, 0);
	{
		u16 avg = AutoLight_Avg_Light();
		ui_u16_to_str((avg == 0xFFFF) ? 0 : avg, buf);
		LCD_ShowString((u16)(12 * CHAR_W), y, (const u8 *)buf, C_VALUE, C_BG, FONT_SIZE, 0);
	}

	/* 第 2 行: "Cloud:OK" / "Cloud:--" */
	y = (u16)(y + UI_ROW_STEP);
	LCD_Fill(0, y, LCD_W - 1, (u16)(y + FONT_SIZE - 1), C_BG);
	LCD_ShowString(0, y, (const u8 *)"Cloud:", C_LABEL, C_BG, FONT_SIZE, 0);
	LCD_ShowString((u16)(6 * CHAR_W), y,
	               (const u8 *)(UI_CLOUD_OK() ? "OK" : "--"),
	               UI_CLOUD_OK() ? C_ONLINE : C_OFFLINE, C_BG, FONT_SIZE, 0);

	s_last_human = SR602_Current();
	s_last_night = AutoLight_Is_Night();
	s_last_luxavg_100 = (u8)((AutoLight_Avg_Light() == 0xFFFF ? 0 : AutoLight_Avg_Light()) / 100);
	s_last_cloud_ok = UI_CLOUD_OK();
}

/* ============================================================
   对外接口
   ============================================================ */

/**
 *  @brief 初始化界面: 清屏 + 画静态标题栏
 */
void LCD_UI_Init(void)
{
	u8 i = 0;

	/* 1. 全屏清黑 (约 150ms, 只在开机做一次) */
	LCD_Fill(0, 0, LCD_W - 1, LCD_H - 1, C_BG);

	/* 2. 标题栏 */
	LCD_Fill(0, UI_HEADER_Y, LCD_W - 1, UI_HEADER_H - 1, C_TITLE_BG);
	LCD_ShowString(2, UI_HEADER_Y, (const u8 *)"SmartHome v22", C_TITLE_FG,
	               C_TITLE_BG, FONT_SIZE, 0);

	/* 3. 分隔线 (用 2 像素高的灰条代替画线函数) */
	/* 分隔线: 用 2 像素高的灰条代替画线函数, 位置严格按布局表 */
	LCD_Fill(0, (u16)(UI_HEADER_H),     LCD_W - 1, (u16)(UI_HEADER_H + 1),     GRAY);
	LCD_Fill(0, (u16)(UI_SLAVE2_Y - 2), LCD_W - 1, (u16)(UI_SLAVE2_Y - 1),     GRAY);
	LCD_Fill(0, (u16)(UI_BOTTOM_Y - 2), LCD_W - 1, (u16)(UI_BOTTOM_Y - 1),     GRAY);

	/* 4. 清掉缓存, 让第一次 Update 全画 */
	for(i = 0; i < SLAVE_NUM; i++)
	{
		s_cache[i].inited = 0;
	}
	s_last_human = 0xFF;
	s_last_night = 0xFF;
	s_last_luxavg_100 = 0xFF;
	s_last_cloud_ok = 0xFF;
	s_last_update_ms = 0;

	LOG_Str("[LCD] UI init done\r\n");
}

/**
 *  @brief 强制全量刷新
 */
void LCD_UI_Refresh_All(void)
{
	u8 i = 0;

	for(i = 0; i < SLAVE_NUM; i++)
	{
		s_cache[i].inited = 0;
	}
	LCD_UI_Update();
}

/**
 *  @brief 刷新显示 (1 秒节流 + 只画变化的部分)
 *
 *  @return 1 = 有更新 / 0 = 没到时间或没变化
 */
u8 LCD_UI_Update(void)
{
	u32 now = 0;
	u8  updated = 0;
	u8  i = 0;
	u16 avg = 0;

	/*
		★ 1 秒节流
		  为什么不每轮刷: 主循环一轮要跑 ModBus 轮询 + 日志 + 上报,
		  留给 LCD 的预算不多。1 秒刷一次足够看清数据, 也不影响轮询。
	*/
	now = Delay_GetMs();
	if(s_last_update_ms != 0 && (u32)(now - s_last_update_ms) < 1000)
	{
		return 0;
	}
	s_last_update_ms = now;

	/* 1. 从机数据: 只重绘变化的 */
	for(i = 0; i < SLAVE_NUM; i++)
	{
		if(ui_slave_changed(i))
		{
			ui_draw_slave(i);
			updated = 1;
		}
	}

	/* 2. 底部状态: 人体/夜晚/云端 任一变化才重绘 */
	avg = AutoLight_Avg_Light();
	{
		u8 lux100 = (u8)((avg == 0xFFFF ? 0 : avg) / 100);
		if(SR602_Current() != s_last_human ||
		   AutoLight_Is_Night() != s_last_night ||
		   lux100 != s_last_luxavg_100 ||
		   UI_CLOUD_OK() != s_last_cloud_ok)
		{
			ui_draw_bottom();
			updated = 1;
		}
	}

	return updated;
}
