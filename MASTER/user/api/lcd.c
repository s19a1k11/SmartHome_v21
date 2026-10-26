#include "lcd.h"
#include "lcd_init.h"
#include "lcdfont.h"

/*
	中景园 1.8 寸 ST7735S 绘图实现
	==================================================================
	移植自官方 lcd.c, 只保留本项目需要的函数, 去掉了中文/画圆/图片。

	★ 一个必须注意的点: LCD_WR_DATA 每写一个像素都要经过 LCD_Writ_Bus,
	  而 LCD_Writ_Bus 每次都翻转 CS。全屏 128x160 = 20480 像素 x 2 字节
	  = 40960 次 CS 翻转, 每次还要等 TXE —— 全屏填充实测约 150ms。

	  所以:
	    1. 只在开机做一次全屏填充
	    2. 平时刷新只填变化的小区域
	    3. 显示更新放在主循环最后, 优先级最低
*/

/**
 *  @brief 在指定区域填充颜色
 *
 *  @param xsta,ysta 起始坐标
 *  @param xend,yend 结束坐标
 *  @param color     RGB565 颜色
 */
void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color)
{
	u16 i = 0;
	u16 j = 0;

	if(xsta > xend) { u16 t = xsta; xsta = xend; xend = t; }
	if(ysta > yend) { u16 t = ysta; ysta = yend; yend = t; }
	if(xend >= LCD_W) xend = LCD_W - 1;
	if(yend >= LCD_H) yend = LCD_H - 1;

	LCD_Address_Set(xsta, ysta, xend, yend);

	for(i = ysta; i <= yend; i++)
	{
		for(j = xsta; j <= xend; j++)
		{
			LCD_WR_DATA(color);
		}
	}
}

/**
 *  @brief 画一个点
 */
void LCD_DrawPoint(u16 x, u16 y, u16 color)
{
	if(x >= LCD_W || y >= LCD_H) return;

	LCD_Address_Set(x, y, x, y);
	LCD_WR_DATA(color);
}

/**
 *  @brief 显示一个字符
 *
 *  @param x,y    左上角坐标
 *  @param num    要显示的 ASCII 字符
 *  @param fc     前景色(字)
 *  @param bc     背景色
 *  @param sizey  字号: 12 / 16 / 24 / 32 (字宽 = sizey/2)
 *  @param mode   0 = 非叠加(带背景色填充) 1 = 叠加(只画字, 背景保留)
 */
void LCD_ShowChar(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 sizey, u8 mode)
{
	u8  temp = 0;
	u8  sizex = 0;
	u8  t = 0;
	u8  m = 0;
	u16 i = 0;
	u16 TypefaceNum = 0;
	u16 x0 = x;

	sizex = (u8)(sizey / 2);
	TypefaceNum = (u16)((sizex / 8 + ((sizex % 8) ? 1 : 0)) * sizey);

	/*
		★ 精简版字模表只有 8x16 一种字号。
		  传其它字号直接返回, 不读越界(改造前这里会读 ascii_1206 等不存在的数组)。
		  需要别的字号时, 从原版 lcdfont.h 把对应字模加回来。
	*/
	if(sizey != 16)
	{
		return;
	}

	/* 可打印字符范围保护: 字模表只覆盖 0x20~0x7E */
	if(num < ' ' || num > '~')
	{
		num = ' ';
	}
	num = (u8)(num - ' ');

	/*
		★ 越界保护(v22 加)
		改造前只有 LCD_DrawPoint 里判越界, 而 mode=0 走的是
		直接写显存的路径(LCD_Address_Set + LCD_WR_DATA), 完全没检查。
		如果 x 或 y 超出屏幕:
		  - 轻则字符被 ST7735 自己裁掉(看起来"少半个字")
		  - 重则 LCD_Address_Set 发出非法坐标, 屏幕可能花屏
		这里在入口统一拦住: 完全在屏幕外的直接返回,
		部分在外的也返回(宁可不显示, 也不要花屏)。
	*/
	if(x >= LCD_W || y >= LCD_H)
	{
		return;
	}
	if((u16)(x + sizex) > LCD_W || (u16)(y + sizey) > LCD_H)
	{
		return;
	}

	if(!mode)
	{
		LCD_Address_Set(x, y, (u16)(x + sizex - 1), (u16)(y + sizey - 1));
	}

	for(i = 0; i < TypefaceNum; i++)
	{
		temp = ascii_1608[num][i];

		for(t = 0; t < 8; t++)
		{
			if(!mode)
			{
				/* 非叠加: 逐像素写, 背景色也写 */
				if(temp & (0x01 << t)) LCD_WR_DATA(fc);
				else                   LCD_WR_DATA(bc);
				m++;
				if(m % sizex == 0)
				{
					m = 0;
					break;
				}
			}
			else
			{
				/* 叠加: 只画前景色点, 不动背景 */
				if(temp & (0x01 << t)) LCD_DrawPoint(x, y, fc);
				x++;
				if((u16)(x - x0) == sizex)
				{
					x = x0;
					y++;
					break;
				}
			}
		}
	}
}

/**
 *  @brief 显示字符串 (遇到 '\0' 结束)
 */
void LCD_ShowString(u16 x, u16 y, const u8 *p, u16 fc, u16 bc, u8 sizey, u8 mode)
{
	while(*p != '\0')
	{
		LCD_ShowChar(x, y, *p, fc, bc, sizey, mode);
		x = (u16)(x + sizey / 2);
		p++;
	}
}

/**
 *  @brief 求 m 的 n 次方 (给数字显示用)
 */
u32 mypow(u8 m, u8 n)
{
	u32 result = 1;

	while(n--)
	{
		result *= m;
	}
	return result;
}

/**
 *  @brief 显示整数
 *
 *  @param len 显示位数(不足左边补 0, 超过则只显示低位)
 */
void LCD_ShowIntNum(u16 x, u16 y, u16 num, u8 len, u16 fc, u16 bc, u8 sizey)
{
	u8  t = 0;
	u8  temp = 0;
	u8  enshow = 0;
	u8  sizex = (u8)(sizey / 2);

	for(t = 0; t < len; t++)
	{
		temp = (u8)((num / mypow(10, (u8)(len - t - 1))) % 10);
		if(enshow == 0 && t < (u8)(len - 1))
		{
			if(temp == 0)
			{
				LCD_ShowChar((u16)(x + t * sizex), y, ' ', fc, bc, sizey, 0);
				continue;
			}
			else
			{
				enshow = 1;
			}
		}
		LCD_ShowChar((u16)(x + t * sizex), y, (u8)(temp + '0'), fc, bc, sizey, 0);
	}
}

/**
 *  @brief 显示一位小数的浮点数
 *
 *  ★ 不用 sprintf("%f"): 那会拉进几百字节的浮点格式化代码。
 *    这里手工拆整数/小数部分, 只用到一次 float -> int 转换。
 */
void LCD_ShowFloatNum1(u16 x, u16 y, float num, u8 len, u16 fc, u16 bc, u8 sizey)
{
	u8  t = 0;
	u8  temp = 0;
	u8  enshow = 0;
	u8  sizex = (u8)(sizey / 2);
	u16 num1 = 0;

	/* 保留一位小数, 四舍五入 */
	num1 = (u16)(num * 10.0f + 0.5f);

	for(t = 0; t < len; t++)
	{
		temp = (u8)((num1 / mypow(10, (u8)(len - t - 1))) % 10);
		if(enshow == 0 && t < (u8)(len - 1))
		{
			if(temp == 0)
			{
				LCD_ShowChar((u16)(x + t * sizex), y, ' ', fc, bc, sizey, 0);
				continue;
			}
			else
			{
				enshow = 1;
			}
		}
		LCD_ShowChar((u16)(x + t * sizex), y, (u8)(temp + '0'), fc, bc, sizey, 0);
	}

	/* 小数点 */
	LCD_ShowChar((u16)(x + len * sizex), y, '.', fc, bc, sizey, 0);

	/* 小数位 */
	temp = (u8)(num1 % 10);
	LCD_ShowChar((u16)(x + (len + 1) * sizex), y, (u8)(temp + '0'), fc, bc, sizey, 0);
}
