#ifndef _LCD_H
#define _LCD_H

#include "common.h"

/*
	中景园 1.8 寸 TFT (ST7735S) 绘图接口
	==================================================================
	★ 只保留本项目实际用到的函数, 去掉了:
	    - 中文显示 (需要 4 套汉字字模, 体积大)
	    - 画圆 / 画线 / 画矩形 (界面不需要)
	    - 显示图片 (需要 16KB 图片表)
	  这样省下约 20KB Flash。以后需要中文再单独加字模。

	坐标系统: 竖屏 128x160, (0,0) 在左上角
	颜色格式: RGB565 (16 位)
*/

/* ---- RGB565 常用颜色 ---- */
#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define GREEN       0x07E0
#define CYAN        0x7FFF
#define YELLOW      0xFFE0
#define MAGENTA     0xF81F
#define GRAY        0x8430
#define LGRAY       0xC618      /* 浅灰, 常用作背景 */
#define DARKBLUE    0x01CF
#define LIGHTBLUE   0x7D7C
#define ORANGE      0xFD20

/* ---- 填充与画点 ---- */
void LCD_Fill(u16 xsta, u16 ysta, u16 xend, u16 yend, u16 color);
void LCD_DrawPoint(u16 x, u16 y, u16 color);

/* ---- 字符与字符串 ---- */
void LCD_ShowChar(u16 x, u16 y, u8 num, u16 fc, u16 bc, u8 sizey, u8 mode);
void LCD_ShowString(u16 x, u16 y, const u8 *p, u16 fc, u16 bc, u8 sizey, u8 mode);

/* ---- 数字 ---- */
u32  mypow(u8 m, u8 n);
void LCD_ShowIntNum(u16 x, u16 y, u16 num, u8 len, u16 fc, u16 bc, u8 sizey);
void LCD_ShowFloatNum1(u16 x, u16 y, float num, u8 len, u16 fc, u16 bc, u8 sizey);

#endif
