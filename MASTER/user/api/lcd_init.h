#ifndef _LCD_INIT_H
#define _LCD_INIT_H

#include "common.h"

/*
	中景园 1.8 寸 TFT (ST7735S) 驱动 —— 引脚定义与底层时序
	==================================================================
	硬件来源:
	  D:\00_开班软件\PCBrun\中景园ZJY180S0800TG01技术资料\
	  02-1.8LCD程序源码\11-1.8LCD显示屏STM32F103硬件SPI+DMA例程

	屏幕规格:
	  驱动 IC  : ST7735S
	  分辨率   : 128 x 160
	  接口     : 4 线 SPI (SCL/SDA/RES/DC/CS/BLK)
	  模块引脚 : BLK CS DC RST SDA SCL VDD GND  (8 针)

	★ 引脚分配: 例程是 PA5/PA7/PA1/PA2/PA3/PA4, 但【不能照抄】!
	  原因: 战神V1 主机板上
	    PA2 = USART2_TX -> RS485 DI
	    PA3 = USART2_RX <- RS485 RO
	    PA4 = ESP8266 CH_PD (模块使能, 拉低就关断)
	  照抄会把 485 和 WiFi 一起废掉。

	  所以数据线保留 SPI1 的默认脚(不重映射时唯一选择),
	  控制线全部挪到手册标注"完全独立"的空闲脚:

	    SCL  -> PA5   (SPI1_SCK,  唯一选择)
	    SDA  -> PA7   (SPI1_MOSI, 唯一选择)
	    RES  -> PA1   (空闲, 拔 P10 跳线帽后完全独立)
	    DC   -> PE0   (手册: 未接任何外设, 完全独立)
	    CS   -> PE1   (同上)
	    BLK  -> PE6   (同上)

	  VDD -> 3.3V, GND -> GND

	★ 接线注意(8 针模块, 背面丝印从左到右):
	    BLK  CS  DC  RST  SDA  SCL  VDD  GND
	  按上面这张表对应接, 不要按丝印顺序想当然。
*/

/* ---- 屏幕尺寸 (竖屏 128x160) ---- */
#define USE_HORIZONTAL  1       /* 0/1 = 竖屏 128x160, 2/3 = 横屏 160x128 */

#if (USE_HORIZONTAL == 0) || (USE_HORIZONTAL == 1)
#define LCD_W 128
#define LCD_H 160
#else
#define LCD_W 160
#define LCD_H 128
#endif

/* ---- 控制引脚 (★ 已避开 PA2/PA3/PA4) ---- */

/* RES 复位: PA1 */
#define LCD_RES_PORT    GPIOA
#define LCD_RES_PIN     GPIO_Pin_1
#define LCD_RES_Clr()   GPIO_ResetBits(LCD_RES_PORT, LCD_RES_PIN)
#define LCD_RES_Set()   GPIO_SetBits(LCD_RES_PORT, LCD_RES_PIN)

/* DC 数据/命令选择: PE0 */
#define LCD_DC_PORT     GPIOE
#define LCD_DC_PIN      GPIO_Pin_0
#define LCD_DC_Clr()    GPIO_ResetBits(LCD_DC_PORT, LCD_DC_PIN)
#define LCD_DC_Set()    GPIO_SetBits(LCD_DC_PORT, LCD_DC_PIN)

/* CS 片选: PE1 */
#define LCD_CS_PORT     GPIOE
#define LCD_CS_PIN      GPIO_Pin_1
#define LCD_CS_Clr()    GPIO_ResetBits(LCD_CS_PORT, LCD_CS_PIN)
#define LCD_CS_Set()    GPIO_SetBits(LCD_CS_PORT, LCD_CS_PIN)

/* BLK 背光: PE6 (高 = 亮) */
#define LCD_BLK_PORT    GPIOE
#define LCD_BLK_PIN     GPIO_Pin_6
#define LCD_BLK_Clr()   GPIO_ResetBits(LCD_BLK_PORT, LCD_BLK_PIN)
#define LCD_BLK_Set()   GPIO_SetBits(LCD_BLK_PORT, LCD_BLK_PIN)

/* ---- 底层接口 ---- */
void LCD_GPIO_Init(void);
void LCD_Init(void);
void LCD_Writ_Bus(u8 dat);
void LCD_WR_DATA8(u8 dat);
void LCD_WR_DATA(u16 dat);
void LCD_WR_REG(u8 dat);
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2);

#endif
