#include "lcd_init.h"
#include "delay.h"
#include "log.h"

/*
	中景园 1.8 寸 ST7735S 驱动 —— 底层实现
	==================================================================
	移植自官方例程, 改动:
	  1. 控制引脚从 PA2/PA3/PA4 改到 PE0/PE1/PE6 (避开 485 和 WiFi)
	  2. SPI 初始化从独立的 spi.c 内联进来 (少一个文件)
	  3. 用工程统一的 delay_ms / delay_us 替代例程的 delay_init

	★ 为什么用【软件 CS 控制 + 硬件 SPI 数据】的混合方式:
	  例程里 LCD_Writ_Bus 每次都拉低 CS 再拉高, 这是"每字节一个片选周期",
	  对 ST7735 完全没问题(它允许这样)。好处是不用关心连续传输的时序,
	  代价是慢一点 —— 但我们的刷新率只要 1 秒一次, 完全够。
*/

/**
 *  @brief 初始化 LCD 的 4 个控制引脚 + SPI1
 */
void LCD_GPIO_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure = {0};
	SPI_InitTypeDef   SPI_InitStructure  = {0};

	/* 1. 开时钟: GPIOA(RES) + GPIOE(DC/CS/BLK) + SPI1 + AFIO */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA |
	                       RCC_APB2Periph_GPIOE |
	                       RCC_APB2Periph_SPI1  |
	                       RCC_APB2Periph_AFIO, ENABLE);

	/* 2. 控制引脚: 推挽输出, 初始都为高
	      RES 高 = 不复位
	      DC  高 = 数据态
	      CS  高 = 未选中
	      BLK 高 = 背光关(初始化里再打开) */
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_InitStructure.GPIO_Pin = LCD_RES_PIN;
	GPIO_Init(LCD_RES_PORT, &GPIO_InitStructure);
	LCD_RES_Set();

	GPIO_InitStructure.GPIO_Pin = LCD_DC_PIN | LCD_CS_PIN | LCD_BLK_PIN;
	GPIO_Init(LCD_DC_PORT, &GPIO_InitStructure);
	LCD_DC_Set();
	LCD_CS_Set();
	LCD_BLK_Clr();      /* 背光先关, 初始化完再开, 避免上电花屏 */

	/*
		3. SPI1: PA5 = SCK, PA7 = MOSI, 复用推挽
		   ★ PA6(MISO) 不用: 屏幕只收不发, 用 SPI_Direction_1Line_Tx
		     这样还能省一个引脚, 也避免 MISO 悬空引入干扰
	*/
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/*
		4. SPI1 参数
		   CPOL=High + CPHA=2Edge 是 ST7735 要求的方式(模式3)
		   预分频 4 -> 72MHz/4 = 18MHz, ST7735S 支持到 15~30MHz, 安全
	*/
	SPI_InitStructure.SPI_Direction         = SPI_Direction_1Line_Tx;
	SPI_InitStructure.SPI_Mode              = SPI_Mode_Master;
	SPI_InitStructure.SPI_DataSize          = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL              = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA              = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS               = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4;
	SPI_InitStructure.SPI_FirstBit          = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial     = 7;
	SPI_Init(SPI1, &SPI_InitStructure);
	SPI_Cmd(SPI1, ENABLE);
}

/**
 *  @brief 往 SPI 总线上写一个字节 (带片选翻转)
 *
 *  ★★ 这里必须等 BSY(总线忙) 清零, 不能只等 TXE —— 踩过的坑 ★★
 *
 *  错误写法(我第一次就是这样, 结果屏幕全白):
 *      LCD_CS_Clr();
 *      while(TXE == RESET);          // 只等发送缓冲空
 *      SPI_I2S_SendData(SPI1, dat);
 *      LCD_CS_Set();                 // 立刻拉高片选
 *
 *  为什么错: TXE=1 只表示"发送缓冲空了, 数据已经移进移位寄存器",
 *           但那个字节【还在移位寄存器里一位一位往外发】。
 *           此时立刻拉高 CS, 屏幕认为这一字节的传输被中止 -> 丢弃。
 *           结果就是初始化序列全部无效, 屏幕保持白屏。
 *
 *  正确做法: 等 BSY 清零 —— 它表示整个字节(含移位寄存器)真的发完了。
 *           这比例程里的 delay(1) 更精确, 也不会引入固定延时。
 */
void LCD_Writ_Bus(u8 dat)
{
	LCD_CS_Clr();

	/* 1. 等发送缓冲空, 才能写新数据 */
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{
		/* 等 */
	}
	SPI_I2S_SendData(SPI1, dat);

	/*
		2. 等总线真正空闲 —— 这一步不能省!
		   BSY=1 表示移位寄存器里还有数据在发。
		   72MHz / 4 = 18MHz, 一个字节 8 bit 约 0.44us,
		   忙等几十个周期就过去了, 开销可以忽略。
	*/
	while(SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET)
	{
		/* 等移位寄存器发完 */
	}

	LCD_CS_Set();
}

/**
 *  @brief 写一个字节数据
 */
void LCD_WR_DATA8(u8 dat)
{
	LCD_Writ_Bus(dat);
}

/**
 *  @brief 写 16 位数据 (高字节在前)
 */
void LCD_WR_DATA(u16 dat)
{
	LCD_Writ_Bus((u8)(dat >> 8));
	LCD_Writ_Bus((u8)(dat & 0x00FF));
}

/**
 *  @brief 写命令 (DC 拉低表示命令, 写完恢复数据态)
 */
void LCD_WR_REG(u8 dat)
{
	LCD_DC_Clr();
	LCD_Writ_Bus(dat);
	LCD_DC_Set();
}

/**
 *  @brief 设置显示窗口 (列/行地址范围)
 *
 *  偏移量说明: ST7735S 的显存是 132x162, 而屏幕可视区是 128x160,
 *  所以坐标要加偏移才能对到可视区。竖屏(0/1) 是 +2/+1,
 *  横屏(2/3) 是 +1/+2 —— 这是官方例程实测出来的值, 不要改。
 */
void LCD_Address_Set(u16 x1, u16 y1, u16 x2, u16 y2)
{
#if (USE_HORIZONTAL == 0) || (USE_HORIZONTAL == 1)
	LCD_WR_REG(0x2A);           /* 列地址 */
	LCD_WR_DATA(x1 + 2);
	LCD_WR_DATA(x2 + 2);
	LCD_WR_REG(0x2B);           /* 行地址 */
	LCD_WR_DATA(y1 + 1);
	LCD_WR_DATA(y2 + 1);
	LCD_WR_REG(0x2C);           /* 开始写显存 */
#else
	LCD_WR_REG(0x2A);
	LCD_WR_DATA(x1 + 1);
	LCD_WR_DATA(x2 + 1);
	LCD_WR_REG(0x2B);
	LCD_WR_DATA(y1 + 2);
	LCD_WR_DATA(y2 + 2);
	LCD_WR_REG(0x2C);
#endif
}

/**
 *  @brief 初始化 ST7735S
 *
 *  ★ 整个初始化要 ~350ms (复位 200ms + SleepOut 120ms + 其他),
 *    只在开机调用一次, 不影响主循环。
 */
void LCD_Init(void)
{
	LCD_GPIO_Init();

	/* 1. 硬复位: RES 低 100ms -> 高 100ms */
	LCD_RES_Clr();
	delay_ms(100);
	LCD_RES_Set();
	delay_ms(100);

	/* 2. 开背光 (先开再初始化, 有些屏不开背光看不到初始化过程) */
	LCD_BLK_Set();
	delay_ms(100);

	/*
		3. 软件复位
		   ★ 有些 ST7735S 模块在硬复位后仍需要一次软复位,
		     否则内部寄存器可能停在不确定状态(表现就是白屏)。
		     官方例程没有这一步, 但加上无害 —— 0x01 是标准 SWRESET。
	*/
	LCD_WR_REG(0x01);           /* SWRESET */
	delay_ms(120);

	/* 4. Sleep out */
	LCD_WR_REG(0x11);
	delay_ms(120);

	/* 帧率 */
	LCD_WR_REG(0xB1);
	LCD_WR_DATA8(0x05); LCD_WR_DATA8(0x3C); LCD_WR_DATA8(0x3C);
	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x05); LCD_WR_DATA8(0x3C); LCD_WR_DATA8(0x3C);
	LCD_WR_REG(0xB3);
	LCD_WR_DATA8(0x05); LCD_WR_DATA8(0x3C); LCD_WR_DATA8(0x3C);
	LCD_WR_DATA8(0x05); LCD_WR_DATA8(0x3C); LCD_WR_DATA8(0x3C);

	LCD_WR_REG(0xB4);           /* Dot inversion */
	LCD_WR_DATA8(0x03);

	/* 电源序列 */
	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x28); LCD_WR_DATA8(0x08); LCD_WR_DATA8(0x04);
	LCD_WR_REG(0xC1);
	LCD_WR_DATA8(0xC0);
	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x0D); LCD_WR_DATA8(0x00);
	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x8D); LCD_WR_DATA8(0x2A);
	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x8D); LCD_WR_DATA8(0xEE);

	/* VCOM */
	LCD_WR_REG(0xC5);
	LCD_WR_DATA8(0x1A);

	/* 扫描方向 */
	LCD_WR_REG(0x36);
#if (USE_HORIZONTAL == 0)
	LCD_WR_DATA8(0x00);
#elif (USE_HORIZONTAL == 1)
	LCD_WR_DATA8(0xC0);
#elif (USE_HORIZONTAL == 2)
	LCD_WR_DATA8(0x70);
#else
	LCD_WR_DATA8(0xA0);
#endif

	/* 伽马校正 */
	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0x04); LCD_WR_DATA8(0x22); LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x0A); LCD_WR_DATA8(0x2E); LCD_WR_DATA8(0x30);
	LCD_WR_DATA8(0x25); LCD_WR_DATA8(0x2A); LCD_WR_DATA8(0x28);
	LCD_WR_DATA8(0x26); LCD_WR_DATA8(0x2E); LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x00); LCD_WR_DATA8(0x01); LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x13);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0x04); LCD_WR_DATA8(0x16); LCD_WR_DATA8(0x06);
	LCD_WR_DATA8(0x0D); LCD_WR_DATA8(0x2D); LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x23); LCD_WR_DATA8(0x27); LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x25); LCD_WR_DATA8(0x2D); LCD_WR_DATA8(0x3B);
	LCD_WR_DATA8(0x00); LCD_WR_DATA8(0x01); LCD_WR_DATA8(0x04);
	LCD_WR_DATA8(0x13);

	/* 65K 色 + 开显示 */
	LCD_WR_REG(0x3A);
	LCD_WR_DATA8(0x05);
	LCD_WR_REG(0x29);

	LOG_Str("[LCD] ST7735S init done, 128x160\r\n");
}
