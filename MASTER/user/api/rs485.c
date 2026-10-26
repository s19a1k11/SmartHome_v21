#include "rs485.h"

/*
	RS485 方向脚 PD7 —— 主机侧全工程唯一操作它的地方
	==================================================================
	PD7 输出 1 = 发送使能(占用总线)
	PD7 输出 0 = 接收使能(释放总线)

	为什么方向脚只能在这里动:
	  RS485 是半双工, 同一时刻总线上只能有一个发送者。
	  如果多处代码都能改 PD7, 就会出现"A 切了发送、B 又切了一次、
	  谁负责切回来"的责任真空 —— 一旦漏切, 整条总线被本机拉死,
	  所有从机都不应答, 复位才好, 而且只在特定时序下复现。

	  ★ 这两个函数是 static: 编译器保证 rs485.c 以外的文件调不到,
	    这是把"约束"变成"编译期强制", 比写注释可靠。
*/
static void RS485_Dir_Tx(void)
{
    GPIO_SetBits(GPIOD, GPIO_Pin_7);
}

static void RS485_Dir_Rx(void)
{
    GPIO_ResetBits(GPIOD, GPIO_Pin_7);
}

/**
 *  @brief 初始化 RS485 模块
 *
 *  硬件连接：
 *      USART2 Tx ==> PA2 【复用推挽输出】
 *      USART2 Rx ==> PA3 【浮空输入】
 *      方向控制  ==> PD7 【推挽输出，1 发送 / 0 接收】
 *
 *  上电顺序: 必须先把 PD7 配成输出并拉低(接收态), 再使能 USART2。
 *  否则 USART2 使能时 PD7 还是复位默认的浮空输入, 若总线上此刻有数据,
 *  主机会误收几个字节, 表现为"上电第一帧 CRC 失败"。
 */
void RS485_Init(u32 brr)
{
    // 1. RCC 时钟使能 GPIOD，方向控制脚在 PD7
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);

    // 2. 方向控制脚配置为推挽输出，默认接收态
    //    上电绝不能占住总线，否则三台从机谁都发不出数据
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    RS485_Dir_Rx();

    // 3. 初始化 USART2 物理层，PA2/PA3 与中断都在里面配置
    USART2_Init(brr);
}

/**
 *  @brief 清空 RS485 接收缓冲区
 */
void RS485_Recv_Clear(void)
{
    USART2_Recv_Clear();
}

/**
 *  @brief 发送指定个数的字节到 RS485 总线
 *
 *  关键点：方向脚必须等最后一个字节真正从移位寄存器里移出去之后才能切回
 *          接收，否则应答帧的最后一个字节会被切掉半个，从机侧就 CRC 报错。
 *          所以这里等的是 TC（发送完成），而不是 TXE（发送数据寄存器空）。
 *
 *          另外 RS485 收发器在发送态时，自己发出去的字节也会回到自己的
 *          接收脚上，所以发送全程必须关掉接收中断，否则会收到自己的回声。
 */
void RS485_SendBuffer(u8 *buffer, u16 len)
{
    u16 i = 0;

    if (buffer == NULL || len == 0)
    {
        return;
    }

    // 1. 关闭接收中断，避免自己发出去的字节被自己收进来
    USART2_Recv_IT_Cmd(DISABLE);

    // 2. 切换为发送态
    RS485_Dir_Tx();

    // 3. 逐字节发送，每个字节都等到 TC 置位
    //    USART_SR 复位值是 0x00C0，TC 位上电默认为 1，必须先清掉再发第一个
    //    字节，否则第一字节会直接跳过等待，把还没移出的数据覆盖掉
    USART_ClearFlag(USART2, USART_FLAG_TC);
    for (i = 0; i < len; i++)
    {
        USART_SendData(USART2, buffer[i]);
        while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
        {
            // 等待当前字节发送完成
        }
    }

    // 4. TC 置位说明最后一字节已经移出，可以切回接收态了
    RS485_Dir_Rx();

    // 5. 丢掉总线回声和残留标志，重新打开接收中断
    RS485_Recv_Clear();
    USART2_Recv_IT_Cmd(ENABLE);
}

/**
 *  @brief 发送单个字节到 RS485 总线
 */
void RS485_SendByte(u8 byte)
{
    RS485_SendBuffer(&byte, 1);
}

/**
 *  @brief 发送字符串到 RS485 总线
 */
void RS485_SendString(const char *str)
{
    // strlen 返回值转成 u16 传入，ModBus 帧不会超过 255 字节
    RS485_SendBuffer((u8 *)str, (u16)strlen(str));
}
