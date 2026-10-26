#include "usart2.h"

USART2_Value usart2_value = {0};

/**
 *  @brief 初始化 USART2 模块
 *
 *  硬件连接：Tx ==> PA2【复用推挽输出】 Rx ==> PA3【浮空输入】
 *  USART2 是 RS485 总线的物理层，只负责收发字节，
 *  方向切换和 ModBus 帧的处理都在 rs485.c / modbus.c 里。
 */
void USART2_Init(u32 brr)
{
    // 1. RCC 时钟使能 GPIOA 和 USART2 模块
    //    USART2 挂在 APB1 总线上，用的是 APB1 的时钟使能函数
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA
                         | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

    // 2. 对 GPIO 模块进行配置
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 对 USART 模块进行配置
    // 明确关闭 USART2 模块
    USART_Cmd(USART2, DISABLE);
    USART_InitTypeDef USART_InitStructure = {0};
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_BaudRate = brr;
    USART_Init(USART2, &USART_InitStructure);

    // 4. NVIC 中断配置，需要注册中断使能有 RXNE，IDLE
    //    两个中断必须分两次调用USART_ITConfig，写在一起只会配上一个（RXNE）
    USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    /*
        ★ USART2 必须是最高抢占优先级(0):
          它是唯一"晚一步就丢字节"的中断。
          USART3(ESP8266) 和 USART1(调试) 都可以被它打断。
    */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 5. 开启 USART2 模块
    USART_Cmd(USART2, ENABLE);

    // 6. 清空接收缓冲区
    USART2_Recv_Clear();
}

/**
 *  @brief USART2 接收中断处理函数 —— 快进快出
 *
 *  ModBus RTU 没有起始符也没有结束符，靠总线静默时间断帧。
 *  这里 RXNE 只做"搬一个字节", IDLE 只做"置一个标志",
 *  校验 / 组帧 / 解析全部在主循环里做。
 *
 *  ★ 为什么不能在这里做更多:
 *    9600 下一个字节只占 1.04ms。中断里任何一次 sprintf / memset /
 *    打印都可能超过这个时间, 后面紧跟的字节直接丢 —— 表现就是
 *    "偶发 CRC 错", 而且越忙越容易错, 极难复现定位。
 */
void USART2_IRQHandler(void)
{
    // 1. RXNE 中断处理，每来一个字节就写入缓冲区
    if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)
    {
        /*
            上一帧还没有被主循环取走, 说明主循环慢了(比如正在建 MQTT 链)。
            这时丢掉新字节、保留旧帧 —— 旧帧已经完整, 新帧刚开始,
            保完整的那个更有价值。
        */
        if (usart2_value.flag)
        {
            (void)USART_ReceiveData(USART2);
        }
        else if (usart2_value.count < USART2_BUFFER_SIZE)
        {
            usart2_value.data[usart2_value.count++] = USART_ReceiveData(USART2);
        }
        else
        {
            // 缓冲区已满，溢出字节直接丢弃，避免数组越界
            (void)USART_ReceiveData(USART2);
        }
    }

    // 2. IDLE 中断处理，总线静默满一个字符时间，认为一帧结束
    if (USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
    {
        // IDLE 标志的清除序列是【先读 SR，再读 DR】
        USART2->SR;
        USART2->DR;

        if (usart2_value.count > 0)
        {
            usart2_value.flag = 1;
        }
    }
}

/**
 *  @brief 清空 USART2 接收缓冲区
 *
 *  ★ 关中断再清: 清到一半被中断写进一个字节的话, 下一帧会变成
 *    "残帧 + 新帧"粘在一起, CRC 必败。
 */
void USART2_Recv_Clear(void)
{
    __disable_irq();
    usart2_value.count = 0;
    usart2_value.flag = 0;
    memset(usart2_value.data, 0, USART2_BUFFER_SIZE);
    __enable_irq();

    // 读一次 SR / DR，清掉方向切换瞬间可能残留的标志位
    USART2->SR;
    USART2->DR;
}

/**
 *  @brief 开关 USART2 的接收中断
 */
void USART2_Recv_IT_Cmd(FunctionalState state)
{
    USART_ITConfig(USART2, USART_IT_RXNE, state);
    USART_ITConfig(USART2, USART_IT_IDLE, state);
}

/**
 *  @brief USART2 发送字符串
 */
void USART2_SendString(const char *str)
{
    while (*str)
    {
        USART2_SendByte(*str);
        str += 1;
    }
}

/**
 *  @brief USART2 发送单个字节
 *
 *  先写数据寄存器，再等 TC 置位。不能写成在 while(!TC) 里反复写 DR，
 *  那等于把同一个字节重发很多遍。
 *
 *  另外 USART_SR 复位值是 0x00C0，TC 位上电就是 1，不先清掉的话第一个
 *  字节发送时会直接跳过等待，导致首字节丢失。
 */
void USART2_SendByte(u8 byte)
{
    USART_ClearFlag(USART2, USART_FLAG_TC);
    USART_SendData(USART2, byte);
    while (USART_GetFlagStatus(USART2, USART_FLAG_TC) == RESET)
    {
        // 等待当前字节发送完成
    }
}

/**
 *  @brief USART2 发送指定个数的字节
 */
void USART2_SendBuffer(u8 *buffer, u16 len)
{
    while (len--)
    {
        USART2_SendByte(*buffer);
        buffer += 1;
    }
}

/**
 *  @brief USART2 发送一个十进制整数
 */
void USART2_SendNumber(u32 num)
{
    char buf[11] = {0};
    u8 pos = 0;

    if (num == 0)
    {
        USART2_SendByte('0');
        return;
    }

    while (num > 0)
    {
        buf[pos++] = (char)('0' + (num % 10));
        num /= 10;
    }

    while (pos > 0)
    {
        USART2_SendByte(buf[--pos]);
    }
}
