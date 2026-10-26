#include "usart3.h"

USART3_Value usart3_value = {0};

/**
 *  @brief 初始化 USART3 模块
 *
 *  硬件连接：Tx ==> PB10【复用推挽输出】 Rx ==> PB11【浮空输入】
 *  这个串口接的是 ESP8266，波特率按模块 AT 固件默认 115200。
 */
void USART3_Init(u32 brr)
{
    // 1. RCC 时钟使能 GPIOB 和 USART3 模块
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // 2. 对 GPIO 模块进行配置
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 3. 对 USART 模块进行配置
    USART_Cmd(USART3, DISABLE);
    USART_InitTypeDef USART_InitStructure = {0};
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_BaudRate = brr;
    USART_Init(USART3, &USART_InitStructure);

    // 4. NVIC 中断配置，需要注册中断使能有 RXNE，IDLE
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    /*
        ★ 优先级 1: 低于 USART2(485 = 0)。
          ESP8266 收到的数据晚几百微秒毫无影响(它是 TCP 流, 不会因为
          我们晚读就丢), 而 485 上的字节晚一步就没了。
    */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 5. 开启 USART3 模块
    USART_Cmd(USART3, ENABLE);

    // 6. 清空接收缓冲区
    USART3_Recv_Clear();
}

/**
 *  @brief USART3 接收中断处理函数 —— 快进快出
 *
 *  之前的实现里 RXNE 分支只做了一次 USART3->DR 空读，收到的字节全被丢掉，
 *  MQTT 下行数据永远读不到。这里改成真正的收字节逻辑。
 *
 *  ★ 中断里【绝对不许】打印:
 *    USART1@9600 打 1 个字符 1.04ms, 打一行 "USART3 RX: ..." 就是几十毫秒,
 *    这期间 ESP8266 继续往 USART3 灌数据(115200 下 86us 一个字节),
 *    硬件只有 1 字节的接收寄存器, 立刻溢出 -> MQTT 报文残缺 -> 下发命令丢失。
 *    改造前这里正是这么写的, 那是"云端下发偶尔不生效"的直接原因。
 *
 *  ★ 中断里也【不许】做 memmove:
 *    缓冲 512 字节, 搬一半就是 256 字节, 72MHz 上约 3~5us ——
 *    看着不多, 但它发生在 115200 的接收中断里(字节间隔 86us),
 *    而且每次溢出都会触发, 累积起来一样会丢字节。
 *    溢出时只做两件常数级的事: 置 flag 通知主循环 + 丢弃当前字节,
 *    搬运/丢弃的决策交给主循环(那里没有时序压力)。
 */
void USART3_IRQHandler(void)
{
    // 1. RXNE 中断处理，每来一个字节就写入缓冲区
    if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
    {
        if (usart3_value.count < USART3_BUFFER_SIZE)
        {
            usart3_value.data[usart3_value.count++] = USART_ReceiveData(USART3);
        }
        else
        {
            /*
                缓冲区满了: 丢掉这个字节, 并置 flag 让主循环立刻来处理。
                为什么不是"清空重来": 清空等于把一帧的中间挖掉,
                后面的字节会拼成半截帧; 而置 flag 能让主循环马上把
                已有的 512 字节拿去解析, 解析完自然就腾出空间了。
                实际使用中 512 字节远大于任何一帧下行, 这个分支只是兜底。
            */
            (void)USART_ReceiveData(USART3);
            usart3_value.flag = 1;
        }
    }

    // 2. IDLE 中断处理，总线空闲表示一帧收完
    if (USART_GetITStatus(USART3, USART_IT_IDLE) != RESET)
    {
        // 先读 SR 再读 DR 是 IDLE 标志的固定清除序列
        USART3->SR;
        USART3->DR;

        if (usart3_value.count > 0)
        {
            usart3_value.flag = 1;
        }
    }
}

/**
 *  @brief 清空 USART3 接收缓冲区
 */
void USART3_Recv_Clear(void)
{
    __disable_irq();
    usart3_value.count = 0;
    usart3_value.flag = 0;
    memset(usart3_value.data, 0, USART3_BUFFER_SIZE);
    __enable_irq();
}

/**
 *  @brief USART3 发送字符串
 */
void USART3_SendString(const char *str)
{
    while (*str)
    {
        USART3_SendByte(*str);
        str += 1;
    }
}

/**
 *  @brief USART3 发送单个字节
 *
 *  之前的写法是在 while(!TC) 里反复写 DR，等于把同一个字节重发很多次，
 *  直到 TC 偶然置位，数据全是错的。正确顺序是先写 DR 再等 TC。
 *
 *  另外 USART_SR 复位值是 0x00C0，TC 位上电就是 1，不先清掉的话第一个
 *  字节发送时会直接跳过等待，导致首字节丢失。
 */
void USART3_SendByte(u8 byte)
{
    USART_ClearFlag(USART3, USART_FLAG_TC);
    USART_SendData(USART3, byte);
    while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
    {
        // 等待当前字节发送完成
    }
}

/**
 *  @brief 取 USART3 当前收到的原始字节(供 @esp 诊断)
 *
 *  ★ 为什么需要: ESP8266 出问题时, 最想知道的是"模块到底回了什么"。
 *    没有这个接口, 只能看到上层打的 "module no response", 分不清是
 *    没收到(接线/供电) 还是 收到乱码(波特率错)。
 */
u16 USART3_Get_Raw(u8 *buf, u16 max)
{
    u16 n = 0;
    u16 i = 0;

    if (buf == NULL || max == 0) return 0;

    n = usart3_value.count;
    if (n > max) n = max;

    for (i = 0; i < n; i++) buf[i] = usart3_value.data[i];
    return n;
}

/**
 *  @brief USART3 发送指定个数的字节
 */
void USART3_SendBuffer(u8 *buffer, u16 len)
{
    while (len--)
    {
        USART3_SendByte(*buffer);
        buffer += 1;
    }
}
