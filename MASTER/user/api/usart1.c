#include "usart1.h"

USART1_Value usart1_value = {0};

/**
 *  @brief 初始化 USART1 模块
 *
 *  硬件连接：Tx ==> PA9【复用推挽输出】 Rx ==> PA10【浮空输入】
 *  这个串口只做调试日志输出与回显，不接 485 总线。
 *
 *  ★ 本工程的硬要求: "串口1 作为调试回显, 必须在所有需要完成的操作
 *    结束后再进行发送"。
 *    落实分两层:
 *      1) 日志内容先进 RAM 缓冲 (见 log.c), 不在业务代码里直接发;
 *      2) main 循环把轮询/上报/下发/心跳全部做完, 最后一步才 LOG_Flush()。
 *    所以本文件的发送函数只被 log.c 的 LOG_Flush 调用一次。
 */
void USART1_Init(u32 brr)
{
    // 1. RCC 时钟使能 GPIOA 和 USART1 模块
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA
                         | RCC_APB2Periph_USART1, ENABLE);

    // 2. 对 GPIO 模块进行配置
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 3. 对 USART 模块进行配置
    // 明确关闭 USART1 模块
    USART_Cmd(USART1, DISABLE);
    USART_InitTypeDef USART_InitStructure = {0};
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_BaudRate = brr;
    USART_Init(USART1, &USART_InitStructure);

    // 4. NVIC 中断配置，需要注册中断使能有 RXNE，IDLE
    //    两个中断必须分两次调用USART_ITConfig，写在一起只会配上一个（RXNE）
    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    NVIC_InitTypeDef NVIC_InitStructure = {0};
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    /*
        ★ 优先级: 抢占 3 / 子 1 (最低)

        调试口晚几毫秒没有任何影响, 但它是【发送口】——
        发送是阻塞轮询 TC 的, 一旦开始发, 只有更高优先级的中断能打断它。
        放到最低, 意味着 485 接收(抢占 1) 和 MQTT 接收(抢占 0)
        都可以随时打断日志发送 —— 这正是"收 > 发"和
        "Modbus 指令发送 > 其他发送(日志)"两条要求的落实:
        日志可以等, 总线上的字节不能等。
    */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // 5. 开启 USART1 模块
    USART_Cmd(USART1, ENABLE);

    // 6. 清空接收缓冲区
    USART1_Recv_Clear();
}

/**
 *  @brief USART1 接收中断处理函数 —— 快进快出
 *
 *  ★ 中断里只做: 读 DR -> 存一格 -> 置标志。清缓冲交给主循环。
 *    改造前这里在开头 `if(flag) { count=0; flag=0; }` ——
 *    也就是"上一帧还没被主循环取走就在中断里把它扔掉"。
 *    虽然省了 memset, 但行为是错的: 主循环还在处理上一帧时,
 *    新来的第一个字节就把旧帧清了, 主循环拿到的是一帧被腰斩的数据。
 *    现在改成"flag 未取走时丢弃新字节", 保住已经完整的那一帧。
 */
void USART1_IRQHandler(void)
{
    // 1. RXNE 中断处理，每来一个字节就写入缓冲区
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        u8 byte = (u8)USART_ReceiveData(USART1);

        /* 上一帧还没被主循环取走 -> 丢弃这个新字节(保住完整的旧帧) */
        if (usart1_value.flag == 0 && usart1_value.count < USART1_BUFFER_SIZE)
        {
            usart1_value.data[usart1_value.count++] = byte;
        }
    }

    // 2. IDLE 中断处理，一帧收完
    if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        // 先读 SR 再读 DR 清除 IDLE 标志
        USART1->SR;
        USART1->DR;

        if (usart1_value.count > 0)
        {
            usart1_value.flag = 1;
        }
    }
}

/**
 *  @brief 清空 USART1 接收缓冲区
 */
void USART1_Recv_Clear(void)
{
    usart1_value.count = 0;
    usart1_value.flag = 0;
    memset(usart1_value.data, 0, USART1_BUFFER_SIZE);
}

/**
 *  @brief USART1 发送字符串
 */
void USART1_SendString(const char *str)
{
    while (*str)
    {
        USART1_SendByte(*str);
        str += 1;
    }
}

/**
 *  @brief USART1 发送单个字节
 *
 *  先写数据寄存器，再等 TC 置位。之前的写法在 while(!TC) 里反复写 DR，
 *  会把同一个字节重发几十遍。
 *
 *  另外 USART_SR 复位值是 0x00C0，TC 位上电就是 1，不先清掉的话第一个
 *  字节发送时会直接跳过等待，导致首字节丢失。
 */
void USART1_SendByte(u8 byte)
{
    USART_ClearFlag(USART1, USART_FLAG_TC);
    USART_SendData(USART1, byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {
        // 等待当前字节发送完成
    }
}

/**
 *  @brief USART1 发送指定个数的字节
 */
void USART1_SendBuffer(u8 *buffer, u16 len)
{
    while (len--)
    {
        USART1_SendByte(*buffer);
        buffer += 1;
    }
}

/**
 *  @brief USART1 发送一个十进制整数
 */
void USART1_SendNumber(u32 num)
{
    char buf[11] = {0};
    u8 pos = 0;

    if (num == 0)
    {
        USART1_SendByte('0');
        return;
    }

    while (num > 0)
    {
        buf[pos++] = (char)('0' + (num % 10));
        num /= 10;
    }

    while (pos > 0)
    {
        USART1_SendByte(buf[--pos]);
    }
}
