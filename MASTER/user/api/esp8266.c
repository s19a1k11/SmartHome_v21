#include "esp8266.h"
#include "usart1.h"
#include "log.h"
#include "delay.h"

/**
 *  @brief 初始化 ESP8266 控制引脚 + 复位模块
 *
 * ★ 这是 v21 补上的关键一步。
 *   改造前代码完全没碰 PA4/PA15, 它们是复位后的浮空输入状态,
 *   而 ESP-01 要求 CH_PD 必须为高、GPIO0 必须为高才能跑 AT 固件。
 *   悬空 = 电平不确定 = 模块可能一直关断或停在下载模式,
 *   表现就是发 AT 永远没回应。
 */
u8 ESP8266_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    /* 1. 开 GPIOA 时钟(PA4 和 PA15 都在 GPIOA) */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

    /*
        2. 禁用 JTAG, 保留 SWD
           ★ PA15 是 JTAG 的 JTDI, 不禁用 JTAG 的话它被 JTAG 占用,
             配置成 GPIO 也不生效。
           ★ 用 JTAGDisable 而不是 SWJ_Disable: 后者会连 SWD 一起关掉,
             那样 ST-Link 就没法下载调试了。
    */
    GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);

    /* 3. PA15 (GPIO0) 推挽输出, 先拉高 = 正常运行模式
          低电平会让模块进下载(烧录)模式, 不跑 AT 固件 */
    GPIO_InitStructure.GPIO_Pin   = ESP_GPIO0_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(ESP_GPIO0_PORT, &GPIO_InitStructure);
    GPIO_SetBits(ESP_GPIO0_PORT, ESP_GPIO0_PIN);

    /* 4. PA4 (CH_PD) 推挽输出
          CH_PD 是模块的使能脚: 高 = 工作, 低 = 关断 */
    GPIO_InitStructure.GPIO_Pin   = ESP_CHPD_PIN;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(ESP_CHPD_PORT, &GPIO_InitStructure);

    /*
        5. 复位时序:
             拉低 CH_PD -> 保持 100ms(确保模块完全断电复位)
             拉高 CH_PD -> 等 500ms(模块启动 + AT 固件就绪)
           顺序不能反: 先拉低再拉高才是"复位",
           一直保持高只是"上电", 如果模块之前处于异常状态(比如停在透传),
           只有真正断电重启才能恢复。
    */
    GPIO_ResetBits(ESP_CHPD_PORT, ESP_CHPD_PIN);
    delay_ms(ESP_RESET_LOW_MS);

    GPIO_SetBits(ESP_CHPD_PORT, ESP_CHPD_PIN);
    delay_ms(ESP_POWER_ON_DELAY_MS);

    /* 6. 丢掉复位期间模块吐出的启动乱码 */
    USART3_Recv_Clear();

    LOG_Str("[ESP] ctrl pins: CH_PD=1 GPIO0=1 (module enabled)\r\n");
    return 1;
}

/**
 *  @brief 读控制引脚当前电平(诊断用)
 *
 *  ★ 为什么用 GPIO_ReadOutputDataBit 而不是 InputDataBit:
 *    这两个脚是【我们推挽驱动】的, 读输入可能读到被外部拉偏的值,
 *    读输出寄存器才是"我们正在让它是什么电平"。
 */
void ESP8266_Get_Ctrl_Pins(u8 *chpd_out, u8 *gpio0_out)
{
    if(chpd_out)
    {
        *chpd_out = GPIO_ReadOutputDataBit(ESP_CHPD_PORT, ESP_CHPD_PIN) ? 1 : 0;
    }
    if(gpio0_out)
    {
        *gpio0_out = GPIO_ReadOutputDataBit(ESP_GPIO0_PORT, ESP_GPIO0_PIN) ? 1 : 0;
    }
}

/**
 *  @brief 清空接收缓冲区
 */
void ESP8266_Recv_Clear(void)
{
    USART3_Recv_Clear();
}

/**
 *  @brief 发送 AT 指令并等待指定关键字
 *
 *  判断关键字时要给缓冲区补一个 '\0'，否则 strstr 会越过有效数据往后找。
 */
/*
	ESP8266 诊断输出开关
	1 = 每次 AT 指令收发都打印(调试模块时用)
	0 = 只在失败时打印"收到了什么"(默认, 够定位问题)

	★ 为什么必须打印"收到了什么"(v21 补):
	  改造前失败时只 return 0, 上层打一句 "module no response" ——
	  这句话只能说明"没匹配到期望关键字", 无法区分三种完全不同的故障:
	    a) 一个字节都没收到   -> 接线 / 供电 / 模块波特率
	    b) 收到了乱码         -> 主机波特率与模块不一致(如模块是 74880)
	    c) 收到了别的内容     -> 模块停在透传态 / 上次命令残留
	  这三种的处理方式完全不同, 不打原始数据就只能瞎猜。
*/
#define ESP_DEBUG_LOG (0)

u8 ESP8266_SendCmd(const char *cmd, const char *ack, u16 timeout_ms)
{
    u16 wait = 0;
    u16 got = 0;            /* 本次实际收到的字节数 */

    if (cmd == NULL || ack == NULL)
    {
        return 0;
    }

    // 1. 发送前清空缓冲，保证只判断本次应答
    ESP8266_Recv_Clear();

    // 2. 发送指令
    USART3_SendString(cmd);

#if ESP_DEBUG_LOG
    LOG_Str("[ESP] TX: ");
    LOG_Str(cmd);
#endif

    /*
        3. 轮询等待模块应答, 中断负责填缓冲, 这里只做比对

        ★ v21 修: 缓冲写满时要【先比对再丢弃】, 不能干等。
          踩过的坑: ESP8266 复位(AT+RST)时会吐一大段启动日志
            "ets Jan  8 2013,rst cause:2, boot mode:(3,6) ... ready"
          整段超过 512 字节缓冲。缓冲满了之后中断里开始丢字节,
          而 "ready" 恰好排在启动日志【最后】-> 永远等不到 ->
          白白超时 2 秒, 而且下一句 AT 要重新等。

          改法: 每次进循环都:
            1) 补 '\0' 后比对当前缓冲
            2) 命中了就返回
            3) 缓冲满了但没命中 -> 丢掉前半, 保留后半继续等
               (报文尾部才是我们要的关键字)
    */
    while (wait < timeout_ms)
    {
        if (usart3_value.count > 0)
        {
            // 3.1 补字符串结束符，留一个字节余量防止越界
            if (usart3_value.count < USART3_BUFFER_SIZE)
            {
                usart3_value.data[usart3_value.count] = '\0';
            }
            else
            {
                usart3_value.data[USART3_BUFFER_SIZE - 1] = '\0';
            }

            // 3.2 命中期望关键字
            if (strstr((const char *)usart3_value.data, ack) != NULL)
            {
#if ESP_DEBUG_LOG
                LOG_Str("[ESP] RX: ");
                LOG_Str((const char *)usart3_value.data);
#endif
                return 1;
            }

            /*
                3.3 缓冲快满了但还没命中 -> 丢掉前半, 保留后半
                  为什么保留后半: 我们要找的关键字通常在报文尾部
                  (比如 ready 在启动日志最后)。
                  为什么不用 memmove 搬全量: 这里已经不在中断里了,
                  搬 256 字节无所谓, 但只搬后半就够了。
            */
            if (usart3_value.count >= (USART3_BUFFER_SIZE - 32))
            {
                u16 half = USART3_BUFFER_SIZE / 2;
                u16 i = 0;

                for (i = 0; i < half; i++)
                {
                    usart3_value.data[i] = usart3_value.data[half + i];
                }
                usart3_value.count = half;
            }
        }

        delay_ms(10);
        wait += 10;
    }

    /*
        ★ 失败时把"到底收到了什么"打出来 —— 这是定位 ESP8266 问题的关键。
          三种情况一眼可分:
            got=0     -> 一个字节都没收到, 查接线/供电/模块波特率
            got=N>0   -> 收到了但没匹配, 把原始内容打出来看是什么
    */
    got = usart3_value.count;
    LOG_Str("[ESP] no '");
    LOG_Str(ack);
    LOG_Str("' got=");
    LOG_Num(got);
    LOG_Str("B");

    if (got > 0)
    {
        /* 有内容就原样打出来(可能含不可打印字符, 但能看出是不是乱码) */
        LOG_Str(" raw=[");
        {
            u16 i = 0;
            for (i = 0; i < got && i < 40; i++)
            {
                u8 c = usart3_value.data[i];
                if (c >= 0x20 && c < 0x7F)
                {
                    LOG_Chr((char)c);
                }
                else if (c == '\r')
                {
                    LOG_Str("\\r");
                }
                else if (c == '\n')
                {
                    LOG_Str("\\n");
                }
                else
                {
                    LOG_Chr('.');
                }
            }
            LOG_Str("]");
        }
    }
    else
    {
        LOG_Str(" (nothing received: check wiring / power / module baud)");
    }
    LOG_Str("\r\n");

    return 0;
}

/**
 *  @brief 测试模块是否在线
 */
u8 ESP8266_Test_AT(void)
{
    return ESP8266_SendCmd("AT\r\n", "OK", ESP8266_SHORT_TIMEOUT);
}

/**
 *  @brief 连接目标热点
 */
u8 ESP8266_ConnectAP(const char *ssid, const char *psk)
{
    char cmd[ESP8266_CMD_BUFFER_SIZE] = {0};

    if (ssid == NULL || psk == NULL)
    {
        return 0;
    }

    // 1. 设置为 STA 模式
    //    注意是 AT+CWMODE=1，之前写成 AT+CWMODE_CUP=1 模块是认不出来的
    if (!ESP8266_SendCmd("AT+CWMODE=1\r\n", "OK", ESP8266_SHORT_TIMEOUT))
    {
        USART1_SendString("[ESP] set STA mode fail\r\n");
        return 0;
    }

    // 2. 关闭自动连接，避免模块记住上一次的热点导致连不上新的
    ESP8266_SendCmd("AT+CWAUTOCONN=0\r\n", "OK", ESP8266_SHORT_TIMEOUT);

    // 3. 连接热点，这一步最慢，入网 + DHCP 通常需要好几秒
    sprintf(cmd, "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, psk);
    if (!ESP8266_SendCmd(cmd, "WIFI GOT IP", ESP8266_LONG_TIMEOUT))
    {
        USART1_SendString("[ESP] join AP fail\r\n");
        return 0;
    }

    return 1;
}

/**
 *  @brief TCP 连接到 MQTT 服务器
 */
u8 ESP8266_ConnectTCP(const char *ip, u16 port)
{
    char cmd[ESP8266_CMD_BUFFER_SIZE] = {0};

    if (ip == NULL)
    {
        return 0;
    }

    // 1. 单连接模式，一次只允许一条 TCP
    if (!ESP8266_SendCmd("AT+CIPMUX=0\r\n", "OK", ESP8266_SHORT_TIMEOUT))
    {
        USART1_SendString("[ESP] set single link fail\r\n");
        return 0;
    }

    // 2. 建立 TCP 连接
    sprintf(cmd, "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", ip, port);
    if (!ESP8266_SendCmd(cmd, "OK", ESP8266_LONG_TIMEOUT))
    {
        USART1_SendString("[ESP] tcp connect fail\r\n");
        return 0;
    }

    return 1;
}

/**
 *  @brief 进入透传模式
 */
u8 ESP8266_EnterTransparent(void)
{
    // 1. 打开透传
    if (!ESP8266_SendCmd("AT+CIPMODE=1\r\n", "OK", ESP8266_SHORT_TIMEOUT))
    {
        USART1_SendString("[ESP] set transparent fail\r\n");
        return 0;
    }

    // 2. 发送 AT+CIPSEND 之后模块才真正切换到透传，提示符是 '>'
    if (!ESP8266_SendCmd("AT+CIPSEND\r\n", ">", ESP8266_SHORT_TIMEOUT))
    {
        USART1_SendString("[ESP] enter send mode fail\r\n");
        return 0;
    }

    // 3. 透传开启后串口上会出现换行等残留字符，先清干净再发 MQTT
    ESP8266_Recv_Clear();

    return 1;
}

/**
 *  @brief 退出透传模式
 */
u8 ESP8266_ExitTransparent(void)
{
    // 1. 保证发送 +++ 前后各有一段总线静默
    delay_ms(30);
    USART3_SendBuffer((u8 *)"+++", 3);
    delay_ms(30);

    // 2. 用一条最基础的 AT 指令确认模块已经回到命令模式
    ESP8266_Recv_Clear();
    if (!ESP8266_SendCmd("AT\r\n", "OK", ESP8266_SHORT_TIMEOUT))
    {
        USART1_SendString("[ESP] exit transparent fail\r\n");
        return 0;
    }

    return 1;
}

/**
 *  @brief ESP8266 完整连云流程
 */
u8 ESP8266_ConnectCloud(const char *ssid, const char *psk,
                        const char *ip, u16 port)
{
    // 1. 测试模块在线
    if (!ESP8266_Test_AT())
    {
        USART1_SendString("[ESP] module no response\r\n");
        return 0;
    }

    /*
        2. 复位模块，清掉上一次运行残留的状态

        ★ v21 修: 不再等 "ready" 这个关键字。
          实测你这颗模块(AT 固件 version 1.7)的启动信息是:
              " version : 1.7(5d6f877)  SPI Speed : 40M ..."
          里面【根本没有 "ready" 这个词】。原来等 "ready" 必然失败,
          每次都要白等 2 秒超时(日志里的 [ESP] no 'ready' got=401B)。

          改法: 复位后只需要"等模块启动完", 不依赖具体关键字。
          用 AT 探测代替: 复位 -> 等 2 秒 -> 发 AT 看回不回 OK。
          AT 能回 OK 就证明模块已经起来了, 这比猜启动字符串可靠得多
          (不同版本/不同厂家的 AT 固件启动信息都不一样)。
    */
    ESP8266_SendCmd("AT+RST\r\n", "OK", ESP8266_SHORT_TIMEOUT);

    /* 复位后给模块 2 秒启动时间(这期间不通讯, 只等) */
    delay_ms(2000);

    /* 清掉启动期间吐出的所有乱码 */
    ESP8266_Recv_Clear();

    /*
        再用一次 AT 确认模块真的起来了。
        失败也不直接返回失败 —— 有些模块 RST 后不会回 OK,
        留一次机会给后面的 ConnectAP 去试。
        但成功的话说明模块状态健康, 值得记一笔。
    */
    if (ESP8266_Test_AT())
    {
        LOG_Str("[ESP] reset ok, module alive\r\n");
    }
    else
    {
        LOG_Str("[ESP] warn: no response after reset, continue anyway\r\n");
    }

    // 3. 连接热点
    if (!ESP8266_ConnectAP(ssid, psk))
    {
        return 0;
    }

    // 4. 建立 TCP 连接
    if (!ESP8266_ConnectTCP(ip, port))
    {
        return 0;
    }

    // 5. 进入透传模式
    if (!ESP8266_EnterTransparent())
    {
        return 0;
    }

    USART1_SendString("[ESP] cloud link ready\r\n");
    return 1;
}
