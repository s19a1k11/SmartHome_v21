#include "cloud.h"
#include "usart1.h"
#include "esp8266.h"
#include "autolight.h"
#include "sr602.h"

// 通过 USART1 打印调试日志开关，1 打开 / 0 关闭
// ★ 无论开关是几, 日志都先进缓冲, 由 main 循环最后一步 LOG_Flush() 发出
#define CLOUD_LOG (1)

// 云平台 CONNACK 标志，1 表示连接已经被服务器确认
volatile u8 g_mqtt_connack = 0;

// PINGRESP 标志，主机发出 PINGREQ 后置 0，收到服务器应答后置 1
volatile u8 g_mqtt_ping_resp = 0;

// MQTT 下行解析临时缓冲
static char s_cloud_topic[CLOUD_TOPIC_SIZE] = {0};
static char s_cloud_payload[CLOUD_RECV_SIZE] = {0};

/* ==========================================================================
   上行 JSON 组装缓冲 —— 必须是 static, 不能放栈上
   ==========================================================================
   ★ v20 复查发现的栈溢出:
     主机栈 1024 字节, 而 Cloud_Upload_All 的局部 payload 就占 560 字节,
     再往下调 MQTT_Publish_Package_Send(576 字节局部) = 1152 字节, 直接溢出。
     栈溢出会踩坏相邻 RAM, 表现是随机的 HardFault, 且只在"上报"时出现。

   ★ 为什么可以共用一块:
     Cloud_Upload_All / Cloud_Upload_Online / Cloud_Upload_Slave 三者
     不会互相嵌套 —— 主循环里是顺序调用的, 同一时刻只有一个在拼 JSON。
     共用一块 448 字节, 省下两份 RAM。
   ========================================================================== */
static char s_cloud_up_buf[CLOUD_PAYLOAD_SIZE];

/**
 *  @brief 在 JSON 中提取整型键值
 */
u8 JSON_GetInt(const char *json, const char *key, int *value)
{
    char target[32] = {0};
    const char *p = NULL;
    int sign = 1;
    long val = 0;
    u8 digits = 0;

    if (json == NULL || key == NULL || value == NULL)
    {
        return 0;
    }

    if (strlen(key) > 24)
    {
        return 0;
    }
    sprintf(target, "\"%s\"", key);

    p = strstr(json, target);
    if (p == NULL)
    {
        return 0;
    }
    p += strlen(target);

    while (*p != '\0' && *p != ':')
    {
        p += 1;
    }
    if (*p != ':')
    {
        return 0;
    }
    p += 1;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
    {
        p += 1;
    }

    if (*p == '-')
    {
        sign = -1;
        p += 1;
    }
    while (*p >= '0' && *p <= '9')
    {
        val = val * 10 + (*p - '0');
        p += 1;
        digits += 1;
    }

    if (digits == 0)
    {
        return 0;
    }

    *value = (int)(sign * val);
    return 1;
}

/**
 *  @brief 在 JSON 中提取布尔键值
 */
u8 JSON_GetBool(const char *json, const char *key, u8 *value)
{
    char target[32] = {0};
    const char *p = NULL;

    if (json == NULL || key == NULL || value == NULL)
    {
        return 0;
    }

    if (strlen(key) > 24)
    {
        return 0;
    }
    sprintf(target, "\"%s\"", key);

    p = strstr(json, target);
    if (p == NULL)
    {
        return 0;
    }
    p += strlen(target);

    while (*p != '\0' && *p != ':')
    {
        p += 1;
    }
    if (*p != ':')
    {
        return 0;
    }
    p += 1;
    while (*p == ' ' || *p == '\t' || *p == '\r' || *p == '\n')
    {
        p += 1;
    }

    if (strncmp(p, "true", 4) == 0)
    {
        *value = 1;
        return 1;
    }
    if (strncmp(p, "false", 5) == 0)
    {
        *value = 0;
        return 1;
    }
    if (*p == '1')
    {
        *value = 1;
        return 1;
    }
    if (*p == '0')
    {
        *value = 0;
        return 1;
    }

    return 0;
}

/**
 *  @brief 拼接 key":value, 形式的 JSON 片段
 *
 *  ★ 每次 append 前都检查余量, 而不是只在最后检查一次总长度:
 *     sprintf 一旦越界就是直接踩坏相邻变量, 症状随机, 极难定位。
 */
static u16 Cloud_Append_Key(char *buf, u16 offset, u16 max,
                            const char *key, u16 value, u8 last)
{
    if (offset + 32 >= max)
    {
        return 0;
    }
    return (u16)sprintf(buf + offset, "\"%s\":%u%s", key, value, last ? "" : ",");
}

/**
 *  @brief 拼接 key":true/false, 形式的 JSON 片段，用于 Switch 类型的属性
 */
static u16 Cloud_Append_Bool(char *buf, u16 offset, u16 max,
                             const char *key, u16 value, u8 last)
{
    if (offset + 32 >= max)
    {
        return 0;
    }
    return (u16)sprintf(buf + offset, "\"%s\":%s%s",
                        key, value ? "true" : "false", last ? "" : ",");
}

/**
 *  @brief 在 JSON 中提取字符串键值
 */
u8 JSON_GetString(const char *json, const char *key, char *out, u8 out_size)
{
    char target[32] = {0};
    const char *p = NULL;
    const char *end = NULL;
    u16 len = 0;

    if (json == NULL || key == NULL || out == NULL || out_size == 0)
    {
        return 0;
    }

    if (strlen(key) > 24)
    {
        return 0;
    }
    sprintf(target, "\"%s\"", key);

    p = strstr(json, target);
    if (p == NULL)
    {
        return 0;
    }
    p += strlen(target);
    while (*p == ' ' || *p == ':')
    {
        p += 1;
    }

    if (*p != '"')
    {
        return 0;
    }
    p += 1;
    end = strchr(p, '"');
    if (end == NULL)
    {
        return 0;
    }

    len = (u16)(end - p);
    if (len >= out_size)
    {
        return 0;
    }
    memcpy(out, p, len);
    out[len] = '\0';

    return 1;
}

// 最近一次上报的时刻，用于限制即时上报的最小间隔
static u32 s_last_upload_ms = 0;

/**
 *  @brief 判断当前是否允许上报，防止连点或状态抖动把平台顶限流
 */
static u8 Cloud_Upload_Allowed(void)
{
    u32 now = Delay_GetMs();

    if (now - s_last_upload_ms < CLOUD_UPLOAD_MIN_GAP)
    {
        return 0;
    }
    s_last_upload_ms = now;
    return 1;
}

/**
 *  @brief 把一台从机的传感器 + 执行器状态追加进 JSON
 *
 *  键顺序(和平台物模型对照时按这个顺序找):
 *      Temp%d / Hum%d / Lux%d / Led%d / Bp%d / SenErr%d
 *
 *  ★ 注意: 这里【没有 SR602_%d】。
 *    SR602 已移到主机侧, 只报一个单值键 "SR602"(见 Cloud_Upload_All)。
 *    从机的 0x0105 现在是保留字段, 固定 0。
 *
 *  @return 本次写入的字节数, 0 表示余量不够(调用方应停止追加)
 */
static u16 Cloud_Append_Slave_Data(char *payload, u16 pos, u16 max, u8 index)
{
    Slave_Device *dev = &g_slave[index];
    char key[24] = {0};
    u16 n = 0;
    u16 start = pos;

    /* 1. 温湿度: 整数寄存器 + 小数寄存器拼成一位小数, 避免浮点运算 */
    sprintf(key, "Temp%d", index + 1);
    if (pos + 40 >= max) return 0;
    n = (u16)sprintf(payload + pos, "\"%s\":%u.%01u,", key, dev->temp_int, dev->temp_dec);
    pos += n;

    sprintf(key, "Hum%d", index + 1);
    if (pos + 40 >= max) return 0;
    n = (u16)sprintf(payload + pos, "\"%s\":%u.%01u,", key, dev->humi_int, dev->humi_dec);
    pos += n;

    /* 2. 光照 */
    sprintf(key, "Lux%d", index + 1);
    pos += Cloud_Append_Key(payload, pos, max, key, dev->light, 0);

    /* 3. 灯的【真实状态】—— 来自 ModBus 回读, 不是下发值
          ★ 这是"上传 Led 状态"的落地: 云端看到的永远是
            从机寄存器里现在的值。哪怕下发被拒或者被本地改动,
            这里报的也是真实情况。 */
    sprintf(key, "Led%d", index + 1);
    pos += Cloud_Append_Bool(payload, pos, max, key, dev->led, 0);

    /* 4. 蜂鸣器真实状态 */
    sprintf(key, "Bp%d", index + 1);
    pos += Cloud_Append_Bool(payload, pos, max, key, dev->beep, 0);

    /* 5. 数据是否可信(两个来源取或)
          a) 从机自己报的 0x0002 位图 bit0 -> 它内部连续 2 次 DHT11 采集失败
          b) 主机这边连续 2 轮读不到输入区 -> 数据根本拿不回来

          ★ 离线时强制报 false:
            从机整个不在线时 data_err_cnt 也会一直涨(因为读不到),
            如果不屏蔽, SenErr 和 Online 会同时为异常,
            云端界面上会出现"离线 + 传感器故障"两条告警,
            把"查供电"这条真正该做的事淹没掉。 */
    sprintf(key, "SenErr%d", index + 1);
    pos += Cloud_Append_Bool(payload, pos, max, key,
                             (u16)((dev->online &&
                                    ((dev->sensor_health & 0x0001) ||
                                     ModBus_Data_Failed(index))) ? 1 : 0), 0);

    return (u16)(pos - start);
}

/**
 *  @brief 上传指定从机的传感器数据
 */
/**
 *  @brief 统一的发布出口 —— 检查 MQTT 层返回值并打诊断
 *
 *  ★ 为什么要包一层:
 *    MQTT_Publish_Package_Send 返回 0 表示"报文超长被丢弃"。
 *    改造前它是 void, 丢弃时什么痕迹都不留 —— 于是出现了
 *    "MQTT 缓冲区 256 < payload 306, 所有上行静默丢失"这种
 *    极难定位的故障(链路全通、心跳正常、云端就是没数据)。
 *    现在每一次丢弃都会在日志里留下 payload 长度和原因。
 *
 *  @param payload 已拼好的 JSON
 *  @return 1 已发出 / 0 未发出
 */
static u8 Cloud_Publish(const char *payload)
{
    u16 need = 0;

    if (MQTT_Publish_Package_Send(CLOUD_PUBLISH_TOPIC, payload))
    {
        return 1;
    }

    /*
        走到这里说明被丢弃了。把"需要多少 / 上限多少"都打出来,
        现场一眼就能判断是 payload 太长还是缓冲区太小。
    */
    need = (u16)(strlen(payload) + 2 + strlen(CLOUD_PUBLISH_TOPIC) + 5);
    LOG_Str("[CLOUD] PUBLISH DROPPED! payload=");
    LOG_Num((u16)strlen(payload));
    LOG_Str(" need=");
    LOG_Num(need);
    LOG_Str(" max=");
    LOG_Num(MQTT_PACKAGE_SIZE);
    LOG_Str("\r\n");
    return 0;
}

/**
 *  @brief 上传指定从机的传感器数据
 *
 *  @return 1 已发出 / 0 未发出
 */
u8 Cloud_Upload_Slave(u8 index)
{
    char *payload = s_cloud_up_buf;
    u16 pos = 0;

    /* 清空共享缓冲: 上一包的残留不能被当成这一包的内容 */
    memset(payload, 0, CLOUD_PAYLOAD_SIZE);

    if (index >= SLAVE_NUM)
    {
        return 0;
    }

    if (Cloud_Upload_Allowed() == 0)
    {
        return 0;
    }

    pos += (u16)sprintf(payload + pos, "{");
    pos += Cloud_Append_Slave_Data(payload, pos, CLOUD_PAYLOAD_SIZE, index);

    if (pos > 1 && payload[pos - 1] == ',')
    {
        pos -= 1;
    }
    pos += (u16)sprintf(payload + pos, "}");

    if (pos >= CLOUD_PAYLOAD_SIZE)
    {
        LOG_Str("[CLOUD] slave payload overflow\r\n");
        return 0;
    }

#if CLOUD_LOG && defined(MODBUS_LOOPBACK)
    LOG_Str("[CLOUD] up: ");
    LOG_Str(payload);
    LOG_Str("\r\n");
#endif

    return Cloud_Publish(payload);
}

/**
 *  @brief 上传所有从机的在线情况
 *
 *  @return 1 已发出 / 0 未发出
 */
u8 Cloud_Upload_Online(void)
{
    char *payload = s_cloud_up_buf;
    char key[24] = {0};
    u16 pos = 0;
    u8 i = 0;

    memset(payload, 0, CLOUD_PAYLOAD_SIZE);

    if (Cloud_Upload_Allowed() == 0)
    {
        return 0;
    }

    pos += (u16)sprintf(payload + pos, "{");

    for (i = 0; i < SLAVE_NUM; i++)
    {
        sprintf(key, "Online%d", i + 1);
        pos += Cloud_Append_Bool(payload, pos, CLOUD_PAYLOAD_SIZE,
                                 key, g_slave[i].online, (u8)(i + 1 >= SLAVE_NUM));
    }

    pos += (u16)sprintf(payload + pos, "}");

    if (pos >= CLOUD_PAYLOAD_SIZE)
    {
        LOG_Str("[CLOUD] online payload overflow\r\n");
        return 0;
    }

    LOG_Str("[CLOUD] online: ");
    LOG_Str(payload);
    LOG_Str("\r\n");

    return Cloud_Publish(payload);
}

/**
 *  @brief 三台从机的数据 + 主机人体感应 + 在线状态合并成一条 MQTT 消息
 *
 *  每条消息都是对平台的一次访问，平台有流控机制，上报太频繁会被直接
 *  掐断连接并惩罚半小时。所以能合并的坚决合并。
 */
u8 Cloud_Upload_All(void)
{
    char *payload = s_cloud_up_buf;
    char key[24] = {0};
    u16 pos = 0;
    u8 i = 0;
    u16 n = 0;

    memset(payload, 0, CLOUD_PAYLOAD_SIZE);

    pos += (u16)sprintf(payload + pos, "{");

    // 1. 三台从机的传感器 + 执行器状态
    for (i = 0; i < SLAVE_NUM; i++)
    {
        n = Cloud_Append_Slave_Data(payload, pos, CLOUD_PAYLOAD_SIZE, i);
        if (n == 0)
        {
            LOG_Str("[CLOUD] payload full, truncate\r\n");
            break;
        }
        pos += n;
    }

    // 2. 主机自己的人体感应(SR602) —— 单值键, 不带编号
    //    ★ 为什么单独一个键: SR602 只在主机上, 不属于任何一台从机。
    pos += Cloud_Append_Key(payload, pos, CLOUD_PAYLOAD_SIZE, "SR602",
                            (u16)SR602_Current(), 0);

    // 3. 用于夜晚判定的平均光照 —— 让云端也能看到判定依据,
    //    标定阈值时不用守着串口看日志
    pos += Cloud_Append_Key(payload, pos, CLOUD_PAYLOAD_SIZE, "LuxAvg",
                            (u16)(AutoLight_Avg_Light() == 0xFFFF ? 0 : AutoLight_Avg_Light()), 0);

    // 4. 当前是否判定为夜晚 —— 调试/验证用
    pos += Cloud_Append_Key(payload, pos, CLOUD_PAYLOAD_SIZE, "Night",
                            (u16)AutoLight_Is_Night(), 0);

    // 5. 在线状态
    for (i = 0; i < SLAVE_NUM; i++)
    {
        sprintf(key, "Online%d", i + 1);
        if (pos + 32 >= CLOUD_PAYLOAD_SIZE)
        {
            break;
        }
        pos += Cloud_Append_Bool(payload, pos, CLOUD_PAYLOAD_SIZE,
                                 key, g_slave[i].online, (u8)(i + 1 >= SLAVE_NUM));
    }

    // 6. 去掉多余的尾部逗号, 保证是合法 JSON
    if (pos > 1 && payload[pos - 1] == ',')
    {
        pos -= 1;
    }
    pos += (u16)sprintf(payload + pos, "}");

    if (pos >= CLOUD_PAYLOAD_SIZE)
    {
        LOG_Str("[CLOUD] all payload overflow\r\n");
        return 0;
    }

#if CLOUD_LOG
    LOG_Str("[CLOUD] up: ");
    LOG_Str(payload);
    LOG_Str("\r\n");
#endif

    return Cloud_Publish(payload);
}

/**
 *  @brief 在线状态发生变化时立刻上报一次
 *
 *  ★ 需求"某个从机连续两次数据采集都没有成功则反馈云端该从机离线"的
 *    最后一环。判定在 modbus.c(连续 2 次全超时 -> online = 0),
 *    这里负责"一变就发", 不等 15 秒的定时上传。
 *
 *  ★ v20 修的关键 bug: reported 标志必须在【发送成功之后】才更新。
 *    改造前是"先置 reported 再发", 而 Cloud_Upload_Online() 可能因为
 *    CLOUD_UPLOAD_MIN_GAP(8 秒) 或报文超长而根本没发出去。
 *    后果: reported 已经等于 online, 下一轮比较"相等" -> 不再重试 ->
 *    这次离线通知【永久丢失】, 云端一直以为这台从机在线, 直到 15 秒的
 *    定时上报才纠正。若那段时间还叠加了别的上报, 定时上报也可能被
 *    MIN_GAP 挡掉 —— 表现就是"从机断电了, 云端半天不报离线"。
 *    现在: 只有真正发出去了才更新 reported; 没发出去就留到下轮重试。
 *
 *  @return 1 有翻转且已成功上报 / 0 无变化 或 上报被挡下(下轮会重试)
 */
u8 Cloud_Upload_Online_If_Changed(void)
{
    u8 i = 0;
    u8 changed = 0;

    for (i = 0; i < SLAVE_NUM; i++)
    {
        if (g_slave[i].online != g_slave[i].online_reported)
        {
            changed = 1;

            LOG_Str("[CLOUD] slave ");
            LOG_Num(i + 1);
            LOG_Str(g_slave[i].online ? " ONLINE\r\n" : " OFFLINE\r\n");

            /*
                ★ 一次只发一条: Cloud_Upload_Online 发的是"全部三台的状态",
                  所以一次翻转就够了, 不需要逐台发。发成功才更新所有
                  reported, 失败则全部保留, 下一轮继续尝试。
            */
            if (Cloud_Upload_Online())
            {
                u8 k = 0;
                for (k = 0; k < SLAVE_NUM; k++)
                {
                    g_slave[k].online_reported = g_slave[k].online;
                }
                return 1;
            }

            /*
                没发出去(被 MIN_GAP 挡下或报文超长) —— reported 保持原值,
                下一轮循环还会进到这里重试。
                ★ 这里 return 0 而不是继续循环: 已经打了一条日志,
                  再对其它从机重复打会把日志刷满, 而且发送通道是同一个,
                  这一轮再试也是白试。
            */
            return 0;
        }
    }

    (void)changed;
    return 0;
}

/**
 *  @brief 把平台下发的舵机枚举文字换算成从机档位
 */
static int Cloud_Gear_From_Text(const char *text)
{
    if (strcmp(text, "clockwise") == 0)
    {
        return CLOUD_SG90_CLOCKWISE_GEAR;
    }
    if (strcmp(text, "counter_clockwise") == 0)
    {
        return CLOUD_SG90_CCW_GEAR;
    }
    return -1;
}

/**
 *  @brief 把某台从机的灯按布尔值写入
 */
static u8 Cloud_Write_LED(u8 index, u8 on)
{
    Slave_Device *dev = NULL;
    u8 stat = 0;

    if (index >= SLAVE_NUM)
    {
        return 0;
    }

    dev = &g_slave[index];

    if (dev->online == 0)
    {
        LOG_Str("[CLOUD] led#");
        LOG_Num(index + 1);
        LOG_Str(" DROPPED: slave offline\r\n");
        return 0;
    }

    /*
        ★ v22: 检查写返回值并打日志。

        踩过的坑: 原来这里是
            ModBus_Write_Single_Reg(...);   // 返回值直接丢掉
        写失败(超时/CRC/从机拒绝)时【什么痕迹都不留】,
        现场只能看到"平台上点了开关, 灯没亮", 完全不知道卡在哪。

        现在把失败原因打出来:
          stat=1 TIMEOUT   从机没应答
          stat=2 CRC       总线干扰
          stat=6 EXCEPT    从机拒绝(值越界)
    */
    stat = ModBus_Write_Single_Reg(dev->addr, OUT_REG_LED,
                                   (u16)(on ? CLOUD_LED_ON_VALUE : 0));
    if (stat != MODBUS_OK)
    {
        LOG_Str("[CLOUD] led#");
        LOG_Num(index + 1);
        LOG_Str(on ? " ON" : " OFF");
        LOG_Str(" WRITE FAIL stat=");
        LOG_Num(stat);
        LOG_Str(" (1=timeout 2=crc 6=except)\r\n");
        return 0;
    }

    LOG_Str("[CLOUD] led#");
    LOG_Num(index + 1);
    LOG_Str(on ? " ON ok\r\n" : " OFF ok\r\n");
    return 1;
}

/**
 *  @brief 把某台从机的蜂鸣器按布尔值写入
 */
static u8 Cloud_Write_Beep(u8 index, u8 on)
{
    Slave_Device *dev = NULL;
    u8 stat = 0;

    if (index >= SLAVE_NUM)
    {
        return 0;
    }

    dev = &g_slave[index];

    if (dev->online == 0)
    {
        LOG_Str("[CLOUD] beep#");
        LOG_Num(index + 1);
        LOG_Str(" DROPPED: slave offline\r\n");
        return 0;
    }

    stat = ModBus_Write_Single_Reg(dev->addr, OUT_REG_BEEP, (u16)(on ? 1 : 0));
    if (stat != MODBUS_OK)
    {
        LOG_Str("[CLOUD] beep#");
        LOG_Num(index + 1);
        LOG_Str(" WRITE FAIL stat=");
        LOG_Num(stat);
        LOG_Str("\r\n");
        return 0;
    }

    LOG_Str("[CLOUD] beep#");
    LOG_Num(index + 1);
    LOG_Str(on ? " ON ok\r\n" : " OFF ok\r\n");
    return 1;
}

/**
 *  @brief 把某台从机的舵机按档位写入
 */
static void Cloud_Write_Servo(u8 index, u8 gear)
{
    Slave_Device *dev = NULL;

    if (index >= SLAVE_NUM)
    {
        return;
    }

    dev = &g_slave[index];

    if (dev->online == 0)
    {
        LOG_Str("[CLOUD] slave offline, drop servo\r\n");
        return;
    }

    ModBus_Write_Single_Reg(dev->addr, OUT_REG_SERVO, (u16)gear);
}

/**
 *  @brief 回读从机输出区并把真实状态同步回云端
 */
static void Cloud_Sync_Output(u8 index)
{
    u16 regs[OUT_REG_COUNT] = {0};
    Slave_Device *dev = NULL;

    if (index >= SLAVE_NUM)
    {
        return;
    }

    dev = &g_slave[index];

    if (dev->online == 0)
    {
        return;
    }

    if (ModBus_Read_Registers(dev->addr, OUT_REG_LED, OUT_REG_COUNT, regs) == MODBUS_OK)
    {
        dev->led = regs[0];
        dev->beep = regs[1];
        dev->servo = regs[2];
    }

    Cloud_Upload_Slave(index);
}

/**
 *  @brief 解析云端下发的 JSON 指令并转发到从机
 */
void Cloud_Handle_Downlink(const char *payload)
{
    u8 enable = 1;
    u8 val = 0;
    int servo = 0;
    int gear = 0;
    u8 index = 0;
    char key[16] = {0};
    char s_gear_text[24] = {0};
    u8 touched[SLAVE_NUM] = {0};

    if (payload == NULL)
    {
        return;
    }

    LOG_Str("[CLOUD] down: ");
    LOG_Str(payload);
    LOG_Str("\r\n");

    // 1. 总使能开关，stm32 = false 时整条命令丢弃
    if (JSON_GetBool(payload, "stm32", &enable) && (enable == 0))
    {
        LOG_Str("[CLOUD] stm32 disabled, drop cmd\r\n");
        return;
    }

    // 2. 灯
    for (index = 0; index < SLAVE_NUM; index++)
    {
        sprintf(key, "Led%d", index + 1);
        if (JSON_GetBool(payload, key, &val))
        {
            Cloud_Write_LED(index, val);
            touched[index] = 1;
        }
    }

    /*
        ★ v22 诊断: 一个键都没匹配上时, 把收到的原文和期望键名打出来。
        踩过的坑: 平台手打 {"LED1":true} 时代码找 "Led1" 找不到,
        整条命令被静默忽略, 现场只看到"点了没反应", 毫无线索。
    */
    {
        u8 any = 0;
        for (index = 0; index < SLAVE_NUM; index++) { if (touched[index]) { any = 1; break; } }
        if (!any)
        {
            LOG_Str("[CLOUD] !! NO KEY MATCHED. expected:");
            for (index = 0; index < SLAVE_NUM; index++) { LOG_Str(" Led"); LOG_Num(index + 1); }
            for (index = 0; index < SLAVE_NUM; index++) { LOG_Str(" Beep"); LOG_Num(index + 1); }
            for (index = 0; index < SLAVE_NUM; index++) { LOG_Str(" SG90_"); LOG_Num(index + 1); }
            LOG_Str(" (case sensitive)\r\n");
        }
    }

    // 3. 蜂鸣器
    for (index = 0; index < SLAVE_NUM; index++)
    {
        sprintf(key, "Beep%d", index + 1);
        if (JSON_GetBool(payload, key, &val))
        {
            Cloud_Write_Beep(index, val);
            touched[index] = 1;
        }
    }

    // 4. 舵机
    for (index = 0; index < SLAVE_NUM; index++)
    {
        sprintf(key, "SG90_%d", index + 1);
        if (JSON_GetString(payload, key, s_gear_text, (u8)sizeof(s_gear_text)))
        {
            gear = Cloud_Gear_From_Text(s_gear_text);
            if (gear >= 0)
            {
                Cloud_Write_Servo(index, (u8)gear);
                touched[index] = 1;
            }
        }
        else if (JSON_GetInt(payload, key, &servo))
        {
            if (servo < 0) servo = 0;
            if (servo > CLOUD_SG90_GEAR_MAX) servo = CLOUD_SG90_GEAR_MAX;
            Cloud_Write_Servo(index, (u8)servo);
            touched[index] = 1;
        }
    }

    // 5. 回读所有被动过的从机，确认生效并把真实状态同步回云端
    for (index = 0; index < SLAVE_NUM; index++)
    {
        if (touched[index])
        {
            Cloud_Sync_Output(index);
        }
    }
}

/**
 *  @brief 处理透传通道上收到的 MQTT 报文
 */
void Cloud_Rx_Process(void)
{
    u16 total = 0;
    u16 offset = 0;
    u16 pkt_len = 0;
    u16 rest = 0;
    u8 type = 0;

    if (!usart3_value.flag)
    {
        return;
    }

    total = usart3_value.count;

    while (offset < total)
    {
        pkt_len = MQTT_Packet_Length(usart3_value.data + offset, (u16)(total - offset));
        if (pkt_len == 0)
        {
            break;
        }

        type = usart3_value.data[offset] & 0xF0;
        switch (type)
        {
            case MQTT_PKT_CONNACK:
                g_mqtt_connack = 1;
                LOG_Str("[MQTT] connack\r\n");
                break;

            case MQTT_PKT_PUBLISH:
                if (MQTT_Publish_Package_Parse(usart3_value.data + offset, pkt_len,
                                               s_cloud_topic, CLOUD_TOPIC_SIZE,
                                               s_cloud_payload, CLOUD_RECV_SIZE))
                {
                    Cloud_Handle_Downlink(s_cloud_payload);
                }
                break;

            case MQTT_PKT_SUBACK:
                LOG_Str("[MQTT] suback\r\n");
                break;

            case MQTT_PKT_PINGRESP:
                g_mqtt_ping_resp = 1;
                break;

            default:
                break;
        }

        offset += pkt_len;
    }

    if (offset >= total)
    {
        USART3_Recv_Clear();
    }
    else if (offset == 0 && total >= USART3_BUFFER_SIZE)
    {
        /*
            ★ 死锁保护(必须有):
              缓冲区已经满了, 但一个完整报文都没解析出来。这只有两种可能:
                a) 收到的根本不是 MQTT 流(模块上电时吐的 AT 回显、乱码)
                b) 报文长度字段坏了, 永远等不到"收全"
              如果还按下面"把尾部挪到开头继续等"处理, rest 会等于 total,
              count 仍是满的, 下一轮又走进同一分支 —— 死循环,
              而且新字节一直被丢, 表现是"链路建立了但云端下发永远不生效"。
        */
        LOG_Str("[MQTT] rx buffer full & unparsable, drop\r\n");
        USART3_Recv_Clear();
    }
    else
    {
        // 还有没收全的尾部，挪到缓冲区开头继续等剩下的字节
        __disable_irq();
        rest = (u16)(usart3_value.count - offset);
        if (rest > 0 && rest <= USART3_BUFFER_SIZE)
        {
            memmove(usart3_value.data, usart3_value.data + offset, rest);
        }
        usart3_value.count = rest;
        usart3_value.flag = 0;
        __enable_irq();
    }
}

/**
 *  @brief 云平台层自检
 */
void Cloud_Self_Test(void)
{
    u8 i = 0;

    LOG_Str("\r\n==== CLOUD SELF TEST ====\r\n");

    for (i = 0; i < SLAVE_NUM; i++)
    {
        ModBus_Poll_Slave(i);
    }
    ModBus_Update_Online();

    Cloud_Handle_Downlink("{\"Led1\":true,\"Beep1\":true,\"SG90_1\":\"clockwise\"}");
    Cloud_Handle_Downlink("{\"Led1\":false}");
    Cloud_Upload_All();

    LOG_Str("==== CLOUD SELF TEST DONE ====\r\n\r\n");
}

/**
 *  @brief 建立到云平台的完整链路
 */
u8 Cloud_Link_Build(void)
{
    u32 wait_ms = 0;
    u8 retry = 0;

    ESP8266_ExitTransparent();

    while (retry < CLOUD_LINK_RETRY)
    {
        if (ESP8266_ConnectCloud(CLOUD_AP_SSID, CLOUD_AP_PSK,
                                 CLOUD_MQTT_IP, CLOUD_MQTT_PORT) != 0)
        {
            break;
        }
        retry += 1;
        LOG_Str("[CLOUD] retry link ");
        LOG_Num(retry);
        LOG_Str("\r\n");
        delay_ms(CLOUD_RELINK_INTERVAL);
    }

    if (retry >= CLOUD_LINK_RETRY)
    {
        LOG_Str("[CLOUD] link build failed\r\n");
        return 0;
    }

    g_mqtt_connack = 0;
    g_mqtt_ping_resp = 0;
    if (!MQTT_Connect_Package_Send(CLOUD_MQTT_CLIENT, CLOUD_MQTT_USERNAME,
                                   CLOUD_MQTT_PASSWORD, CLOUD_MQTT_KEEP_ALIVE))
    {
        /* CONNECT 报文本身超长(用户名/密码过长) -> 根本发不出去, 直接报错 */
        LOG_Str("[CLOUD] CONNECT packet too long!\r\n");
        return 0;
    }

    wait_ms = 0;
    while (wait_ms < CLOUD_CONNACK_TIMEOUT)
    {
        Cloud_Rx_Process();
        if (g_mqtt_connack)
        {
            break;
        }
        delay_ms(10);
        wait_ms += 10;
    }

    if (!g_mqtt_connack)
    {
        LOG_Str("[CLOUD] no connack\r\n");
        return 0;
    }
    LOG_Str("[CLOUD] mqtt connected\r\n");

    if (!MQTT_Subscribe_Package_Send(CLOUD_SUBSCRIBE_TOPIC))
    {
        LOG_Str("[CLOUD] SUBSCRIBE packet too long!\r\n");
        return 0;
    }
    delay_ms(300);
    Cloud_Rx_Process();

    return 1;
}
