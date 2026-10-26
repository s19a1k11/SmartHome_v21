#include "mqtt.h"

/* ==========================================================================
   MQTT 报文组装缓冲 —— 必须是 static, 不能放栈上
   ==========================================================================
   ★ 为什么不能放栈上(v20 复查发现的问题):
     主机栈只有 0x400 = 1024 字节。而调用链
         Cloud_Upload_All(560 字节局部) -> Cloud_Publish -> MQTT_Publish(576)
     三层加起来 1152 字节, 已经【超过整个栈】——
     结果会踩坏相邻 RAM(全局变量/返回地址), 表现是随机的 HardFault
     或数据错乱, 而且只在"上报"这个动作发生时才出现, 极难定位。

     改成 static 之后, 这几个缓冲移到 .bss, 不占栈;
     栈上只剩几十字节的局部变量, 1024 字节绰绰有余。

   ★ 为什么可以共用一块: 这三个发送函数【不会嵌套、不会重入】——
     它们都在主循环的单线程上下文里被顺序调用(cloud.c -> mqtt.c),
     一次只有一条报文在组装。共用一块 512 字节既省 RAM 又避免三份浪费。
   ========================================================================== */
static u8 s_mqtt_tx_buf[MQTT_PACKAGE_SIZE];

/**
 *  @brief 把剩余长度按 MQTT 变长编码写入缓冲区
 *
 *  MQTT 的剩余长度每字节只有低 7 位有效，最高位是 continuation 位，
 *  低位在前、最后一字节的最高位清零。之前的代码直接用一字节存长度，
 *  超过 127 字节的 JSON 会被云平台直接判为格式错误。
 *
 *  @return 编码占用的字节数
 */
static u8 MQTT_Encode_Length(u8 *buf, u32 length)
{
    u8 pos = 0;
    u8 encoded = 0;

    do
    {
        encoded = (u8)(length % 128);
        length = length / 128;
        if (length > 0)
        {
            encoded |= 0x80;
        }
        buf[pos++] = encoded;
    } while (length > 0 && pos < 4);

    return pos;
}

/**
 *  @brief 解码 MQTT 变长字段
 *
 *  @return 变长字段占用的字节数，解码结果写入 length
 */
static u8 MQTT_Decode_Length(u8 *buf, u32 *length)
{
    u8 pos = 0;
    u32 multiplier = 1;
    u32 value = 0;

    *length = 0;

    do
    {
        if (pos >= 4)
        {
            return 0;
        }
        value += (u32)(buf[pos] & 0x7F) * multiplier;
        multiplier *= 128;
        pos += 1;
    } while ((buf[pos - 1] & 0x80) != 0);

    *length = value;
    return pos;
}

/**
 *  @brief 计算一包 MQTT 报文的总长度
 */
u16 MQTT_Packet_Length(u8 *buf, u16 len)
{
    u32 remaining = 0;
    u8 head = 0;
    u16 total = 0;

    if (buf == NULL || len < 2)
    {
        return 0;
    }

    head = MQTT_Decode_Length(buf + 1, &remaining);
    if (head == 0)
    {
        return 0;
    }

    total = (u16)(1 + head + remaining);

    // 还没收全就返回 0，让调用方继续等
    if (total > len)
    {
        return 0;
    }

    return total;
}

/**
 *  @brief 请求连接到 MQTT 云平台
 */
u8 MQTT_Connect_Package_Send(const char *client, const char *username,
                             const char *password, u16 keep_alive_time)
{
    // 协议名固定为 "MQTT"，长度 4 字节
    const char mqtt_protocol_name[4] = {'M', 'Q', 'T', 'T'};
    const u16 mqtt_protocol_len = 4;

    u16 client_len = 0;
    u16 username_len = 0;
    u16 password_len = 0;
    u32 payload_len = 0;
    u32 variable_header_len = 0;
    u32 remaining_len = 0;

    u8 *MQTT_Connect_Package = s_mqtt_tx_buf;
    u16 pos = 0;

    /* 清空共享缓冲: 上一包的残留字节不能被当成这一包的内容 */
    memset(MQTT_Connect_Package, 0, MQTT_PACKAGE_SIZE);

    if (client == NULL || username == NULL || password == NULL)
    {
        return 0;
    }

    client_len = (u16)strlen(client);
    username_len = (u16)strlen(username);
    password_len = (u16)strlen(password);

    variable_header_len = 2 + mqtt_protocol_len + 1 + 1 + 2;
    payload_len = 2 + client_len + 2 + username_len + 2 + password_len;
    remaining_len = variable_header_len + payload_len;

    // 装不下直接放弃，绝不越界写数组
    // ★ 返回 0 而不是静默 return: 让调用方能知道"这条没发出去"
    if (remaining_len + 5 > MQTT_PACKAGE_SIZE)
    {
        return 0;
    }

    // 1. 固定头：报文类型 CONNECT(0x10) + 变长剩余长度
    MQTT_Connect_Package[pos++] = 0x10;
    pos += MQTT_Encode_Length(MQTT_Connect_Package + pos, remaining_len);

    // 2. 可变头：协议名长度、协议名、协议等级、连接标志、Keep Alive
    MQTT_Connect_Package[pos++] = (u8)(mqtt_protocol_len >> 8);
    MQTT_Connect_Package[pos++] = (u8)(mqtt_protocol_len & 0x00FF);
    memcpy(MQTT_Connect_Package + pos, mqtt_protocol_name, mqtt_protocol_len);
    pos += mqtt_protocol_len;
    MQTT_Connect_Package[pos++] = 0x04;    // 协议等级 MQTT 3.1.1
    MQTT_Connect_Package[pos++] = 0xC2;    // 用户名 + 密码 + 清除会话
    MQTT_Connect_Package[pos++] = (u8)(keep_alive_time >> 8);
    MQTT_Connect_Package[pos++] = (u8)(keep_alive_time & 0x00FF);

    // 3. 有效载荷：顺序固定为 ClientID -> UserName -> Password
    MQTT_Connect_Package[pos++] = (u8)(client_len >> 8);
    MQTT_Connect_Package[pos++] = (u8)(client_len & 0x00FF);
    memcpy(MQTT_Connect_Package + pos, client, client_len);
    pos += client_len;

    MQTT_Connect_Package[pos++] = (u8)(username_len >> 8);
    MQTT_Connect_Package[pos++] = (u8)(username_len & 0x00FF);
    memcpy(MQTT_Connect_Package + pos, username, username_len);
    pos += username_len;

    MQTT_Connect_Package[pos++] = (u8)(password_len >> 8);
    MQTT_Connect_Package[pos++] = (u8)(password_len & 0x00FF);
    memcpy(MQTT_Connect_Package + pos, password, password_len);
    pos += password_len;

    // 4. 利用 WiFi 模块的透传通道发送到云平台
    USART3_SendBuffer(MQTT_Connect_Package, pos);
    return 1;
}

/**
 *  @brief MCU 发送数据到云平台
 *
 *  ★ 返回值必须接。返回 0 = 报文超长, 这条消息被丢弃。
 *    改造前这里是 void + 静默 return, 而 MQTT_PACKAGE_SIZE 只有 256、
 *    实际 payload 有 306 字节 —— 结果【每一条上行都被丢掉】,
 *    云端一个字节收不到, 日志里却什么异常都没有。
 *    这类"链路全通但数据不见"的故障, 不返回错误码根本查不出来。
 */
u8 MQTT_Publish_Package_Send(const char *publish_topic, const char *payload)
{
    u16 publish_topic_len = 0;
    u16 payload_len = 0;
    u32 remaining_len = 0;

    u8 *MQTT_Publish_Package = s_mqtt_tx_buf;
    u16 pos = 0;

    /* 清空共享缓冲: 上一包的残留字节不能被当成这一包的内容 */
    memset(MQTT_Publish_Package, 0, MQTT_PACKAGE_SIZE);

    if (publish_topic == NULL || payload == NULL)
    {
        return 0;
    }

    publish_topic_len = (u16)strlen(publish_topic);
    payload_len = (u16)strlen(payload);
    remaining_len = 2 + publish_topic_len + payload_len;

    /*
        超长: 返回 0, 由 cloud.c 打日志。
        ★ 为什么用 "+5" 而不是精确算: MQTT 变长剩余长度最多占 4 字节,
          再加 1 字节类型, 5 是最坏情况的固定头长度。用最坏值判断
          偏保守(可能多留几字节), 但绝不会越界 —— 这里安全比紧凑重要。
    */
    if (remaining_len + 5 > MQTT_PACKAGE_SIZE)
    {
        return 0;
    }

    // 1. 固定头：PUBLISH QoS0(0x30) + 变长剩余长度
    MQTT_Publish_Package[pos++] = 0x30;
    pos += MQTT_Encode_Length(MQTT_Publish_Package + pos, remaining_len);

    // 2. 可变头：主题长度（高字节在前）+ 主题字符串
    MQTT_Publish_Package[pos++] = (u8)(publish_topic_len >> 8);
    MQTT_Publish_Package[pos++] = (u8)(publish_topic_len & 0x00FF);
    memcpy(MQTT_Publish_Package + pos, publish_topic, publish_topic_len);
    pos += publish_topic_len;

    // 3. 有效载荷：QoS0 没有报文标识符，紧跟就是数据
    memcpy(MQTT_Publish_Package + pos, payload, payload_len);
    pos += payload_len;

    // 4. 发送到云平台
    USART3_SendBuffer(MQTT_Publish_Package, pos);
    return 1;
}

/**
 *  @brief MCU 发送订阅主题数据包到 MQTT 云平台
 */
u8 MQTT_Subscribe_Package_Send(const char *subscribe_topic)
{
    u16 subscribe_topic_len = 0;
    u32 remaining_len = 0;

    u8 *MQTT_Subscribe_Package = s_mqtt_tx_buf;
    u16 pos = 0;

    /* 清空共享缓冲: 上一包的残留字节不能被当成这一包的内容 */
    memset(MQTT_Subscribe_Package, 0, MQTT_PACKAGE_SIZE);

    if (subscribe_topic == NULL)
    {
        return 0;
    }

    subscribe_topic_len = (u16)strlen(subscribe_topic);
    remaining_len = 2 + 2 + subscribe_topic_len + 1;

    if (remaining_len + 5 > MQTT_PACKAGE_SIZE)
    {
        return 0;
    }

    // 1. 固定头：SUBSCRIBE(0x82) + 变长剩余长度
    MQTT_Subscribe_Package[pos++] = 0x82;
    pos += MQTT_Encode_Length(MQTT_Subscribe_Package + pos, remaining_len);

    // 2. 可变头：报文标识符，固定用 10
    MQTT_Subscribe_Package[pos++] = (u8)((u16)10 >> 8);
    MQTT_Subscribe_Package[pos++] = (u8)((u16)10 & 0x00FF);

    // 3. 有效载荷：主题长度 + 主题 + 服务等级
    MQTT_Subscribe_Package[pos++] = (u8)(subscribe_topic_len >> 8);
    MQTT_Subscribe_Package[pos++] = (u8)(subscribe_topic_len & 0x00FF);
    memcpy(MQTT_Subscribe_Package + pos, subscribe_topic, subscribe_topic_len);
    pos += subscribe_topic_len;
    MQTT_Subscribe_Package[pos++] = 0;    // QoS 0

    // 4. 发送到云平台
    USART3_SendBuffer(MQTT_Subscribe_Package, pos);

    /*
        ★ 这个 return 1 曾经漏掉(v21 修)。
          把函数从 void 改成 u8 时, 我只加了失败分支的 return 0,
          忘了加成功分支的 return 1 —— 函数走到末尾没有返回值,
          调用方拿到的是栈上的随机值。
          Keil 报 warning #940-D: missing return statement 抓出来的;
          gcc 的 -Wall 也会报 -Wreturn-type, 但之前只跑了 -fsyntax-only
          (只做语法检查, 不做控制流分析), 所以没发现。

          教训: 改返回类型时必须检查【每一条出口路径】。
    */
    return 1;
}

/**
 *  @brief MCU 发送心跳请求包到 MQTT 云平台
 */
void MQTT_PingReq_Package_Send(void)
{
    u8 ping_package[2] = {0xC0, 0x00};

    USART3_SendBuffer(ping_package, 2);
}

/**
 *  @brief 解析云平台下发的 PUBLISH 报文
 */
u8 MQTT_Publish_Package_Parse(u8 *buf, u16 len, char *topic, u16 topic_max,
                              char *payload, u16 payload_max)
{
    u32 remaining = 0;
    u16 pos = 0;
    u16 topic_len = 0;
    u16 copy_len = 0;

    if (buf == NULL || topic == NULL || payload == NULL)
    {
        return 0;
    }
    if (topic_max == 0 || payload_max == 0)
    {
        return 0;
    }

    // 1. 只处理 PUBLISH 报文
    if ((buf[0] & 0xF0) != MQTT_PKT_PUBLISH)
    {
        return 0;
    }

    // 2. 跳过固定头里的变长剩余长度字段
    pos = 1;
    pos += MQTT_Decode_Length(buf + pos, &remaining);
    if (len < pos + (u16)remaining)
    {
        return 0;
    }

    // 3. 取主题，两字节长度 + 主题内容
    topic_len = (u16)(((u16)buf[pos] << 8) | (u16)buf[pos + 1]);
    pos += 2;
    remaining -= 2;

    copy_len = (topic_len < (topic_max - 1)) ? topic_len : (topic_max - 1);
    memcpy(topic, buf + pos, copy_len);
    topic[copy_len] = '\0';
    pos += topic_len;
    remaining -= topic_len;

    // 4. QoS 大于 0 时中间夹着两字节报文标识符
    if ((buf[0] & 0x06) != 0)
    {
        pos += 2;
        remaining -= 2;
    }

    // 5. 剩下的就是有效载荷
    /* 两边都转成 u32 再比较, 避免 signed/unsigned 混比(原本只是警告, 顺手清掉) */
    copy_len = (remaining < (u32)(payload_max - 1)) ? (u16)remaining : (u16)(payload_max - 1);
    memcpy(payload, buf + pos, copy_len);
    payload[copy_len] = '\0';

    return 1;
}
