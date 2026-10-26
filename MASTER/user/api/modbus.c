#include "modbus.h"
#include "usart1.h"
#include "cli.h"

// 通过 USART1 打印调试日志开关，1 打开 / 0 关闭。
// ★ 无论开关是几, 日志都先进缓冲, 由 main 循环最后一步 LOG_Flush() 发出
#define MODBUS_LOG (0)

// 总线上所有从机的管理数组
Slave_Device g_slave[SLAVE_NUM] = {0};
// 最近一次事务的异常码
u8 g_modbus_exception = 0;

// 事务输出的应答帧副本，防止被下一次发送覆盖
static u8 s_reply[RS485_BUFFER_SIZE] = {0};
static u16 s_reply_len = 0;

/*
	超时日志限流(每台从机一个时间戳)

	★ v21 修: 从"按次数限流"改成"按时间限流"。
	  原来按次数, 并且 any_ok 就清零 —— 但"部分成功"的情况会让它失效:
	    第1笔读系统区 成功 -> any_ok=1 -> 计数清零
	    第2笔读输入区 超时 -> 打一条 n=1
	    第3笔读输出区 成功
	  下一轮又从 n=1 开始 -> 每轮都打, 限流形同虚设。

	  现在按时间: 同一台从机的超时日志 MODBUS_TIMEOUT_LOG_GAP_S 秒内最多一条。
	  与成功/失败无关, 不会因为部分成功而失效。
*/
static u32 s_timeout_log_ms[SLAVE_NUM] = {0};

/*
	判断这次超时要不要打日志
	返回 1 = 该打 / 0 = 刚打过, 跳过
*/
static u8 ModBus_Timeout_Log_Should(u8 index)
{
    u32 now = 0;

    if (index >= SLAVE_NUM) return 1;

    now = Delay_GetMs();

    /* 首次(时间戳为0)或者距上次超过间隔 -> 打 */
    if (s_timeout_log_ms[index] == 0 ||
        (u32)(now - s_timeout_log_ms[index]) >= (MODBUS_TIMEOUT_LOG_GAP_S * 1000UL))
    {
        s_timeout_log_ms[index] = now;
        return 1;
    }
    return 0;
}

/**
 *  @brief CRC-16/MODBUS 校验
 *
 *  多项式 0xA001（反转形式），初值 0xFFFF，结果低字节在前。
 */
u16 ModBus_CRC16(u8 *buf, u16 len)
{
    u16 crc = 0xFFFF;
    u16 i = 0;
    u8 j = 0;

    for (i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
            {
                crc = (crc >> 1) ^ 0xA001;
            }
            else
            {
                crc = crc >> 1;
            }
        }
    }

    return crc;
}

/**
 *  @brief 拼接读取寄存器请求帧（功能码 0x03）
 *
 *  @return 请求帧长度，固定 8 字节
 */
static u16 ModBus_Build_Read_Frame(u8 *frame, u8 slave_addr,
                                   u16 start_addr, u16 reg_num)
{
    u16 crc = 0;

    // 1. 地址域与 PDU
    frame[0] = slave_addr;
    frame[1] = MODBUS_FC_READ_REG;
    frame[2] = (u8)(start_addr >> 8);   // 起始地址高位在前
    frame[3] = (u8)(start_addr & 0x00FF);
    frame[4] = (u8)(reg_num >> 8);      // 寄存器数量高位在前
    frame[5] = (u8)(reg_num & 0x00FF);

    // 2. CRC 低字节在前，这是 ModBus RTU 唯一一处顺序例外
    crc = ModBus_CRC16(frame, 6);
    frame[6] = (u8)(crc & 0x00FF);
    frame[7] = (u8)(crc >> 8);

    return 8;
}

/**
 *  @brief 拼接写单个寄存器请求帧（功能码 0x06）
 *
 *  @return 请求帧长度，固定 8 字节
 */
static u16 ModBus_Build_Write_Frame(u8 *frame, u8 slave_addr,
                                    u16 reg_addr, u16 value)
{
    u16 crc = 0;

    frame[0] = slave_addr;
    frame[1] = MODBUS_FC_WRITE_SINGLE;
    frame[2] = (u8)(reg_addr >> 8);
    frame[3] = (u8)(reg_addr & 0x00FF);
    frame[4] = (u8)(value >> 8);
    frame[5] = (u8)(value & 0x00FF);

    crc = ModBus_CRC16(frame, 6);
    frame[6] = (u8)(crc & 0x00FF);
    frame[7] = (u8)(crc >> 8);

    return 8;
}

/**
 *  @brief 校验从机应答帧的合法性
 *
 *  ★ 每一类失败都在这里留下【原始数据】:
 *    只打一句 "CRC FAIL" 是没用的 —— 现场要能回答
 *    "收到的 CRC 是多少、算出来的 CRC 是多少、帧长多少、内容是什么"。
 *    所以失败时把这三个信息全打出来, 配合 RX 原始帧可以直接对照手册。
 */
static u8 ModBus_Check_Reply(u8 slave_addr, u8 expect_fc, u8 *reply, u16 len)
{
    u16 recv_crc = 0;
    u16 calc_crc = 0;

    if (len < 5)
    {
        LOG_Str("[MB] REPLY TOO SHORT len=");
        LOG_Num(len);
        LOG_Str(" (need >=5)\r\n");
        if (len > 0) LOG_HexDump("RX", reply, len);
        return MODBUS_ERR_LEN;
    }

    recv_crc = (u16)reply[len - 2] | ((u16)reply[len - 1] << 8);
    calc_crc = ModBus_CRC16(reply, len - 2);

    /*
        ★ RX 原始帧: 出错时一定打; 正常时只在【运行时打开了逐帧日志】才打。
          这里的判据不能只放在 CRC 失败分支里 —— 那样 @log on 之后
          正常帧看不到 RX(只能看到 TX), 调试时反而缺一半信息。
          正确的做法是: 先用开关判断一次, 覆盖正常路径;
          出错路径再无条件打(不管开关), 因为出错必须留下证据。
    */
    if (CLI_Get_ModBus_Log())
    {
        LOG_HexDump("RX", reply, len);
    }

    if (recv_crc != calc_crc)
    {
        /*
            ★ 只在【出错时】才打原始帧。
              v20 曾经在正常路径也无条件打 RX/TX, 实测代价:
                每笔事务 TX+RX 约 95 个字符
                一轮轮询(3 台 x 3 笔) = 855 字符
                9600 下 = 891ms
              而轮询周期只有 1000ms —— 日志吃掉 89% 的周期,
              主循环被严重拖慢, MQTT 心跳和云端下行都得不到及时处理。
              所以改成: 正常静默, 出错才回显。
              需要看正常帧时把 MODBUS_LOG 打开即可(那种模式是给调试用的,
              本来就不指望它跑得快)。
        */
        LOG_Str("[MB] CRC FAIL len=");
        LOG_Num(len);
        LOG_Str(" recv=0x");
        LOG_HexNum(recv_crc);
        LOG_Str(" calc=0x");
        LOG_HexNum(calc_crc);
        LOG_Str("\r\n");
        /*
            出错时必须留证据: 如果逐帧日志已经开着, 上面已经打过 RX 了,
            这里就不再重复; 如果没开, 这里补打一次 —— 出错是低频事件,
            几行日志换一个可定位的线索, 值得。
        */
        if (!CLI_Get_ModBus_Log())
        {
            LOG_HexDump("RX", reply, len);
        }
        return MODBUS_ERR_CRC;
    }

    if (reply[0] != slave_addr)
    {
        LOG_Str("[MB] ADDR MISMATCH exp=");
        LOG_Num(slave_addr);
        LOG_Str(" got=");
        LOG_Num(reply[0]);
        LOG_Str("\r\n");
        return MODBUS_ERR_ADDR;
    }

    if ((reply[1] & 0x80) != 0)
    {
        g_modbus_exception = reply[2];
        LOG_Str("[MB] SLAVE EXCEPTION fc=0x");
        LOG_HexNum(reply[1]);
        LOG_Str(" code=0x");
        LOG_HexNum(reply[2]);
        LOG_Str(" (01=func 02=addr 03=value)\r\n");
        return MODBUS_ERR_EXCEPT;
    }

    if (reply[1] != expect_fc)
    {
        LOG_Str("[MB] FUNC MISMATCH exp=0x");
        LOG_HexNum(expect_fc);
        LOG_Str(" got=0x");
        LOG_HexNum(reply[1]);
        LOG_Str("\r\n");
        return MODBUS_ERR_FUNC;
    }

    g_modbus_exception = 0;
    return MODBUS_OK;
}

/**
 *  @brief 以十六进制打印一帧数据
 *  ★ 走日志缓冲, 不直接发串口 —— 真正的发送在 main 循环最后一步
 */
void ModBus_Print_Frame(const char *tag, u8 *frame, u16 len)
{
    if (frame == NULL)
    {
        return;
    }
    LOG_HexDump(tag, frame, len);
}

#if defined(MODBUS_LOOPBACK)
// ===== 模拟从机寄存器镜像，下标 0 对应从机1，三块分别对应三个区 =====
static u16 s_sim_sys[SLAVE_NUM][SYS_REG_COUNT] = {0};
static u16 s_sim_in[SLAVE_NUM][IN_REG_COUNT] = {0};
static u16 s_sim_out[SLAVE_NUM][OUT_REG_COUNT] = {0};

static u8 ModBus_Sim_Read(u8 idx, u16 reg_addr, u16 *value)
{
    u16 off = (u16)(reg_addr & 0x00FF);
    u8 zone = (u8)(reg_addr >> 8);
    u32 sec = Delay_GetMs() / 1000;

    if (zone == 0x00 && off < SYS_REG_COUNT)
    {
        if (reg_addr == SYS_REG_RUN_SECONDS)
        {
            *value = (u16)sec;
        }
        else
        {
            *value = s_sim_sys[idx][off];
        }
        return 1;
    }

    if (zone == 0x01 && off < IN_REG_COUNT)
    {
        if (reg_addr == IN_REG_LIGHT)
        {
            *value = (u16)(800 + (sec % 100));
        }
        else
        {
            *value = s_sim_in[idx][off];
        }
        return 1;
    }

    if (zone == 0x02 && off < OUT_REG_COUNT)
    {
        *value = s_sim_out[idx][off];
        return 1;
    }

    return 0;
}

static u8 ModBus_Sim_Write(u8 idx, u16 reg_addr, u16 value)
{
    u16 off = (u16)(reg_addr & 0x00FF);
    u8 zone = (u8)(reg_addr >> 8);

    if (zone == 0x02 && off < OUT_REG_COUNT)
    {
        if (off == 0x00 && value > 1) return 0;
        if (off == 0x01 && value > 1) return 0;
        if (off == 0x02 && value > 2) return 0;

        s_sim_out[idx][off] = value;
        return 1;
    }

    return 0;
}

static u16 ModBus_Slave_Simulate(u8 *req, u16 req_len, u8 *reply)
{
    u16 crc_recv = 0;
    u16 crc_calc = 0;
    u16 start = 0;
    u16 num = 0;
    u16 val = 0;
    u16 pos = 0;
    u16 i = 0;
    u8 idx = 0xFF;

    if (req == NULL || reply == NULL || req_len < 8)
    {
        return 0;
    }

    crc_recv = (u16)req[req_len - 2] | ((u16)req[req_len - 1] << 8);
    crc_calc = ModBus_CRC16(req, (u16)(req_len - 2));
    if (crc_recv != crc_calc)
    {
        return 0;
    }

    for (i = 0; i < SLAVE_NUM; i++)
    {
        if (g_slave[i].addr == req[0])
        {
            idx = (u8)i;
            break;
        }
    }
    if (idx == 0xFF)
    {
        return 0;
    }

    if (req[1] == MODBUS_FC_READ_REG)
    {
        start = (u16)(((u16)req[2] << 8) | req[3]);
        num = (u16)(((u16)req[4] << 8) | req[5]);
        if (num == 0 || num > 0x7D)
        {
            return 0;
        }

        reply[0] = req[0];
        reply[1] = MODBUS_FC_READ_REG;
        reply[2] = (u8)(num * 2);
        pos = 3;
        for (i = 0; i < num; i++)
        {
            val = 0;
            ModBus_Sim_Read(idx, (u16)(start + i), &val);
            reply[pos++] = (u8)(val >> 8);
            reply[pos++] = (u8)(val & 0x00FF);
        }

        crc_calc = ModBus_CRC16(reply, pos);
        reply[pos++] = (u8)(crc_calc & 0x00FF);
        reply[pos++] = (u8)(crc_calc >> 8);
        return pos;
    }

    if (req[1] == MODBUS_FC_WRITE_SINGLE)
    {
        start = (u16)(((u16)req[2] << 8) | req[3]);
        val = (u16)(((u16)req[4] << 8) | req[5]);
        if (ModBus_Sim_Write(idx, start, val) == 0)
        {
            reply[0] = req[0];
            reply[1] = (u8)(req[1] | 0x80);
            reply[2] = 0x03;
            crc_calc = ModBus_CRC16(reply, 3);
            reply[3] = (u8)(crc_calc & 0x00FF);
            reply[4] = (u8)(crc_calc >> 8);
            return 5;
        }
        memcpy(reply, req, req_len);
        return req_len;
    }

    reply[0] = req[0];
    reply[1] = (u8)(req[1] | 0x80);
    reply[2] = 0x01;
    crc_calc = ModBus_CRC16(reply, 3);
    reply[3] = (u8)(crc_calc & 0x00FF);
    reply[4] = (u8)(crc_calc >> 8);
    return 5;
}
#endif

/**
 *  @brief 发一笔完整事务：发请求帧 -> 等待应答帧 -> 校验
 *
 *  ★ 超时用的是 MODBUS_REPLY_TIMEOUT = 250ms, 这个数字的来历
 *    在 modbus.h 里逐段算过(最坏 64.7ms x 3.9 倍余量)。
 */
static u8 ModBus_Transaction(u8 slave_addr, u8 *req, u16 req_len, u8 expect_fc)
{
#if defined(MODBUS_LOOPBACK)
    u8 stat = MODBUS_OK;
#else
    u16 wait = 0;
    u8 stat = MODBUS_OK;
#endif

#if defined(MODBUS_LOOPBACK)
    ModBus_Print_Frame("TX", req, req_len);

    memset(s_reply, 0, RS485_BUFFER_SIZE);
    s_reply_len = ModBus_Slave_Simulate(req, req_len, s_reply);
    if (s_reply_len == 0)
    {
        LOG_Str("[MB] sim slave no reply\r\n");
        return MODBUS_ERR_TIMEOUT;
    }

    ModBus_Print_Frame("RX", s_reply, s_reply_len);
    stat = ModBus_Check_Reply(slave_addr, expect_fc, s_reply, s_reply_len);
    return stat;
#else

    /*
        ★ 0. 帧间静默 T3.5(v21 加, 修协议违规)

        为什么必须有: 上一笔事务的应答(可能是别的从机的)刚在总线上跑完,
        如果立刻发新请求, 间隔小于 T3.5(3.65ms), 那么【其它】从机的
        TIM2 判帧定时器不会溢出, 它们会把"上一帧的尾巴 + 这一帧"当成一帧,
        CRC 必然失败, 于是永远不应答。

        实测症状: 1号从机正常, 2号从机满屏 CRC FAIL 且永不响应。
        详见 modbus.h 里 MODBUS_FRAME_GAP_MS 的说明。
    */
    delay_ms(MODBUS_FRAME_GAP_MS);

    // 1. 发送前清空接收缓冲，保证只判断本次应答
    RS485_Recv_Clear();

    // 2. 发送问询帧（内部自己切 PD7: 发送 -> 等 TC -> 接收）
    RS485_SendBuffer(req, req_len);

        /*
        ★ TX 原始帧只在【运行时打开了逐帧日志】才打。
          默认关闭的原因: 每笔事务都打 TX/RX 会让日志吃掉约 89% 的轮询周期
          (实测: 一轮 3 台 x 3 笔 = 855 字符 = 891ms @9600, 而周期只有 1000ms),
          主循环被严重拖慢, MQTT 心跳和云端下行都得不到及时处理。
          需要看时用串口1 敲 @log on 打开(见 cli.c), 调完敲 @log off 关掉。
    */
    if (CLI_Get_ModBus_Log())
    {
        LOG_HexDump("TX", req, req_len);
    }

    // 3. 等待从机应答，IDLE 中断置位即代表收到一帧
    while (wait < MODBUS_REPLY_TIMEOUT)
    {
        if (usart2_value.flag)
        {
            break;
        }
        delay_ms(1);
        wait += 1;
    }

    // 4. 超时处理，从机不应答的情况直接返回
    if (!usart2_value.flag)
    {
        /*
            超时是最常见的故障, 必须把"问的是谁、等了多久、等了多少次"打全。
            如果只打一句 TIMEOUT, 现场没法判断是地址错、接线错还是波特率错。
        */
        /*
            ★ 超时日志限流(v21 加)。
              从机长期离线时, 每秒会打 2 条 TIMEOUT(addr=2 和 addr=3),
              一小时就是 7200 行 —— 串口助手的接收区被刷满,
              真正有用的信息(比如云端下发、ESP 报错)反而被冲走。

              策略: 每台从机【连续超时】只在第 1 次和之后每 30 次打一条,
              一旦恢复成功(err_cnt 清零)就重置计数, 下次断线还能立刻看到。
            */
        {
            u8 idx = 0xFF;
            u8 i = 0;

            /* 找到这个地址对应的下标 */
            for (i = 0; i < SLAVE_NUM; i++)
            {
                if (g_slave[i].addr == slave_addr) { idx = i; break; }
            }

            if (idx != 0xFF && ModBus_Timeout_Log_Should(idx))
            {
                LOG_Str("[MB] TIMEOUT addr=");
                LOG_Num(slave_addr);
                LOG_Str(" wait=");
                LOG_Num(wait);
                LOG_Str("ms/");
                LOG_Num(MODBUS_REPLY_TIMEOUT);
                LOG_Str("ms fc=0x");
                LOG_HexNum(expect_fc);
                LOG_Str(" err=");
                LOG_Num(g_slave[idx].err_cnt);
                LOG_Str("\r\n");
            }
        }
        return MODBUS_ERR_TIMEOUT;
    }

    // 5. 把应答帧拷出来，防止被后续函数覆盖
    s_reply_len = usart2_value.count;
    if (s_reply_len > RS485_BUFFER_SIZE)
    {
        s_reply_len = RS485_BUFFER_SIZE;
    }
    memset(s_reply, 0, RS485_BUFFER_SIZE);
    memcpy(s_reply, usart2_value.data, s_reply_len);

    // 6. 释放接收缓冲，让下一帧可以进来
    RS485_Recv_Clear();

    // 7. 校验应答帧
    stat = ModBus_Check_Reply(slave_addr, expect_fc, s_reply, s_reply_len);
    return stat;
#endif
}

/**
 *  @brief 读取从机的若干个寄存器（功能码 0x03）
 */
u8 ModBus_Read_Registers(u8 slave_addr, u16 start_addr, u16 reg_num, u16 *regs)
{
    u8 req[8] = {0};
    u16 req_len = 0;
    u16 expect_len = 0;
    u16 idx = 0;
    u16 i = 0;
    u8 stat = MODBUS_OK;

    if (regs == NULL || reg_num == 0 || reg_num > 0x7D)
    {
        return MODBUS_ERR_LEN;
    }

    req_len = ModBus_Build_Read_Frame(req, slave_addr, start_addr, reg_num);
    stat = ModBus_Transaction(slave_addr, req, req_len, MODBUS_FC_READ_REG);
    if (stat != MODBUS_OK)
    {
        return stat;
    }

    expect_len = (u16)(5 + 2 * reg_num);
    if (s_reply_len != expect_len)
    {
        LOG_Str("[MB] LEN MISMATCH exp=");
        LOG_Num(expect_len);
        LOG_Str(" got=");
        LOG_Num(s_reply_len);
        LOG_Str("\r\n");
        return MODBUS_ERR_LEN;
    }
    if (s_reply[2] != (u8)(2 * reg_num))
    {
        LOG_Str("[MB] BYTE COUNT MISMATCH exp=");
        LOG_Num((u16)(2 * reg_num));
        LOG_Str(" got=");
        LOG_Num(s_reply[2]);
        LOG_Str("\r\n");
        return MODBUS_ERR_LEN;
    }

    idx = 3;
    for (i = 0; i < reg_num; i++)
    {
        regs[i] = (u16)(((u16)s_reply[idx] << 8) | (u16)s_reply[idx + 1]);
        idx += 2;
    }

    return MODBUS_OK;
}

/**
 *  @brief 写从机的单个寄存器（功能码 0x06）
 */
u8 ModBus_Write_Single_Reg(u8 slave_addr, u16 reg_addr, u16 value)
{
    u8 req[8] = {0};
    u16 req_len = 0;
    u8 stat = MODBUS_OK;

    req_len = ModBus_Build_Write_Frame(req, slave_addr, reg_addr, value);
    stat = ModBus_Transaction(slave_addr, req, req_len, MODBUS_FC_WRITE_SINGLE);
    if (stat != MODBUS_OK)
    {
        return stat;
    }

    if (s_reply_len != req_len)
    {
        LOG_Str("[MB] WRITE REPLY LEN MISMATCH exp=");
        LOG_Num(req_len);
        LOG_Str(" got=");
        LOG_Num(s_reply_len);
        LOG_Str("\r\n");
        return MODBUS_ERR_LEN;
    }
    if (memcmp(s_reply, req, req_len) != 0)
    {
        LOG_Str("[MB] WRITE REPLY NOT ECHO\r\n");
        return MODBUS_ERR_FUNC;
    }

    return MODBUS_OK;
}

/**
 *  @brief 初始化 ModBus 主机
 */
void ModBus_Init(void)
{
    u8 i = 0;
    /*
        ★ 地址表必须正好 SLAVE_NUM 个元素。
          原来写死 3 个初值, 把 SLAVE_NUM 改成 2 之后,
          数组只有 2 格却给了 3 个初值 —— Keil 会报
          "too many initializers" 或静默截断, 两种都不好。
          这里用 #if 保证元素个数和 SLAVE_NUM 严格一致。
    */
#if (SLAVE_NUM == 1)
    u8 addr_list[SLAVE_NUM] = {SLAVE1_ADDR};
#elif (SLAVE_NUM == 2)
    u8 addr_list[SLAVE_NUM] = {SLAVE1_ADDR, SLAVE2_ADDR};
#else
    u8 addr_list[SLAVE_NUM] = {SLAVE1_ADDR, SLAVE2_ADDR, SLAVE3_ADDR};
#endif

    for (i = 0; i < SLAVE_NUM; i++)
    {
        memset(&g_slave[i], 0, sizeof(Slave_Device));
        g_slave[i].addr = addr_list[i];
        g_slave[i].online = 0;
        g_slave[i].online_reported = 0;
        g_slave[i].last_alive_tick = 0;
        g_slave[i].err_cnt = 0;
        g_slave[i].data_err_cnt = 0;

#if defined(MODBUS_LOOPBACK)
        s_sim_sys[i][0] = 0x0102;   // 固件版本 v1.2
        s_sim_sys[i][2] = 0;        // 传感器健康位图：全部正常
        s_sim_sys[i][3] = 0;        // DHT11 连续失败次数：0
        s_sim_in[i][0] = 50;
        s_sim_in[i][1] = 0;
        s_sim_in[i][2] = 26;
        s_sim_in[i][3] = 5;
        s_sim_in[i][5] = 0;         // 0x0105 保留, 固定 0
        s_sim_out[i][0] = 0;
        s_sim_out[i][1] = 0;
        s_sim_out[i][2] = 1;
#endif
    }

    g_modbus_exception = 0;
}

/**
 *  @brief 轮询一台从机的全部数据
 *
 *  一轮做三笔事务：系统区 / 输入区 / 输出区。
 *  ★ 在线判定的口径（对应需求"连续两次采集失败判离线"）:
 *      三笔事务里【只要有一笔成功】-> 这台从机在应答 -> err_cnt 清零, online = 1
 *      三笔【全部失败】          -> err_cnt += 1
 *      err_cnt 达到 MODBUS_OFFLINE_FAIL_LIMIT(2) -> online = 0
 *    为什么"一笔成功就算在线"而不是"三笔全成功":
 *      某笔偶发 CRC 错(总线干扰)不代表从机死了, 用"三笔全成功"会把
 *      一次干扰放大成一次"采集失败", 两次干扰就误报离线。
 */
u8 ModBus_Poll_Slave(u8 index)
{
    u16 regs[IN_REG_COUNT] = {0};
    u8 stat = MODBUS_OK;
    u8 any_ok = 0;          /* 三笔里有没有任意一笔成功(判在线用) */
    u8 data_ok = 0;         /* 输入区有没有拿到(判"采集成功"用) */
    u8 dead = 0;            /* 1 = 第一笔就超时, 判定从机不在, 跳过剩余两笔 */
    Slave_Device *dev = NULL;

    if (index >= SLAVE_NUM)
    {
        return 0;
    }

    dev = &g_slave[index];

    /*
        ★ 已离线从机降频探测(v21 加, 解决主循环饱和)

        问题: 2 台从机离线时, 每轮要付 2 x 250ms = 500ms 超时代价,
              加上日志和上报, 一轮就逼近 1000ms 的轮询周期,
              日志里出现 "[SYS] loop blocked, missed=1 ticks"。

        做法: 一台从机被判离线(online==0 且 err_cnt 已达门限)之后,
              不再每轮都花 250ms 去问它, 而是每 MODBUS_OFFLINE_RETRY_TICKS
              个节拍才探一次。
              - 在线从机不受影响(仍然每秒问)
              - 离线从机 3 秒探一次, 恢复后最多 3 秒被发现, 完全可接受
              - 一轮的代价从 500ms 降到 0(大多数轮) 或 500ms(探测轮)

        注意: 这个降频只影响【已经判离线】的设备。第一次失败到判离线
              的那两轮仍然是每秒探测, 所以"连续 2 次失败判离线"的
              响应速度没有变慢。
    */
    if (dev->online == 0 && dev->err_cnt >= MODBUS_OFFLINE_FAIL_LIMIT)
    {
        static u32 s_last_probe[SLAVE_NUM] = {0};
        u32 now_tick = Tick_Count();
        u32 last = s_last_probe[index];

        /* last == 0 表示还没探过, 立刻探一次 */
        if (last != 0 && (u32)(now_tick - last) < MODBUS_OFFLINE_RETRY_TICKS)
        {
            return 0;       /* 本轮跳过, 不消耗超时 */
        }
        s_last_probe[index] = now_tick;
    }

    // 1. 读系统区【固件版本 + 上电运行秒数 + 传感器健康 + DHT11 失败次数】
    stat = ModBus_Read_Registers(dev->addr, SYS_REG_FW_VERSION, SYS_REG_COUNT, regs);
    if (stat == MODBUS_OK)
    {
        dev->fw_version    = regs[0];
        dev->run_seconds   = regs[1];
        dev->sensor_health = regs[2];
        dev->dht_fail_cnt  = regs[3];
        any_ok = 1;
    }
    else
    {
#if MODBUS_LOG
        LOG_Str("[MB] read sys fail ");
        LOG_Num(stat);
        LOG_Str("\r\n");
#endif

        /*
            ★ 快速失败:
              第一笔就【超时】说明从机根本没应答 —— 断电 / 线断 / 地址错。
              这时后面两笔必然也是各等满 250ms, 白白耗掉 500ms。
              一台离线从机 3 笔 = 750ms; 三台全离线 = 2.25 秒,
              已经超过 1 秒的轮询周期, 主循环被彻底拖住,
              连 MQTT 心跳和下行都处理不了。
              快速失败之后: 一台离线只花 250ms, 三台全离线 750ms, 仍在周期内。

              为什么只在 TIMEOUT 时快速失败, 而 CRC 错 / 异常帧不快速失败:
                CRC 错和异常帧都证明【从机是活的】, 只是这一笔有问题,
                后面两笔很可能正常 —— 这时继续问才能把数据补齐。
        */
        if (stat == MODBUS_ERR_TIMEOUT)
        {
            dead = 1;
        }
    }

    // 2. 读输入区【湿度整/小 + 温度整/小 + 光照 + 保留】
    if (!dead)
    {
        stat = ModBus_Read_Registers(dev->addr, IN_REG_HUMI_INT, IN_REG_COUNT, regs);
        if (stat == MODBUS_OK)
        {
            dev->humi_int = regs[0];
            dev->humi_dec = regs[1];
            dev->temp_int = regs[2];
            dev->temp_dec = regs[3];
            dev->light    = regs[4];
            dev->reserved = regs[5];
            any_ok = 1;
            data_ok = 1;
        }
        else
        {
#if MODBUS_LOG
            LOG_Str("[MB] read in fail ");
            LOG_Num(stat);
            LOG_Str("\r\n");
#endif
        }
    }

    // 3. 回读输出区【灯 / 蜂鸣器 / 舵机】
    if (!dead)
    {
        stat = ModBus_Read_Registers(dev->addr, OUT_REG_LED, OUT_REG_COUNT, regs);
        if (stat == MODBUS_OK)
        {
            dev->led   = regs[0];
            dev->beep  = regs[1];
            dev->servo = regs[2];
            any_ok = 1;
        }
        else
        {
#if MODBUS_LOG
            LOG_Str("[MB] read out fail ");
            LOG_Num(stat);
            LOG_Str("\r\n");
#endif
        }
    }

    // 4. 更新心跳与连续失败计数
    if (any_ok)
    {
        dev->last_alive_tick = Tick_Count();
        dev->err_cnt = 0;
        dev->online = 1;
        /*
            ★ 注意: 这里【不】清零超时日志的时间戳。
              按时间限流不需要清零 —— 清零反而会让"部分成功"的
              设备每轮都能打一条(就是之前的 bug)。
              时间到了自然会再打。
        */
    }
    else
    {
        if (dev->err_cnt < 0xFFFF)
        {
            dev->err_cnt += 1;
        }
    }

    /*
        5. 数据采集层的连续失败计数（与上面的通信层分开）
        ★ 口径: 这一轮【输入区没读到】就算一次采集失败。
    */
    if (data_ok)
    {
        dev->data_err_cnt = 0;
    }
    else
    {
        if (dev->data_err_cnt < 0xFFFF)
        {
            dev->data_err_cnt += 1;
        }
    }

    return data_ok;
}

/**
 *  @brief 轮询总线上全部从机
 *
 *  为什么每轮全扫而不是"一次只问一台":
 *    每台 3 笔事务正常约 30ms，3 台约 90~120ms，1 秒的轮询周期完全放得下。
 *    全扫让"连续两次采集失败"的时间尺度是 2 秒而不是 6 秒，
 *    离线判定更快，云端看到的温湿度也更连续。
 *
 *  代价: 1 秒内总线占用约 120ms(12%)；三台全离线时 750ms(75%)，
 *        仍在周期内(这就是快速失败存在的意义)。
 */
void ModBus_Poll_All(void)
{
    u8 i = 0;

    for (i = 0; i < SLAVE_NUM; i++)
    {
        (void)ModBus_Poll_Slave(i);
    }
}

/**
 *  @brief 刷新所有从机的在线状态
 *
 *  ★ 用 TIM4 节拍作为时间基准(Tick_Count), 并检查"是否错过了节拍"。
 *
 *  为什么要检查 missed:
 *    主机的 Cloud_Link_Build() 在建链失败时会阻塞几十秒。
 *    这段时间里 TIM4 还在正常走, 但主循环没机会轮询 ——
 *    从机的 err_cnt 不会增加(压根没发起事务), 所以不会误判离线。
 *    但反过来: 如果主循环在【两次轮询之间】被阻塞了 30 秒,
 *    这一轮轮询回来时"上次成功应答是 30 秒前", 用超时判就会误报离线。
 *    这里用 err_cnt(次数)判, 本身就不受时间影响 —— missed 只用于
 *    日志提示, 让人知道"这段时间的数据不可信"。
 *
 *  ★ 这里只做"把 err_cnt 翻译成 online"这一件事,
 *    不负责"什么时候算一次失败" —— 那是 ModBus_Poll_Slave 的事。
 */
void ModBus_Update_Online(void)
{
    u8 i = 0;

    for (i = 0; i < SLAVE_NUM; i++)
    {
        if (g_slave[i].err_cnt >= MODBUS_OFFLINE_FAIL_LIMIT)
        {
            g_slave[i].online = 0;
        }
    }
}

/**
 *  @brief 判断某台从机的"数据采集"是否连续失败已达门限
 */
u8 ModBus_Data_Failed(u8 index)
{
    if (index >= SLAVE_NUM)
    {
        return 0;
    }
    return (u8)(g_slave[index].data_err_cnt >= MODBUS_DATA_FAIL_LIMIT);
}

/**
 *  @brief 协议层自检
 */
u8 ModBus_Self_Test(void)
{
    /*
        ★ 数组大小必须按【最大的那个区】开, 不能按 OUT_REG_COUNT(3)。
          这个函数里既读系统区 4 个(regs[0..3]) 又读输出区 3 个,
          开成 regs[3] 会让 regs[3] 越界写 2 字节 ——
          踩到相邻的 ok 变量上, 表现是"自检结果随机 PASS/FAIL"。
          (这个 bug 是 UV4 编译警告 #175-D 抓出来的, gcc 没报)
    */
    u16 regs[SYS_REG_COUNT] = {0};
    u8 ok = 1;
    u8 i = 0;

    LOG_Str("\r\n==== MODBUS SELF TEST ====\r\n");
#if defined(MODBUS_LOOPBACK)
    LOG_Str("[MB] loopback ON, slave is simulated\r\n");
#else
    LOG_Str("[MB] loopback OFF, real bus\r\n");
#endif

    if (ModBus_Read_Registers(SLAVE1_ADDR, SYS_REG_FW_VERSION, SYS_REG_COUNT, regs) == MODBUS_OK)
    {
        LOG_Str("[MB] sys read ok, fw=");
        LOG_Num(regs[0]);
        LOG_Str(" run=");
        LOG_Num(regs[1]);
        LOG_Str(" health=");
        LOG_Num(regs[2]);
        LOG_Str(" dht_fail=");
        LOG_Num(regs[3]);
        LOG_Str("\r\n");
    }
    else
    {
        LOG_Str("[MB] sys read FAIL\r\n");
        ok = 0;
    }

    if (ModBus_Write_Single_Reg(SLAVE1_ADDR, OUT_REG_LED, 1) != MODBUS_OK)
    {
        LOG_Str("[MB] led write FAIL\r\n");
        ok = 0;
    }

    if (ModBus_Write_Single_Reg(SLAVE1_ADDR, OUT_REG_BEEP, 1) != MODBUS_OK)
    {
        LOG_Str("[MB] beep write FAIL\r\n");
        ok = 0;
    }

    if (ModBus_Write_Single_Reg(SLAVE1_ADDR, OUT_REG_SERVO, 2) != MODBUS_OK)
    {
        LOG_Str("[MB] servo write FAIL\r\n");
        ok = 0;
    }

    if (ModBus_Read_Registers(SLAVE1_ADDR, OUT_REG_LED, OUT_REG_COUNT, regs) == MODBUS_OK)
    {
        LOG_Str("[MB] out read back: led=");
        LOG_Num(regs[0]);
        LOG_Str(" beep=");
        LOG_Num(regs[1]);
        LOG_Str(" servo=");
        LOG_Num(regs[2]);
        LOG_Str("\r\n");

        if (regs[0] != 1 || regs[1] != 1 || regs[2] != 2)
        {
            LOG_Str("[MB] read back MISMATCH\r\n");
            ok = 0;
        }
    }
    else
    {
        LOG_Str("[MB] out read FAIL\r\n");
        ok = 0;
    }

    for (i = 0; i < SLAVE_NUM; i++)
    {
        if (ModBus_Poll_Slave(i))
        {
            LOG_Str("[MB] slave ");
            LOG_Num(i + 1);
            LOG_Str(" poll ok\r\n");
        }
        else
        {
            LOG_Str("[MB] slave ");
            LOG_Num(i + 1);
            LOG_Str(" poll FAIL\r\n");
            ok = 0;
        }
    }

    LOG_Str(ok ? "==== SELF TEST PASS ====\r\n\r\n" : "==== SELF TEST FAILED ====\r\n\r\n");
    return ok;
}
