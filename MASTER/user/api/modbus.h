#ifndef _MODBUS_H
#define _MODBUS_H

#include "common.h"
#include "rs485.h"
#include "delay.h"
#include "tick.h"
#include "log.h"

/*
当前固件编译为【主机】，负责轮询总线上的从机、转发 MQTT 上下行数据。
从机固件里请把这一行注释掉，两边不要同时打开。
*/
#define MODBUS_MASTER

/*
模拟从机回环开关：没接从机时取消下面这行的注释，主机发出的问询帧由本地
模拟从机应答，这样【JSON 解析 -> 组帧 -> 解包 -> 回读 -> 云端上行】整条
链路都能真跑一遍。

!! 接上真实从机时必须保持注释状态，否则永远读不到真实传感器数据，
   而且三台从机会被误判为在线。
*/
// #define MODBUS_LOOPBACK

// ============== 总线上的从机清单 ==============
/*
	SLAVE_NUM = 总线上实际挂了几台
	SLAVE1/2/3_ADDR = 每台的地址, 必须和从机固件里 MODBUS_SLAVE_ADDR 一一对应

	★ 从机侧是"同一份源码只改一个宏", 主机侧是"这里列出来"。
	  两边必须对齐: 从机 2 号板如果错烧成 0x01, 主机会看到"1 号有应答、
	  2 号一直超时", 而且总线上会有两台同时应答导致 CRC 乱 —— 排查时先查这个。
*/
/*
	★ SLAVE_NUM = 总线上【实际接了】几台, 不是"最多支持几台"。

	当前配置: 2 台(1 号 + 2 号)。第三块板暂时没接, 所以设 2。
	这样主机不会去轮询 addr=3, 日志干净、轮询周期也更宽裕。

	以后接上第三块板:
	  1) 把这里改成 3
	  2) 把 SLAVE3_ADDR 那行前面的注释去掉(如果注释了)
	  3) 重新编译烧录

	★ 为什么必须和实际数量一致(而不是干脆设大一点):
	  每多一台"配了但不存在"的从机, 主机就要为它付探测代价 ——
	  第一轮 250ms 超时 x2 判离线, 之后每 3 秒还要再花 250ms 探一次。
	  设成 3 而只接 2 台, 就是白白浪费这些时间,
	  还可能把轮询周期顶爆(日志里的 loop blocked / missed)。
*/
#define SLAVE_NUM (2)
#define SLAVE1_ADDR (0x01)
#define SLAVE2_ADDR (0x02)
#define SLAVE3_ADDR (0x03)      /* 备用: SLAVE_NUM 改回 3 时生效 */

/* 总线上允许的最大从机号, 用于校验配置没写错 */
#define MODBUS_SLAVE_MAX (3)

#if (SLAVE_NUM < 1) || (SLAVE_NUM > MODBUS_SLAVE_MAX)
#error "SLAVE_NUM must be in 1..MODBUS_SLAVE_MAX"
#endif

// ModBus 功能码，本工程只用到这三个
#define MODBUS_FC_READ_REG (0x03)
#define MODBUS_FC_READ_INPUT (0x04)
#define MODBUS_FC_WRITE_SINGLE (0x06)

// ============== 寄存器地址表（必须和从机 ModBus.h 完全一致） ==============
// 系统区
#define SYS_REG_FW_VERSION (0x0000)
#define SYS_REG_RUN_SECONDS (0x0001)
#define SYS_REG_SENSOR_HEALTH (0x0002)
#define SYS_REG_DHT_FAIL_CNT (0x0003)
// 输入区（传感器，只读）
#define IN_REG_HUMI_INT (0x0100)
#define IN_REG_HUMI_DEC (0x0101)
#define IN_REG_TEMP_INT (0x0102)
#define IN_REG_TEMP_DEC (0x0103)
#define IN_REG_LIGHT (0x0104)
#define IN_REG_RESERVED (0x0105)     /* 保留: SR602 已移到主机侧, 从机固定返回 0 */
// 输出区（执行器，可读可写）
#define OUT_REG_LED (0x0200)
#define OUT_REG_BEEP (0x0201)
#define OUT_REG_SERVO (0x0202)

// 各区连续读取的寄存器个数
#define SYS_REG_COUNT (4)
#define IN_REG_COUNT (6)
#define OUT_REG_COUNT (3)

/* 一次读取的最大寄存器个数(输入区 6 个最多), 用于算最长应答帧长度 */
#define REG_MAX_MAX (IN_REG_COUNT)

/* ==========================================================================
   从机应答超时 —— 精确计算, 不是拍脑袋填的
   ==========================================================================
   ★ 需求: "这个时间应该大于
            主机问询帧发送完毕 >> 从机接收完毕 >> 从机做出相应操作之后组包
            >> 从机发出应答帧 >> 主机收到应答帧 这么一套下来的时间"

   逐段算(9600 8N1, 1 个字符 = 10 bit = 1.042ms):

     ① 主机发问询帧 8 字节          8 x 1.042 =  8.3 ms
        (从机在同一段时间里逐字节收完, 所以这一段已经把"从机接收完毕"算进去了)

     ② 从机处理: CRC 校验 + 组应答帧   ~0.05 ms (纯计算, 可忽略)
        ★ 但真正的大头是【从机可能正在采集】:
          从机的采集节拍是 2 秒一拍, 一拍里 DHT11 独占 35ms
          (起始信号 delay_ms(35) + 40 位判定)。
          如果主机的问询帧正好落在这 35ms 里, 从机要等采集做完
          才会回到主循环看到 flag。所以这一段必须按【最坏情况】算:
                                       35.0 ms

     ③ 从机发应答帧, 最长 17 字节    17 x 1.042 = 17.7 ms
        (最长的是读输入区 6 个寄存器: 1+1+1+12+2 = 17 字节)

     ④ 主机判帧结束: 总线安静够 T3.5   3.65 ms
        (主机用 IDLE 判帧, 必须等这一小段静默才算"收完")

     合计最坏情况: 8.3 + 35.0 + 17.7 + 3.65 = 64.7 ms

   再乘一个安全系数:
     x2  -> 129 ms   覆盖"从机刚好在采集 + 总线上有轻微干扰导致重发"这类情况
     x3  -> 194 ms   留更多余量

   取值 250ms:
     既远大于最坏情况 64.7ms(3.9 倍余量), 又不会让"三台全离线"拖太久
     (一台 250ms, 三台 750ms, 仍在 1 秒轮询周期内)。

   ★ 为什么不能更大:
     离线从机的超时会直接叠加进轮询周期。250ms x 3 台 = 750ms,
     已经接近 1 秒节拍; 再大就会让主循环被超时吃满,
     MQTT 心跳和云端下发都得不到执行机会。

   ★ 为什么不能更小:
     小于 65ms 就会把"从机正在采集 DHT11"误判成"从机离线"。
     这个误判在真机上是间歇性的(2 秒一次采集, 每次都 35ms 窗口),
     表现是"从机时不时掉线一下又回来", 极难查。
   ========================================================================== */
#define MODBUS_BYTE_TIME_US     (1000000UL / (RS485_BRR / 10))     /* 1042us */
#define MODBUS_REQ_BYTES        (8)                                 /* 03/06 请求帧固定 8 字节 */
#define MODBUS_RESP_BYTES_MAX   (5 + 2 * REG_MAX_MAX)               /* 最长应答 17 字节 */
#define MODBUS_T35_MS           (4)                                 /* 判帧静默 */
#define MODBUS_SLAVE_BUSY_MS    (40)                                /* 从机最坏采集阻塞(DHT11 35ms + 余量) */
#define MODBUS_REPLY_TIMEOUT    (250)

/* ==========================================================================
   帧间静默间隔 —— ModBus RTU 的强制要求
   ==========================================================================
   ★ v21 修的一个严重协议违规。

   ModBus RTU 规范: 帧与帧之间必须有 >= 3.5 个字符时间的静默(T3.5),
   接收方靠这段静默判断"上一帧结束了"。

   @9600 8N1: 1 字符 = 10 bit = 1.042ms
              T3.5 = 3.5 x 1.042 = 3.65ms

   踩过的坑(实测日志):
     主机收到 1 号从机的应答后【立刻】发下一个请求, 间隔 < 1ms。
     结果 2 号从机看到的是:
        [1号的应答 13字节] --0.5ms--> [主机的新请求 8字节]
     两帧粘成一帧 21 字节, 它的 TIM2(3.65ms) 没来得及溢出,
     于是判成一帧 -> CRC 必败 -> 丢弃 -> 永远不响应 addr=2
     表现就是: 1号正常, 2号一直 TIMEOUT。

   为什么 1 号自己没事:
     1 号发应答时关掉了接收中断, 发完清缓冲, 所以它收不到自己的回声,
     也不会把"自己的应答 + 主机的请求"粘起来。
     而 2 号一直在听, 就遭了殃。

   所以必须在【每一笔事务之前】留出 T3.5 静默。
   取 5ms 而不是 3.65ms: 留 37% 余量, 抵消晶振误差和中断延迟。
   ========================================================================== */
#define MODBUS_FRAME_GAP_MS     (5)

/*
	同一台从机的超时日志最小间隔(秒)
	从机长期半死(部分事务成功部分超时)时, 不限流会每轮刷一条。
	按时间限流比按次数可靠 —— 详见 modbus.c 里 ModBus_Timeout_Log_Should 的说明。
*/
#define MODBUS_TIMEOUT_LOG_GAP_S (30)

/* 上面几个宏只是为了让 250 这个数字"有出处", 下面做个编译期一致性提醒 */
#if (MODBUS_REPLY_TIMEOUT < 100)
#error "MODBUS_REPLY_TIMEOUT too small: must cover 8.3+35+17.7+3.65 = ~65ms"
#endif

/*
	★ 离线判据（本项目的核心需求之一）
	"某个从机如果连续两次数据采集都没有成功则反馈云端该从机离线"

	落在代码里就是: 一台从机连续 MODBUS_OFFLINE_FAIL_LIMIT 次轮询
	都没能完整取回数据 -> online = 0 -> 立即上报云端 Online=false。

	★ 为什么用"连续次数"而不是"多少秒没应答":
	  次数是确定的, 和时间无关。主循环某次被 MQTT 建链阻塞了 30 秒,
	  用超时判会误判离线; 用次数判只会让"两次失败"来得晚一点, 但不会误判。
	  现场排查时也好解释: "主机问了它两次, 两次都没答上来, 就是离线"。
*/
#define MODBUS_OFFLINE_FAIL_LIMIT (2)

/*
	★ 数据采集失败门限
	一轮轮询里【输入区没读到】记一次采集失败,
	连续 MODBUS_DATA_FAIL_LIMIT 次 -> 判这台从机的数据不可信。

	★ 这一条在云端怎么表达, 是【两个键】而不是一个:
	    Online%d = false   -> 从机整个不在了(通信层, 由 err_cnt 判)
	    SenErr%d = true    -> 从机还在, 但连续两次没采到数据(采集层)
	  为什么要分开:
	    运维看到 Online=false 会去查供电和 485 接线;
	    看到 Online=true + SenErr=true 才知道该去换 DHT11。
*/
#define MODBUS_DATA_FAIL_LIMIT (2)

/*
	已离线从机的探测间隔(单位: 节拍, 1 节拍 = 1 秒)
	-----------------------------------------------------------------
	一台从机被判离线后, 不必每轮都花 250ms 去问它 —— 那会让
	"2 台离线"的场景每轮付 500ms 超时, 把 1 秒的轮询周期顶爆
	(现象: 日志里 [SYS] loop blocked, missed=N ticks)。

	改成 3 秒探一次:
	  - 在线从机不受影响(仍然每秒问)
	  - 离线从机恢复后最多 3 秒被发现, 完全够用
	  - 大多数轮次不再付超时代价

	★ 不影响"连续 2 次失败判离线"的响应速度:
	  从第一次失败到判离线的那两轮仍然是每秒探测,
	  只有【已经判离线】之后才降频。
*/
#define MODBUS_OFFLINE_RETRY_TICKS (3)

// 事务处理结果状态码
#define MODBUS_OK (0)
#define MODBUS_ERR_TIMEOUT (1)
#define MODBUS_ERR_CRC (2)
#define MODBUS_ERR_ADDR (3)
#define MODBUS_ERR_LEN (4)
#define MODBUS_ERR_FUNC (5)
#define MODBUS_ERR_EXCEPT (6)

// 单台从机的全部信息
typedef struct slave_device
{
    // 从机地址
    u8 addr;
    // 在线状态，1 在线 / 0 离线
    u8 online;
    // 上一次上报给云端的在线状态，用于判断状态是否发生变化
    u8 online_reported;
    // 最近一次成功应答的节拍数（Tick_Count 的值，单位秒）
    u32 last_alive_tick;
    /*
        连续无应答次数（达到 MODBUS_OFFLINE_FAIL_LIMIT 判离线）
        口径: 一整轮轮询里【三笔事务全部失败】才算一次"通信失败"
    */
    u16 err_cnt;
    /*
        连续"数据采集失败"次数
        口径: 一整轮轮询里【输入区没读到】才算一次"采集失败"
        为什么要和 err_cnt 分开: 见 MODBUS_DATA_FAIL_LIMIT 的说明
    */
    u16 data_err_cnt;

    // 系统区数据
    u16 fw_version;
    u16 run_seconds;
    u16 sensor_health;
    u16 dht_fail_cnt;

    // 输入区数据
    u16 humi_int;
    u16 humi_dec;
    u16 temp_int;
    u16 temp_dec;
    u16 light;
    u16 reserved;       /* 0x0105 保留(SR602 在主机侧) */

    // 输出区数据（执行一次【写 + 回读】后才是真实生效值）
    u16 led;
    u16 beep;
    u16 servo;
} Slave_Device;

// 总线上所有从机的管理数组
extern Slave_Device g_slave[SLAVE_NUM];
// 最近一次事务的异常码，正常时为 0
extern u8 g_modbus_exception;

/**
 *  @brief 初始化 ModBus 主机，填充从机地址并清空缓存数据
 */
void ModBus_Init(void);

/**
 *  @brief CRC-16/MODBUS 校验计算
 */
u16 ModBus_CRC16(u8 *buf, u16 len);

/**
 *  @brief 读取从机的若干个保持寄存器（功能码 0x03）
 */
u8 ModBus_Read_Registers(u8 slave_addr, u16 start_addr, u16 reg_num, u16 *regs);

/**
 *  @brief 写从机的单个寄存器（功能码 0x06）
 */
u8 ModBus_Write_Single_Reg(u8 slave_addr, u16 reg_addr, u16 value);

/**
 *  @brief 轮询一台从机的系统区、输入区、输出区全部数据
 *
 *  @return 本次轮询是否完整成功，1 是 / 0 否
 */
u8 ModBus_Poll_Slave(u8 index);

/**
 *  @brief 轮询总线上全部从机（一台接一台，每轮都全扫一遍）
 */
void ModBus_Poll_All(void);

/**
 *  @brief 根据"连续失败次数"刷新所有从机的在线状态
 *
 *  ★ 用 TIM4 节拍(Tick_Count)作为时间基准, 并会检查"是否错过了节拍"。
 *    如果主循环刚才被长时间阻塞(比如 MQTT 建链), 这一轮跳过离线判定 ——
 *    因为"我没去看"和"从机不在"是两件完全不同的事。
 */
void ModBus_Update_Online(void);

/**
 *  @brief 判断某台从机的"数据采集"是否连续失败已达门限
 *
 *  用于云端上报 SenErr%d —— "从机在线但连续两次没采到数据"
 */
u8 ModBus_Data_Failed(u8 index);

/**
 *  @brief 打印一帧数据的十六进制内容(走日志缓冲, 不直接发串口)
 */
void ModBus_Print_Frame(const char *tag, u8 *frame, u16 len);

/**
 *  @brief 跑一遍协议层自检
 */
u8 ModBus_Self_Test(void);

#endif
