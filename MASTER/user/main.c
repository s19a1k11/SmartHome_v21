#include "common.h"
#include "delay.h"
#include "usart1.h"
#include "usart3.h"
#include "esp8266.h"
#include "mqtt.h"
#include "rs485.h"
#include "modbus.h"
#include "cloud.h"
#include "tick.h"
#include "log.h"
#include "sr602.h"
#include "autolight.h"
#include "cli.h"
#include "lcd_init.h"
#include "lcd_ui.h"

#define USART1_BAUDRATE (9600)
#define USART3_BAUDRATE (115200)
#define RS485_BAUDRATE (9600)

/*
	MQTT 心跳间隔，必须小于 Keep Alive(60s)，否则会被云平台判定离线
	★ 60 秒的 Keep Alive 配 30 秒心跳, 留一半余量
*/
#define MQTT_PING_INTERVAL (30000)
/* 心跳应答等待超时 */
#define MQTT_PING_TIMEOUT (3000)
/* 连续多少次心跳无应答就重建整条链路 */
#define MQTT_LINK_ERR_MAX (2)

/* ==========================================================================
	★ 主机中断优先级表 (NVIC_PriorityGroup_2: 2 位抢占 0~3 + 2 位子优先级 0~3)

	按需求: 收 > 发, SR602 次之, MQTT 指令接收 > Modbus 接收,
	        Modbus 指令发送 > 其他发送。

	┌────────────────────┬──────┬──────┬──────────────────────────────────┐
	│ 中断源              │ 抢占 │ 子   │ 为什么排这个位置                  │
	├────────────────────┼──────┼──────┼──────────────────────────────────┤
	│ USART3 MQTT 接收    │  0   │  0   │ 115200 下一个字节 86us, 硬件只有  │
	│                    │      │      │ 1 字节寄存器, 晚一步就丢报文      │
	│                    │      │      │ -> MQTT 下发命令丢失              │
	├────────────────────┼──────┼──────┼──────────────────────────────────┤
	│ USART2 Modbus 接收  │  1   │  1   │ 9600 下一个字节 1.04ms, 晚一步就  │
	│                    │      │      │ 丢字节 -> 整帧 CRC 失败           │
	├────────────────────┼──────┼──────┼──────────────────────────────────┤
	│ EXTI2 SR602         │  2   │  0   │ 人体边沿晚几十 us 无所谓, 但漏掉  │
	│                    │      │      │ 一次"有人来了"就漏一次开灯        │
	├────────────────────┼──────┼──────┼──────────────────────────────────┤
	│ TIM4 轮询节拍       │  3   │  0   │ 只是"打拍子", 晚几毫秒毫无影响    │
	├────────────────────┼──────┼──────┼──────────────────────────────────┤
	│ USART1 调试回显     │  3   │  1   │ 最不重要, 而且发送本身是阻塞的;   │
	│                    │      │      │ 放最低 = 任何中断都能打断它       │
	└────────────────────┴──────┴──────┴──────────────────────────────────┘

	★ 关于"发 > 其他发":
	  USART2(485) 和 USART3(ESP8266) 的【发送】都是阻塞轮询 TC, 不占中断,
	  所以"发送优先级"实际体现在【谁能打断谁】:
	    - 485 发送期间, USART3 接收(抢占 0) 可以打断它 -> MQTT 指令不丢
	    - 日志发送(USART1, 抢占 3) 最低, 485 接收(抢占 1) 可以打断它
	  这正是"Modbus 指令发送 > 其他发送(日志)"的落实:
	  485 应答永远优先于调试打印。

	★ 关于"MQTT 指令接收 > Modbus 接收":
	  USART3 给抢占 0、USART2 给抢占 1。
	  理由是两者都是硬实时, 但 115200 的字节间隔(86us)比 9600(1042us)
	  紧 12 倍, 一旦丢字节恢复代价更大(整个 MQTT 报文作废)。
	  而 Modbus 帧丢了还有 250ms 超时 + 下一轮轮询可以重试,
	  MQTT 下发指令丢了就没有第二次机会(平台不会重发 QoS0 报文)。
   ========================================================================== */

int main(void)
{
    u32 last_upload_ms = 0;
    u32 last_ping_ms = 0;
    u32 ping_send_ms = 0;
    u8  ping_wait = 0;
    u8  link_err = 0;
    u8  first_round = 1;
    u8  human_now = 0;

    // 1. 配置优先级分组: 2 位抢占 + 2 位子优先级
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    // 2. 初始化延时时基，delay_ms / Delay_GetMs 全部依赖它
    Delay_Init();

    // 3. 初始化设备
    /*
    USART1 ==> 调试串口，只打印日志(通过日志缓冲, 最后统一发)
    USART3 ==> ESP8266 WiFi 模块，负责 MQTT 上云
    USART2 ==> RS485 总线，负责轮询三台从机
    */
    USART1_Init(USART1_BAUDRATE);
    LOG_Init();                         /* 日志缓冲必须在 USART1 之后初始化 */
    CLI_Init();                         /* 串口1 本地命令解析(模拟 MQTT 下发) */

    USART3_Init(USART3_BAUDRATE);

    /*
        ★ ESP8266 控制引脚初始化(v21 新增, 关键)
          战神V1 板子上:
            PA4  = ESP-01 的 CH_PD (使能脚, 必须为高才工作)
            PA15 = ESP-01 的 GPIO0 (必须为高才是 AT 模式, 低电平进下载模式)
          改造前代码完全没碰这两个脚, 它们是复位后的浮空输入 ->
          电平不确定 -> 模块可能一直关断或停在下载模式 ->
          发 AT 永远没回应, 就是 [ESP] module no response。

          顺序要求: 必须在 USART3_Init 之后(要清掉模块启动时的乱码),
                    在 Cloud_Link_Build 之前(建链第一步就是发 AT)。
    */
    ESP8266_Init();

    /*
        ★ LCD 初始化(v22 新增)
          1.8 寸 ST7735S, 128x160, SPI1 + PE0/PE1/PE6 控制线
          初始化耗时约 350ms(复位+SleepOut+背光), 只在开机做一次。
          放在 ESP8266_Init 之后: 两者都用不到对方的资源, 顺序无所谓,
          但 LCD 初始化比较慢, 放在 WiFi 建链之前可以先看到屏幕亮起来。
    */
    LCD_Init();
    LCD_UI_Init();

    /*
        ★ 顺序: 先 RS485_Init(内部先配 PD7 为接收态, 再使能 USART2),
          再 ModBus_Init。反过来会让 USART2 在 PD7 还是浮空输入时就开始收,
          上电瞬间总线上的杂波会被当成第一帧, 表现为"上电第一帧 CRC 失败"。
    */
    RS485_Init(RS485_BAUDRATE);
    ModBus_Init();

    /* SR602 人体红外(主机侧, EXTI2 双边沿) */
    SR602_Init();
    AutoLight_Init();

    /*
        ★ TIM4 轮询节拍: 1 秒一次中断, 中断里只置标志, 轮询在主循环做。
          为什么用定时器而不是 if(now - last >= 1000):
          主循环被 MQTT 建链阻塞几十秒时, 条件判断回来只看到"过去了 30 秒",
          无法知道自己错过了 30 次轮询; 定时器能通过 missed 计数告知,
          这样就能区分"我没去看"和"从机不在"(详见 tick.c)。
    */
    Tick_Init();

    LOG_Str("[SYS] host firmware start\r\n");
    LOG_Str("[SYS] USART1 CLI ready: type @help\r\n");
    /*
        ★ 版本串必须每次改动都更新!
          v19 之后改了三轮(MQTT容量/栈溢出/CLI/@esp), 但版本串一直没动,
          导致"烧没烧进去"根本判断不出来 —— 现场看到的还是 v19,
          和改动前完全一样。这个坑已经踩过一次。
          规则: 改代码 -> 同时改这一行 -> 否则无法确认板上跑的是哪一版。
    */
    LOG_Str("[VER] 2026-10-24-v21 MASTER slaves=");
    LOG_Num(SLAVE_NUM);
    LOG_Str("\r\n");

    /*
        ★ [BUILD] 行: 打印【编译时刻】, 由编译器自动填 __DATE__ / __TIME__。
          为什么需要它:
            光靠手写的版本号无法区分"改了但没重新编译"和"编译了但没烧进去"。
            这一行是编译时生成的, 只要代码没重编就永远不变 ——
            烧完看这一行就知道板上跑的是不是刚才编译的那个文件。
          用法: 编译前记下这个时间, 烧完对照; 不一致就是没烧进去。
    */
    LOG_Str("[BUILD] ");
    LOG_Str(__DATE__);
    LOG_Str(" ");
    LOG_Str(__TIME__);
    LOG_Str("\r\n");
    LOG_Str("[CFG] reply_timeout=");
    LOG_Num(MODBUS_REPLY_TIMEOUT);
    LOG_Str("ms upload=");
    LOG_Num(CLOUD_UPLOAD_INTERVAL / 1000);
    LOG_Str("s night_th=");
    LOG_Num(NIGHT_LIGHT_THRESHOLD);
    LOG_Str("\r\n");
    LOG_Flush();                        /* 开机信息: 此刻还没进入循环, 可以直接发 */

    // 4. 建立云平台连接
    Cloud_Link_Build();
    LOG_Flush();

    // 5. 主循环
    while (1)
    {
        /*
            ---------- 5.0 串口1 本地命令 ----------
            ★ 放在最前面: 这是人在串口助手敲的命令, 越早响应越好。
              内部走的是 Cloud_Handle_Downlink(和云端下发同一个函数),
              所以用它验证通了, 接上云平台就一定通。
        */
        (void)CLI_Poll();

        /*
            ---------- 5.1 处理云端下行 ----------
            ★ 用户点了开关, 越早执行越好。
              内部会拆 MQTT 包并回调 Cloud_Handle_Downlink。
        */
        Cloud_Rx_Process();

        /*
            ---------- 5.2 采样 SR602 ----------
            每轮读一次引脚并缓存, 之后的开灯判断和上报都用这个缓存值。
            放在轮询之前: 开灯判断要用到"这一轮开始时有没有人"。
        */
        SR602_Sample();
        human_now = SR602_Current();

        /*
            ---------- 5.3 轮询节拍(TIM4 打拍子) ----------
            Tick_Take() 返回 1 表示这一秒该轮询了。
            Tick_Take_Missed() 返回"主循环刚才错过了几拍", 用于日志提示。
        */
        if (first_round || Tick_Take())
        {
            u16 missed = 0;

            first_round = 0;
            missed = Tick_Take_Missed();
            if (missed > 0)
            {
                /*
                    ★ 主循环刚才被长时间阻塞过(典型是 MQTT 建链)。
                      只记日志, 不做别的 —— 因为离线判定用的是"连续失败次数"
                      而不是"多少秒没应答", 本身就不受时间影响。
                      这一行只是让现场知道"这段时间的数据是断的"。
                */
                LOG_Str("[SYS] loop blocked, missed=");
                LOG_Num(missed);
                LOG_Str(" ticks\r\n");
            }

            /*
                ★ 三件事的顺序不能乱:
                  1) 先轮询 -> 拿到最新数据, 同时累加/清零"连续失败次数"
                  2) 再把次数翻译成 online
                  3) 最后判断 online 有没有翻转, 翻了立刻上报
                如果先判 online 再轮询, 上报的永远是上一轮的状态,
                从机断电后要等两个周期云端才看到离线。
            */
            ModBus_Poll_All();
            ModBus_Update_Online();
            Cloud_Upload_Online_If_Changed();

            /*
                ---------- 5.4 夜晚回家自动开灯 ----------
                ★ 只在主机实现: SR602 在门口(主机), 要开的是门口的灯
                  = 1 号从机的 0x0200。
                判定: 有人 + 三台在线从机光照平均值 < 阈值 -> 开灯
                      无人 + 超过 10 秒            -> 关灯
                全部逻辑在 autolight.c, 这里只调一次。
            */
            AutoLight_Poll(human_now);
        }

        /*
            ---------- 5.5 定时上传 ----------
            间隔 15 秒(见 cloud.h 里的说明: 10 秒是下限, 贴下限容易被流控)。
            所有键合并成一条消息发，压住对平台的访问频率。
        */
        {
            u32 now = Delay_GetMs();
            if (now - last_upload_ms >= CLOUD_UPLOAD_INTERVAL)
            {
                last_upload_ms = now;
                Cloud_Upload_All();
            }

            /*
                ---------- 5.6 MQTT 心跳保活 ----------
                用状态机实现，不阻塞轮询。
            */
            if (ping_wait == 0)
            {
                if (now - last_ping_ms >= MQTT_PING_INTERVAL)
                {
                    last_ping_ms = now;
                    g_mqtt_ping_resp = 0;
                    MQTT_PingReq_Package_Send();
                    ping_send_ms = now;
                    ping_wait = 1;
                }
            }
            else
            {
                if (g_mqtt_ping_resp)
                {
                    ping_wait = 0;
                    link_err = 0;
                }
                else if (now - ping_send_ms >= MQTT_PING_TIMEOUT)
                {
                    ping_wait = 0;
                    link_err += 1;

                    if (link_err >= MQTT_LINK_ERR_MAX)
                    {
                        link_err = 0;
                        LOG_Str("[SYS] rebuild cloud link\r\n");
                        LOG_Flush();        /* 建链前先把攒的日志发掉 */

                        // 内部会先退出透传再重连
                        Cloud_Link_Build();

                        /*
                            ★ 建链是阻塞的(最长几十秒), 期间 485 轮询完全停摆。
                              回来之后必须重置时间戳, 否则:
                               - last_upload_ms 还是几十秒前 -> 立刻连发好几条
                               - 更要命的是 ping: 若不重置, 下一轮循环立刻又判定
                                 "该发心跳了", 在链路刚建好时又发一次, 容易再次超时,
                                 形成"刚连上就断"的死循环。
                            ★ 节拍那边不用重置: Tick_Take_Missed 已经记录了
                              错过几拍, 轮询逻辑自己会补上。
                        */
                        last_upload_ms = Delay_GetMs();
                        last_ping_ms = Delay_GetMs();

                        /* 建链后立刻重扫一轮, 让云端尽快看到真实状态 */
                        ModBus_Poll_All();
                        ModBus_Update_Online();
                        Cloud_Upload_Online_If_Changed();
                    }
                }
            }
        }

        /*
            ---------- 5.7 ★ 最后一步才发日志 ----------
            这是"串口1 作为调试回显, 必须在所有需要完成的操作结束后再发送"
            这条要求的落地点:
            云端下行处理完了、485 轮询完了、自动开灯判断完了、
            上报发出去了、心跳也处理了 —— 才把这一轮攒下的日志一次性打出去。

            为什么顺序这么重要:
              USART1 是 9600 阻塞发送, 1 个字符 1.04ms。
              如果边轮询边打印, 打印时间会直接叠加进轮询周期
              (每台从机 3 笔事务, 打一行 40 字符就是 40ms);
              更糟的是打印期间从机的应答可能已经到达并被下一帧覆盖。
            现在无论这一轮攒了多少日志, 都不会挡在轮询前面。
        */
        /*
            ---------- 7. 刷新 LCD ----------
            ★ 放在最后: LCD 刷新是纯显示, 优先级最低。
              内部有 1 秒节流 + 只重绘变化区域, 数据不变时开销为 0。
              全屏填充要 150ms, 所以绝不能每轮都刷。
        */
        (void)LCD_UI_Update();
        LOG_Flush();
    }
}
