#ifndef _ESP8266_H
#define _ESP8266_H

#include "common.h"
#include "usart3.h"
#include "delay.h"

#define ESP8266_CMD_BUFFER_SIZE (128)

/* ==========================================================================
   ESP8266 控制引脚 —— 战神V1 开发板的实际接法
   ==========================================================================
   来源: D:\00_开班软件\PCBrun\STM开发资料\战神V1的IO引脚分配表.xlsx
         STM32F1战神V1_原理图V3.0.pdf (第4页 WIFI ESP-01)

   手册原文:
     PA4  | STM_DAC | GBC_KEY | "2. WIFI模块的复位引脚"
     PA15 | JTDI    | GBC_LED | "2. WIFI模块的GPIO0和蓝牙模块的状态引脚"

   ★ 为什么必须主动控制这两个脚(v21 补):
     改造前代码里【完全没有初始化 PA4 和 PA15】, 上电后它们是复位默认的
     浮空输入状态。而 ESP-01 模块要求:
       CH_PD (接 PA4)   必须为【高电平】芯片才工作, 低电平 = 关断
       GPIO0 (接 PA15)  必须为【高电平】才进正常 AT 模式,
                        低电平会进下载(烧录)模式, 不跑 AT 固件
     这两个脚悬空时电平不确定, 模块很可能一直处于关断/下载模式,
     表现就是主机发 AT 永远收不到回应 -> [ESP] module no response。

   ★ PA15 的特殊性:
     它是 JTAG 的 JTDI 脚, 复位后默认被 JTAG 占用。
     要当普通 IO 用, 必须先调 GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable)
     关掉 JTAG(保留 SWD, 不影响用 ST-Link 下载调试)。
*/
#define ESP_CHPD_PORT        GPIOA
#define ESP_CHPD_PIN         GPIO_Pin_4
#define ESP_CHPD_RCC         RCC_APB2Periph_GPIOA

#define ESP_GPIO0_PORT       GPIOA
#define ESP_GPIO0_PIN        GPIO_Pin_15
#define ESP_GPIO0_RCC        RCC_APB2Periph_GPIOA

/*
	复位时序参数(单位 ms)
	ESP8266 上电/复位后需要一段时间启动, 期间发 AT 会丢。
	数据手册: 上电到 AT 可用约 100~300ms, 保守取 500ms。
*/
#define ESP_POWER_ON_DELAY_MS   (500)
#define ESP_RESET_LOW_MS        (100)

// AT 指令默认等待时间，单位 ms
#define ESP8266_SHORT_TIMEOUT (2000)
#define ESP8266_LONG_TIMEOUT (15000)

/**
 *  @brief 清空 ESP8266 对应的接收缓冲区
 */
void ESP8266_Recv_Clear(void);

/**
 *  @brief 发送一条 AT 指令并等待模块返回指定关键字
 *
 *  @param cmd  AT 指令字符串，需要自带 "\r\n"
 *  @param ack  期望的响应关键字，例如 "OK" / "WIFI GOT IP" / ">"
 *  @param timeout_ms 等待超时时间
 *
 *  @return 1 收到期望关键字 / 0 超时未收到
 */
u8 ESP8266_SendCmd(const char *cmd, const char *ack, u16 timeout_ms);

/**
 *  @brief 测试 ESP8266 模块是否在线
 *
 *  @return 1 在线 / 0 无响应
 */
/**
 *  @brief 初始化 ESP8266 的控制引脚 (CH_PD / GPIO0), 并复位模块
 *
 *  必须在 USART3_Init 之后、第一次发 AT 之前调用。
 *  内部会:
 *    1. 禁用 JTAG(保留 SWD), 让 PA15 能当普通 IO
 *    2. PA4 (CH_PD) 拉低 -> 延时 -> 拉高 -> 延时等模块启动
 *    3. PA15 (GPIO0) 拉高 = 正常运行模式(不是下载模式)
 *
 *  @return 1 初始化完成 / 0 参数错误
 */
u8 ESP8266_Init(void);

/**
 *  @brief 打印控制引脚当前电平(诊断用)
 *
 *  @param chpd_out 输出: PA4(CH_PD) 电平 0/1
 *  @param gpio0_out 输出: PA15(GPIO0) 电平 0/1
 */
void ESP8266_Get_Ctrl_Pins(u8 *chpd_out, u8 *gpio0_out);

u8 ESP8266_Test_AT(void);

/**
 *  @brief 连接目标热点
 *
 *  @param ssid 热点名称
 *  @param psk 热点密码
 *
 *  @return 1 连接成功 / 0 失败
 */
u8 ESP8266_ConnectAP(const char *ssid, const char *psk);

/**
 *  @brief 用 TCP 协议连接到目标 MQTT 服务器
 *
 *  @param ip 服务器域名或 IP
 *  @param port 服务器端口
 *
 *  @return 1 连接成功 / 0 失败
 */
u8 ESP8266_ConnectTCP(const char *ip, u16 port);

/**
 *  @brief 进入透传模式
 *
 *  进入透传后模块不再解析 AT 指令，所有串口数据直接转发到 TCP 连接上，
 *  MQTT 报文必须在这个状态下发送
 *
 *  @return 1 成功 / 0 失败
 */
u8 ESP8266_EnterTransparent(void);

/**
 *  @brief 退出透传模式，回到 AT 指令模式
 *
 *  模块的约定是：前后各留 20ms 以上的总线静默，中间裸发一串 +++ 且不带
 *  换行符，否则模块会把 +++ 当成数据发走
 *
 *  @return 1 已回到 AT 模式 / 0 失败
 */
u8 ESP8266_ExitTransparent(void);

/**
 *  @brief ESP8266 完整连云流程：测试 -> 复位 -> STA -> 入网 -> TCP -> 透传
 *
 *  @param ssid 热点名称
 *  @param psk 热点密码
 *  @param ip MQTT 服务器域名或 IP
 *  @param port MQTT 服务器端口
 *
 *  @return 1 全流程成功 / 0 中途失败
 */
u8 ESP8266_ConnectCloud(const char *ssid, const char *psk,
                        const char *ip, u16 port);

#endif
