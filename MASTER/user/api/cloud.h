#ifndef _CLOUD_H
#define _CLOUD_H

#include "common.h"
#include "delay.h"
#include "mqtt.h"
#include "modbus.h"
#include "log.h"

// 云平台 MQTT 主题：上行走 attributes，下行订阅 attributes/push
#define CLOUD_PUBLISH_TOPIC "attributes"
#define CLOUD_SUBSCRIBE_TOPIC "attributes/push"

/*
云平台连接参数，换热点或换设备账号时只改这一块

注意 MQTT 的用户名密码是【设备级】凭据，在平台设备详情页获取，
不是控制台登录账号，填错的话服务器会直接断连而不回 CONNACK

★ 凭据不写在这里。
  本文件随仓库公开，把 WiFi 密码和设备凭据写进来等于直接发布到互联网。
  实际取值来自 `cloud_credentials.h`（该文件已被 .gitignore 排除，不进仓库）。
  首次编译前把 `cloud_credentials.example.h` 复制成 `cloud_credentials.h` 再填值。
*/
#include "cloud_credentials.h"

/* ---- 私有配置的完整性检查：缺一项就编译失败，不要拖到现场连不上 ---- */
#ifndef CLOUD_AP_SSID
#error "CLOUD_AP_SSID 未定义：请在 cloud_credentials.h 中填写 WiFi 热点名称"
#endif
#ifndef CLOUD_AP_PSK
#error "CLOUD_AP_PSK 未定义：请在 cloud_credentials.h 中填写 WiFi 密码"
#endif
#ifndef CLOUD_MQTT_IP
#error "CLOUD_MQTT_IP 未定义：请在 cloud_credentials.h 中填写你的 MQTT 接入点"
#endif
#ifndef CLOUD_MQTT_CLIENT
#error "CLOUD_MQTT_CLIENT 未定义：请在 cloud_credentials.h 中填写你的设备 Client ID"
#endif
#ifndef CLOUD_MQTT_USERNAME
#error "CLOUD_MQTT_USERNAME 未定义：请在 cloud_credentials.h 中填写设备级 MQTT 用户名"
#endif
#ifndef CLOUD_MQTT_PASSWORD
#error "CLOUD_MQTT_PASSWORD 未定义：请在 cloud_credentials.h 中填写设备级 MQTT 密码"
#endif

/* ---- 以下为不可变的协议常量，与账号无关，可以公开 ---- */

#define CLOUD_MQTT_PORT (1883)

// MQTT Keep Alive，单位秒。主循环里的心跳间隔必须小于这个值
#define CLOUD_MQTT_KEEP_ALIVE (60)

// 发送 CONNECT 后等待 CONNACK 的超时时间，单位 ms
#define CLOUD_CONNACK_TIMEOUT (5000)
// 建链失败后隔多久重试一次，单位 ms
#define CLOUD_RELINK_INTERVAL (3000)
// 建链时的最大重试次数
#define CLOUD_LINK_RETRY (5)

/* ==========================================================================
   上报间隔 —— 为什么选 15 秒而不是 10 秒
   ==========================================================================
   ★ 需求: "云平台上传的时间间隔需要大于 10 秒, 否则会被判定为攻击服务器"

   取 15 秒的理由:
     1) 10 秒是"下限", 不是"目标值"。平台侧的流控判据通常是
        "单位时间内的消息条数", 贴着下限跑等于每次都在红线边上 ——
        一旦某次因为"状态翻转立刻上报"(Cloud_Upload_Online_If_Changed)
        额外多发一条, 那一分钟的条数就会超。
        15 秒给这个"额外的一条"留出了 50% 的余量。
     2) 数据新鲜度够用: 温湿度/光照本来就是 2 秒采集一次,
        但室内环境在 15 秒内的变化对用户没有意义;
        真正需要"立刻知道"的是开关状态和离线告警 ——
        这两类已经走"状态一变就立刻上报"的通道, 不受这个间隔影响。
     3) 数值上更好算: 60 秒 / 15 秒 = 4 条/分钟。
        配合心跳 30 秒一次(2 条/分钟), 总报文量约 6 条/分钟,
        远低于任何平台的流控阈值。

   为什么不取 20 秒或更长:
     云端界面上的温度曲线会变得很稀疏(每分钟 3 个点),
     看起来像"卡住了"; 15 秒是"曲线平滑"和"不触流控"之间的折中。
   ========================================================================== */
#define CLOUD_UPLOAD_INTERVAL (15000)

/*
	即时上报（下线、下发回读）的最小间隔，单位 ms。
	从机时通时断会让在线状态反复翻转，没有间隔限制的话上报频率会失控，
	再次触发平台流控。定时上传是 15 秒一次，不受这个间隔影响。

	取 8 秒: 比 15 秒的定时上传短(保证"离线"能比定时上传更快到达云端),
	         又足够长(防止状态抖动连发)。
*/
#define CLOUD_UPLOAD_MIN_GAP (8000)

// 下行 JSON 解析缓冲大小
#define CLOUD_TOPIC_SIZE (48)
#define CLOUD_RECV_SIZE (128)

/*
	回家自动开灯：人体感应触发后写入的灯寄存器值
	★ 必须是 1: 从机灯寄存器只接受 0/1(纯开关), 写 100 会被回异常帧拒绝
*/
#define AUTO_LIGHT_VALUE (1)

/*
	舵机平台枚举到从机档位（0x0202 寄存器）的映射
	clockwise = 顺时针转，counter_clockwise = 逆时针转

	★ 从机只认 0/1/2 三个档位(0 度 / 90 度 / 180 度), 不认角度值。
*/
#define CLOUD_SG90_CLOCKWISE_GEAR (2)
#define CLOUD_SG90_CCW_GEAR (0)

/*
	★ 布尔量开灯时写入从机 0x0200 的值

	历史 bug（已修复）：这里原来是 100。
	从机 ModBus_Set_Reg 对 0x0200 的判据是 if(val > 1) return 0,
	所以写 100 一定被拒 -> 回异常帧 01 86 03 -> 灯根本不会亮。
	从机灯寄存器是纯开关（本项目不做 PWM 调光），所以只能写 1。
*/
#define CLOUD_LED_ON_VALUE (1)

// 舵机档位取值范围
#define CLOUD_SG90_GEAR_MAX (2)

/*
	上行 JSON 缓冲大小
	本版每台从机上报 7 个键(Temp/Hum/Lux/Led/Bp/SenErr/Online),
	另加主机的 SR602 和 LuxAvg, 三台约 320 字节。留 448 有余量。
*/
#define CLOUD_PAYLOAD_SIZE (448)

/*
	编译期一致性检查: MQTT 报文缓冲区必须装得下最大 payload。
	★ 为什么要有这条:
	  v19 的严重 bug 就是 MQTT_PACKAGE_SIZE(256) 比实际 payload(306) 还小,
	  结果每一条上行都被静默丢弃, 云端一个字节收不到。
	  加了这个 #if 之后, 以后谁调大 payload 却忘了同步 MQTT 缓冲区,
	  编译直接失败, 不会拖到现场才发现在线却没数据。
*/
#if (MQTT_PACKAGE_SIZE < (CLOUD_PAYLOAD_SIZE + 5 + 2 + 16))
#error "MQTT_PACKAGE_SIZE too small: must hold CLOUD_PAYLOAD_SIZE + mqtt header"
#endif

/**
 *  @brief 在 JSON 字符串中提取一个整型键值
 */
u8 JSON_GetInt(const char *json, const char *key, int *value);

/**
 *  @brief 在 JSON 字符串中提取一个布尔键值
 */
u8 JSON_GetBool(const char *json, const char *key, u8 *value);

/**
 *  @brief 在 JSON 字符串中提取一个字符串键值
 */
u8 JSON_GetString(const char *json, const char *key, char *out, u8 out_size);

/**
 *  @brief 上传指定从机的传感器数据到云平台
 *
 *  @return 1 已发出 / 0 未发出(离线抑制、间隔限制、报文超长)
 */
u8 Cloud_Upload_Slave(u8 index);

/**
 *  @brief 一次性上传三台从机的全部数据、主机人体感应与在线状态
 *
 *  @return 1 已发出 / 0 未发出(报文超长)
 */
u8 Cloud_Upload_All(void);

/**
 *  @brief 上传所有从机的在线情况
 *
 *  @return 1 已发出 / 0 未发出(间隔限制或报文超长)
 */
u8 Cloud_Upload_Online(void);

/**
 *  @brief 检查是否有从机在线状态发生变化，有就立刻上报一次
 *
 *  ★ 这是"从机离线后立即反馈云端"的落地函数
 *
 *  @return 1 有状态翻转且已上报 / 0 无变化或上报被间隔限制挡下
 */
u8 Cloud_Upload_Online_If_Changed(void);

/**
 *  @brief 解析云端下发的 JSON 指令，转换成 ModBus 写寄存器帧发给从机
 */
void Cloud_Handle_Downlink(const char *payload);

// 云平台 CONNACK 标志
extern volatile u8 g_mqtt_connack;
// PINGRESP 标志
extern volatile u8 g_mqtt_ping_resp;

/**
 *  @brief 处理 ESP8266 透传通道上收到的 MQTT 报文
 */
void Cloud_Rx_Process(void);

/**
 *  @brief 云平台层自检
 */
void Cloud_Self_Test(void);

/**
 *  @brief 建立到云平台的完整链路
 */
u8 Cloud_Link_Build(void);

#endif
