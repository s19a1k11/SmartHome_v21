#ifndef _MQTT_H
#define _MQTT_H

#include "common.h"
#include "usart3.h"

/* ==========================================================================
   MQTT 报文缓冲区大小 —— 这个值必须 >= 最大上行 payload + 报文头
   ==========================================================================
   ★ 历史严重 bug (v20 复查发现):
     这里原来是 256。而本工程上行 JSON 实测 306 字节(3 台 x 6 键 + 主机 3 键
     + 在线 3 键), MQTT_Publish_Package_Send 里的判据是
         if (remaining_len + 5 > MQTT_PACKAGE_SIZE) return;   // 静默返回!
     于是【每一条上行消息都被静默丢弃】—— 云端一个字节都收不到,
     而且日志里什么都不会打, 现象是"设备在线、心跳正常、但云端没有任何数据"。
     这是最难查的一类故障: 链路全通, 只有数据不见。

   算一下需要的空间:
     上行 payload 实际最长 ≈ 330 字节
     MQTT 固定头    = 1 (类型) + 最多 2 (变长剩余长度, <16384 只需 2 字节) = 3
     MQTT 可变头    = 2 (主题长度) + strlen("attributes") = 12
     合计最坏       = 330 + 3 + 12 = 345 字节

   取 512:
     - 覆盖 447(理论最大 payload) + 15(头) = 462, 留 10% 余量
     - 是 2 的幂, 编译器对齐友好
     - 栈上 512 字节: 主机栈 0x400 = 1024 字节, 这个函数里没有别的
       大局部变量, 单次占用 512 是安全的

   ★ 配套改动: 发送函数改成返回 u8 表示成功/失败,
     让调用方能发现"没发出去", 而不是静默丢包。
   ========================================================================== */
#define MQTT_PACKAGE_SIZE (512)

/* 一次发布最多能承载的 payload 字节数(留给调用方做编译期校验) */
#define MQTT_PAYLOAD_MAX  (MQTT_PACKAGE_SIZE - 3 - 2 - 16)   /* 512-21 = 491 */

// MQTT 控制报文类型（高 4 位）
#define MQTT_PKT_CONNACK (0x20)
#define MQTT_PKT_PUBLISH (0x30)
#define MQTT_PKT_PINGRESP (0xD0)
#define MQTT_PKT_SUBACK (0x90)

/**
 *  @brief 请求连接到 MQTT 云平台，要求用户提供的参数
 *
 *  @return 1 已发出 / 0 报文超长未发出
 */
u8 MQTT_Connect_Package_Send(const char *client, const char *username,
                             const char *password, u16 keep_alive_time);

/**
 *  @brief MCU 发送数据到云平台
 *
 *  ★ 返回值必须接: 返回 0 表示报文超长被丢弃(不会静默失败)
 *
 *  @return 1 已发出 / 0 报文超长未发出
 */
u8 MQTT_Publish_Package_Send(const char *publish_topic, const char *payload);

/**
 *  @brief MCU 发送订阅主题数据包到 MQTT 云平台
 *
 *  @return 1 已发出 / 0 报文超长未发出
 */
u8 MQTT_Subscribe_Package_Send(const char *subscribe_topic);

/**
 *  @brief MCU 发送心跳请求包 PINGREQ 到 MQTT 云平台
 *
 *  必须在 Keep Alive 时间内发出，否则云平台会主动踢掉连接
 */
void MQTT_PingReq_Package_Send(void);

/**
 *  @brief 计算一包 MQTT 报文的总长度（固定头 + 剩余长度）
 *
 *  @return 报文总字节数；返回 0 表示数据还没收全，需要继续等待
 */
u16 MQTT_Packet_Length(u8 *buf, u16 len);

/**
 *  @brief 解析云平台下发的 PUBLISH 报文，取出主题和有效载荷
 *
 *  @return 1 解析成功 / 0 报文不完整或不是 PUBLISH
 */
u8 MQTT_Publish_Package_Parse(u8 *buf, u16 len, char *topic, u16 topic_max,
                              char *payload, u16 payload_max);

#endif
