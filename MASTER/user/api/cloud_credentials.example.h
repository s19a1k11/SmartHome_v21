/*
 * cloud_credentials.example.h —— 云平台凭据模板（可公开）
 *
 * 用法：
 *   1. 把本文件复制为同目录下的 `cloud_credentials.h`
 *        copy cloud_credentials.example.h cloud_credentials.h
 *   2. 把下面的占位值换成你自己的真实参数
 *   3. `cloud_credentials.h` 已被 .gitignore 排除，不会进入仓库
 *
 * 注意：MQTT 的用户名密码是【设备级】凭据，在 ThingsCloud 的
 *       设备详情页获取，不是控制台登录账号。填错的话服务器会直接
 *       断连而不回 CONNACK，现场表现是"一直显示离线"。
 */

#ifndef _CLOUD_CREDENTIALS_H
#define _CLOUD_CREDENTIALS_H

/* ---- WiFi 热点（ESP8266 要连的那个 2.4GHz 热点，不支持 5GHz） ---- */
#define CLOUD_AP_SSID "YOUR_WIFI_SSID"
#define CLOUD_AP_PSK  "YOUR_WIFI_PASSWORD"

/* ---- ThingsCloud 接入点：项目概览页给出的 MQTT 地址 ---- */
#define CLOUD_MQTT_IP "YOUR_PROJECT.mqtt.iot-api.com"

/* ---- 设备身份：同一项目下多台设备必须各用各的，不能重复 ---- */
#define CLOUD_MQTT_CLIENT   "YOUR_DEVICE_CLIENT_ID"
#define CLOUD_MQTT_USERNAME "YOUR_DEVICE_MQTT_USERNAME"
#define CLOUD_MQTT_PASSWORD "YOUR_DEVICE_MQTT_PASSWORD"

#endif
