# STM32 智能家居系统 v21

STM32F103 主从架构的智能家居系统：主机（MASTER）负责 LCD 界面、ModBus 轮询、ESP8266 上云与串口命令行；三台从机（SLAVE）负责温湿度/光照采集与灯、蜂鸣器、舵机执行。云端使用 ThingsCloud，MQTT 协议。

## 目录结构

```
MASTER/            主机工程（Keil MDK）
  project/         Keil 工程文件
  user/api/        业务代码：ModBus、MQTT、ESP8266、LCD、CLI 等
  user/main.c      主循环
  libraries/       STM32F10x 标准外设库 + CMSIS
SLAVE/             从机工程（三台共用同一份源码，靠 ModBus 地址区分）
HEX/               编译好的固件（本地生成，不入库，见下）
```

## 快速开始

### 1. 准备凭据文件

云平台凭据（WiFi 密码、MQTT 设备账号）**不在仓库里**，需要自己填一份：

```bash
cd MASTER/user/api
cp cloud_credentials.example.h cloud_credentials.h
```

然后编辑 `cloud_credentials.h`，填入你自己的参数：

| 宏 | 含义 | 从哪里拿 |
|---|---|---|
| `CLOUD_AP_SSID` / `CLOUD_AP_PSK` | WiFi 热点名与密码 | 你的 2.4GHz 热点（ESP8266 不支持 5GHz） |
| `CLOUD_MQTT_IP` | MQTT 接入点 | ThingsCloud 项目概览页 |
| `CLOUD_MQTT_CLIENT` | 设备 Client ID | ThingsCloud 设备详情页 |
| `CLOUD_MQTT_USERNAME` / `CLOUD_MQTT_PASSWORD` | 设备级 MQTT 凭据 | ThingsCloud 设备详情页 |

`cloud_credentials.h` 已被 `.gitignore` 排除，不会误提交。缺少任一项时编译会直接 `#error` 报错，不会拖到烧录后才发现连不上。

### 2. 编译烧录

用 Keil MDK 打开 `MASTER/project/STM32F103ZET6_Project.uvprojx`（从机用 `SLAVE/project/` 下的同名工程），编译后烧录。

从机的 ModBus 地址靠拨码/固件区分，三台从机对应 `SLAVE_addr1/2/3`。

## 功能概览

- **传感器采集**：从机 DHT11 温湿度 + 光敏电阻光照，主机 SR602 人体感应
- **执行器**：灯、蜂鸣器（开关量），SG90 舵机（0/90/180 三档）
- **通信**：主机↔从机走 RS485 ModBus；主机↔云走 ESP8266 + MQTT
- **本地界面**：LCD 显示三台从机状态，按键操作
- **串口命令行**：串口1 可直接发 JSON 模拟云端下发，或 `@` 开头的诊断命令（`@help` 查看全部）
- **自动照明**：SR602 触发 + 夜晚判定后自动开灯

## 文档

| 文件 | 内容 |
|---|---|
| `说明_v21.md` | v21 版本变更说明与验证记录 |
| `串口测试命令手册.md` | 全部串口命令与 JSON 指令用法 |
| `云平台配置清单.md` | ThingsCloud 物模型 20 个属性的完整配置 |
| `代码复查_v20.md` | v20 代码复查记录 |
| `STM32智能家居系统_技术汇报_v4.pptx` | 技术汇报材料 |

## 关于固件（HEX）

`HEX/` 目录不在仓库内。固件是把源码连同当时的 `cloud_credentials.h` 一起编译出来的，二进制里以**明文**保存 WiFi 密码和 MQTT 凭据——任何人拿到 HEX 都能直接提取出来。

需要固件请自己用 Keil 编译（填好自己的凭据），或私下传输。

## 许可

个人学习项目，代码可自由参考。第三方库（STM32F10x StdPeriph Driver、CMSIS）版权归 ST 与 ARM 所有。
