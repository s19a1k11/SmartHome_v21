#ifndef _MODBUS_H
#define _MODBUS_H

#include "common.h"
#include "USART2_485.h"
#include "USART1.h"
#include "GPIO_Init_One.h"
#include "log.h"

// 485 模块对应波特率参数
#define RS485_BRR (9600)

// 当前485 单一字节传递时间
#define BYTE_INTERVAL_TIME (1000000 / (RS485_BRR / 10))

/* 单次读写的最大寄存器个数: 输入区一次读 6 个, 所以至少 6 */
#define REG_MAX 6

/* ==========================================================================
   一、从机地址 —— 全套固件只有这一个地方需要改
   ==========================================================================
   三块从机板烧的是【同一份源码、同一个工程】，唯一区别就是下面这个数字：
       1 号板: #define MODBUS_SLAVE_ADDR   0x01
       2 号板: #define MODBUS_SLAVE_ADDR   0x01
       3 号板: #define MODBUS_SLAVE_ADDR   0x01

   为什么必须用宏而不是在代码里到处写 0x01:
       总线上所有从机并联在同一对 485 线上，靠地址域区分谁该应答。
       如果地址散落在多个文件里，改一台就要改好几处，漏一处就变成
       "两台同时应答 -> 总线上出现叠加乱码 -> CRC 必败 -> 两台都像坏的"。

   全工程引用地址的唯一入口是 ModBus.c 里的 RS485_IP，
   它由本宏初始化；其它文件一律通过 RS485_IP 间接使用，不再出现字面量。
   ========================================================================== */
#define MODBUS_SLAVE_ADDR   0x01

/*
	组网时总线上最大从机号: 1 ~ MODBUS_SLAVE_MAX
	主机侧只放行这个范围内的地址(主机自己没有地址);
	从机侧仍然严格只认 MODBUS_SLAVE_ADDR, 不受本宏影响。
	本工程按 3 台组网。要加第 4 台就把这里改成 4,
	同时主机那边的 SLAVE_NUM 一起改。
*/
#define MODBUS_SLAVE_MAX    3

/*
	编译期自检: 地址必须落在 1 ~ MODBUS_SLAVE_MAX 之间。
	越界不会报错, 只会在运行时不吭声(谁都不应答, 或者 0 号地址永远没人理),
	那种故障最难查, 所以在编译期就拦下来。
*/
#if (MODBUS_SLAVE_ADDR < 1) || (MODBUS_SLAVE_ADDR > MODBUS_SLAVE_MAX)
#error "MODBUS_SLAVE_ADDR must be in 1..MODBUS_SLAVE_MAX"
#endif

/* ==========================================================================
   二、寄存器区段 (高字节 = 区段号, 低字节 = 段内偏移)
   ========================================================================== */
#define REG_SYS_BASE        0x0000
#define REG_SYS_NUM         4
#define REG_INPUT_BASE      0x0100
#define REG_INPUT_NUM       6
#define REG_OUTPUT_BASE     0x0200
#define REG_OUTPUT_NUM      3

/*
	系统区 4 个寄存器:
	   0x0000 固件版本 (高字节主版本 / 低字节次版本)
	   0x0001 上电运行秒数
	   0x0002 传感器健康位图 (bit0 = DHT11 连续失败已达判定门限)
	   0x0003 DHT11 连续失败次数 (0 表示正常, 用于现场定位)
*/
#define REG_SYS_FW_VERSION  (REG_SYS_BASE + 0)
#define REG_SYS_RUN_SEC     (REG_SYS_BASE + 1)
#define REG_SYS_SENSOR      (REG_SYS_BASE + 2)
#define REG_SYS_DHT_FAIL    (REG_SYS_BASE + 3)

/*
	输入区 6 个寄存器: 湿度整 / 湿度小 / 温度整 / 温度小 / 光照 / 保留
	   0x0105 是【保留】, 固定返回 0。

	★ 2026-10-24 变更: SR602 人体红外移到主机侧了。
	  理由: 实际应用里 SR602 装在家门内, 只有门口那台设备(主机/总控)
	        需要它; 三台从机各接一个既浪费又没意义。
	  这里保留地址只为了让"读输入区 6 个寄存器"的应答长度不变,
	  主机和云端既有的解析代码一个字都不用改。
*/
#define REG_IN_HUMI         (REG_INPUT_BASE + 0)
#define REG_IN_HUMI_DEC     (REG_INPUT_BASE + 1)
#define REG_IN_TEMP         (REG_INPUT_BASE + 2)
#define REG_IN_TEMP_DEC     (REG_INPUT_BASE + 3)
#define REG_IN_LIGHT        (REG_INPUT_BASE + 4)
#define REG_IN_RESERVED     (REG_INPUT_BASE + 5)

/*
	输出区 3 个寄存器: 灯 / 蜂鸣器 / 舵机
	   灯: 0 = 灭, 1 = 亮 (只接受 0/1, 写 2 及以上回异常帧 03)
	   舵机: 0 -> 0 度 / 1 -> 90 度 / 2 -> 180 度
*/
#define REG_OUT_LED         (REG_OUTPUT_BASE + 0)
#define REG_OUT_BEEP        (REG_OUTPUT_BASE + 1)
#define REG_OUT_SERVO       (REG_OUTPUT_BASE + 2)

/* 系统区 0x0002 健康位图里的位定义 */
#define SENSOR_FAULT_DHT11  (0x0001)

/*
	连续多少次采集失败才判定"传感器异常"
	连续两次数据采集都没有成功 -> 置故障位, 让主机/云端知道这台从机的
	传感器坏了(它本身还在线, 485 通信正常, 只是数据不可信)。
*/
#define SENSOR_FAIL_LIMIT   2

/*
	从机调试日志开关
	1 = 把每一帧的收发过程记进日志缓冲(在 LOG_Flush 时才真正发出)
	0 = 只记采集结果和错误

	★ 无论开关是几, 日志都不会出现在中断里、也不会插在应答之前 ——
	  统一由 main.c 在"所有操作结束后"调 LOG_Flush()。
*/
#define MODBUS_SLAVE_LOG    0

/*
	ModBus 错误码
	约定: 0 表示成功，与各函数"成功返回 0"保持一致。
*/
typedef enum
{
	MODBUS_OK           = 0,   /* 成功 */
	MODBUS_ERR_SHORT    = 1,   /* 帧长度不足 */
	MODBUS_ERR_CRC      = 2,   /* CRC 校验失败 */
	MODBUS_ERR_ADDR     = 3,   /* 从机地址不匹配 */
	MODBUS_ERR_FUNC     = 4,   /* 功能码不支持 */
	MODBUS_ERR_EXC      = 5,   /* 从机返回异常帧 */
	MODBUS_ERR_OVERFLOW = 6    /* 缓冲区溢出 */
} ModBus_ErrorCode;

/*
	主机/从机模式开关
	★ 本工程是从机固件, 只打开 MODBUS_SLAVE
*/
//#define MODBUS_MASTER
#define MODBUS_SLAVE

// 宏开关/宏标记，使用哪一个时间间隔就打开哪个宏标志
#define USING_T3_5
//#define USING_T4_0

//T3_5
#ifdef USING_T3_5

/*
	DATA_FRAME_TIME_INTERVAL 宏定义名称保持一致，但是在不同的时间间隔
	对应数据不同，方便后续项目开发，并且不需要修改代码内容
*/
#define DATA_FRAME_TIME_INTERVAL (BYTE_INTERVAL_TIME * 3 + (BYTE_INTERVAL_TIME >> 1))

#endif

//T4_0
#ifdef USING_T4_0

#define DATA_FRAME_TIME_INTERVAL (BYTE_INTERVAL_TIME * 4)

#endif

// Modbus 数据接收控制定时，使用 TIM2 基础定时实现
void ModBus_TIM_Config(void);

/*
 * @brief Modbus CRC-16 计算函数
 *
 * @param data   	待计算的数据指针
 * @param length  数据长度(字节数)
 * @return CRC校验位 (16位)
*/
uint16_t Modbus_CRC16(const uint8_t *data, size_t length);

/*
	寄存器缓存 ("柜子里的数")
	定义在 ModBus.c; 采集层负责写, ModBus_Get_Reg 只读
*/
/*
	本机地址(定义在 ModBus.c, 初值 = MODBUS_SLAVE_ADDR)
	为什么要在这里 extern: main.c 开机打印地址、ModBus.c 组应答帧都要用它,
	而它是普通全局变量, 不声明就编译不过。
*/
extern u8  RS485_IP;

extern u16 g_humi,  g_humi_dec;
extern u16 g_temp,  g_temp_dec;
extern u16 g_light;
extern u16 g_led,   g_beep, g_servo;

/* 寄存器读写辅助 */
u16  ModBus_Get_Reg(u16 addr);

u8   ModBus_Set_Reg(u16 addr, u16 val);

/* 异常响应帧 */
void ModBus_Send_Exception(u8 addr, u8 func, u8 errcode);

/*
	从机应答: 收到主机的请求帧后按功能码处理并回帧
	调用者: main.c 从机分支, usart2_value.flag == 1 时调一次
	返回值: 1 已回正常应答 / 0 已回异常帧 / MODBUS_ERR_* 静默丢弃
*/
u8 ModBus_Read_03_Cmd(void);

/*
	采集层上报一次 DHT11 采集结果
	ok = 1 采集成功 / 0 采集失败
	内部维护"连续失败次数", 达到 SENSOR_FAIL_LIMIT 就置系统区 0x0002 的
	SENSOR_FAULT_DHT11 位, 并保留旧值不覆盖寄存器缓存
*/
void ModBus_Sensor_Result(u8 ok);

/* 当前 DHT11 连续失败次数(0 = 正常) */
u8 ModBus_Sensor_Fail_Count(void);

/* 当前传感器健康位图(0 = 全部正常) */
u16 ModBus_Sensor_Health(void);

/*
	★ 中断优先级分配 (NVIC_PriorityGroup_2: 2 位抢占 0~3 + 2 位子优先级 0~3)

	从机侧的原则是"收 > 发":
	  USART2 收字节   抢占 0 / 子 1   收错一个字节就整帧废掉, 最高
	  TIM2   判帧结束 抢占 1 / 子 0   晚了只是应答晚几微秒
	  TIM6   1ms 时基 抢占 2 / 子 0
	  USART3 本地直控 抢占 3 / 子 0   不参与时序
	  USART1 调试回显 抢占 3 / 子 1   最不重要, 且发送本身是阻塞的

	★ "发"没有中断: USART2 发送是阻塞轮询 TC, 不占中断。
	  阻塞发送期间允许被 USART2 接收中断打断 —— 这正是"收 > 发"的体现:
	  正在往总线发应答时, 如果有新字节进来(多机竞争), 先收进来再说。
*/

/* 调试辅助(全部走日志缓冲, 不直接发串口) */
void ModBus_PrintHex(const char *tag, u8 *buffer, u16 len);
void ModBus_PrintNum(u16 n);
void ModBus_PrintHexNum(u16 v);

#endif
