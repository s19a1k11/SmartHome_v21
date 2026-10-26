#include "all.h"

/*
	【卡10】光照采集: 光敏模块 AO -> PA1 -> ADC1 通道 1
	==================================================================
	它是干什么的:
		把 PA1 上的模拟电压(0~3.3V)变成 0~4095 的数字, 存进 g_light,
		主机读寄存器 0x0104 就能拿到。

	为什么会有"模拟量"这回事:
		光和声音一样是连续变化的, 不像按键只有"按下/松开"两种状态。
		数字引脚只能判断"高于 1.65V 算 1, 低于算 0", 中间的细节全丢。
		ADC 就是那把"尺子", 把 0~3.3V 切成 4096 格, 每格约 0.8mV。

	本文件用标准外设库函数写
	-----------------------------------------------------------------
	库函数不是另一套硬件, 它只是把"往哪个寄存器第几位写什么"包装成了
	一个带名字的函数。左边是库函数, 右边是它实际干的事:

		RCC_APB2PeriphClockCmd(...)      -> RCC->APB2ENR 的对应位
		RCC_ADCCLKConfig(RCC_PCLK2_Div6) -> RCC->CFGR 的 bit15~14 写 10
		GPIO_Init(GPIOA, &结构体)         -> GPIOA->CRL 里那 4 位写 0000
		ADC_Init(ADC1, &结构体)           -> CR1/CR2 的一堆位
		ADC_RegularChannelConfig(...)     -> SQR1/SQR3/SMPR2
		ADC_Cmd(ADC1, ENABLE)             -> CR2 的 bit0 (ADON)
		ADC_StartCalibration / Get...     -> CR2 的 bit3/bit2

	所以读代码时只要认函数名就够了, 需要抠细节时再回去看寄存器。

	三个必须配对的参数(配错就读不出正确值):
		1. ADC 时钟不能超过 14MHz  -> PCLK2(72MHz) / 6 = 12MHz
		2. 引脚必须是"模拟输入"     -> GPIO_Mode_AIN
		3. 采样时间要够长           -> 239.5 个周期, 12MHz 下约 20us
		   (外部是电阻分压, 阻抗不低, 采样时间太短电容充不满, 读数会偏低)
*/


/*
	初始化: PA1 模拟输入 + ADC1 单通道单次转换 + 校准
	-----------------------------------------------------------------
	调用一次即可, 放在 main 的 while(1) 之前。
*/
void Light_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef  ADC_InitStructure;

	/* 1. 开时钟: GPIOA 和 ADC1 都挂在 APB2 上 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_ADC1, ENABLE);

	/* 2. ADC 预分频 = PCLK2 / 6 = 12MHz
	      可选 Div2 / Div4 / Div6 / Div8, 只有 Div6 和 Div8 不超过 14MHz 上限 */
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);

	/* 3. PA1 配成模拟输入
	      GPIO_Mode_AIN 会让引脚内部的施密特触发器断开, 电压原样送到 ADC
	      结构体里没用到的成员(Speed)不用管, 模拟模式下它不起作用 */
	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* 4. ADC1 工作方式: 独立模式 + 单通道 + 单次转换
	      ADC_Mode_Independent      : 只用 ADC1, 不和 ADC2 联动
	      ADC_ScanConvMode = DISABLE: 不扫描多通道
	      ADC_ContinuousConvMode    : DISABLE = 转换一次就停, 下次要用再手动启动
	      ADC_ExternalTrigConv_None : 软件触发(写 SWSTART 才开始)
	      ADC_DataAlign_Right       : 数据右对齐 -> 结果就是 0~4095
	      ADC_NbrOfChannel = 1      : 规则序列里只有 1 个通道 */
	ADC_InitStructure.ADC_Mode               = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode       = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStructure.ADC_ExternalTrigConv   = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign          = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel       = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* 5. 规则转换序列: 第 1 个转换的是通道 1, 采样时间 239.5 周期
	      参数顺序 = (ADCx, 通道号, 排在序列第几个, 采样时间)
	      239.5 周期是可选值里最长的一档, 12MHz 下约 20us, 读数最稳 */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);

	/* 6. 打开 ADC */
	ADC_Cmd(ADC1, ENABLE);

	/* 7. 上电后要等至少 2 个 ADC 时钟周期才能校准, 这里给 10us 富余 */
	delay_us(10);

	/* 8. 复位校准: 启动后硬件自动清标志, 所以循环等它自己变 0 */
	ADC_ResetCalibration(ADC1);
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* 9. 启动校准: 同样等标志被硬件清 0, 校准完才能读到准确值
	      不做这一步读数会整体偏移几十个 LSB */
	ADC_StartCalibration(ADC1);
	while(ADC_GetCalibrationStatus(ADC1));
}


/*
	读一次光照
	-----------------------------------------------------------------
	返回值: 0~4095
	内部连续采 8 次求平均:
		单次读数会因为电源毛刺、舵机转动跳几个数, 平均一下曲线就平了。
		8 次总共约 170us, 对 485 应答没有影响(T3.5 断帧靠中断, 不受阻塞影响)。

	三个库函数的分工:
		ADC_SoftwareStartConvCmd(ADC1, ENABLE) : 写 SWSTART, 开始一次转换
		ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)  : 查 EOC 标志, 1 = 转换完成
		ADC_GetConversionValue(ADC1)           : 读 DR, 顺带把 EOC 清 0
*/
u16 Light_Read(void)
{
	u32 sum = 0;
	u8  i;

	for(i = 0 ; i < 8 ; i++)
	{
		ADC_SoftwareStartConvCmd(ADC1, ENABLE);			/* 开始一次转换 */

		while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));	/* 等转换完成 */

		sum += ADC_GetConversionValue(ADC1);			/* 读结果, 同时清 EOC */
	}

	return (u16)(sum / 8);
}
