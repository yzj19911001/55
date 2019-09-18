/*
 * 文件名：ADC.c
 * 描述：实现AD采集电压相关功能
 * 创建日期：2015-3-17
 * 编写：Hudi
 */
#include "ADC.h"
#define ADC_INT  7

#define MAX_SAMPLES 4	  //循环缓冲大小
static int32  adc_data[MAX_SAMPLES];  //循环缓冲，存储转换的AD值
static uint32 adc_index;	//循环缓冲索引
uint8  g_ADCFlag = 1; //ADC转换完成标志
uint8  g_ADCInit;	//ADC是否初始化成功标志


/*
 *函数名：IRQ_ADC
 *功能：ADC中断服务函数，存储转换的AD值
 */
void IRQ_ADC (void)
{
   if(adc_index<MAX_SAMPLES)
   {
	  adc_data[adc_index] = (ADC_VALUE & 0x000003FF);	//低十位有效
   }
   else
   {
	  adc_index = 0;
	  adc_data[adc_index] = (ADC_VALUE & 0x000003FF);	//低十位有效
   }
   adc_index++;
   g_ADCFlag = 1; //转换完成
}


/*
 *函数名：adc_init
 *功能：adc初始化
 */
void adc_init(void)
{	
	//AD时钟设置
	ADCLK_CTRL1 =  0x0000010C;   //设置AD的分频比与时钟源，选择时钟源为Fpclk=13M，13分频为1M后给AD使用,分频数value=divider-1
	ADCLK_CTRL  =  0x00000001;	 //使能AD时钟

	//给AD模块上电
	ADC_SELECT  =  0x00000284;  //设置为默认值
	ADC_CTRL	=  0x00000004;	//bit2为1，其它位为0，打开ADC模块

	//AD通道选择
	ADC_SELECT  |= 0x00000020;  //选择通道2为采样通道	

 	sic1IrqFuncSet(ADC_INT, 1, (unsigned int)IRQ_ADC);
	g_ADCInit = 1;  //adc初始化成功标志

}


/*
 *函数名：Get_ADC_Value
 *功能：根据缓冲中的AD值计算实际电压值
 */
float Get_ADC_Value(void)
{
   uint16 i;
   int16  adcCnt = 0;
   int16  adcSum = 0;
   float  value = 0.0;
   for(i=0;i<MAX_SAMPLES;i++)
   {
     if(adc_data[i] != 0)
	 {
	   adcSum += adc_data[i];
	   adcCnt++;
	 }
   }

//计算说明：由于AD采样进来的值缩小十倍，实际电压值需要乘以10
//AD值采用4次采样求平均，整个计算为(adcSum/adcCnt)/1024*3.3V*20.53

   if(adcCnt)
   	  value = (float)(adcSum*3.3*10512)/(adcCnt*1024*512); //采样得到的AD值,
  
   return  value;
}

/*
 *函数名：adc_start_conversion
 *功能：开启进行一次AD转换
 */
void adc_start_conversion(void)
{
   ADC_CTRL |= 0x00000002;
}

