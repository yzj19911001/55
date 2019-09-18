/*
 * 文件名：ADC.h
 * 描述：ADC.c的头文件
 * 创建日期：2015-3-17
 * 编写：Hudi
 */

#ifndef __ADC_H
#define __ADC_H

#include "config.h"

//全局变量，判断AD的状态标志
extern uint8  g_ADCFlag; //ADC转换完成标志
extern uint8  g_ADCInit; //ADC是否初始化成功标志

//函数
extern void adc_init(void);
extern float Get_ADC_Value(void);
extern void adc_start_conversion(void);

#endif

