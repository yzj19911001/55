/*
 * �ļ�����ADC.h
 * ������ADC.c��ͷ�ļ�
 * �������ڣ�2015-3-17
 * ��д��Hudi
 */

#ifndef __ADC_H
#define __ADC_H

#include "config.h"

//ȫ�ֱ������ж�AD��״̬��־
extern uint8  g_ADCFlag; //ADCת����ɱ�־
extern uint8  g_ADCInit; //ADC�Ƿ��ʼ���ɹ���־

//����
extern void adc_init(void);
extern float Get_ADC_Value(void);
extern void adc_start_conversion(void);

#endif

