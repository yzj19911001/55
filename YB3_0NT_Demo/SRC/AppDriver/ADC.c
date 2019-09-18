/*
 * �ļ�����ADC.c
 * ������ʵ��AD�ɼ���ѹ��ع���
 * �������ڣ�2015-3-17
 * ��д��Hudi
 */
#include "ADC.h"
#define ADC_INT  7

#define MAX_SAMPLES 4	  //ѭ�������С
static int32  adc_data[MAX_SAMPLES];  //ѭ�����壬�洢ת����ADֵ
static uint32 adc_index;	//ѭ����������
uint8  g_ADCFlag = 1; //ADCת����ɱ�־
uint8  g_ADCInit;	//ADC�Ƿ��ʼ���ɹ���־


/*
 *��������IRQ_ADC
 *���ܣ�ADC�жϷ��������洢ת����ADֵ
 */
void IRQ_ADC (void)
{
   if(adc_index<MAX_SAMPLES)
   {
	  adc_data[adc_index] = (ADC_VALUE & 0x000003FF);	//��ʮλ��Ч
   }
   else
   {
	  adc_index = 0;
	  adc_data[adc_index] = (ADC_VALUE & 0x000003FF);	//��ʮλ��Ч
   }
   adc_index++;
   g_ADCFlag = 1; //ת�����
}


/*
 *��������adc_init
 *���ܣ�adc��ʼ��
 */
void adc_init(void)
{	
	//ADʱ������
	ADCLK_CTRL1 =  0x0000010C;   //����AD�ķ�Ƶ����ʱ��Դ��ѡ��ʱ��ԴΪFpclk=13M��13��ƵΪ1M���ADʹ��,��Ƶ��value=divider-1
	ADCLK_CTRL  =  0x00000001;	 //ʹ��ADʱ��

	//��ADģ���ϵ�
	ADC_SELECT  =  0x00000284;  //����ΪĬ��ֵ
	ADC_CTRL	=  0x00000004;	//bit2Ϊ1������λΪ0����ADCģ��

	//ADͨ��ѡ��
	ADC_SELECT  |= 0x00000020;  //ѡ��ͨ��2Ϊ����ͨ��	

 	sic1IrqFuncSet(ADC_INT, 1, (unsigned int)IRQ_ADC);
	g_ADCInit = 1;  //adc��ʼ���ɹ���־

}


/*
 *��������Get_ADC_Value
 *���ܣ����ݻ����е�ADֵ����ʵ�ʵ�ѹֵ
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

//����˵��������AD����������ֵ��Сʮ����ʵ�ʵ�ѹֵ��Ҫ����10
//ADֵ����4�β�����ƽ������������Ϊ(adcSum/adcCnt)/1024*3.3V*20.53

   if(adcCnt)
   	  value = (float)(adcSum*3.3*10512)/(adcCnt*1024*512); //�����õ���ADֵ,
  
   return  value;
}

/*
 *��������adc_start_conversion
 *���ܣ���������һ��ADת��
 */
void adc_start_conversion(void)
{
   ADC_CTRL |= 0x00000002;
}

