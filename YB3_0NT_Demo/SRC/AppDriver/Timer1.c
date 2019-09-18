/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)
**
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Timer1h.c
** Last modified Date:  2012-5-9
** Last Version:		1.0
** Descriptions:		��ʱ��1��غ���
**
**--------------------------------------------------------------------------------------------------------
** Created by:			Hongxy
** Created date:		2
** Version:
** Descriptions:		Timer1
**
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__TIMER1_C
#include "Timer1.h"
#include "Common.h"
#include "UART5.h"
#include "socket.h"
#include "Task_Checknet.h"

static	void IRQ_Timer1(void);		//�жϴ���
__align(2)   uint8 RdToLMSSendData[24] = {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x0F,0x73,0x52,0x4E,0x20,0x4C,0x4D,0x44,0x73,0x63,0x61,0x6E,0x64,0x61,0x74,0x61,0x05};
__align(2)  uint8 LianxuLMSSendData[26]= {0x02,0x02,0x02,0x02,0x00,0x00,0x00,0x11,0x73,0x45,0x4E,0x20,0x4C,0x4D,0x44,0x73,0x63,0x61,0x6E,0x64,0x61,0x74,0x61,0x20,0x01,0x33};

/****************HH*****************************************************************************************
** ��������:  IRQ_Timer1
** ��������:  ��ʱ��1�жϷ������
** ��ڲ���:  ��
** ���ڲ���:  ��
** ����˵��:  20ms����һ���ж�
*********************************************************************************************************/
void IRQ_Timer1(void)
{

	T1IR			= 0x01;					                /* ����жϱ�־	                */

	if(Flag_NetConnect==1)
	{

		//���ͼ���0��������
		S_tx_process(0,RdToLMSSendData, 24);  
		
	}


}

/***************************************************************************************
**
**   Time1��ʼ������
**
****************************************************************************************/
void Time1Init(void)
{
	MIC_ER			|= (1<<17);									//ʹ��Timer0�ж�
	//�жϿ��ƼĴ�����Book��P.142��
	//
	TIMCLK_CTRL1	= TIMCLK_CTRL1|0x08;     //P.131            /* ʹ�ܶ�ʱ��1��ʱ��            */
	T1TCR			= 0x02;                           			/* ��λ�����ܶ�ʱ��1            */
	T1IR			= 0xFF;                           			/* ��������ж�                 */
	T1TC			= 0x00000000;//��ʱ��������					/* ��ʱ������Ϊ0                */
	T1PR			= 0x0000000F;						 		/* ʱ��16��Ƶ                   */
	T1PC			= 0x00000000;
	T1MCR			= 0x00000003;						       		/* ����T1MR0ƥ���λT1TC��    */
	/* �������жϱ�־               */
	//T1TC��ʱ����������ͨ��������ֵ�����Լ��㾫ȷ��ʱ�䣻
	//ϵͳ13MHz,ÿ1������13M����
	//16��Ƶ��ÿ16������������1��
	//����һ���ڣ�������������13M/16������13M/16����������
	T1CTCR			= 0x00;
	T1MR0			= Fpclk / (16*50);			/* 20ms��ʱ    10ms��ʱΪ(16*100)                */
	//			(13M/16)/OS_TICKS_PER_SEC ������һ���ڵļ������ڳ���OS_TICKS_PER_SEC��

	T1TCR			= 0x01;						   				/* ������ʱ��1                  */

	micIrqFuncSet(17, 0, (unsigned int)IRQ_Timer1);				/* �����ж�������				*/
	//17��ʱ��1�жϣ�16��ʱ��0�жϣ�
}
