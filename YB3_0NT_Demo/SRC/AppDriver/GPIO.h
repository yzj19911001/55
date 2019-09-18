/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			GPIO.h
** Last modified Date:  2011-03-26
** Last Version:		1.0
** Descriptions:		ͨ�ýӿں�����������ȣ�����LCD
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		2011-03-26
** Version:				1.0
** Descriptions:		The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__GPIO_H
#define	__GPIO_H

#include "Config.h"

#ifdef __GPIO_C
#define		GPIO_EXT
#else
#define		GPIO_EXT	extern
#endif

#define		BEEP				1 << 6					/* GPO_07����BEEP���͵�ƽ����   */

#define		BeepOFF()	        P3_OUTP_CLR = BEEP		//������
#define		BeepON()			P3_OUTP_SET = BEEP		//���岻��

#define		LEDBIT				(1<<9)					//GPO_9

#define		LED_RUN_ON()			P3_OUTP_CLR = LEDBIT	//LED��,RUN��
#define		LED_RUN_OFF()			P3_OUTP_SET = LEDBIT	//LED��,RUN��


#define		BDMODE				(P3_INP_STATE & (1<<2))		//���뿪�أ��Ƿ����궨״̬

//#define		LCM_LEDCON			(1<<14)					//���Ʊ��⿪��  GPO14
//#define		LCDRST				(1<<1)					//LCD��λ����   GPO01

#define		DELAY_TIME			100						/* �����ӳ�ʱ��					*/
#define		TICKS_PER_SEC		100						/* ÿ����Tick��,��OS_CFG.Hһ��	*/

//#define		GSTriggerPIN		(((P3_INP_STATE >> 28) & 1) & g_u32SoftGS )		//�ڵ���դ�źţ��ж��Ƿ��ڵ���դ	0:�ڵ�  1:�ɿ�
//#define		LPTriggerPIN		((P3_INP_STATE >> 16) & 1)		//������Ȧ�ź�
//#define		GSErrPIN			((P3_INP_STATE >> 5) & 1)		//��դ�����ź�
//#define		LPErrPIN			((P3_INP_STATE >> 9) & 1)		//��Ȧ�����ź�

//#define		OUTBGON				((P3_INP_STATE >> 4) & 1)		//�ⲿ������ʾ������

//#define     GPO_02              (1<<2)
//#define     GPO_03              (1<<3)
//#define     GPO_07              (1<<7)
//#define     GPO_08              (1<<8)

//#define     GPI_19              (1<<19)
//#define     GPI_22              (1<<22)

#define		DirTriPin			((P3_INP_STATE >> 8) & 1)       //�����Ȧ�����źţ��г�����ƽΪ��
GPIO_EXT	void	Delay(uint32 p_u32TickNumber);				/* ��ʱ����������Ϊ��ʱ��		*/


#endif		//__GPIO_C
