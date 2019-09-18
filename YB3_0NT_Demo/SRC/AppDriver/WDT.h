/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			WDT.h
** Last modified Date:  2011-04-12
** Last Version:		1.0
** Descriptions:		Timer0��ʼ�����жϴ���
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__WDT_H
#define	__WDT_H

#include "config.h"

#ifdef	__WDT_C
#define	WDT_EXT
#else
#define	WDT_EXT	extern
#endif

//��ʼ���ڲ���
WDT_EXT	void WDTInit(uint8 start);

//�ⲿWDT
WDT_EXT	uint8			g_u8WDTFlag;

#define	OUTWDT	(1<<17)			//�ⲿ��
#define	FeedExternDog()		if(g_u8WDTFlag==0)			\
								P3_OUTP_SET = OUTWDT;	\
							else						\
								P3_OUTP_CLR = OUTWDT;	\
							g_u8WDTFlag	^= 0x01;

#endif		//__WDT_H
