/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Timer0.h
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
#ifndef	__TIMER0_H
#define	__TIMER0_H

#include "config.h"

#ifdef	__TIMER0_C
#define	T0_EXT
#include "WDT.h"
#else
#define	T0_EXT	extern
#endif

//��ʼ��
static	void IRQ_Timer0(void);		//�жϴ���
T0_EXT	void Time0Init(void);


#endif		//__TIMER0_H
