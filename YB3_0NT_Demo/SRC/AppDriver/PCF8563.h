/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			PCF8563.h
** Last modified Date:  20110503
** Last Version:		1.0
** Descriptions:		ʵʱʱ�����
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110503
** Version:				1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			ZHANG Ye			
** Modified date:		20110518		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef __PCF8563_H

#define __PCF8563_H

#ifdef __PCF8563_C
#define	PCF_EXT	
#include "I2C2.h"
#else
#define	PCF_EXT	extern
#endif

#include "config.h"

typedef		SystemTime		RTCDateTime;

PCF_EXT	void	RTC8563Init(void);						//RTC��8563��ʼ��
PCF_EXT	void	SetRTCTime(SystemTime * p_pstTime);		//����RTCʱ��
PCF_EXT	void	GetRTCTime(SystemTime * p_pstTime);		//��ȡRTCʱ�� 
PCF_EXT	void	Synchro8563(void);						//RTC��8563ͬ��

#endif		//__PCF8563_H
