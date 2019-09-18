/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			DEFConfig.h
** Last modified Date:  20110718
** Last Version:		1.0
** Descriptions:		���ú궨��
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110718
** Version:				1.0
** Descriptions:		The original version
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			Wang ZiFeng
** Modified date:		20130318
** Version:				2.0
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__DEFCONFIG_H
#define	__DEFCONFIG_H

#define		APP_VERSION		"S1BCKG170106"	   //����汾��yyyy-mm-dd
#define		DEVICEINIT		"DEV20170701"	   //�豸Ĭ��ID
#define		STATIONINIT		"STA20170701"	   //վ����Ĭ��ֵ


#define		SHOWVEHPASSDEBUG 	0			//��ʾ��̬����״̬


#if	YBVERSION >= 30	 
#define		TASKSTACKSIZE		10240	//�����ջ��С
#else								
#define		TASKSTACKSIZE		512
#endif
 


#define		DEBUG_NT			0			//����״̬:1 ���ԣ�	0 �ǵ���   

#endif	//#ifndef	__DEFCONFIG_H
