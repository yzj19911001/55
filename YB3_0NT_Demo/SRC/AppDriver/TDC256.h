/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			TDC256.h
** Last modified Date:  20110517
** Last Version:		1.0
** Descriptions:		����
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110517
** Version:				1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#ifndef __TDC256_H

#define __TDC256_H

#ifdef __TDC256_C
#define	TD_EXT
#include "I2C1.h"
#else
#define	TD_EXT	extern
#endif

#include "config.h"

#define     SYS_SET           0x0100         //ϵͳ����
#define		MDL_SET			  0x0200		 //ģ�Ͳ�����ַ
#define		MDL_BAK			  0x0300		 //ģ�Ͳ������ݵ�ַ
#define     CTR_NETSET        0x0450         //�������������
#define     CTR_UARTSET       0x0500         //����������
#define     CTR_RESET		  0x0550         //����������  
#define     CTR_VEHID         0x0600         //����ID��¼  
#define     VEH_OUTTH         0x0650         //������β��ֵ����
#define     WEI_ATSET         0x0700         //��̨��������


TD_EXT	uint8	ReadC256(uint16 p_u16Addr, uint8 * p_pu8ReadBuf, uint16 p_u16Len);
TD_EXT	uint8	WriteC256(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len);

uint8 Read256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len);	 //��ȡ�����ͬ������
uint8 Write256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len); 	 //д�����ͬ������
uint8 Clear256_full(uint16 p_u16Addr,uint16 p_u16Len);//�������ָ����ַָ��λ���ռ�

#endif		//__TDC256_H
