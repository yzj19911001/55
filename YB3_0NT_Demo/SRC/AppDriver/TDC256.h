/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			TDC256.h
** Last modified Date:  20110517
** Last Version:		1.0
** Descriptions:		铁电
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

#define     SYS_SET           0x0100         //系统设置
#define		MDL_SET			  0x0200		 //模型参数地址
#define		MDL_BAK			  0x0300		 //模型参数备份地址
#define     CTR_NETSET        0x0450         //控制器网络参数
#define     CTR_UARTSET       0x0500         //控制器串口
#define     CTR_RESET		  0x0550         //控制器重启  
#define     CTR_VEHID         0x0600         //车辆ID记录  
#define     VEH_OUTTH         0x0650         //车辆收尾阈值设置
#define     WEI_ATSET         0x0700         //称台交互设置


TD_EXT	uint8	ReadC256(uint16 p_u16Addr, uint8 * p_pu8ReadBuf, uint16 p_u16Len);
TD_EXT	uint8	WriteC256(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len);

uint8 Read256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len);	 //读取铁电的同步方法
uint8 Write256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len); 	 //写铁电的同步方法
uint8 Clear256_full(uint16 p_u16Addr,uint16 p_u16Len);//清空铁电指定地址指定位数空间

#endif		//__TDC256_H
