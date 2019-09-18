/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			NORFlash.h
** Last modified Date:  20110419
** Last Version:		1.0
** Descriptions:		NORFlash����
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110419
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
#ifndef	__NORFLASH_H
#define	__NORFLASH_H

#include "config.h"

#ifdef	__NORFLASH_C
#define	NOR_EXT
#else
#define	NOR_EXT	extern
#endif

#define		IRAMADDR		0x08020000			//IRAM�л���2K���ݵ��׵�ַ

#define		NORSTARTREC		0x1E2000			//����������¼��ַ���ܹ�8K:0x1E2000~0x1E3FFF
#define		VEHBUFREC		0x190000			//�������ݼ�¼��64B*2048= 256K�� 0x190000~0x1CFFFF

//��������
NOR_EXT	uint8	EraseNORFlash(void);					//��������NORFlash����
NOR_EXT	uint8	EraseBlock(uint32 p_u32BlockID);		//����Block����
NOR_EXT	uint8	EraseSector(uint32 p_u32SectorAddr);	//����Sector����
NOR_EXT uint8	WordProgram(uint32 p_u32Address, uint16 p_u16Data);		//дWORD 
//
////��չ����
////��дFlash��ָ��λ������
NOR_EXT	uint8	ReadNORFlash(uint32 p_u32Address, uint32 p_u32ReadBytes, uint8 * p_pu8Data);
NOR_EXT	uint8	WriteNORFlash(uint32 p_u32Address, uint32 p_u32WriteNumber, uint8 * p_pu8Data);

#endif		//__NORFLASH_H
