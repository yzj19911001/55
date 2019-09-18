/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			TDC256.c
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
#define	__TDC256_C
#include "TDC256.h"

#define		TDC256_ADDR_WRITE		0xA0
#define		TDC256_ADDR_READ		0xA1

/*********************************************************************************************************
** Function name:		ReadC256
** Descriptions:		读铁电数据
** input parameters:	p_u16Addr	读取数据的首地址
**						p_u16Len	读取数据长度 
** output parameters:	p_pu8ReadBuf	读取数据Buf
**
** Created by:			ZHANG Ye		  
** Created Date:		20110517	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 ReadC256(uint16 p_u16Addr, uint8 * p_pu8ReadBuf, uint16 p_u16Len)
{
	return	I2C1_ReadNByte(TDC256_ADDR_READ, 2, p_u16Addr, p_pu8ReadBuf, p_u16Len);
}

/*********************************************************************************************************
** Function name:		WriteC256
** Descriptions:		读铁电数据
** input parameters:	p_u16Addr	写数据的首地址
**						p_u16Len	写数据长度 
** 						p_pu8WriteBuf	写数据Buf
**
** Created by:			ZHANG Ye		  
** Created Date:		20110517	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 WriteC256(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len)
{
	return	I2C1_WriteNByte(TDC256_ADDR_WRITE, 2, p_u16Addr, p_pu8WriteBuf, p_u16Len);
}


/*******************************************************/
uint8 Read256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len)
{
	uint8 err;
	uint8 ret;
	OSSemPend(g_TdOpMutex,5000,&err);
	if(err==0)
	{
		ret = ReadC256(p_u16Addr,p_pu8WriteBuf,p_u16Len);
		OSSemPost(g_TdOpMutex);
	}
	return ret;
}

uint8 Write256_full(uint16 p_u16Addr, uint8 * p_pu8WriteBuf, uint16 p_u16Len)
{
	uint8 err;
	uint8 ret; 
	OSSemPend(g_TdOpMutex,5000,&err);
	if(err==0)
	{
		ret = WriteC256(p_u16Addr,p_pu8WriteBuf,p_u16Len);
		OSSemPost(g_TdOpMutex);
	}
	return ret;
}


uint8 Clear256_full(uint16 p_u16Addr,uint16 p_u16Len)
{
	uint8 err;
	uint8 *p_pu8WriteBuf=NULL;
	p_pu8WriteBuf=(uint8 *)malloc(sizeof(uint8)*p_u16Len);
	if(p_pu8WriteBuf!=NULL)
	{
	
		memset(p_pu8WriteBuf,0,p_u16Len);
		OSSemPend(g_TdOpMutex,5000,&err);
		if(err==0)
		{
			WriteC256(p_u16Addr,p_pu8WriteBuf,p_u16Len);
			OSSemPost(g_TdOpMutex);
		}
		return 1;
	}
	else
	{
		return 0;
	}
	
}
