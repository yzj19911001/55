/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			NORFlash.c
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
#define	__NORFLASH_C
#include "NORFlash.h"
				 
#define  FLASH_ADDR     0xe0000000                                      // NOR Flash ����BANK0��
#define  GetAddr(addr)	(volatile uint16  *)(FLASH_ADDR|(addr << 1))	//��Ե�ַת��Ϊ���Ե�ַ

/*********************************************************************************************************
** Function name:		EraseNORFlash
** Descriptions:		��������NORFlash����
**
** input parameters:	None
** output parameters:	None
** Returned value:		�������	TRUE:�ɹ��� FALSE��ʧ��
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 EraseNORFlash(void)
{
	volatile	uint16 *	l_pu16Temp;
				uint16		l_u16Temp1;
				uint16		l_u16Temp2;
	
    l_pu16Temp		= GetAddr(0x5555);
    l_pu16Temp[0]	= 0xaa;						/* ��һ������,��ַ0x5555,����0xAA*/
    l_pu16Temp		= GetAddr(0x2aaa);
    l_pu16Temp[0]	= 0x55;						/* �ڶ�������,��ַ0x2aaa,����0x55*/
    l_pu16Temp		= GetAddr(0x5555);
    l_pu16Temp[0]	= 0x80;						/* ����������,��ַ0x5555,����0x80*/
    l_pu16Temp		= GetAddr(0x5555);
    l_pu16Temp[0]	= 0xaa;						/* ���ĸ�����,��ַ0x5555,����0xAA*/
    l_pu16Temp		= GetAddr(0x2aaa);
    l_pu16Temp[0]	= 0x55;						/* ���������,��ַ0x2aaa,����0x55*/
    l_pu16Temp		= GetAddr(0x5555);
    l_pu16Temp[0]	= 0x10;						/* ����������,��ַ0x5555,����0x10*/
    /* 
     *  �ȴ�������� (����������û����ɣ�ÿ�ζ�����DQ6������) 
     */
    while (1)				
    {  
        l_u16Temp1	= *l_pu16Temp;
        l_u16Temp2	= *l_pu16Temp;
        if (l_u16Temp1 == l_u16Temp2)
        {  
            if (l_u16Temp1 != 0xffff)
                return(FALSE);
            else
                return(TRUE);
        }
    }
}

/*********************************************************************************************************
** Function name:		EraseBlock
** Descriptions:		����Block����
**
** input parameters:	p_u32BlockID
** output parameters:	None
** Returned value:		�������	TRUE:�ɹ��� FALSE��ʧ��
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 EraseBlock(uint32 p_u32BlockID)
{
	volatile	uint16 *	l_pu16Temp;
				uint16		l_u16Temp1;
				uint16		l_u16Temp2;
	
    l_pu16Temp		= GetAddr(0x5555);
    l_pu16Temp[0]	= 0xaa;						/* ��һ������,��ַ0x5555,����0xAA*/
    l_pu16Temp		= GetAddr(0x2aaa);
    l_pu16Temp[0]	= 0x55;						/* �ڶ�������,��ַ0x2aaa,����0x55*/
    l_pu16Temp		= GetAddr(0x5555);
    l_pu16Temp[0]	= 0x80;						/* ����������,��ַ0x5555,����0x80*/
    l_pu16Temp		= GetAddr(0x5555);
    l_pu16Temp[0]	= 0xaa;						/* ���ĸ�����,��ַ0x5555,����0xAA*/
    l_pu16Temp		= GetAddr(0x2aaa);
    l_pu16Temp[0]	= 0x55;						/* ���������,��ַ0x2aaa,����0x55*/
    l_pu16Temp		= GetAddr(p_u32BlockID>>1);
//#ifdef	__ZLG
    l_pu16Temp[0]	= 0x5050;						/* ����������,���ַ,����0x50  */
//#else				//��������
//	l_pu16Temp[0]	= 0x3030;						/* ����������,���ַ,����0x30  */
//#endif
    
    /* 
     *  �ȴ�������� (����������û����ɣ�ÿ�ζ�����DQ6������) 
     */
    while (1)				
    {  
        l_u16Temp1	= *l_pu16Temp;
        l_u16Temp2	= *l_pu16Temp;
        if (l_u16Temp1 == l_u16Temp2)
        {  
            if (l_u16Temp1 != 0xffff)
                return(FALSE);
            else
                return(TRUE);
        }
    }
}

/*********************************************************************************************************
** Function name:		EraseSector
** Descriptions:		����Sector����
**
** input parameters:	p_u32SectorAddr
** output parameters:	None
** Returned value:		�������	TRUE:�ɹ��� FALSE��ʧ��
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 EraseSector(uint32 p_u32SectorAddr)
{
	volatile	uint16 *	l_pu16Temp;
				uint16		l_u16Temp1;
				uint16		l_u16Temp2;
	
    l_pu16Temp		= GetAddr(0x5555);
    l_pu16Temp[0]	= 0xaa;						/* ��һ������,��ַ0x5555,����0xAA*/
    l_pu16Temp		= GetAddr(0x2aaa);
    l_pu16Temp[0]	= 0x55;						/* �ڶ�������,��ַ0x2aaa,����0x55*/
    l_pu16Temp		= GetAddr(0x5555);
    l_pu16Temp[0]	= 0x80;						/* ����������,��ַ0x5555,����0x80*/
    l_pu16Temp		= GetAddr(0x5555);
    l_pu16Temp[0]	= 0xaa;						/* ���ĸ�����,��ַ0x5555,����0xAA*/
    l_pu16Temp		= GetAddr(0x2aaa);
    l_pu16Temp[0]	= 0x55;						/* ���������,��ַ0x2aaa,����0x55*/
    l_pu16Temp		= GetAddr(p_u32SectorAddr>>1);
//#ifdef	__ZLG
    l_pu16Temp[0]	= 0x3030;						/* ����������,���ַ,����0x30  */
//#else				//��������
//	l_pu16Temp[0]	= 0x5050;						/* ����������,���ַ,����0x50  */
//#endif
    /* 
     *  �ȴ�������� (����������û����ɣ�ÿ�ζ�����DQ6������) 
     */
    while (1)				
    {  
        l_u16Temp1	= *l_pu16Temp;
        l_u16Temp2	= *l_pu16Temp;
        if (l_u16Temp1 == l_u16Temp2)
        {  
            if (l_u16Temp1 != 0xffff)
                return(FALSE);
            else
                return(TRUE);
        }
    }
}

/*********************************************************************************************************
** Function name:		WordProgram
** Descriptions:		дWORD
**
** input parameters:	p_u32Address	��ַ
**						p_u16Data		����
** output parameters:	None
** Returned value:		�������	TRUE:�ɹ��� FALSE��ʧ��
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 WordProgram(uint32 p_u32Address, uint16 p_u16Data)
{
	volatile	uint16 *	l_pu16Temp;
				uint16		l_u16Temp1;
				uint16		l_u16Temp2;
	
	l_pu16Temp		= GetAddr(0x5555);
	l_pu16Temp[0]	= 0xaa;						/* ��һ������,��ַ0x5555,����0xAA*/
	l_pu16Temp		= GetAddr(0x2aaa);
	l_pu16Temp[0]	= 0x55;						/* �ڶ�������,��ַ0x2aaa,����0x55*/
	l_pu16Temp		= GetAddr(0x5555);
	l_pu16Temp[0]	= 0xa0;						/* ����������,��ַ0x5555,����0x80*/
	
	l_pu16Temp		= (uint16 *)(FLASH_ADDR|(p_u32Address & 0x1FFFFF));
	*l_pu16Temp		= p_u16Data;					/* ���ĸ�����,��ַ,����  */
    
	/* 
	 *  �ȴ�������� (����������û����ɣ�ÿ�ζ�����DQ6������) 
	 */
	while (1)				
	{  
		l_u16Temp1	= *l_pu16Temp;
		l_u16Temp2	= *l_pu16Temp;
		if (l_u16Temp1 == l_u16Temp2)
		{  
			if (l_u16Temp1 != p_u16Data)
				return(FALSE);
			else
				return(TRUE);
		}
	}
}

/*********************************************************************************************************
** Function name:		ReadNORFlash
** Descriptions:		��Flash��ָ��λ�����ݣ�ÿ��λ��ȡ
**
** input parameters:	p_u32Address	���NOR�׵�ַ��ƫ�Ƶ�ַ0~0x1fffff
**						p_u32ReadBytes	��ȡ�ֽ���
**						p_pu8Data		����ָ��
** output parameters:	None
** Returned value:		�������	TRUE:�ɹ��� FALSE��ʧ��
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 ReadNORFlash(uint32 p_u32Address, uint32 p_u32ReadBytes, uint8 * p_pu8Data)
{
	uint32		i;
	uint32 *	l_pu32Data;

	l_pu32Data	= (uint32 *)p_pu8Data;

	for (i = 0; i < p_u32ReadBytes; i += 4)
	{
		//��NORFlash
		*(l_pu32Data ++)	= *(uint32 *)(FLASH_ADDR|((p_u32Address+i) & 0x1FFFFF));
	}
	return	TRUE;
}

/*********************************************************************************************************
** Function name:		WriteNORFlash
** Descriptions:		дFlash��ָ��λ������
**
** input parameters:	p_u32Address	���NOR�׵�ַ��ƫ�Ƶ�ַ0~0x1fffff
**						p_u32WriteNumber	��ȡ�ֽ���
** output parameters:	p_u16Data		����ָ��
** Returned value:		�������	TRUE:�ɹ��� FALSE��ʧ��
**
** Created by:			ZHANG Ye
** Created Date:		20110419
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint8 WriteNORFlash(uint32 p_u32Address, uint32 p_u32WriteNumber, uint8 * p_pu8Data)
{
	uint32		i;
	uint16 *	l_pu16Data;
	uint8		l_u8Res;
	
	l_u8Res		= TRUE;

#ifdef	__UCOS		//��UCOS�У�дNORFlashʱ���жϣ���ֹдFlash�����б����
	OS_ENTER_CRITICAL();
#endif

	l_pu16Data	= (uint16 *)p_pu8Data;
	
//	EraseSector(p_u32Address);		//����һ�������������ܶ�����ռ����д�������ռ�Ϊ2K	
	
	for (i = 0; i < p_u32WriteNumber; i += 2)
	{
		while(FALSE == WordProgram(p_u32Address + i, *(l_pu16Data + (i >> 1))));		//������д��NORFlash
	}
	
#ifdef	__UCOS
	OS_EXIT_CRITICAL();
#endif

	return	l_u8Res;
}

