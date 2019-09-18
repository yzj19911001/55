/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Uart5.c
** Last modified Date:  2011-04-12
** Last Version:		1.0
** Descriptions:		����5��غ���
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		2011-04-12
** Version:				1.0
** Descriptions:		Uart5
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#define	__UART5_C
#include "UART5.h"
#include "JZGlobal.h"





/*********************************************************************************************************
** Function name:		U5SendBytes
** Descriptions:		�򴮿�5����һ���ֽ�����
** input parameters:	p_u8SendBuf		��������ָ�� 
** 						p_u32Len		�������ݳ���
**
** Created by:			ZHANG Ye		  
** Created Date:		20110331	  
**-------------------------------------------------------------------------------------------------------
** Modified by:			ZHANG Ye	
** Modified date:		20110517	
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void U5SendBytes(uint8 * p_u8SendBuf, uint32 p_u32Len)
{
	uint32	l_u32Tmp;

	for (l_u32Tmp = 0; l_u32Tmp < p_u32Len; l_u32Tmp ++)
	{
		//�ȴ����ͼĴ�������ǰ���ݷ������
		while ((U5LSR & 0x40) == 0);	
		
		U5THR = *(p_u8SendBuf+l_u32Tmp);
	}
}

/*********************************************************************************************************
** Function name:		U5ReciveByte
** Descriptions:		�򴮿�5����һ���ֽ�����
** input parameters:	*p_pu8RcvDataBuf	���յ���ָ��					 
** output parameters:	p_u16RcvNum			��������(��ֵΪ0ʱ�����������ݴ���)
** Return Value:		ʵ�ʽ���������
**						����ֵС����Ҫ��������ʱ�������������ݣ����Ƿ���Buf����Ч�ֽڵ�����
**
** Created by:			ZHANG Ye		  
** Created Date:		20110331	  
**-------------------------------------------------------------------------------------------------------
** Modified by:			ZHANG Ye	
** Modified date:		20110518	
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
uint16 U5ReciveByte(uint8 *p_pu8RcvDataBuf, uint8 p_u16RcvNum)
{
	return 0;
}

/*********************************************************************************************************
** Function name:     IRQ_UART5
** Descriptions:      ����5�����жϷ������
** input parameters:  none        
** output parameters: none
**
** Created by:		  
** Created Date:	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void IRQ_UART5(void)
{
	uint32 i = 0;
	uint8 l_u8recvTemp = 0;
	/* 			
	*  UART5�����жϴ��� 			
	*/
	i = (uint32)(U5IIR&0x0f);                                  

	if (i!= 0x01) 
	{		                  					/* �������жϴ�����             */
		switch (i)
		{
		    case 0x04:		
		    case 0x0c:		
			    i	= (uint32)U5RXLEV;				/* ���ж��ﱣ����i����Ч����*/
				for (;i>0;i--) 
				{
					//����2048buff�ٴ�ͷ��ʼ��
					if( ((g_UART5_Recv_Index+1)&(UARTRCVBUFSIZE-1)) == g_UART5_Proc_Index)		/* �����������       */
					{
						l_u8recvTemp = (uint8)(HSU1_RX & 0xff); 			/* �������յ�������    */
					}
					else
					{
						g_u8U5RcvBuf[g_UART5_Recv_Index] = (uint8)(U5RBR & 0xff);  /* �������ݴ�����ջ�����       */
						g_UART5_Recv_Index = (g_UART5_Recv_Index+1)&(UARTRCVBUFSIZE-1);
					}	
				}
				OSSemPost(g_Uart5Recv);	
				break;	                                                    
	        default:
		        break;
		}
	
	}
	
}


/*********************************************************************************************************
** Function name:     UART5Init
** Descriptions:      ���5���ȫ����ʼ������,Baund=Fpcld/[16*(DLM:DLL)]*X/Y
** input parameters:  none        
** output parameters: none
**
** Created by:		  
** Created Date:	  
**-------------------------------------------------------------------------------------------------------
** Modified by:		  		
** Modified date:	  		
**-------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
void UART5_Init(int p_iBaudRate)
{
	/*  
    *  ���ò�����
    */
	U5LCR        = 0x83;                                                /* ʹ�ܲ����ʷ�Ƶ������ķ���   */
	U5DLM        = UARTDLM;                                                 /* ����Ƶ����ֵ�����︳ֵΪ1    */ 
	U5DLL        = UARTDLL; 
	U5LCR        = 0x03;                                                /* ʹ��THR,RBR,IER�Ĵ���        */

	
	switch (p_iBaudRate)
	{	
	case UBR_4800:
		U5CLK        = (1 << 8) | (169 << 0);
		break;
	
	case UBR_9600:
		U5CLK        = (3 << 8) | (254 << 0);
		break;
	
	case UBR_19200:
		U5CLK        = (3 << 8) | (127 << 0);
		break;

	case UBR_38400:
		U5CLK        = (6 << 8) | (127 << 0);
		break;

	case UBR_57600:
		U5CLK        = (9 << 8) | (127 << 0);
		break;

	case UBR_115200:
		U5CLK        = (19 << 8) | (134 << 0);
		break;

	case UBR_230400:
		U5CLK        = (19 << 8) | (67 << 0);
		break;

	case UBR_460800:
		U5CLK        = (38 << 8) | (67 << 0);
		break;

	default:
		U5CLK        = (19 << 8) | (134 << 0);
		break;
  	}
	
	/*
    *  ʹ��UART5�жϣ�����������ʱ��
    */	 
	U5FCR  	     = 0x3f;                   					            /* ʹ��FIFO��������FIFO������� */ //16
	U5FCR  	     = 0xff;
	U5IER        = 0x01;		 					                    /* ʹ��RDA                      */
	UART_CLKMODE |= 0x0200;                                              /* ѡ��UART5��ʱ��ģʽ          */ 
    
	/* 
    *  ��ʼ��UART VIC �жϽӿڣ������ش��� 
    */
	micIrqFuncSet(9, 1, (unsigned int)IRQ_UART5 );             	
}


void UART5_SendBuf_full(uint8  *RcvBufPt,  uint32  Snd_Len)		 	//����ʽ����UART5
{
	 uint8 err;
	 OSSemPend(g_Uart5Send, 250, &err);
	 if(err == OS_NO_ERR)
	 {
		U5SendBytes(RcvBufPt,Snd_Len);
		OSSemPost(g_Uart5Send);
	 }
}
