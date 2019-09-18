/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Uart1.c
** Last modified Date:  2012-7-18
** Last Version:		1.0
** Descriptions:		����1��غ���
**
**--------------------------------------------------------------------------------------------------------
** Created by:			Hong Xiangyuan
** Created date:		2012-7-18
** Version:				1.0
** Descriptions:		Uart1
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			
** Modified date:		
** Version:
** Descriptions:
**
*********************************************************************************************************/
#include "Uart1.h"



#define   UART1_INT      26                                             /* UART1�ж�ʹ��λ              */


//uint8     Rcv_Buf[2048];                                                  /* �ַ���������                 */
volatile  uint32         Rcv_Len = 0;
volatile  uint32         Rcv_N   = 0;                                   /* ��¼����������Ϻ��跢���� */

/*********************************************************************************************************
** �������� ��UART1_SendBuf
** �������� ���򴮿ڷ�������
** ��ڲ��� ��uint32  Rcv_N��  �������ݵĸ�����
**            uint8* RcvBufPt�������ַ  
** ���ڲ��� ����
**********************************************************************************************************/
void UART1_SendBuf(uint8  *RcvBufPt,  uint32  Snd_Len)
{
    uint32 i = 0;
    for (i = 0; i < Snd_Len; i++)
	 {                                     /* ʹ�÷���FIFO��������          */
        while((HSU1_LEVEL>>8) == 64);
		HSU1_TX = RcvBufPt[i];
    }
    Snd_Len = 0;
}

/*********************************************************************************************************
** �������� ��IRQ_UART1
** �������� ������1�����жϷ������
** ��ڲ��� ����
** ���ڲ��� ����
*********************************************************************************************************/
void IRQ_UART1 (void)
{
 	volatile static uint32 i = 0;
	uint8 l_u8recvTemp = 0;

    if ((HSU1_IIR & 0x06) == 0x02)
     {                                   																			  /* ����FIFO���ڴ�������ж�     */ 
    
        Rcv_N += (uint32)(HSU1_LEVEL & 0xff) - 1;                      					/* ����һ���ֽ��Բ�����ʱ�ж�,δ*/
																																								/* ������������ж��д���       */          
        for (; i < Rcv_N; i++) {
			if(((g_UART1_Recv_Index+1)&(UARTRCVBUFSIZE-1)) == g_UART1_Proc_Index)
			{
				l_u8recvTemp=(uint8)(HSU1_RX & 0xff);
			}
			else
			{
				l_u8recvTemp=(uint8)(HSU1_RX & 0xff);
				g_u8U1RcvBuf[g_UART1_Recv_Index] = l_u8recvTemp;  /* �������ݴ�����ջ�����       */
				g_UART1_Recv_Index = (g_UART1_Recv_Index+1)&(UARTRCVBUFSIZE-1);				
			}
        }

    } else {                                                            				/* ��������ʱ�ж�               */
        Rcv_N   += (uint32)(HSU1_LEVEL & 0xff);
        Rcv_Len =  Rcv_N;                                               				/* ������յ��������ֽڳ���     */

        for (; i < Rcv_N; i++) {
 			if(((g_UART1_Recv_Index+1)&(UARTRCVBUFSIZE-1)) == g_UART1_Proc_Index)
			{
				l_u8recvTemp=(uint8)(HSU1_RX & 0xff);
			}
			else
			{
				l_u8recvTemp=(uint8)(HSU1_RX & 0xff);
				g_u8U1RcvBuf[g_UART1_Recv_Index] = l_u8recvTemp;  /* �������ݴ�����ջ�����       */
				g_UART1_Recv_Index = (g_UART1_Recv_Index+1)&(UARTRCVBUFSIZE-1);				
			}
	    }
        
        i     = 0;
        Rcv_N = 0;                                                      				/* ������ճ���                 */
 	   OSSemPost(g_Uart1Recv);
	}
}

/*********************************************************************************************************
** �������� ��UART1_Init
** �������� �����1���ȫ����ʼ������,Baund=Fpclk/[14*(HSU1_RATE+1)]
** ��ڲ��� ����
** ���ڲ��� ����
*********************************************************************************************************/
void UART1_Init (uint32 p_iBaudRate)
{
	//��MOS���� ��UART1����
	P3_OUTP_CLR |= (0x01 << 3);
	
    MIC_ER       |=  0xC0000003; 
    HSU1_IIR     |= 0xFF;
   /* 
    *  ���ò�����	 7 115200; 15 57600; 23 38400; 47 19200; 96 9600; 192 4800; 386 2400;
    */
	switch (p_iBaudRate)
	{
	case UBR_2400:
		HSU1_RATE        = 386;
		break;
				
	case UBR_4800:
		HSU1_RATE        = 192;
		break;
	
	case UBR_9600:
		HSU1_RATE        = 96;
		break;
	
	case UBR_19200:
		HSU1_RATE        = 47;
		break;

	case UBR_38400:
		HSU1_RATE        = 23;
		break;

	case UBR_57600:
		HSU1_RATE        = 15;
		break;

	case UBR_115200:
		HSU1_RATE        = 7;
		break;

	default:
		HSU1_RATE        = 7;
		break;
  	}


    //HSU1_RATE    = 0x07;                                                
    
   /*
    *  ʹ�ܽ����жϣ�����ʱ��ƫ�������շ��������
    */
//    HSU1_CTRL    = 0x00012848;       //1E   20
	HSU1_CTRL    = 0x00012850;        										//20130519                           
    //						0001	0010	1000	0100	1000
   /* 
    *  ��ʼ��UART VIC �жϽӿڣ������ش��� 
    */
    micIrqFuncSet(UART1_INT, 1, (unsigned int)IRQ_UART1);             
}

void UART1_SendBuf_full(uint8  *RcvBufPt,  uint32  Snd_Len)		 	//����ʽ����UART1
{
	 uint8 err;
	 OSSemPend(g_Uart1Send, 250, &err);
	 if(err == OS_NO_ERR)
	 {
		UART1_SendBuf(RcvBufPt,Snd_Len);
		OSSemPost(g_Uart1Send);
	 }
}
