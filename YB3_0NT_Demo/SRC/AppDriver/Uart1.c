/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Uart1.c
** Last modified Date:  2012-7-18
** Last Version:		1.0
** Descriptions:		串口1相关函数
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



#define   UART1_INT      26                                             /* UART1中断使能位              */


//uint8     Rcv_Buf[2048];                                                  /* 字符接收数组                 */
volatile  uint32         Rcv_Len = 0;
volatile  uint32         Rcv_N   = 0;                                   /* 记录接收数据完毕后，需发送数 */

/*********************************************************************************************************
** 函数名称 ：UART1_SendBuf
** 函数功能 ：向串口发送数据
** 入口参数 ：uint32  Rcv_N：  发送数据的个数。
**            uint8* RcvBufPt：缓存地址  
** 出口参数 ：无
**********************************************************************************************************/
void UART1_SendBuf(uint8  *RcvBufPt,  uint32  Snd_Len)
{
    uint32 i = 0;
    for (i = 0; i < Snd_Len; i++)
	 {                                     /* 使用发送FIFO发送数据          */
        while((HSU1_LEVEL>>8) == 64);
		HSU1_TX = RcvBufPt[i];
    }
    Snd_Len = 0;
}

/*********************************************************************************************************
** 函数名称 ：IRQ_UART1
** 函数功能 ：串口1接收中断服务程序
** 入口参数 ：无
** 出口参数 ：无
*********************************************************************************************************/
void IRQ_UART1 (void)
{
 	volatile static uint32 i = 0;
	uint8 l_u8recvTemp = 0;

    if ((HSU1_IIR & 0x06) == 0x02)
     {                                   																			  /* 接收FIFO高于触发深度中断     */ 
    
        Rcv_N += (uint32)(HSU1_LEVEL & 0xff) - 1;                      					/* 留下一个字节以产生超时中断,未*/
																																								/* 保存的数据在中断中处理       */          
        for (; i < Rcv_N; i++) {
			if(((g_UART1_Recv_Index+1)&(UARTRCVBUFSIZE-1)) == g_UART1_Proc_Index)
			{
				l_u8recvTemp=(uint8)(HSU1_RX & 0xff);
			}
			else
			{
				l_u8recvTemp=(uint8)(HSU1_RX & 0xff);
				g_u8U1RcvBuf[g_UART1_Recv_Index] = l_u8recvTemp;  /* 接收数据存入接收缓冲区       */
				g_UART1_Recv_Index = (g_UART1_Recv_Index+1)&(UARTRCVBUFSIZE-1);				
			}
        }

    } else {                                                            				/* 接收器超时中断               */
        Rcv_N   += (uint32)(HSU1_LEVEL & 0xff);
        Rcv_Len =  Rcv_N;                                               				/* 保存接收到的数据字节长度     */

        for (; i < Rcv_N; i++) {
 			if(((g_UART1_Recv_Index+1)&(UARTRCVBUFSIZE-1)) == g_UART1_Proc_Index)
			{
				l_u8recvTemp=(uint8)(HSU1_RX & 0xff);
			}
			else
			{
				l_u8recvTemp=(uint8)(HSU1_RX & 0xff);
				g_u8U1RcvBuf[g_UART1_Recv_Index] = l_u8recvTemp;  /* 接收数据存入接收缓冲区       */
				g_UART1_Recv_Index = (g_UART1_Recv_Index+1)&(UARTRCVBUFSIZE-1);				
			}
	    }
        
        i     = 0;
        Rcv_N = 0;                                                      				/* 清零接收长度                 */
 	   OSSemPost(g_Uart1Recv);
	}
}

/*********************************************************************************************************
** 函数名称 ：UART1_Init
** 函数功能 ：令串口1完成全部初始化工作,Baund=Fpclk/[14*(HSU1_RATE+1)]
** 入口参数 ：无
** 出口参数 ：无
*********************************************************************************************************/
void UART1_Init (uint32 p_iBaudRate)
{
	//打开MOS开关 给UART1供电
	P3_OUTP_CLR |= (0x01 << 3);
	
    MIC_ER       |=  0xC0000003; 
    HSU1_IIR     |= 0xFF;
   /* 
    *  设置波特率	 7 115200; 15 57600; 23 38400; 47 19200; 96 9600; 192 4800; 386 2400;
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
    *  使能接收中断，设置时钟偏移量和收发触发深度
    */
//    HSU1_CTRL    = 0x00012848;       //1E   20
	HSU1_CTRL    = 0x00012850;        										//20130519                           
    //						0001	0010	1000	0100	1000
   /* 
    *  初始化UART VIC 中断接口，上升沿触发 
    */
    micIrqFuncSet(UART1_INT, 1, (unsigned int)IRQ_UART1);             
}

void UART1_SendBuf_full(uint8  *RcvBufPt,  uint32  Snd_Len)		 	//独享方式操作UART1
{
	 uint8 err;
	 OSSemPend(g_Uart1Send, 250, &err);
	 if(err == OS_NO_ERR)
	 {
		UART1_SendBuf(RcvBufPt,Snd_Len);
		OSSemPost(g_Uart1Send);
	 }
}
