/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)
**
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_Checknet.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		������������
**
**--------------------------------------------------------------------------------------------------------
** Created by:			Hong XiangYuan
** Created date:		20120718
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
#include "Task_Checknet.h"
#include "WT_Task.h"
#include "socket.h"
#include "w5300.h"
#include "GPIO.h"
#include "UART1.h"
uint8 Flag_NetConnect=0;
uint8 Flag_NetToPC=0;

void  Task_Checknet(void *tdata)
{
    uint32   l_u32Linknum;
    uint8	 JGRdy=0;
    uint8  l_disconnect[25];
    tdata	= tdata;
    OSTimeDly(1250);
    Flag_NetConnect = 0;
//=================================================
    LED_RUN_ON();
//================ ==================================
    while(1)
    {
        if(W5300_LINK) //�͵�ƽ��ʾ��������
        {
            l_u32Linknum++;
            OSTimeDly(5);
            if(l_u32Linknum>2)
            {
                LED_RUN_OFF();
                l_u32Linknum=0;
                JGRdy = 0;
                Flag_NetConnect = 0;	 //��������
                W5300_Init(&g_NetInfo);
                OSTimeDly(1250);		 //��ʱ����ɾ����ɾ�������Ӳ���

                W5300_Socket_Connect(W5300_SOCKET0,S0_IP,S0_DPort);//���Ӽ�����
                W5300_Socket_Connect(W5300_SOCKET1,S1_IP,S1_DPort);//�����ⲿƽ̨������
                W5300_Socket_Connect(W5300_SOCKET2,S2_IP,S2_DPort);//���ӵ�������
                W5300_Socket_Connect(W5300_SOCKET3,S3_IP,S3_DPort);
            }
        }
        else	//���߲�������
        {
            LED_RUN_ON();
            l_u32Linknum=0;

            if(W5300_State_management(W5300_SOCKET0,S0_Port,S0_Mode,S0_IP,S0_DPort))
		      	{
                JGRdy |= 0x01;
			        	Flag_NetConnect=1;
			      }
            else
            {
                Net1_DisconnectNum++;	//JG0 ���������ۼ�
                W5300_Socket_Connect(W5300_SOCKET0,S0_IP,S0_DPort);
                JGRdy &= ~0x01;
						  	l_disconnect[0]= Net1_DisconnectNum/1000 +0x30;
							  l_disconnect[1]= Net1_DisconnectNum%1000/100+0x30;
							  l_disconnect[2]= Net1_DisconnectNum%100/10+0x30;
						 	  l_disconnect[3]= Net1_DisconnectNum%10 +0x30;
							  l_disconnect[4]= '\n';
													
								GetRTCTime(&g_SysCurTime);
								YEAR 	=	g_SysCurTime.u16Year;	//��
								MONTH	=	g_SysCurTime.u8Month;			//��
								DAY		=	g_SysCurTime.u8Day;			//��
								WEEK	=	g_SysCurTime.u8Week;
								HOUR	=	g_SysCurTime.u8Hour;			//ʱ
								MIN		=	g_SysCurTime.u8Minute;		//��
								SEC		=	g_SysCurTime.u8Second;		//��
			  	      l_disconnect[5]= YEAR/1000 +0x30;
							  l_disconnect[6]= YEAR%1000/100+0x30;
							  l_disconnect[7]= YEAR%100/10+0x30;
						 	  l_disconnect[8]= YEAR%10 +0x30;
								l_disconnect[9]='-';
								l_disconnect[10]= MONTH/10 +0x30;
								l_disconnect[11]= MONTH%10 +0x30;
								l_disconnect[12]='-';
								l_disconnect[13]= DAY/10 +0x30;
								l_disconnect[14]= DAY%10 +0x30;
								l_disconnect[15]='-';
								l_disconnect[16]= HOUR/10 +0x30;
								l_disconnect[17]= HOUR%10 +0x30;
								l_disconnect[18]='-';
								l_disconnect[19]= MIN/10 +0x30;
								l_disconnect[20]= MIN%10 +0x30;								
								l_disconnect[21]='-';
								l_disconnect[22]= SEC	/10 +0x30;
								l_disconnect[23]= SEC	%10 +0x30;										
							  l_disconnect[24]= '\n';
							  UART1_SendBuf(l_disconnect,sizeof(l_disconnect));
                OSTimeDly(30);
            }

						
            if(W5300_State_management(W5300_SOCKET1,S1_Port,S1_Mode,S1_IP,S1_DPort))
		      	{
                JGRdy |= 0x02;
			        	Flag_NetToPC = 1;
		      	}
             else
            {
                Net2_DisconnectNum++;//JG1���������ۼ�
                W5300_Socket_Connect(W5300_SOCKET1,S1_IP,S1_DPort);
                JGRdy &= ~0x02;
                OSTimeDly(30);
            }
		
            OSTimeDly(1000);
        }
    }
}
//======================
