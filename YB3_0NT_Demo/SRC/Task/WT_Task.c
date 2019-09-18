/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)
**
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			WT_Task.C
** Last modified Date:  20110511
** Last Version:		1.0
** Descriptions:		��������
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110511
** Version:				1.0
** Descriptions:
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			Hong Xiang Yuan
** Modified date:		20120718
** Version:
** Descriptions:		TaskStart;��������
**
*********************************************************************************************************/
#define __JZGLOBAL
#include "WT_Task.h"


#include "UART5.h"
#include "Timer0.h"
#include "sdconfig.h"
#include "sdcommon.h"
#include "sddriver.h"
#include "FAT32APP.h"

#include "Uart1.h"
#include "Task_TiPo.h"
#include "UART5.h"

#include "Task_Uart5Rec.h"
#include "Task_Uart1Rec.h"
#include "Task_CheckDir.h"
#include "SysRecovery.h"
#include "Task_AqcWave.h"

/*********************************************************************************************************
** Function name:           TaskStart
**
** Descriptions:            �������񣬳�ʼ��ϵͳ����������������
**
** input parameters:        None
** output parameters:       None
** Returned value:          None
**
** Created by:              ZHANG Ye
** Created Date:            20110511
**--------------------------------------------------------------------------------------------------------
** Modified by:
** Modified date:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/

void  TaskStart (void *tdata)
{
	tdata = tdata;

	TargetInit();

	InitAllIRQ();//��������Ҫ���жϽ��г�ʼ��,��ʱ��I2C,UCOS����ʱ�������г�ʼ��/���������г�ʼ��
																													  //������������  
	RTC8563Init();	//


//��Ӳ���CPUʹ����
#if OS_TASK_STAT_EN >0
	OSStatInit();
#endif		      	
				
	//�ָ�ϵͳ
	RecoverySystem();
	
	//���ÿ��Ź�״̬
	SetWatchDog();
	
	//��¼����
	RecordReset();	
	
#if	YBVERSION >= 30
	InitAllSP();
#endif

	//��ʼ���ź���
	InitSemaphore();

	//�崮�ڻ�������
	ClearUartCache();

	//��ʼ��SD����
	SDCardInit();//sd����ʼ���ɹ� ����1�� ʧ�ܷ���2��

	OSTaskCreate(Task_Checknet,(void *)0, &TaskChecknet[TASKSTACKSIZE-1], TASKChecknetPRIO);  //�������״̬����
	OSTaskCreate(Task_JG0,(void *)0,&TaskJG0[TASKSTACKSIZE-1],TaskJG0PRIO);//����0���մ�������
	OSTaskCreate(Task_Data_JG,(void *)0,&TaskDataJG[TASKSTACKSIZE-1],TaskDataJGPRIO);//�������ݴ�������
	OSTaskCreate(Task_CheckDir,(void *)0,&TaskCheckDir[TASKSTACKSIZE-1],TASKCHECKDIRPRIO);//��鳵����������
	OSTaskCreate(Task_TiPo,(void *)0,&TaskTiPo[TASKSTACKSIZE-1],TASKTiPoPRIO);//ʱ��ˢ������
	OSTaskCreate(Task_Uart1Rec,(void *)0,&TaskUart1Rec[TASKSTACKSIZE-1],TASKUART1RECPRIO);//����1���մ�������
	OSTaskCreate(Task_Uart5Rec,(void *)0,&TaskUart5Rec[TASKSTACKSIZE-1],TASKUART5RECPRIO);//����5���մ�������
	OSTaskCreate(Task_AqcWave,(void *)0,&TaskAqcWave[TASKSTACKSIZE-1],TaskAqcWavePRIO);//�����浥����������
	
	
	BuzzerBeep(2,50);	//��������ʱ��������

	while(1)
	{
		OSTimeDly(200);
		WDTIM_COUNTER	= 1;									/* ι��	*/	
	}
}



/*		������������		*/
void RunStartTask(void)
{
	OSTaskCreate(TaskStart,(void *)0, &TaskStartStk[TASKSTACKSIZE-1], TASKSTARTPRIO);  	
}
/*		��ʼ��		*/


//���Ʒ������ĺ���
void BuzzerBeep(int cnt,int time)									
{
	int i = 0;
	for(;i<cnt;i++)
	{
		BeepON();
		OSTimeDly(time);	
		BeepOFF();	 
		OSTimeDly(time);
	}
}


//�崮�ڻ�������
void  ClearUartCache(void)
{
	uint16 ReadLen=0;
	uint16 i=0;
	uint8 ReadBuf=0;
	
	//����1������س�ʼ��
	memset(g_u8U1RcvBuf, 0 , UARTRCVBUFSIZE);			//����1���ջ���
	g_UART1_Recv_Index = 0;	   //����1�����������
	g_UART1_Proc_Index = 0;	   //����1���洦������

 	//������1����
	ReadLen = (uint16)(HSU1_LEVEL & 0xff);
	if(ReadLen > 0)
	{			                                                            
		for (i=0; i < ReadLen; i++) 
		{
			ReadBuf = (uint8)(HSU1_RX & 0xff);                    
		}
	}

	//����5������س�ʼ��
	memset(g_u8U5RcvBuf, 0 , UARTRCVBUFSIZE);			//����5���ջ���
	g_UART5_Recv_Index = 0;	   //����5�����������
	g_UART5_Proc_Index = 0;	   //����5���洦������

	//������5����
	ReadLen = (uint32)U5RXLEV;
	if(ReadLen > 0)
	{			                                                            
		for (i=0; i < ReadLen; i++) 
		{
			ReadBuf = (uint8)(U5RBR & 0xff);                    
		}
	}  
}


//�����ź���
void InitSemaphore(void)
{
	g_Uart1Recv  = OSSemCreate(0);//����1�����ź���
    g_Uart1Send  = OSSemCreate(1);//����1�����ź���
    g_Uart5Send  = OSSemCreate(1);//����5�����ź���
	g_Uart5Recv	 = OSSemCreate(0);//����5�����ź���
    g_NetS0Recv	 = OSSemCreate(0);//����S0�����ź���
    g_NetS1Recv	 = OSSemCreate(0);//����S1�����ź���
    g_NetS2Recv	 = OSSemCreate(0);//����S2�����ź���
	g_NetS3Recv  = OSSemCreate(0);//����S3�����ź���
    g_WavesProc	 = OSSemCreate(0);//���δ����ź���
    
    g_TdOpMutex  = OSSemCreate(1);//��������ź���
    g_SdOpMutex  = OSSemCreate(1);//sd�������ź���
    g_UpdateApp  = OSSemCreate(0);//Զ�̸��³���
}

//���ÿ��Ź�״̬
void SetWatchDog()
{
	if(g_SysSetUp.u8WorkMode)
	{
		WDTInit(0);
	}
	else
	{
		WDTInit(1);
	}
}




