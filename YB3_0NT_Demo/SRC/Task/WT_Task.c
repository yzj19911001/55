/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)
**
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			WT_Task.C
** Last modified Date:  20110511
** Last Version:		1.0
** Descriptions:		程序任务
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
** Descriptions:		TaskStart;创建任务；
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
** Descriptions:            启动任务，初始化系统，并加载其他任务
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

	InitAllIRQ();//对所有需要的中断进行初始化,定时器I2C,UCOS下在时间任务中初始化/键盘任务中初始化
																													  //更新重启次数  
	RTC8563Init();	//


//添加测试CPU使用率
#if OS_TASK_STAT_EN >0
	OSStatInit();
#endif		      	
				
	//恢复系统
	RecoverySystem();
	
	//设置看门狗状态
	SetWatchDog();
	
	//记录重启
	RecordReset();	
	
#if	YBVERSION >= 30
	InitAllSP();
#endif

	//初始化信号量
	InitSemaphore();

	//清串口缓存数据
	ClearUartCache();

	//初始化SD卡；
	SDCardInit();//sd卡初始化成功 蜂鸣1声 失败蜂鸣2声

	OSTaskCreate(Task_Checknet,(void *)0, &TaskChecknet[TASKSTACKSIZE-1], TASKChecknetPRIO);  //检查网络状态任务
	OSTaskCreate(Task_JG0,(void *)0,&TaskJG0[TASKSTACKSIZE-1],TaskJG0PRIO);//激光0接收处理任务
	OSTaskCreate(Task_Data_JG,(void *)0,&TaskDataJG[TASKSTACKSIZE-1],TaskDataJGPRIO);//激光数据处理任务
	OSTaskCreate(Task_CheckDir,(void *)0,&TaskCheckDir[TASKSTACKSIZE-1],TASKCHECKDIRPRIO);//检查车辆方向任务
	OSTaskCreate(Task_TiPo,(void *)0,&TaskTiPo[TASKSTACKSIZE-1],TASKTiPoPRIO);//时钟刷新任务
	OSTaskCreate(Task_Uart1Rec,(void *)0,&TaskUart1Rec[TASKSTACKSIZE-1],TASKUART1RECPRIO);//串口1接收处理任务
	OSTaskCreate(Task_Uart5Rec,(void *)0,&TaskUart5Rec[TASKSTACKSIZE-1],TASKUART5RECPRIO);//串口5接收处理任务
	OSTaskCreate(Task_AqcWave,(void *)0,&TaskAqcWave[TASKSTACKSIZE-1],TaskAqcWavePRIO);//过车存单车波形任务
	
	
	BuzzerBeep(2,50);	//程序启动时蜂鸣两声

	while(1)
	{
		OSTimeDly(200);
		WDTIM_COUNTER	= 1;									/* 喂狗	*/	
	}
}



/*		调用启动任务		*/
void RunStartTask(void)
{
	OSTaskCreate(TaskStart,(void *)0, &TaskStartStk[TASKSTACKSIZE-1], TASKSTARTPRIO);  	
}
/*		初始化		*/


//控制蜂鸣器的函数
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


//清串口缓存数据
void  ClearUartCache(void)
{
	uint16 ReadLen=0;
	uint16 i=0;
	uint8 ReadBuf=0;
	
	//串口1接收相关初始化
	memset(g_u8U1RcvBuf, 0 , UARTRCVBUFSIZE);			//串口1接收缓存
	g_UART1_Recv_Index = 0;	   //串口1缓存接收索引
	g_UART1_Proc_Index = 0;	   //串口1缓存处理索引

 	//清理串口1缓存
	ReadLen = (uint16)(HSU1_LEVEL & 0xff);
	if(ReadLen > 0)
	{			                                                            
		for (i=0; i < ReadLen; i++) 
		{
			ReadBuf = (uint8)(HSU1_RX & 0xff);                    
		}
	}

	//串口5接收相关初始化
	memset(g_u8U5RcvBuf, 0 , UARTRCVBUFSIZE);			//串口5接收缓存
	g_UART5_Recv_Index = 0;	   //串口5缓存接收索引
	g_UART5_Proc_Index = 0;	   //串口5缓存处理索引

	//清理串口5缓存
	ReadLen = (uint32)U5RXLEV;
	if(ReadLen > 0)
	{			                                                            
		for (i=0; i < ReadLen; i++) 
		{
			ReadBuf = (uint8)(U5RBR & 0xff);                    
		}
	}  
}


//创建信号量
void InitSemaphore(void)
{
	g_Uart1Recv  = OSSemCreate(0);//串口1接收信号量
    g_Uart1Send  = OSSemCreate(1);//串口1发送信号量
    g_Uart5Send  = OSSemCreate(1);//串口5发送信号量
	g_Uart5Recv	 = OSSemCreate(0);//串口5接收信号量
    g_NetS0Recv	 = OSSemCreate(0);//网络S0接收信号量
    g_NetS1Recv	 = OSSemCreate(0);//网络S1接收信号量
    g_NetS2Recv	 = OSSemCreate(0);//网络S2接收信号量
	g_NetS3Recv  = OSSemCreate(0);//网络S3接收信号量
    g_WavesProc	 = OSSemCreate(0);//波形处理信号量
    
    g_TdOpMutex  = OSSemCreate(1);//铁电操作信号量
    g_SdOpMutex  = OSSemCreate(1);//sd卡操作信号量
    g_UpdateApp  = OSSemCreate(0);//远程更新程序
}

//设置看门狗状态
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




