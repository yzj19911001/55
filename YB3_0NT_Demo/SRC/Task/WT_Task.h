/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			WT_Task.h
** Last modified Date:  20110511
** Last Version:		1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		20110511
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

#ifndef	__WT_TASK_H
#define	__WT_TASK_H

#include "JZGlobal.h" 

#include "sdconfig.h"
#include "sdcommon.h"
#include "sddriver.h"
#include "Common.h"
					   
#if	YBVERSION >= 30		//3.0仪表功能
#include "IRQ.h"	   				 
#include "Uart5.h"
//#include "TaskSD.h"
#else	//2.2
#include "AllIRQ.h"					
#include "Uart0.h"	 
#endif	//#if	YBVERSION >= 30		//3.0仪表功能
#include "Config.h"


JZG_EXT 	OS_STK	TaskStartStk[TASKSTACKSIZE];					
JZG_EXT 	OS_STK	TaskChecknet[TASKSTACKSIZE];	
JZG_EXT 	OS_STK  TaskTiPo[TASKSTACKSIZE];
JZG_EXT 	OS_STK  TaskJG0[TASKSTACKSIZE];
JZG_EXT 	OS_STK  TaskDataJG[TASKSTACKSIZE];
JZG_EXT 	OS_STK  TaskUart1Rec[TASKSTACKSIZE];
JZG_EXT 	OS_STK  TaskUart5Rec[TASKSTACKSIZE];
JZG_EXT 	OS_STK  TaskCheckDir[TASKSTACKSIZE];
JZG_EXT 	OS_STK  TaskAqcWave[TASKSTACKSIZE];

JZG_EXT	void	Task_TiPo(void *pdata);
JZG_EXT	void	TaskStart(void *pdata); 
JZG_EXT	void	Task_Checknet(void *pdata);
JZG_EXT	void	Task_JG0(void *tdata);
JZG_EXT	void 	Task_Data_JG(void *tdata);
JZG_EXT	void 	Task_Test(void *tdata);
JZG_EXT	void	Task_CheckDir(void *pdata);
JZG_EXT	void	RunStartTask(void);					//调用启动任务	 


JZG_EXT void BuzzerBeep(int cnt,int time);										 //控制器蜂鸣器的函数
void GlobalVarInit(void);													     //全局变量初始化
void RecordReset(void);														 //更新重启次数
void InitAllSP(void);
void InitSemaphore(void);//创建信号量
void SetWatchDog(void);//设置看门狗状态
void  ClearUartCache(void);//清串口缓存数据
#endif	//__WT_TASK_H
