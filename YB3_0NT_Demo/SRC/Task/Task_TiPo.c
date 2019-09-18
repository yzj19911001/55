/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_TiPo.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		时间刷新任务
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

#include "Task_TiPo.h"
#include "WT_Task.h"
#include "common.h"	
#include "PCF8563.h"  
#include "tdc256.h"
#include "Task_Update_App.h"
#include "PrtlProc.h"
#include "SysRecovery.h"
#include "Avi_AxleCheck.h"

uint8 JG_8[10]={0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa,0xaa};


void Task_TiPo(void *tdata)
{
//时间刷新任务，包括：喂狗、采集温度、检查SD卡插入、每小时定时操作等；
	uint16 l_count=0;
  uint8 tmp;
	uint8 last_sec=60;
	tdata = tdata; 

	while(1)
	{
		GetRTCTime(&g_SysCurTime);
		YEAR 	=	g_SysCurTime.u16Year;	//年
		MONTH	=	g_SysCurTime.u8Month;			//月
		DAY		=	g_SysCurTime.u8Day;			//日
		WEEK	=	g_SysCurTime.u8Week;
		HOUR	=	g_SysCurTime.u8Hour;			//时
		MIN		=	g_SysCurTime.u8Minute;		//分
		SEC		=	g_SysCurTime.u8Second;		//秒
		
		if(g_Update_App_Step==0x01||g_Update_App_Step==0x02||g_Update_App_Step==0x03)
		{
		   //if(MIN-MIN_Remote>=5)	  
		   if((MIN + 60 - MIN_Remote)%60 >= 15)	 //如果远程更新程序持续时间大于15min还没有更新完成，重启控制器
		   {
				SendBuf_full(JG_8,10,g_Update_App_CH);
				g_bStarted = 0;
				g_Update_App_CH=CH_NOCH;
			    OSTimeDly(250);OSTimeDly(250);
			    OSTimeDly(250);OSTimeDly(250);
			    // OSTimeDly(250);OSTimeDly(250);
			    ResetSystem() ;  //重启	
		   }
		}
		
	  l_count++;
		if(l_count>=2)
		{  
			l_count=0;
			tmp= OSCPUUsage;
			UART1_SendBuf(&tmp,1);
		}	

		//称台交互心跳
		if(g_WeigthAt.u8wAckSendFlag)
		{
			ReprotAckSInfo();
			g_WeigthAt.u8wAckSendFlag=0;
		}
        
        if(g_VehIndex.u8Day != DAY)
        {
			/* 新的一天开始,将上传单车的序号重置为1 */
			DefualtVehIndex();
        }
       
		if(last_sec != SEC)
		{
			last_sec=SEC;
		}
			
		OSTimeDly(250);
	}
}


