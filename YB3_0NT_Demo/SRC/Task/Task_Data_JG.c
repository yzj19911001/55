/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_Data_JG.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		�������ݴ�������
**
**--------------------------------------------------------------------------------------------------------
** Created by:			Hong XiangYuan
** Created date:		20120718
** Version:				1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			Wang ZiFeng
** Modified date:		2013-03-11
** Version:				2.0
** Descriptions:
**
*********************************************************************************************************/
#include "Task_Data_JG.h"
#include "Avi_MainProc.h"
#include "Task_JG0.h"
#include "PrtlProc.h"
#include "Task_CheckDir.h"
#include "WT_Task.h"

uint8 JG0_CurBuff[Full_Size]={0};//��ǰ֡��������

void Task_Data_JG(void *tdata)
{
	uint8 err;
	int32 i=0;			   
	int32 j=0;
	int32 Len=0;
	uint8 LedCnt=0;
	tdata=tdata;
	
	while(1)
	{
		OSSemPend(g_WavesProc,0,&err);
		if(err != OS_NO_ERR)
			continue;
		
	    //ָʾ���յ��������ݲ��������ݴ���
		LedCnt++;
		if(LedCnt<10)
		{
			LED_RUN_ON();
		}
		else 
		{
			LED_RUN_OFF();
			if(LedCnt>19)
				LedCnt=0;
		}
		
		if(JG0Rec.WaitPass==1)
		{
			PassThroughToUart(0);
		}
		
		if(g_LmdCacheOut!= g_LmdCacheIn)
		{	
			//����һ����ղ���
			memset(JG0_CurBuff,0,sizeof(JG0_CurBuff));
			memcpy(JG0_CurBuff,g_LmdCache[g_LmdCacheOut],Full_Size);	
			g_LmdCacheOut = g_LmdCacheOut +1;
			g_LmdCacheOut = g_LmdCacheOut % LMDMAXCACHE;
			JG0Rec.ProWvCnt++;
			
			Len=(JG0_CurBuff[83]<<8)+JG0_CurBuff[84];
			memset(LMS_data_1,0,sizeof(LMS_data_1));

			if (Len == POINT_SUM)
			{ 
				for(i=85,j=0;j < POINT_SUM; i=i+2,j++)	//
				{
					LMS_data_1[j]=	(JG0_CurBuff[i]<<8)+JG0_CurBuff[i+1];
					if(LMS_data_1[j] < 0)
						LMS_data_1[j] = 0;
				}
				
				/*��ǰ֡��ʱ���*/	
				LMS_data_1[POINT_SUM] = ((JG0_CurBuff[42]<<24)+(JG0_CurBuff[43]<<16)+(JG0_CurBuff[44]<<8)+JG0_CurBuff[45])/1000;//�������1000 ���������ж�֡
				g_n32WaveNumber=LMS_data_1[POINT_SUM];
				
				//���г���ʶ����
				if(g_SysSetUp.u8WorkMode)
				{
					g_DirTrigged=JG0_CurBuff[810];//ģ�Ⲩ��״̬�� 810λΪ��Ȧ״̬
					memcpy(g_u8ItemName,&JG0_CurBuff[52],24);//���͵��ļ�������
				}
				VehIdentProc();
			}
		}
	} 
}



