/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)
**
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			Task_JG0.C
** Last modified Date:  20120718
** Last Version:		1.0
** Descriptions:		��ֱ���������������˿�0���գ�
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
#include "Task_JG0.h"
#include "socket.h"
#include "WT_Task.h"
#include "Timer0.h"
#include "JZStructure.h"
#include "JZGlobal.h"
#include "math.h"
#include "Task_Data_JG.h"
#include "Task_CheckDir.h"


JGRecSet JG0Rec;

void Task_JG0(void *tdata)
{
    uint8 err;
    uint32 i=0;
    tdata=tdata;
	memset(&JG0Rec,0,sizeof(JG0Rec));
    while(1)
    {
        OSSemPend(g_NetS0Recv,0,&err);
		i = S_rx_process(0);//��������
		JG0Rec.PostCnt++;//��¼���ж���
        if(i>0)
        {
			JG0Rec.RecLen=i;
			if(i>2*Full_Size)
			{
				JG0Rec.ErrFrCnt++;//�����������³���
			}
			else
			{
				memcpy(JG0Rec.Data,Rx_Buffer,i);
				JG0_RecPro();
			}
        }
		else
		{
			//����Ϊ0��֡����
			JG0Rec.ZeroCnt++;
		}
		OSTimeDly(5);
    }
}

void JG0_RecPro(void)
{
	
	if(JG0Rec.Data[0]==0x02 && JG0Rec.Data[0]==0x02 
		&& JG0Rec.Data[0]==0x02 && JG0Rec.Data[0]==0x02)
	{
		if(JG0Rec.RecLen == Full_Size)
		{
			 //�����������
			if((JG0Rec.Data[83]<<8)+JG0Rec.Data[84]==POINT_SUM)
			{
				g_LmdCacheIn=g_LmdCacheIn+1;
				g_LmdCacheIn=g_LmdCacheIn%LMDMAXCACHE;
				//�������豸�Ĵ�����Ϣд�뼤��810λ��
				if(g_SysSetUp.u8WorkMode<1)
				{
					if(g_DirTrigged)
						JG0Rec.Data[810]=1;
					else
						JG0Rec.Data[810]=0;
				}
				//�������豸�Ĵ�����Ϣд�뼤��810λ��
				memcpy(g_LmdCache[g_LmdCacheIn],JG0Rec.Data,Full_Size);
				g_LmdCache[g_LmdCacheIn][Full_Size]=10;		//����JG0
				JG0Rec.RecWvCnt++;
			}
			else
			{
				JG0Rec.ErrFrCnt++;
			}
		}
		else
		{
			 //͸������ ������ѯ�ظ�
			if(JG0Rec.RecLen<MAXPASSLEN-9)
			{
			   //͸�����ݻظ�
				JG0Rec.RecCmdCnt++;
				JG0Rec.WaitPass=0;
				memcpy(g_LaserPass.PassCont,JG0Rec.Data,JG0Rec.RecLen);
				JG0Rec.WaitPass=1;
			}
		}
		
		OSSemPost(g_WavesProc);
	}
	else
	{
		JG0Rec.ErrFrCnt++;
	}
}



