#include "Task_CheckDir.h"
#include "GPIO.h"

void Task_CheckDir(void *tdata)
{ 
	tdata = tdata;

	while(1)
	{
		if(g_SysSetUp.u8WorkMode<1)
		{
			if(g_SysSetUp.u8DirDevType==DEV_COIL)
			{
				if(DirTriPin) //�ظ���Ȧ����źţ��г�ʱΪ�͵�ƽ
				{
					g_DirTrigged = 0;
				}
				else   //��⵽��������
				{
					g_DirTrigged = 1;
					BeepON();
					OSTimeDly(10);
					BeepOFF();
				}
			}
			else if(g_SysSetUp.u8DirDevType==DEV_GRAT)
			{
				if(DirTriPin) //��դ���г�ʱΪ�͵�ƽ
				{
					g_DirTrigged = 0;
				}
				else   //��⵽��������
				{
					g_DirTrigged = 1;
					BeepON();
					OSTimeDly(10);
					BeepOFF();
				}
			}
			else if(g_SysSetUp.u8DirDevType==DEV_LASE)
			{
				//���� ��Ҫ��������߼�����
			}
			else
			{
				
			}
			OSTimeDly(10);
		}
		else
		{
			OSTimeDly(500);
		}
	}
}
