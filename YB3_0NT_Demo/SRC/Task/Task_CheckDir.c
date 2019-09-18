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
				if(DirTriPin) //地感线圈检测信号，有车时为低电平
				{
					g_DirTrigged = 0;
				}
				else   //检测到车辆进入
				{
					g_DirTrigged = 1;
					BeepON();
					OSTimeDly(10);
					BeepOFF();
				}
			}
			else if(g_SysSetUp.u8DirDevType==DEV_GRAT)
			{
				if(DirTriPin) //光栅，有车时为低电平
				{
					g_DirTrigged = 0;
				}
				else   //检测到车辆进入
				{
					g_DirTrigged = 1;
					BeepON();
					OSTimeDly(10);
					BeepOFF();
				}
			}
			else if(g_SysSetUp.u8DirDevType==DEV_LASE)
			{
				//激光 需要任务进行逻辑处理
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
