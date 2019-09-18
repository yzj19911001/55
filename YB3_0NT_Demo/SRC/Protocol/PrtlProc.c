#include "PrtlProc.h"
#include "PCF8563.h"
#include "Socket.h"
#include "TDC256.h"
#include "SysRecovery.h"
#include "Uart1.h"
#include "Uart5.h"
#include "Common.h"
#include "Avi_MainProc.h"
#include "Task_JG0.h"
#include "Task_Checknet.h"

//�ָ�������������״̬
void Ctl2FactoryState(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		tempindex = 5;
		TmpBuff[tempindex++] = ERR_OK;
		
		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_CTL2FACTROY;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;
		
		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);
		
		MakeCtr2Factory();
		
		OSTimeDly(1000);
		OSSchedLock();
		ResetSystem();
	}
	else
	{
		
	}
}


//��ȡ�ɼ����ε�״̬
void GetAqcWvState(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		tempindex = 5;
		
		//���жϼ���
		TmpBuff[tempindex++] = ((JG0Rec.PostCnt>>24)&0xFF);
		TmpBuff[tempindex++] = ((JG0Rec.PostCnt>>16)&0xFF);
		TmpBuff[tempindex++] = ((JG0Rec.PostCnt>>8)&0xFF);
		TmpBuff[tempindex++] = (JG0Rec.PostCnt&0xFF);		
		
		//���ղ���֡��
		TmpBuff[tempindex++] = ((JG0Rec.RecWvCnt>>24)&0xFF);
		TmpBuff[tempindex++] = ((JG0Rec.RecWvCnt>>16)&0xFF);
		TmpBuff[tempindex++] = ((JG0Rec.RecWvCnt>>8)&0xFF);
		TmpBuff[tempindex++] = (JG0Rec.RecWvCnt&0xFF);
		//������֡��
		TmpBuff[tempindex++] = ((JG0Rec.ProWvCnt>>24)&0xFF);
		TmpBuff[tempindex++] = ((JG0Rec.ProWvCnt>>16)&0xFF);
		TmpBuff[tempindex++] = ((JG0Rec.ProWvCnt>>8)&0xFF);
		TmpBuff[tempindex++] = (JG0Rec.ProWvCnt&0xFF);
		//ʣ��ռ�Ϊ0֡��
		TmpBuff[tempindex++] = ((JG0Rec.ZeroCnt>>24)&0xFF);
		TmpBuff[tempindex++] = ((JG0Rec.ZeroCnt>>16)&0xFF);
		TmpBuff[tempindex++] = ((JG0Rec.ZeroCnt>>8)&0xFF);
		TmpBuff[tempindex++] = (JG0Rec.ZeroCnt&0xFF);	
		//֡ͷ����֡��
		TmpBuff[tempindex++] = ((JG0Rec.ErrFrCnt>>24)&0xFF);
		TmpBuff[tempindex++] = ((JG0Rec.ErrFrCnt>>16)&0xFF);
		TmpBuff[tempindex++] = ((JG0Rec.ErrFrCnt>>8)&0xFF);
		TmpBuff[tempindex++] = (JG0Rec.ErrFrCnt&0xFF);	
		//͸������֡��
		TmpBuff[tempindex++] = ((JG0Rec.RecCmdCnt>>24)&0xFF);
		TmpBuff[tempindex++] = ((JG0Rec.RecCmdCnt>>16)&0xFF);
		TmpBuff[tempindex++] = ((JG0Rec.RecCmdCnt>>8)&0xFF);
		TmpBuff[tempindex++] = (JG0Rec.RecCmdCnt&0xFF);		
		
		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_GETAQCSTATE;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;
		
		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);
	}
	else
	{
		
	}
}

//��ղɼ����ε�״̬
void ClrAqcWvState(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	if(PRTL==PTRL_FF)
	{
		//��ռ���״̬
		JG0Rec.PostCnt=0;
		JG0Rec.RecWvCnt=0;
		JG0Rec.ProWvCnt=0;
		JG0Rec.ZeroCnt=0;
		JG0Rec.ErrFrCnt=0;
		JG0Rec.RecCmdCnt=0;
		
		tempindex = 5;
		TmpBuff[tempindex++] = ERR_OK;
		
		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_CLRAQCSTATE;
		
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;
		TmpBuff[4] = (tempindex + 2)&0xFF;
		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);
	}
	else
	{
		
	}
	
}




//��ȡ��֡����
void GetSingleWave(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[Full_Size+50]={0};
	
	if(PRTL==PTRL_FF)
	{
		tempindex = 5;
		memcpy(TmpBuff+tempindex,g_LmdCache[g_LmdCacheIn],Full_Size);
		tempindex+=Full_Size;
		
		
		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_GETSINGWAVE;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;
		
		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);
	}
	else
	{
		
	}
}


//����ϵͳ����
void SetUartInfo(uint8 *pBuff,uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	DevUartSet TmpUartSet;
	memset(&TmpUartSet,0,sizeof(TmpUartSet));
	
	if(PRTL==PTRL_FF)
	{
		memcpy(TmpBuff, pBuff, sizeof(TmpBuff));
		tempindex = 5;
		
		TmpUartSet.BR_COM1=TmpBuff[tempindex++];
		TmpUartSet.BR_COM2=TmpBuff[tempindex++];
		TmpUartSet.TS_CHCOM=TmpBuff[tempindex++];
		TmpUartSet.RM_CHCOM=TmpBuff[tempindex++];
		
		if(TmpUartSet.BR_COM1>5||TmpUartSet.BR_COM2>5||
			(TmpUartSet.RM_CHCOM!=CH_COM1 && TmpUartSet.RM_CHCOM!=CH_COM2 
		&& TmpUartSet.RM_CHCOM!=CH_COMA && TmpUartSet.RM_CHCOM!=CH_NOCH)||
			(TmpUartSet.TS_CHCOM!=CH_COM1 && TmpUartSet.TS_CHCOM!=CH_COM2 
		&& TmpUartSet.TS_CHCOM!=CH_COMA && TmpUartSet.TS_CHCOM!=CH_NOCH))
		{
			tempindex = 5;
			TmpBuff[tempindex++] = CMD_SETUARTINFO;
			TmpBuff[tempindex++] = ERR_AB;//����ʧ�ܣ��쳣
			
			TmpBuff[0] = 0xFF;
			TmpBuff[1] = 0xFF;
			TmpBuff[2] = CMD_OPERATEERRS;
			TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
			TmpBuff[4] = (tempindex + 2)&0xFF;

			AddCrc16(TmpBuff,	tempindex);
			SendBuf_full(TmpBuff,tempindex + 2,CH);	
			
		}
		else
		{
			memcpy(&g_UartSet,&TmpUartSet,sizeof(g_UartSet));
			AddCrc16((uint8 *)&g_UartSet,sizeof(g_UartSet)-2);				
			if(Write256_full(CTR_UARTSET, (uint8 *)&g_UartSet, sizeof(g_UartSet)))
			{
				tempindex = 5;
				TmpBuff[tempindex++] = ERR_OK;//���óɹ�
				
				TmpBuff[0] = 0xFF;
				TmpBuff[1] = 0xFF;
				TmpBuff[2] = CMD_SETUARTINFO;
				TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
				TmpBuff[4] = (tempindex + 2)&0xFF;

				AddCrc16(TmpBuff,tempindex);
				SendBuf_full(TmpBuff,tempindex + 2,CH);	
				
				//��Ч����
				UART1_Init(GetUBR(g_UartSet.BR_COM1));
				UART5_Init(GetUBR(g_UartSet.BR_COM2));
				
			}
			else
			{
				tempindex = 5;
				TmpBuff[tempindex++] = CMD_SETUARTINFO;
				TmpBuff[tempindex++] = ERR_TD;//���óɹ�
				
				TmpBuff[0] = 0xFF;
				TmpBuff[1] = 0xFF;
				TmpBuff[2] = CMD_SETUARTINFO;
				TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
				TmpBuff[4] = (tempindex + 2)&0xFF;

				AddCrc16(TmpBuff,	tempindex);
				SendBuf_full(TmpBuff,tempindex + 2,CH);
			}
		}
	}
	else
	{
		
	}
}




//��ȡ���ڲ���
void GetUartInfoParam(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		tempindex = 5;
		
		TmpBuff[tempindex++] = g_UartSet.BR_COM1;
		TmpBuff[tempindex++] = g_UartSet.BR_COM2;
		TmpBuff[tempindex++] = g_UartSet.TS_CHCOM;
		TmpBuff[tempindex++] = g_UartSet.RM_CHCOM;

		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_GETUARTINFO;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;
		
		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);
	}
	else
	{
		
	}
}





//�����������
void SetNetParam(uint8 *pBuff,uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		memcpy(TmpBuff, pBuff, sizeof(TmpBuff));
		tempindex = 5;
		
		//����IP
		memcpy(g_NetInfo.au8IPAddr,TmpBuff+tempindex,4);
		tempindex+=4;
		
		//�����˿�
		g_NetInfo.u32LocalPortNO=(TmpBuff[tempindex]<<8)+ TmpBuff[tempindex+1];
		tempindex+=2;
			
		//������������
		memcpy(g_NetInfo.au8SubMask,TmpBuff+tempindex,4);
		tempindex+=4;
		
		//����Ĭ������
		memcpy(g_NetInfo.au8GatewayIP,TmpBuff+tempindex,4);
		tempindex+=4;	
		
		//����MAC��ַ
		memcpy(g_NetInfo.au8MACAddr,TmpBuff+tempindex,6);
		tempindex+=6;
		
		//SOCKET0  IP �� �˿�
		memcpy(g_NetInfo.SocketIP[0],TmpBuff+tempindex,4);
		tempindex+=4;	
		
		
		g_NetInfo.SocketPort[0]=(TmpBuff[tempindex]<<8)+ TmpBuff[tempindex+1];
		tempindex+=2;
	
		
		//SOCKET1  IP �� �˿�
		memcpy(TmpBuff+tempindex,g_NetInfo.SocketIP[1],4);
		tempindex+=4;	
		
		
		g_NetInfo.SocketPort[1]=(TmpBuff[tempindex]<<8)+ TmpBuff[tempindex+1];
		tempindex+=2;


		//����ֵ
		tempindex = 5;
		TmpBuff[tempindex++] = ERR_OK;

		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_SETNETPARAM;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;
		
		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);
		
		AddCrc16((uint8 *)&g_NetInfo,sizeof(g_NetInfo)-2);
		Write256_full(CTR_NETSET, (uint8 *)&g_NetInfo, sizeof(g_NetInfo));	
		
		W5300_Init(&g_NetInfo);//��ʼ��5300
	}
	else
	{
		
	}
}


//��ȡ�������
void GetNetParam(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		tempindex = 5;
		
		//����IP
		memcpy(TmpBuff+tempindex,g_NetInfo.au8IPAddr,4);
		tempindex+=4;
		
		//�����˿�
		TmpBuff[tempindex++] = (g_NetInfo.u32LocalPortNO>>8)&0xFF;
		TmpBuff[tempindex++] = g_NetInfo.u32LocalPortNO&0xFF;		
		
		//������������
		memcpy(TmpBuff+tempindex,g_NetInfo.au8SubMask,4);
		tempindex+=4;
		
		//����Ĭ������
		memcpy(TmpBuff+tempindex,g_NetInfo.au8GatewayIP,4);
		tempindex+=4;	
		
		//����MAC��ַ
		memcpy(TmpBuff+tempindex,g_NetInfo.au8MACAddr,6);
		tempindex+=6;
		
		//SOCKET0  IP �� �˿�
		memcpy(TmpBuff+tempindex,g_NetInfo.SocketIP[0],4);
		tempindex+=4;	
		
		TmpBuff[tempindex++] = (g_NetInfo.SocketPort[0]>>8)&0xFF;
		TmpBuff[tempindex++] = g_NetInfo.SocketPort[0]&0xFF;		
		
		//SOCKET1  IP �� �˿�
		memcpy(TmpBuff+tempindex,g_NetInfo.SocketIP[1],4);
		tempindex+=4;	
		
		TmpBuff[tempindex++] = (g_NetInfo.SocketPort[1]>>8)&0xFF;
		TmpBuff[tempindex++] = g_NetInfo.SocketPort[1]&0xFF;	


		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_GETNETPARAM;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;
		
		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);
	}
	else
	{
		
	}
}



//����������
void CtlSysReset(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	if(PRTL==PTRL_FF)
	{
		tempindex = 5;
		TmpBuff[tempindex++] = ERR_OK;
		
		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_CTLSYSRESET;
		
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;
		TmpBuff[4] = (tempindex + 2)&0xFF;
		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);
		
		OSTimeDly(1000);
		OSSchedLock();
		ResetSystem();
	}
	else
	{
		
	}
	
}




//��տ�����������Ϣ
void ClrCtlResetInfo(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 Ret=0;
	ResetRecord TmpRecord;
	uint8 TmpBuff[512]={0};
	if(PRTL==PTRL_FF)
	{
		memset(&TmpRecord,0,sizeof(TmpRecord));
		Ret=Clear256_full(CTR_RESET, sizeof(ResetRecord));

		//����
		if(Ret==0)
		{
			tempindex = 5;
			TmpBuff[tempindex++] = CMD_GETRESETINF;
			TmpBuff[tempindex++] = ERR_AL;
			
			TmpBuff[0] = 0xFF;
			TmpBuff[1] = 0xFF;
			TmpBuff[2] = CMD_OPERATEERRS;
			TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;
			TmpBuff[4] = (tempindex + 2)&0xFF;
			AddCrc16(TmpBuff,	tempindex);
			SendBuf_full(TmpBuff,tempindex + 2,CH);
		}
		else
		{
			tempindex = 5;
			TmpBuff[tempindex++] = ERR_OK;

			
			TmpBuff[0] = 0xFF;
			TmpBuff[1] = 0xFF;
			TmpBuff[2] = CMD_CLRRESETINF;
			TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;
			TmpBuff[4] = (tempindex + 2)&0xFF;
			AddCrc16(TmpBuff,	tempindex);
			SendBuf_full(TmpBuff,tempindex + 2,CH);			
		}
	}
	else
	{
		
	}
	
}



//��ȡ������������Ϣ
void GetCtlResetInfo(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	uint8 i = 0;
	uint8 j = 0;
	uint8 u8TmpPos = 0;
	uint8 u8StartPos = 0;
	uint8 u8TmpEntries = 0;
	ResetRecord  TmpRecord;
	if(PRTL==PTRL_FF)
	{
		memset(&TmpRecord,0,sizeof(TmpRecord));
		
		Read256_full(CTR_RESET, (uint8 *)&TmpRecord, sizeof(TmpRecord));

			//��λʱ��
		u8TmpEntries = TmpRecord.u8CurCnt;
		u8TmpPos = TmpRecord.u8CurPos;
		u8StartPos = (MAX_RESETCNT + u8TmpPos - u8TmpEntries)%MAX_RESETCNT;

		//����
		tempindex = 5;
		//��λ����
		TmpBuff[tempindex++] =u8TmpEntries;
		
		for(j=0; j<u8TmpEntries;j++)	//20130703�޸�
		{
			i = (j+u8StartPos) % MAX_RESETCNT;
			TmpBuff[tempindex++] = b2bcd((TmpRecord.SysTimeTable[i].u16Year - 2000)&0xFF);
			TmpBuff[tempindex++] = b2bcd(TmpRecord.SysTimeTable[i].u8Month);
			TmpBuff[tempindex++] = b2bcd(TmpRecord.SysTimeTable[i].u8Day);
			TmpBuff[tempindex++] = b2bcd(TmpRecord.SysTimeTable[i].u8Week);
			TmpBuff[tempindex++] = b2bcd(TmpRecord.SysTimeTable[i].u8Hour);
			TmpBuff[tempindex++] = b2bcd(TmpRecord.SysTimeTable[i].u8Minute);
			TmpBuff[tempindex++] = b2bcd(TmpRecord.SysTimeTable[i].u8Second);
		}

		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_GETRESETINF;
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;
		TmpBuff[4] = (tempindex + 2)&0xFF;
		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);
	}
	else
	{
		
	}	
}

//��ȡģ�Ͳ���
void GetModelParam(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		tempindex = 5;
		
		//�������ĵ�
		TmpBuff[tempindex++] = (Smp.ZeroPos>>8)&0xFF;
		TmpBuff[tempindex++] = Smp.ZeroPos&0xFF;
		
		//����߶�
		TmpBuff[tempindex++] = (Smp.JGHeight>>8)&0xFF;
		TmpBuff[tempindex++] = Smp.JGHeight&0xFF;
		
		//���������복������
		TmpBuff[tempindex++] =(Smp.StartX>>8)&0xFF; 
		TmpBuff[tempindex++] = Smp.StartX&0xFF;
		
		//�������
		TmpBuff[tempindex++] =(Smp.LaneWidth>>8)&0xFF; 
		TmpBuff[tempindex++] = Smp.LaneWidth&0xFF;	

		//��ȫ���߶�
		TmpBuff[tempindex++] =(Smp.IsLanHt>>8)&0xFF; 
		TmpBuff[tempindex++] = Smp.IsLanHt&0xFF;

		//��ȫ����� 
		TmpBuff[tempindex++] =(Smp.IsLanWd>>8)&0xFF; 
		TmpBuff[tempindex++] = Smp.IsLanWd&0xFF;

		//���ͨ�����ո�
		TmpBuff[tempindex++] =(Smp.MaxHt>>8)&0xFF; 
		TmpBuff[tempindex++] = Smp.MaxHt&0xFF;	
		
		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_GETMDLPARAM;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;
		
		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);
	}
	else
	{
		
	}
}

//����ģ�Ͳ���
void SetModelParam(uint8 *pBuff,uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		
		//���浱ǰ����ֵ������
		AddCrc16((uint8 *)&Smp,sizeof(Smp)-2);	
		Write256_full(MDL_BAK, (uint8 *)&Smp, sizeof(Smp));
		
		//������ģ�Ͳ���
		memcpy(TmpBuff, pBuff, sizeof(TmpBuff));
		tempindex = 5;
		
		Smp.ZeroPos = (TmpBuff[tempindex]<<8)+ TmpBuff[tempindex+1];
		tempindex += 2;
		
		Smp.JGHeight = (TmpBuff[tempindex]<<8)+ TmpBuff[tempindex+1];
		tempindex += 2;
		
		Smp.StartX = (TmpBuff[tempindex]<<8)+ TmpBuff[tempindex+1];
		tempindex += 2;
		
		Smp.LaneWidth = (TmpBuff[tempindex]<<8)+ TmpBuff[tempindex+1];
		tempindex += 2;

		Smp.IsLanHt = (TmpBuff[tempindex]<<8)+ TmpBuff[tempindex+1];
		tempindex += 2;	
		
		Smp.IsLanWd = (TmpBuff[tempindex]<<8)+ TmpBuff[tempindex+1];
		tempindex += 2;
		
		Smp.MaxHt = (TmpBuff[tempindex]<<8)+ TmpBuff[tempindex+1];
		tempindex += 2;	
				
		
		if(CheckMdlParam())
		{
			InitModeParam();
			AddCrc16((uint8 *)&Smp,sizeof(Smp)-2);
			if(Write256_full(MDL_SET, (uint8 *)&Smp, sizeof(Smp)))
			{
				tempindex = 5;
				TmpBuff[tempindex++] = ERR_OK;//���óɹ�
				
				
				TmpBuff[0] = 0xFF;
				TmpBuff[1] = 0xFF;
				TmpBuff[2] = CMD_SETMDLPARAM;
				TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
				TmpBuff[4] = (tempindex + 2)&0xFF;

				AddCrc16(TmpBuff,	tempindex);
				SendBuf_full(TmpBuff,tempindex + 2,CH);	
				//��Чģ�Ͳ��� ����
				OSTimeDly(1000);
				OSSchedLock();
				ResetSystem();
				
			}
			else
			{
				tempindex = 5;
				TmpBuff[tempindex++] = CMD_SETMDLPARAM;//����ʧ�ܣ�������ʧ��
				TmpBuff[tempindex++] = ERR_TD;
				
				
				TmpBuff[0] = 0xFF;
				TmpBuff[1] = 0xFF;
				TmpBuff[2] = CMD_OPERATEERRS;
				TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
				TmpBuff[4] = (tempindex + 2)&0xFF;

				AddCrc16(TmpBuff,	tempindex);
				SendBuf_full(TmpBuff,tempindex + 2,CH);	
			}
		}
		else
		{
			tempindex = 5;
			TmpBuff[tempindex++] = CMD_SETMDLPARAM;//����ʧ�ܣ�������ʧ��
			TmpBuff[tempindex++] = ERR_LG;
			
			TmpBuff[0] = 0xFF;
			TmpBuff[1] = 0xFF;
			TmpBuff[2] = CMD_OPERATEERRS;
			TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
			TmpBuff[4] = (tempindex + 2)&0xFF;

			AddCrc16(TmpBuff,	tempindex);
			SendBuf_full(TmpBuff,tempindex + 2,CH);	
			
			//�ӱ��ݲ����лָ�
			RecBkModelParam();
			
		}
	}
	else
	{
		
	}
}

//��ȡϵͳ����
void GetSysParam(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		tempindex = 5;
		memcpy(TmpBuff+tempindex,g_SysSetUp.au8ProgramVersion,11);	 //�汾��
		tempindex += 11;
		
		TmpBuff[tempindex++] = g_SysSetUp.hResetCnt;
		TmpBuff[tempindex++] = g_SysSetUp.sResetCnt;
		//TmpBuff[tempindex++]=OSCPUUsage;
		
		TmpBuff[tempindex++] = g_SysSetUp.u8DOGEnable;
		TmpBuff[tempindex++] = g_SysSetUp.u8SDEnable;
		
		TmpBuff[tempindex++] = g_SysSetUp.u8DirDevType;
		TmpBuff[tempindex++] = g_SysSetUp.u8DirDevEnable;
		TmpBuff[tempindex++] = g_SysSetUp.u8WorkMode;
		
		memcpy(TmpBuff+tempindex,g_SysSetUp.u8DeviceID,15);
		tempindex+=15;
		
		memcpy(TmpBuff+tempindex,g_SysSetUp.u8StationID,16);
		tempindex+=16;
		
		
		
		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_GETSYSPARAM;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;
		
		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);
	}
	else
	{
		
	}
}


//����ϵͳ����
void SetSysParam(uint8 *pBuff,uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		memcpy(TmpBuff, pBuff, sizeof(TmpBuff));
		tempindex = 5;
		//�����汾������//
		tempindex += 11;
		g_SysSetUp.hResetCnt=TmpBuff[tempindex++];
		g_SysSetUp.hResetCnt=TmpBuff[tempindex++];
		
		g_SysSetUp.u8DOGEnable=TmpBuff[tempindex++];
		g_SysSetUp.u8SDEnable=TmpBuff[tempindex++];
		
		g_SysSetUp.u8DirDevType=TmpBuff[tempindex++];
		g_SysSetUp.u8DirDevEnable=TmpBuff[tempindex++];
		g_SysSetUp.u8WorkMode=TmpBuff[tempindex++];

		memcpy(g_SysSetUp.u8DeviceID,TmpBuff+tempindex,15);
		tempindex+=15;
		
		memcpy(g_SysSetUp.u8StationID,TmpBuff+tempindex,16);
		tempindex+=16;		
		
		AddCrc16((uint8 *)&g_SysSetUp,sizeof(g_SysSetUp)-2);				

		if(Write256_full(SYS_SET, (uint8 *)&g_SysSetUp, sizeof(g_SysSetUp)))
		{
			tempindex = 5;
			TmpBuff[tempindex++] = ERR_OK;//���óɹ�
			
			
			TmpBuff[0] = 0xFF;
			TmpBuff[1] = 0xFF;
			TmpBuff[2] = CMD_SETSYSPARAM;
			TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
			TmpBuff[4] = (tempindex + 2)&0xFF;

			AddCrc16(TmpBuff,	tempindex);
			SendBuf_full(TmpBuff,tempindex + 2,CH);	
			
		}
		else
		{
			tempindex = 5;
			TmpBuff[tempindex++] = CMD_SETSYSPARAM;
			TmpBuff[tempindex++] = ERR_TD;//����ʧ�ܣ�������ʧ��
			
			TmpBuff[0] = 0xFF;
			TmpBuff[1] = 0xFF;
			TmpBuff[2] = CMD_OPERATEERRS;
			TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
			TmpBuff[4] = (tempindex + 2)&0xFF;

			AddCrc16(TmpBuff,	tempindex);
			SendBuf_full(TmpBuff,tempindex + 2,CH);	
		}
	}
	else
	{
		
	}
}

//��ȡϵͳʱ��
void GetSysTime(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	if(PRTL==PTRL_FF)
	{
		tempindex = 5;
		TmpBuff[tempindex++] = SEC;
		TmpBuff[tempindex++] = MIN;
		TmpBuff[tempindex++] = HOUR;
		TmpBuff[tempindex++] = WEEK;
		TmpBuff[tempindex++] = DAY;
		TmpBuff[tempindex++] = MONTH;
		TmpBuff[tempindex++] = YEAR % 100;
		TmpBuff[tempindex++] = YEAR / 100;

		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_GETSYSTIMES;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;

		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);	
	}
	else
	{
		
	}
}

//��ȡϵͳʱ��
void SetSysTime(uint8 *pBuff,uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 temp=0;
	uint32 l_n32Temp=0;
	uint8 TmpBuff[512]={0};
	SystemTime dev_time;
	if(PRTL==PTRL_FF)
	{
		memcpy(TmpBuff, pBuff, sizeof(TmpBuff));
		temp = (TmpBuff[3]<<8)+TmpBuff[4];
		l_n32Temp = TmpBuff[11] + TmpBuff[12]*100;
		if (temp == 0x0f && l_n32Temp >= 1970 && l_n32Temp <= 2100)
		{
			dev_time.u8Second =	TmpBuff[5];
			dev_time.u8Minute = TmpBuff[6];
			dev_time.u8Hour = TmpBuff[7];
			dev_time.u8Week = TmpBuff[8];
			dev_time.u8Day = TmpBuff[9];
			dev_time.u8Month = TmpBuff[10];
			dev_time.u16Year = TmpBuff[11] + TmpBuff[12]*100; 
			SetRTCTime(&dev_time);

			OSTimeDly(5);
			tempindex = 5;
			TmpBuff[0] = 0xFF;
			TmpBuff[1] = 0xFF;
			TmpBuff[2] = CMD_SETSYSTIMES;			
			TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
			TmpBuff[4] = (tempindex + 2)&0xFF;

			AddCrc16(TmpBuff,	tempindex);
			SendBuf_full(TmpBuff, tempindex + 2,CH);	
					
		}
		else
		{
			
		}
	}	
}




/********DTU�뼤����͸������*************
**��ڣ�TouchuanBuffָ��
**���أ���
**����������DTU��Ӧ�Ĵ���1���յ�����02��
        �������ַ������IP�Ͷ˿��Լ��ַ����ݽ��зַ�
*****************************************/
void PassThroughToJG(uint8 *ToJGBuff)
{
	uint16 bufflen;
	uint8 P_IPbuff[6]={0};
	uint8 s;
	bufflen=(ToJGBuff[4]<<24)+(ToJGBuff[5]<<16)+(ToJGBuff[6]<<8)+ToJGBuff[7];
	memcpy(P_IPbuff,ToJGBuff+9+bufflen,6);
	s=SeekSocketNo(P_IPbuff);
	if(s<MAX_SOCK_NUM)
	{	
		S_tx_process(s,ToJGBuff,bufflen+9);
	}
}

void PassThroughToUart(uint8 SocketIdx)
{
	uint8 i;
	uint32 port;
	uint8 P_buf[200]={0};
	uint32 ToUartLen;
	P_buf[0]=0xFF;
	P_buf[1]=0xFF;
	if(SocketIdx==0)
	{
	    ToUartLen=(g_LaserPass.PassCont[4]<<24)+(g_LaserPass.PassCont[5]<<16)
				+(g_LaserPass.PassCont[6]<<8)+g_LaserPass.PassCont[7]+9;
		if (ToUartLen <= MAXPASSLEN-9)
		{
			for(i=0;i<ToUartLen;i++)
			{
			 	P_buf[2+i]=g_LaserPass.PassCont[i];
			}
			port=g_NetInfo.SocketPort[SocketIdx]; 

			for(i=0;i<4;i++)
			{
			 	P_buf[ToUartLen+i+2]=g_NetInfo.SocketIP[SocketIdx][3-i]; 
			}
			P_buf[ToUartLen+6]=port&0xff;
			P_buf[ToUartLen+7]=(port>>8)&0xff;
			P_buf[ToUartLen+8]=0xEE;
		}

	}

	SendBuf_full(P_buf,ToUartLen+9,g_LaserPass.PassCH);	

}

/********02���ַ�sokectѰ�Һ���*************
**��ڣ�p_IPbuffָ��
**���أ�socket���
**����������DTU��Ӧ�Ĵ���1���յ�����02��
        �������ַ������IP�Ͷ˿��Լ��ַ����ݽ��зַ�
*****************************************/
uint8 SeekSocketNo(uint8 *p_IPbuff)
{
	uint8 s=MAX_SOCK_NUM;
	uint8 j=0;
	for(j=0;j<MAX_SOCK_NUM;j++)
	{
		if(g_NetInfo.SocketIP[j][0]==p_IPbuff[3]
			&&g_NetInfo.SocketIP[j][1]==p_IPbuff[2]
			&&g_NetInfo.SocketIP[j][2]==p_IPbuff[1]
			&&g_NetInfo.SocketIP[j][3]==p_IPbuff[0])
		{
			s=j;
			break;
		}
	}
	return s;
}

//��ѯ������ֵ����
void GetVehOutTH(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		tempindex = 5;
		
		//��β�հ�֡
		TmpBuff[tempindex++] = g_VehOutSet.u8EmptyFrCnt;
		//��3��ǿ�Ƴ���
		TmpBuff[tempindex++] = g_VehOutSet.u8Ax3Force;		
		//�����豸����ֵ����
		TmpBuff[tempindex++] = g_VehOutSet.u8DirHiCnt;	
		//�����豸����ֵ����
		TmpBuff[tempindex++] = g_VehOutSet.u8DirLwCnt;	
		//�����豸�뿪����
		TmpBuff[tempindex++] = g_VehOutSet.u8DirLvCnt;	
		//��դ�����߶�����
		TmpBuff[tempindex++] = (g_VehOutSet.u16GratHt>>8)&0xFF;
		TmpBuff[tempindex++] = (g_VehOutSet.u16GratHt&0xFF);
		
		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_GETVEHOUTTH;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;
		
		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);
	}
	else
	{
		
	}
}

//���ó�����ֵ����
void SetVehOutTH(uint8 *pBuff,uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		//������ģ�Ͳ���
		memcpy(TmpBuff, pBuff, sizeof(TmpBuff));
		tempindex = 5;
		
		g_VehOutSet.u8EmptyFrCnt = TmpBuff[tempindex++];
		g_VehOutSet.u8Ax3Force = TmpBuff[tempindex++];
		g_VehOutSet.u8DirHiCnt = TmpBuff[tempindex++];
		g_VehOutSet.u8DirLwCnt = TmpBuff[tempindex++];			
		g_VehOutSet.u8DirLvCnt = TmpBuff[tempindex++];	
		g_VehOutSet.u16GratHt  = ((TmpBuff[tempindex]<<8) + TmpBuff[tempindex+1]);
		tempindex+=2;
		
		if(CheckVehOutTH())
		{
			AddCrc16((uint8 *)&g_VehOutSet,sizeof(g_VehOutSet)-2);
			if(Write256_full(VEH_OUTTH, (uint8 *)&g_VehOutSet, sizeof(g_VehOutSet)))
			{
				tempindex = 5;
				TmpBuff[tempindex++] = ERR_OK;//���óɹ�
				
				
				TmpBuff[0] = 0xFF;
				TmpBuff[1] = 0xFF;
				TmpBuff[2] = CMD_SETVEHOUTTH;
				TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
				TmpBuff[4] = (tempindex + 2)&0xFF;

				AddCrc16(TmpBuff,	tempindex);
				SendBuf_full(TmpBuff,tempindex + 2,CH);	
				
			}
			else
			{
				tempindex = 5;
				TmpBuff[tempindex++] = CMD_SETVEHOUTTH;//����ʧ�ܣ�������ʧ��
				TmpBuff[tempindex++] = ERR_TD;
				
				
				TmpBuff[0] = 0xFF;
				TmpBuff[1] = 0xFF;
				TmpBuff[2] = CMD_OPERATEERRS;
				TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
				TmpBuff[4] = (tempindex + 2)&0xFF;

				AddCrc16(TmpBuff,	tempindex);
				SendBuf_full(TmpBuff,tempindex + 2,CH);	
			}
		}
		else
		{
			tempindex = 5;
			TmpBuff[tempindex++] = CMD_SETVEHOUTTH;//����ʧ�ܣ��߼�����
			TmpBuff[tempindex++] = ERR_LG;
			
			TmpBuff[0] = 0xFF;
			TmpBuff[1] = 0xFF;
			TmpBuff[2] = CMD_OPERATEERRS;
			TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
			TmpBuff[4] = (tempindex + 2)&0xFF;

			AddCrc16(TmpBuff,	tempindex);
			SendBuf_full(TmpBuff,tempindex + 2,CH);	
			
			//�������лָ�
			RecoVehOutThresh();
		}
	}
	else
	{
		
	}
}

//��ѯ���̨����״̬
void GetWeightAtSet(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		tempindex = 5;
		
		//��̨����ͨ��
		TmpBuff[tempindex++] = g_WeigthAt.u8wAtChannel;
		//��̨����ʹ��
		TmpBuff[tempindex++] = g_WeigthAt.u8wAckEnable;		
		//��̨�����ϱ�ʹ��
		TmpBuff[tempindex++] = g_WeigthAt.u8wAxleReprot;	
		//��̨�ֳ��ϱ�ʹ��
		TmpBuff[tempindex++] = g_WeigthAt.u8wCarClassify;	
			
		
		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = CMD_GETWEIGHTAT;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;
		
		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);
	}
	else
	{
		
	}
}

//���ó�̨��������
void SetWeightAtSet(uint8 *pBuff,uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		//������ģ�Ͳ���
		memcpy(TmpBuff, pBuff, sizeof(TmpBuff));
		tempindex = 5;
		
		g_WeigthAt.u8wAtChannel = TmpBuff[tempindex++];
		g_WeigthAt.u8wAckEnable = TmpBuff[tempindex++];
		g_WeigthAt.u8wAxleReprot = TmpBuff[tempindex++];
		g_WeigthAt.u8wCarClassify = TmpBuff[tempindex++];			
		
		if(CheckWeightAtSet())
		{
			AddCrc16((uint8 *)&g_WeigthAt,sizeof(g_WeigthAt)-2);
			if(Write256_full(WEI_ATSET, (uint8 *)&g_WeigthAt, sizeof(g_WeigthAt)))
			{
				tempindex = 5;
				TmpBuff[tempindex++] = ERR_OK;//���óɹ�
				
				
				TmpBuff[0] = 0xFF;
				TmpBuff[1] = 0xFF;
				TmpBuff[2] = CMD_SETWEIGHTAT;
				TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
				TmpBuff[4] = (tempindex + 2)&0xFF;

				AddCrc16(TmpBuff,	tempindex);
				SendBuf_full(TmpBuff,tempindex + 2,CH);	
				
			}
			else
			{
				tempindex = 5;
				TmpBuff[tempindex++] = CMD_SETWEIGHTAT;//����ʧ�ܣ�������ʧ��
				TmpBuff[tempindex++] = ERR_TD;
				
				
				TmpBuff[0] = 0xFF;
				TmpBuff[1] = 0xFF;
				TmpBuff[2] = CMD_OPERATEERRS;
				TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
				TmpBuff[4] = (tempindex + 2)&0xFF;

				AddCrc16(TmpBuff,	tempindex);
				SendBuf_full(TmpBuff,tempindex + 2,CH);	
			}
		}
		else
		{
			tempindex = 5;
			TmpBuff[tempindex++] = CMD_SETWEIGHTAT;//����ʧ�ܣ��߼�����
			TmpBuff[tempindex++] = ERR_LG;
			
			TmpBuff[0] = 0xFF;
			TmpBuff[1] = 0xFF;
			TmpBuff[2] = CMD_OPERATEERRS;
			TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
			TmpBuff[4] = (tempindex + 2)&0xFF;

			AddCrc16(TmpBuff,	tempindex);
			SendBuf_full(TmpBuff,tempindex + 2,CH);	
			
			//�������лָ�
			RecoWeightAtSet();
		}
	}
	else
	{
		
	}
}



//���ݴ����[���ݡ������ȡ���ͨ��]���ⷢ������
void SendBuf_full(uint8  *RcvBufPt, uint32  Snd_Len,uint8 CH)
{
   if(CH_COM1==CH)
   {
		UART1_SendBuf_full(RcvBufPt,Snd_Len);
   }
   else if(CH_COM2==CH)
   {
		UART5_SendBuf_full(RcvBufPt,Snd_Len);
   }
   else if(CH_COMA==CH)
   {
		UART1_SendBuf_full(RcvBufPt,Snd_Len);
		UART5_SendBuf_full(RcvBufPt,Snd_Len);
   }
   else
   {
   }
}


//DLL������豸
void DllOpenCHannel(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	if(PRTL==PTRL_FF)
	{
		g_DllAtState.DllOpened=1;
		
		tempindex = 5;
		TmpBuff[tempindex++] = 0x00;
		TmpBuff[tempindex++] = 0x00;
		TmpBuff[tempindex++] = 0x00;
		TmpBuff[tempindex++] = 0x00;	

		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = DLL_OPENCHANNEL;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;

		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);	
	}
	else
	{
		
	}
}

//DLL��ѯ�豸״̬
void DllGetDeviState(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	if(PRTL==PTRL_FF)
	{
		g_DllAtState.DllOpened=1;
		
		tempindex = 5;
		if(Flag_NetConnect<1)
		{
			//����
			TmpBuff[tempindex++] = 0x16;
		}
		else
		{
			TmpBuff[tempindex++] = 0x00;
		}	

		TmpBuff[tempindex++] = 0x00;
		TmpBuff[tempindex++] = 0x00;
		TmpBuff[tempindex++] = 0x00;
		
		
		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = DLL_GETDEVSTATE;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;

		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);	
	}
	else
	{
		
	}	
}

//DLLӦ���ֶ���β����
void DllReplyManOut(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	if(PRTL==PTRL_FF)
	{
		g_DllAtState.ManHanded=1;
		
		tempindex = 5;
		TmpBuff[tempindex++] = 0x00;
		TmpBuff[tempindex++] = 0x00;
		TmpBuff[tempindex++] = 0x00;
		TmpBuff[tempindex++] = 0x00;	

		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = DLL_MANCLOSEOUT;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;

		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);	
	}
	else
	{
		
	}
}

//DLL����ر��豸
void DllCloseCHannel(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	if(PRTL==PTRL_FF)
	{
		g_DllAtState.DllOpened=0;
		
		tempindex = 5;
		TmpBuff[tempindex++] = 0x00;
		TmpBuff[tempindex++] = 0x00;
		TmpBuff[tempindex++] = 0x00;
		TmpBuff[tempindex++] = 0x00;	

		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = DLL_CLOSECHANEL;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;

		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);	
	}
	else
	{
		
	}
}

//DLL����1����
void DllAdd1VehInfo(uint8 *pBuff,uint8 CH,uint8 PRTL)
{
	SendAddVeh(pBuff[5]);
}


//DLLɾ��1����
void DllDel1VehInfo(uint8 *pBuff,uint8 CH,uint8 PRTL)
{
	SendBackVeh();
}


//DLL�ϴ�������Ϣ
void DllUpLoadQueInfo(uint8 *pBuff,uint8 CH,uint8 PRTL)
{
	g_DllAtState.CheckFrCnt=0;
	g_u8VehQueNum = pBuff[5];
	SendQueNum(CH,PRTL);
	if(g_u8VehQueNum >= 2)
	{
		g_DllAtState.QueCheckFlag = 1;
	}
}
