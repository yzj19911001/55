#include "SysRecovery.h"
#include "TDC256.h"
#include "common.h"
#include "Avi_AxleCheck.h"
#include "JZGlobal.h"
#include "PCF8563.h"
#include "w5300.h"
#include "WT_Task.h"
#include "Avi_MainProc.h"

//�ָ�ϵͳ
void RecoverySystem(void)
{
	RecoSystemParam();
	RecoModelParam();
	RecoVehOutThresh();
	RecoveryNetParam();
	RecoveryUartSet();
	RecoWeightAtSet();
	InitGlobalParam();
	RecoVehIndex();
}

//�ָ�����ID���
void RecoVehIndex(void)
{
	SystemTime	SysTime;
	GetRTCTime(&SysTime); 
	ReadC256(CTR_VEHID,(uint8 *)&g_VehIndex, sizeof(g_VehIndex));
    if(CheckCrc((uint8 *)&g_VehIndex,sizeof(g_VehIndex)-2) == 0 )
    {
       DefualtVehIndex();
    }
	if(g_VehIndex.u16Year!=SysTime.u16Year
		||g_VehIndex.u8Mon!=SysTime.u8Month
	||g_VehIndex.u8Day!=SysTime.u8Day)
	{
		DefualtVehIndex();
	}
}

//���ó�����ż�¼��ΪĬ��ֵ
void DefualtVehIndex(void)
{
	SystemTime	SysTime;
	GetRTCTime(&SysTime);
	g_VehIndex.u16Year=SysTime.u16Year;
	g_VehIndex.u8Mon=SysTime.u8Month;
	g_VehIndex.u8Day=SysTime.u8Day;
	g_VehIndex.u32Index=0;
	AddCrc16((uint8 *)&g_VehIndex,sizeof(g_VehIndex)-2);
	Write256_full(CTR_VEHID, (uint8 *)&g_VehIndex, sizeof(g_VehIndex));
}

//�洢����ID
void SaveVehIndex(void)
{
	AddCrc16((uint8 *)&g_VehIndex,sizeof(g_VehIndex)-2);
	Write256_full(CTR_VEHID, (uint8 *)&g_VehIndex, sizeof(g_VehIndex));
}


//�ָ�ϵͳ����
void RecoSystemParam(void)
{
	ReadC256(SYS_SET,(uint8 *)&g_SysSetUp, sizeof(g_SysSetUp));
    if(CheckCrc((uint8 *)&g_SysSetUp,sizeof(g_SysSetUp)-2) == 0 )
    {
       DefualtSysParam();
    }
}


//Ĭ��ϵͳ����
void DefualtSysParam(void)
{
	g_SysSetUp.u8DOGEnable=1;
	g_SysSetUp.u8SDEnable=1;
	g_SysSetUp.hResetCnt=0;
	g_SysSetUp.sResetCnt=0;
	g_SysSetUp.u8DirDevType=DEV_COIL;
	g_SysSetUp.u8DirDevEnable=1;
	g_SysSetUp.u8WorkMode=0;
	memcpy(g_SysSetUp.u8DeviceID,DEVICEINIT,11);
	memcpy(g_SysSetUp.u8StationID,STATIONINIT,11);
	memcpy(g_SysSetUp.au8ProgramVersion,APP_VERSION,11);
	AddCrc16((uint8 *)&g_SysSetUp,sizeof(g_SysSetUp)-2);
	Write256_full(SYS_SET, (uint8 *)&g_SysSetUp, sizeof(g_SysSetUp));
}


//�ָ�ģ�Ͳ���
void RecoModelParam(void)
{
	ReadC256(MDL_SET,(uint8 *)&Smp, sizeof(Smp));
    if(CheckCrc((uint8 *)&Smp,sizeof(Smp)-2) == 0 )
    {
       DefualtModelParam();
    }
	
	//������������Ҫ�� ��Ĭ�ϲ���
	if(CheckMdlParam()==0)
	{
		RecBkModelParam();
	}
	
	InitModeParam();
}
//���ñ��ݲ�������ģ�;��廯
void RecBkModelParam()
{
	ReadC256(MDL_BAK,(uint8 *)&Smp, sizeof(Smp));
    if(CheckCrc((uint8 *)&Smp,sizeof(Smp)-2) == 0 )
    {
       DefualtModelParam();
	   BuzzerBeep(3,50);
    }
	
	InitModeParam();
}

//Ĭ��ģ�Ͳ���
void DefualtModelParam(void)
{
	//���ɽ���� 20170524
	Smp.ZeroPos = 320;
	Smp.JGHeight = 1200;
	Smp.StartX=600;
	Smp.LaneWidth=2800;
	Smp.MaxHt=5000;
	Smp.IsLanHt=ISLANHT;
	Smp.IsLanWd=ISLANWD;
	
	InitModeParam();
	
	AddCrc16((uint8 *)&Smp,sizeof(Smp)-2);
	Write256_full(MDL_SET, (uint8 *)&Smp, sizeof(Smp));
}

//�ָ��������������ֵ����
void RecoVehOutThresh(void)
{
	ReadC256(VEH_OUTTH,(uint8 *)&g_VehOutSet, sizeof(g_VehOutSet));
    if(CheckCrc((uint8 *)&g_VehOutSet,sizeof(g_VehOutSet)-2) == 0 )
    {
       DefualtVehOutTH();
    }
	
	//������������Ҫ�� ��Ĭ�ϲ���
	if(CheckVehOutTH()==0)
	{
		DefualtVehOutTH();
	}
}

//Ĭ�ϳ�����ֵ����
void DefualtVehOutTH(void)
{
	g_VehOutSet.u8DirHiCnt=NORMAL_MAX_EMPTYFRAME;
	g_VehOutSet.u8DirLwCnt=NORMAL_MAX_EMPTYFRAME;
	g_VehOutSet.u8DirLwCnt=NORMAL_MAX_EMPTYFRAME-1;
	g_VehOutSet.u8EmptyFrCnt=NORMAL_MAX_EMPTYFRAME;
	g_VehOutSet.u8Ax3Force=0;
	AddCrc16((uint8 *)&g_VehOutSet,sizeof(g_VehOutSet)-2);
	Write256_full(VEH_OUTTH, (uint8 *)&g_VehOutSet, sizeof(g_VehOutSet));
}


//��������ֵ����
uint8 CheckVehOutTH(void)
{
	if(g_VehOutSet.u8EmptyFrCnt<NORMAL_MAX_EMPTYFRAME)
	{
		return 0;
	}
	else if(g_VehOutSet.u16GratHt<300||g_VehOutSet.u16GratHt>3000)
	{
		return 0;
	}
	return 1;
}


//�ָ����̨������������
void RecoWeightAtSet(void)
{
	ReadC256(WEI_ATSET,(uint8 *)&g_WeigthAt, sizeof(g_WeigthAt));
    if(CheckCrc((uint8 *)&g_WeigthAt,sizeof(g_WeigthAt)-2) == 0 )
    {
       DefualtWeightAtSet();
    }
	
	//������������Ҫ�� ��Ĭ�ϲ���
	if(CheckWeightAtSet()==0)
	{
		DefualtWeightAtSet();
	}
}


//Ĭ�����̨������������
void DefualtWeightAtSet(void)
{
	g_WeigthAt.u8wAtChannel=CH_NOCH;
	g_WeigthAt.u8wAckEnable=0;
	g_WeigthAt.u8wAxleReprot=0;
	g_WeigthAt.u8wCarClassify=0;
	AddCrc16((uint8 *)&g_WeigthAt,sizeof(g_WeigthAt)-2);
	Write256_full(WEI_ATSET, (uint8 *)&g_WeigthAt, sizeof(g_WeigthAt));
}

//����̨��������
uint8 CheckWeightAtSet(void)
{
	if(g_WeigthAt.u8wAtChannel!=CH_COM1 
		&& g_WeigthAt.u8wAtChannel!=CH_COM2
	    && g_WeigthAt.u8wAtChannel!=CH_COMA)
	{
		return 0;
	}
	else if(g_WeigthAt.u8wAckEnable>2)
	{
		return 0;
	}
	else if(g_WeigthAt.u8wAxleReprot>2)
	{
		return 0;
	}
	else if(g_WeigthAt.u8wCarClassify>2)
	{
		return 0;
	}
	return 1;
}


//������������
void RecordReset(void)	
{		
	SystemTime	l_sstResetTime;
	
	ResetRecord TmpRecord;
	memset(&l_sstResetTime,0,sizeof(l_sstResetTime));	
	memset((uint8 *)&TmpRecord, 0, sizeof(TmpRecord));
	
	GetRTCTime(&l_sstResetTime); 
	Read256_full(CTR_RESET, (uint8 *)&TmpRecord, sizeof(TmpRecord));
	if(TmpRecord.u8CurCnt>MAX_RESETCNT)
	{
		memset((uint8 *)&TmpRecord, 0, sizeof(TmpRecord));
	}	
	memcpy(&TmpRecord.SysTimeTable[TmpRecord.u8CurPos], &l_sstResetTime, sizeof(l_sstResetTime));
	if((WDTIM_RES&0x01)==0x00)
	{
		TmpRecord.SysTimeTable[TmpRecord.u8CurPos].u8Week=0;//ʱ������û�� ֱ��������������  00����Ӳ������ 01 �����������
		g_SysSetUp.hResetCnt++;
	}
	else if((WDTIM_RES&0x01)==0x01)
	{
	  	TmpRecord.SysTimeTable[TmpRecord.u8CurPos].u8Week=1;
		g_SysSetUp.sResetCnt++;
	}
	TmpRecord.u8CurPos=(TmpRecord.u8CurPos+1) % MAX_RESETCNT;
	TmpRecord.u8CurCnt++;
	if(TmpRecord.u8CurCnt>MAX_RESETCNT)
	{
		TmpRecord.u8CurCnt=MAX_RESETCNT;
	}
	Write256_full(CTR_RESET, (uint8 *)&TmpRecord, sizeof(TmpRecord));
}


//�ָ��������
void RecoveryNetParam(void)
{
	ReadC256(CTR_NETSET,(uint8 *)&g_NetInfo, sizeof(g_NetInfo));
    if(CheckCrc((uint8 *)&g_NetInfo,sizeof(g_NetInfo)-2) == 0 )
    {
       DefualtNetParam();
    }
	W5300_Init(&g_NetInfo);//��ʼ��5300
}

//Ĭ���������
void DefualtNetParam(void)
{
	g_NetInfo.au8IPAddr[0] = 192;
	g_NetInfo.au8IPAddr[1] = 168;
	g_NetInfo.au8IPAddr[2] = 0;
	g_NetInfo.au8IPAddr[3] = 111;
	g_NetInfo.u32LocalPortNO=4000;
	g_NetInfo.au8SubMask[0]=255;
	g_NetInfo.au8SubMask[1]=255;
	g_NetInfo.au8SubMask[2]=255;
	g_NetInfo.au8SubMask[3]=0;
	g_NetInfo.au8MACAddr[0] =	0x52;
	g_NetInfo.au8MACAddr[1] =	0x54;
	g_NetInfo.au8MACAddr[2] =	0x4c;
	g_NetInfo.au8MACAddr[3] =	0x19;
	g_NetInfo.au8MACAddr[4] =	0xf7;
	g_NetInfo.au8MACAddr[5] =	0x55;

	g_NetInfo.SocketIP[0][0]=192;
	g_NetInfo.SocketIP[0][1]=168;
	g_NetInfo.SocketIP[0][2]=0;
	g_NetInfo.SocketIP[0][3]=2;
	g_NetInfo.SocketPort[0]=2110;

	g_NetInfo.SocketIP[1][0]=192;
	g_NetInfo.SocketIP[1][1]=168;
	g_NetInfo.SocketIP[1][2]=0;
	g_NetInfo.SocketIP[1][3]=3;
	g_NetInfo.SocketPort[0]=2114;
	
	AddCrc16((uint8 *)&g_NetInfo,sizeof(g_NetInfo)-2);
	Write256_full(CTR_NETSET, (uint8 *)&g_NetInfo, sizeof(g_NetInfo));	
}

//�ָ���������
void RecoveryUartSet()
{
	ReadC256(CTR_UARTSET,(uint8 *)&g_UartSet, sizeof(g_UartSet));
    if(CheckCrc((uint8 *)&g_UartSet,sizeof(g_UartSet)-2) == 0 )
    {
       DefualtUartSet();
    }
	
}

//Ĭ���������
void DefualtUartSet(void)
{
	g_UartSet.BR_COM1=5;
	g_UartSet.BR_COM2=5;
	g_UartSet.RM_CHCOM=CH_COM1;
	g_UartSet.TS_CHCOM=CH_COM2;
	AddCrc16((uint8 *)&g_UartSet,sizeof(g_UartSet)-2);
	Write256_full(CTR_UARTSET, (uint8 *)&g_UartSet, sizeof(g_UartSet));	
}


//ͨ�������ʵȼ��õ�������ֵ
uint32 GetUBR(uint8 UBR_I)
{
   uint32 UBR_VALUE=0;
   switch(UBR_I)
   {
	 case 1:
		UBR_VALUE=UBR_4800;
	 	break;
	 case 2:
		UBR_VALUE=UBR_9600;
	 	break;
	 case 3:
		UBR_VALUE=UBR_19200;
	 	break;
	 case 4:
		UBR_VALUE=UBR_57600;
	 	break;
	 case 5:
		UBR_VALUE=UBR_115200;
	 	break;
	 default:
		UBR_VALUE=UBR_115200;
	 	break;
   }
   return UBR_VALUE;
}

//���ģ�Ͳ����Ƿ�ǡ��
uint8 CheckMdlParam(void)
{	
	if(Smp.ZeroPos>POINT_END)
	{
		//���ĵ�������������
		return 0;
	}
	else if(Smp.IsLanWd<ISLANWD ||Smp.IsLanHt<ISLANHT)
	{
		//��ȫ���߶ȺͿ��̫С
		return 0;
	}
	else if(Smp.MaxHt<5500)
	{
		return 0;
	}
	return 1;
}


//��ʼ��ȫ�ֲ���
void InitGlobalParam(void)																
{ 
	g_LmdCacheIn =0;
	g_LmdCacheOut=0;
	g_DllAtState.DllOpened=1;
	g_u8VehQueIndex = 0; //g_u8VehQue�������������
	memset(g_u8VehQue,0,sizeof(uint8)*MAXVEHQUE); //����ϴ���10��������Ϣ���
}


//ʹ�������ָ���������
void MakeCtr2Factory(void)
{
	DefualtSysParam();
	DefualtModelParam();
	DefualtNetParam();
	DefualtUartSet();
}
