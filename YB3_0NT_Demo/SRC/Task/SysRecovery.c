#include "SysRecovery.h"
#include "TDC256.h"
#include "common.h"
#include "Avi_AxleCheck.h"
#include "JZGlobal.h"
#include "PCF8563.h"
#include "w5300.h"
#include "WT_Task.h"
#include "Avi_MainProc.h"

//恢复系统
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

//恢复车辆ID序号
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

//设置车辆序号记录集为默认值
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

//存储车辆ID
void SaveVehIndex(void)
{
	AddCrc16((uint8 *)&g_VehIndex,sizeof(g_VehIndex)-2);
	Write256_full(CTR_VEHID, (uint8 *)&g_VehIndex, sizeof(g_VehIndex));
}


//恢复系统参数
void RecoSystemParam(void)
{
	ReadC256(SYS_SET,(uint8 *)&g_SysSetUp, sizeof(g_SysSetUp));
    if(CheckCrc((uint8 *)&g_SysSetUp,sizeof(g_SysSetUp)-2) == 0 )
    {
       DefualtSysParam();
    }
}


//默认系统参数
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


//恢复模型参数
void RecoModelParam(void)
{
	ReadC256(MDL_SET,(uint8 *)&Smp, sizeof(Smp));
    if(CheckCrc((uint8 *)&Smp,sizeof(Smp)-2) == 0 )
    {
       DefualtModelParam();
    }
	
	//检查参数不符合要求 则默认参数
	if(CheckMdlParam()==0)
	{
		RecBkModelParam();
	}
	
	InitModeParam();
}
//采用备份参数进行模型具体化
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

//默认模型参数
void DefualtModelParam(void)
{
	//凤凰山参数 20170524
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

//恢复车辆出车相关阈值设置
void RecoVehOutThresh(void)
{
	ReadC256(VEH_OUTTH,(uint8 *)&g_VehOutSet, sizeof(g_VehOutSet));
    if(CheckCrc((uint8 *)&g_VehOutSet,sizeof(g_VehOutSet)-2) == 0 )
    {
       DefualtVehOutTH();
    }
	
	//检查参数不符合要求 则默认参数
	if(CheckVehOutTH()==0)
	{
		DefualtVehOutTH();
	}
}

//默认出车阈值参数
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


//检查出车阈值参数
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


//恢复与称台交互参数设置
void RecoWeightAtSet(void)
{
	ReadC256(WEI_ATSET,(uint8 *)&g_WeigthAt, sizeof(g_WeigthAt));
    if(CheckCrc((uint8 *)&g_WeigthAt,sizeof(g_WeigthAt)-2) == 0 )
    {
       DefualtWeightAtSet();
    }
	
	//检查参数不符合要求 则默认参数
	if(CheckWeightAtSet()==0)
	{
		DefualtWeightAtSet();
	}
}


//默认与称台交互参数设置
void DefualtWeightAtSet(void)
{
	g_WeigthAt.u8wAtChannel=CH_NOCH;
	g_WeigthAt.u8wAckEnable=0;
	g_WeigthAt.u8wAxleReprot=0;
	g_WeigthAt.u8wCarClassify=0;
	AddCrc16((uint8 *)&g_WeigthAt,sizeof(g_WeigthAt)-2);
	Write256_full(WEI_ATSET, (uint8 *)&g_WeigthAt, sizeof(g_WeigthAt));
}

//检查称台交互参数
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


//更新重启次数
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
		TmpRecord.SysTimeTable[TmpRecord.u8CurPos].u8Week=0;//时间内周没用 直接用作重启类型  00代表硬件重启 01 代表软件重启
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


//恢复网络参数
void RecoveryNetParam(void)
{
	ReadC256(CTR_NETSET,(uint8 *)&g_NetInfo, sizeof(g_NetInfo));
    if(CheckCrc((uint8 *)&g_NetInfo,sizeof(g_NetInfo)-2) == 0 )
    {
       DefualtNetParam();
    }
	W5300_Init(&g_NetInfo);//初始化5300
}

//默认网络参数
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

//恢复串口设置
void RecoveryUartSet()
{
	ReadC256(CTR_UARTSET,(uint8 *)&g_UartSet, sizeof(g_UartSet));
    if(CheckCrc((uint8 *)&g_UartSet,sizeof(g_UartSet)-2) == 0 )
    {
       DefualtUartSet();
    }
	
}

//默认网络参数
void DefualtUartSet(void)
{
	g_UartSet.BR_COM1=5;
	g_UartSet.BR_COM2=5;
	g_UartSet.RM_CHCOM=CH_COM1;
	g_UartSet.TS_CHCOM=CH_COM2;
	AddCrc16((uint8 *)&g_UartSet,sizeof(g_UartSet)-2);
	Write256_full(CTR_UARTSET, (uint8 *)&g_UartSet, sizeof(g_UartSet));	
}


//通过波特率等级得到波特率值
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

//检查模型参数是否恰当
uint8 CheckMdlParam(void)
{	
	if(Smp.ZeroPos>POINT_END)
	{
		//中心点序大于最大点序号
		return 0;
	}
	else if(Smp.IsLanWd<ISLANWD ||Smp.IsLanHt<ISLANHT)
	{
		//安全岛高度和宽度太小
		return 0;
	}
	else if(Smp.MaxHt<5500)
	{
		return 0;
	}
	return 1;
}


//初始化全局参数
void InitGlobalParam(void)																
{ 
	g_LmdCacheIn =0;
	g_LmdCacheOut=0;
	g_DllAtState.DllOpened=1;
	g_u8VehQueIndex = 0; //g_u8VehQue数组的索引清零
	memset(g_u8VehQue,0,sizeof(uint8)*MAXVEHQUE); //最近上传的10个车辆信息清空
}


//使控制器恢复出厂设置
void MakeCtr2Factory(void)
{
	DefualtSysParam();
	DefualtModelParam();
	DefualtNetParam();
	DefualtUartSet();
}
