#include "stdafx.h"
#include "Avi_MainProc.h"
#include "Avi_MathProc.h"
#include "Avi_AxleCheck.h"
#include "Avi_FeatureCal.h"
#include "Avi_Classifier.h"
#include "Avi_AxleCheck.h"


#ifdef _KeilEnVi
#include "PrtlProc.h"
#include "Common.h"
#include "Task_CheckDir.h"
#include "Task_AqcWave.h"
#include "SysRecovery.h"
#else
#include "Avi_Main.h"
#include "LmdsProc.h"
uint32 g_n32WaveNumber;//vc中用
SysModelParam Smp;//系统模型参数
FramInfo CurFramInfo;//当前帧信息
VehOutSet g_VehOutSet;//车辆收尾相关阈值
int32 g_FiltX[POINT_SUM]; //扫描点X坐标滤波后坐标
int32 g_FiltZ[POINT_SUM]; //扫描点Z坐标滤波后坐标
int32 LMS_data_1[POINT_SUM+1];//扫描距离数组
VehIndexSet  g_VehIndex;//车辆序号记录集
int32 g_CoorX[POINT_SUM] = {0}; //扫描点X坐标
int32 g_CoorZ[POINT_SUM] = {0}; //扫描点Z坐标
#endif



//uint8 g_VehicleSetIndex[VEHICLE_MAX];//车辆缓存索引数组	
//VehicleSet g_VehicleSet[VEHICLE_MAX];//车辆记录集结构体
VehicleSet g_VehicleSet;
//uint8 g_totalVehicle=0;//当前总车数

/************************************************************************/
/* 车型识别处理                                                         */
/************************************************************************/
void VehIdentProc(void)
{
	//清空中间件
	InitMiddleWare();

#ifdef _KeilEnVi
	CheckAbFramCnt();
	ForceCloseProc();
	CheckQueState();
#endif	
	
	Polar2Rect();

	FramAnlsye();

	VehRegionMatch();
	
#ifdef _KeilEnVi
	VehDirCheck();
#endif	
	CheckVehState();

}

/************************************************************************/
/* 清空中间件                                                           */
/************************************************************************/
void InitMiddleWare(void)
{
	//清空坐标数组
	memset(g_FiltZ,0,sizeof(g_FiltZ));
	memset(g_FiltX,0,sizeof(g_FiltX));
	memset(g_CoorX,0,sizeof(g_CoorX));
	memset(g_CoorZ,0,sizeof(g_CoorZ));

	//清空帧记录集
	memset(&CurFramInfo,0,sizeof(CurFramInfo));
}

/************************************************************************/
/* 模型参数初始化                                                       */
/************************************************************************/
void InitModeParam(void)
{	
#ifdef _KeilEnVi	
	
#else 
	//凤凰山参数 20170524
	//Smp.ZeroPos = 320;
	//Smp.JGHeight = 1150;
	//Smp.StartX=600;
	//Smp.EndX=3200;
	//Smp.MaxHt=5000;
	//Smp.StartX0=Smp.StartX-ISLANWD;
	//Smp.IsLanHt=ISLANHT;
	//Smp.ScanMode=H2LSCAN;	
#endif	
	
	if (Smp.ZeroPos>POINT_CEN)
	{
		Mid_Point_Cal=Smp.ZeroPos;
	}
	else
	{
		Mid_Point_Cal=POINT_END-Smp.ZeroPos;
	}
	SetAxleParam();
}

/************************************************************************/
/* 极坐标转直角坐标 附带滤波   分大小点进车                             */
/************************************************************************/
uint8 Polar2Rect(void)
{
	int16 u16Index=0;
	uint32 Rx=0;
	uint32 Rz=0;
	int32 dif = 0;
	uint32 ThresDis[POINT_SUM] = {0};//每点滤波阈值
	uint32 min_neighour = 65535;
	int32 TmpL=0;
	int32 abs_temp = 0;

	if( Smp.ZeroPos > POINT_CEN ) 
	{
		//由高到低
	}
	else 
	{
		// 由低到高
		for(u16Index = 0; u16Index <POINT_CEN; u16Index++)
		{
			TmpL  = LMS_data_1[u16Index];
			LMS_data_1[u16Index] = LMS_data_1[POINT_END -u16Index];
			LMS_data_1[POINT_END -u16Index] = TmpL ;
		}
	}


	for (u16Index = 0; u16Index<=Mid_Point_Cal; u16Index++)
	{
		if (LMS_data_1[u16Index]<LMS_MINTH)
		{
			//认为打飞或测距不可信扫描点
			g_CoorX[u16Index]=0;
			g_CoorZ[u16Index]=0;
		}
		else
		{
			//进行直角坐标转换
			if(u16Index < Mid_Point_Cal && u16Index >=0)//坐标转换，
			{
				if(ABS(Mid_Point_Cal-u16Index)<=POINT_CEN)
				{
					abs_temp = Smp.JGHeight - (LMS_data_1[u16Index] * Tabcos[ABS(Mid_Point_Cal - u16Index)] >> 15);
					g_CoorZ[u16Index] = (abs_temp);
					g_CoorX[u16Index] = (LMS_data_1[u16Index]*Tabsin[ABS(Mid_Point_Cal-u16Index)]>>15);
				}
				else
				{
					abs_temp = Smp.JGHeight + (LMS_data_1[u16Index] * Tabsin[ABS(Mid_Point_Cal - u16Index) % POINT_CEN] >> 15);
					g_CoorZ[u16Index] = (abs_temp);
					g_CoorX[u16Index] = (LMS_data_1[u16Index]*Tabcos[ABS(Mid_Point_Cal-u16Index)%POINT_CEN]>>15);
				}
			}
			else
			{
				abs_temp = Smp.JGHeight - ((LMS_data_1[u16Index] * Tabcos[ABS(Mid_Point_Cal - u16Index)]) >> 15);
				g_CoorZ[u16Index] = (abs_temp);
				g_CoorX[u16Index] = -1*(LMS_data_1[u16Index]*Tabsin[ABS(Mid_Point_Cal-u16Index)])>>15;

			}

			//不在车道范围和不在正常高度范围内扫描点 置为0
			if (g_CoorX[u16Index]>Smp.StartX+Smp.LaneWidth||g_CoorZ[u16Index]>Smp.MaxHt)
			{
				g_CoorX[u16Index]=0;
				g_CoorZ[u16Index]=0;
			}
		}

		//只对一定高度和X范围内的点进行滤波操作 减少CPU使用率
		if(g_CoorX[u16Index]<Smp.StartX-300||g_CoorX[u16Index]>g_CoorX[u16Index]<Smp.StartX+Smp.LaneWidth/2
			||g_CoorZ[u16Index]>1200)
		{
			g_FiltX[u16Index]=g_CoorX[u16Index];
			g_FiltZ[u16Index]=g_CoorZ[u16Index];
			continue;
		}
		
		
		Rx = TabRsin[ABS(u16Index - Mid_Point_Cal)];
		Rz = TabRcos[ABS(u16Index - Mid_Point_Cal)];


		//以最大值为滤波阈值
		if( Rx > Rz )
			ThresDis[u16Index] = Rx*LMS_data_1[u16Index]/1000;
		else
			ThresDis[u16Index] = Rz*LMS_data_1[u16Index]/1000;

		//如果没有，阈值过小默认为30mm
		if( ThresDis[u16Index] < 30 )
			ThresDis[u16Index] = 30;

		//3邻域滤波
		if( u16Index > 2 && u16Index < 358 )
		{
			min_neighour = 65535;
			//abs_temp = 0;
			//if (g_CoorX[u16Index] < 100 && g_CoorX[u16Index] > Smp.StartX  &&  g_CoorX[u16Index] < Smp.StartX+Smp.LaneWidth ) //对高度100以下的点进行3点均值滤波
			//{
			//	for (dif = -1; dif <= 1; dif++)
			//		abs_temp =+ g_CoorZ[u16Index + dif];

			//	g_CoorZ[u16Index] = abs_temp / 3;
			//}
			abs_temp = 0;
			for(dif = -3; dif <= 3; dif++)
			{	
				if (dif == 0)
					continue;
				abs_temp = (LMS_data_1[u16Index] - LMS_data_1[u16Index + dif]) / dif;
				abs_temp = ABS(abs_temp);
				if ( abs_temp < min_neighour)
				{
					min_neighour = abs_temp;
				}
			}
			if(min_neighour > ThresDis[u16Index] * 115/100 && LMS_data_1[u16Index+1] > 20 && LMS_data_1[u16Index -1]>20)
			{
				//滤掉不要
			}
			else
			{
				g_FiltZ[u16Index] = g_CoorZ[u16Index];
				g_FiltX[u16Index] = g_CoorX[u16Index];
			}
		}
		else if( u16Index >= 0 && u16Index <= 360  && g_CoorX[u16Index] && g_CoorZ[u16Index] )
		{
			g_FiltZ[u16Index] = g_CoorZ[u16Index];
			g_FiltX[u16Index] = g_CoorX[u16Index];
		}			
	}
	return ERROR_NONE;
}

/************************************************************************/
/* 帧分析                                                           */
/************************************************************************/
void FramAnlsye()
{
	uint16 u16Index=0;
	int32 iTmpMaxX=0;
	int32 iTmpMinX=0xFFFF;
	uint32 TmpSumX=0;
	//初始化帧记录集
	CurFramInfo.u16MaxHt=0;
	CurFramInfo.u16SecHt=0;
	CurFramInfo.n32xMin=0xFFFF;
	CurFramInfo.n32xMax=0;
	CurFramInfo.u32TimeStamp=LMS_data_1[POINT_SUM];
	//第一次遍历  寻找车身扫描点  及各高度限起始点
	for( u16Index = Mid_Point_Cal; u16Index>1; u16Index--)
	{
		//用于车型识别 寻找车辆车身扫描点
		if (g_FiltX[u16Index] > Smp.StartX-Smp.IsLanWd &&  g_FiltX[u16Index] <Smp.StartX+Smp.LaneWidth)
		{
			if (g_FiltZ[u16Index] > VEHTH_LOW  &&  g_FiltZ[u16Index]<VEHTH_UP  &&  
				((g_FiltX[u16Index]<Smp.StartX  &&  g_FiltZ[u16Index] > Smp.IsLanHt)||(g_FiltX[u16Index]>Smp.StartX)))
			{
#ifdef _KeilEnVi 				
				if(g_FiltZ[u16Index]<MIN(g_VehOutSet.u16GratHt,3000))
				{
					CurFramInfo.u16GrFitCnt++;
				}
#endif				
				if(CurFramInfo.u16EndPos == 0)
				{
					//找到车身终止点
					CurFramInfo.u16StaPos=u16Index;
					CurFramInfo.u16EndPos=u16Index;
				}
				else
				{
					//车身起始点
					CurFramInfo.u16StaPos=u16Index;
				}

				if (CurFramInfo.u16MaxHt<g_FiltZ[u16Index])
				{
					//车身最高点 
					CurFramInfo.u16SecHt=CurFramInfo.u16MaxHt;
					CurFramInfo.u16MaxHt=g_FiltZ[u16Index];
					CurFramInfo.u16MaxHtPt=u16Index;
					
				}
				else if (CurFramInfo.u16SecHt<g_FiltZ[u16Index])
				{
					CurFramInfo.u16SecHt=g_FiltZ[u16Index];
				}
				

				if (g_FiltX[u16Index]>CurFramInfo.n32xMax)
				{
					//车身最远点
					CurFramInfo.n32xMax=g_FiltX[u16Index];
					CurFramInfo.u16xMaxPt=u16Index;
				}

				if (g_FiltX[u16Index]<CurFramInfo.n32xMin)
				{
					//车身最近点
					CurFramInfo.n32xMin=g_FiltX[u16Index];
					CurFramInfo.u16xMinPt=u16Index;
				}

				TmpSumX+=g_FiltX[u16Index];

				if (g_FiltZ[u16Index]>500)
				{
					if (g_FiltX[u16Index]<iTmpMinX)
					{
						iTmpMinX=g_FiltX[u16Index];
					}
					if (g_FiltX[u16Index]>iTmpMaxX)
					{
						iTmpMaxX=g_FiltX[u16Index];
					}
					CurFramInfo.u16Width=iTmpMaxX-iTmpMinX;
				}
				CurFramInfo.u16PtSum++;
			}
			else
			{
				//地面点过低的 置为-1 区别打飞
				//g_FiltZ[u16Index]=-1;
			}
		}
		else
		{
			//不在范围内的点直接去掉，防止平面镜反光出现散点 20170729
			g_FiltZ[u16Index]=0;
			g_FiltX[u16Index]=0;
		}


	}

	//判断该区域是否为有车合法区域
	if (CurFramInfo.u16PtSum>2 && CurFramInfo.u16MaxHt>300)
	{
		CurFramInfo.u8Valid=1;
		CurFramInfo.u16AvgX=TmpSumX/CurFramInfo.u16PtSum;
#ifdef _KeilEnVi 		
		if(CurFramInfo.u16GrFitCnt>8)
		{
			g_VehicleSet.DirSet.u8GR_Valided=1;
			g_VehicleSet.DirSet.u16GR_FitFrCnt++;
		}
#endif		
	}
	else
	{
#ifdef _KeilEnVi 		
		g_VehicleSet.DirSet.u8GR_Valided=0;
#endif		
	}
}


/************************************************************************/
/* 区域车辆匹配                                                         */
/************************************************************************/
void VehRegionMatch()
{
	if (CurFramInfo.u8Valid == 1)
	{
		if (g_VehicleSet.u8State!=NO_USED)
		{
			if (IsRegionMatch(CurFramInfo,g_VehicleSet.CurFrmInfo))
			{
				CurFramInfo.u8Matched=1;
				g_VehicleSet.EndWaveIndex=g_n32WaveNumber;
				UpdateVehRegion(&g_VehicleSet);
			}
		}

		if (CurFramInfo.u8Matched == 0)
		{
			//该区域未匹配成功，认为有新车
			if(g_VehicleSet.u8State  ==  NO_USED)
			{
				g_VehicleSet.u8State = OCCURING_USED;
				g_VehicleSet.StaWaveIndex=g_n32WaveNumber;
				g_VehicleSet.EndWaveIndex=g_n32WaveNumber;
				CurFramInfo.u8Matched=1;
				UpdateVehRegion(&g_VehicleSet);
			}
		}
	}
}

/************************************************************************/
/* 检查区域是否匹配                                                     */
/* 单车道默认匹配成功    此处存如果不加入位置约束存在分车的隐患         */
/************************************************************************/
uint8 IsRegionMatch(FramInfo FI1,FramInfo FI2)
{
	//同一车道 默认匹配成功  此处存如果不加入位置约束存在分车的隐患
	if (IS_INSIDE(FI1.n32xMin,FI1.n32xMax,FI2.n32xMin,FI2.n32xMax))
	{
		//区域重叠
		return 1;
	}
	else if (abs((FI1.n32xMax+FI1.n32xMin)-(FI2.n32xMax+FI2.n32xMin))<800)
	{
		//区域中心点差距400以内
		return 1;
	}
	else if ((FI1.n32xMax<FI2.n32xMin&&FI2.n32xMin-FI1.n32xMax<400)
		||(FI2.n32xMax<FI1.n32xMin&&FI1.n32xMin-FI2.n32xMax<400))
	{
		//区域边界差距400以内
		return 1;
	}
	else
	{
		return 1;
	}
}

#ifdef _KeilEnVi
/************************************************************************/
/* 感知车辆行进方向                                                     */
/************************************************************************/
void VehDirCheck(void)
{
	
	if(g_VehicleSet.u8State!=NO_USED)
	{
		if(g_SysSetUp.u8WorkMode)
			g_VehicleSet.DirSet.u8DirTrigged=1;
		switch(g_SysSetUp.u8DirDevType)
		{
			case DEV_COIL:
				Dev_CoilDirCheck();
				break;
			case DEV_GRAT:
				Dev_GratDirCheck();
			    break;
			case DEV_LASE:
				Dev_LaseDirCheck();
				break;
			default:
				break;
		}
	}
}

/************************************************************************/
/* 线圈感知方向计数                                                     */
/************************************************************************/
void Dev_CoilDirCheck(void)
{
	if(g_DirTrigged)
	{
		//方向设备使能 离开状态检测才生效
		g_VehicleSet.DirSet.u8DirTrigged=1;
		if(g_VehicleSet.u16FrameCnt<g_VehOutSet.u8DirHiCnt)
		{
			g_VehicleSet.DirSet.u8CL_StateHi++;
		}
		if(g_VehicleSet.u16FrameCnt<g_VehOutSet.u8DirLwCnt)
		{
			g_VehicleSet.DirSet.u8CL_StateLw++;
		}
		if(g_VehicleSet.EmptyFrame<1)
		{
			g_VehicleSet.DirSet.u8CL_StateLv=0;
		}
		else
		{
			g_VehicleSet.DirSet.u8CL_StateLv++;
		}
	}
}

/************************************************************************/
/* 光栅感知方向计数                                                     */
/************************************************************************/
void Dev_GratDirCheck(void)
{
	
	//g_VehOutSet.u8DirLwCnt 推荐为5
	//g_VehOutSet.u8DirHiCnt 推荐为40
	if (g_VehicleSet.DirSet.u8GR_Valided && g_VehicleSet.DirSet.u16GR_FitFrCnt< g_VehOutSet.u8DirLwCnt && !g_DirTrigged) //前5帧激光有车 光栅无车
			g_VehicleSet.DirSet.u8GR_StateLS++;
	
	if (g_VehicleSet.DirSet.u8GR_Valided && g_DirTrigged)//激光有光栅也有
	{
		g_VehicleSet.DirSet.u8GR_StateLG++;
		if (g_VehicleSet.DirSet.u8GR_StateLG> g_VehOutSet.u8DirHiCnt)
			g_VehicleSet.DirSet.u8GR_StateLG = g_VehOutSet.u8DirHiCnt;
	}
	
	if (g_VehicleSet.DirSet.u8GR_Valided)
		g_VehicleSet.DirSet.u8GR_StateGR = 0;
	else
	{
		//车中激光无车、光栅有车
		if (g_DirTrigged)
			g_VehicleSet.DirSet.u8GR_StateGR++;
	}
}

#endif


/************************************************************************/
/* 激光感知方向计数                                                     */
/************************************************************************/
void Dev_LaseDirCheck(void)
{
	
}

#ifdef _KeilEnVi

/************************************************************************/
/* 帧数超常检测		                                                    */
/************************************************************************/
void CheckAbFramCnt(void)
{
	g_DllAtState.CheckFrCnt++;
	//车辆帧数大于6000，并且队列里有车 那么删除头车	
	if(g_DllAtState.CheckFrCnt >= 6000 )
	{
		if( g_u8VehQueNum >=1 )
		{
			SendHeadVeh();
		}
		g_DllAtState.CheckFrCnt = 0;
	}
}

/************************************************************************/
/* 检查队列			                                                    */
/************************************************************************/
void CheckQueState(void)
{
	if(g_DllAtState.QueCheckFlag)
	{
		g_DllAtState.QueCheckFlag = 0;
		VehQueCheck(); //进行队列的检查
	
//		if( ((uint32)RTC_UCOUNT - g_u32SecondSecCnt) > 60 )
//		{
//			g_u32SecondSecCnt		= (uint32)RTC_UCOUNT; // 强制出车，也算有车
//			SendClearVeh();
//		}
		
	}
}

/************************************************************************/
/* 强制出车处理		                                                    */
/************************************************************************/
void ForceCloseProc(void)
{
	uint8 JudgeType=0;
	
	if(g_DllAtState.ManHanded||g_DllAtState.ForceVout)
	{
		if(g_DllAtState.ManHanded)
		{
			g_DllAtState.ManHanded=0;
			g_VehicleSet.u8VoutFlag=HANDED;
		}
		if(g_DllAtState.ForceVout)
		{
			g_DllAtState.ForceVout=0;
			g_VehicleSet.u8VoutFlag=FORCED;
		}
		
		JudgeType=ForceVehJudge(&g_VehicleSet);
		
		if(JudgeType>0)
		{
			//上传车型前进行队列检查
			if(g_u8VehQueNum) //队列中有车辆
			{
				if(g_u8VehQueIndex>=1)
				{
					if(g_u8VehQue[g_u8VehQueIndex-1] >= 7 || g_u8VehQue[g_u8VehQueIndex-1] == 3 || g_u8VehQue[g_u8VehQueIndex-1] == 4)
					{
						SendBackVeh();
						g_u8VehQueIndex = g_u8VehQueIndex - 1;
						g_u8VehQue[g_u8VehQueIndex] = 0;
					}	
				}  
				else //g_u8VehQueIndex == 0
				{
					if(g_u8VehQue[MAXVEHQUE-1] >= 7 || g_u8VehQue[MAXVEHQUE-1] == 3 || g_u8VehQue[MAXVEHQUE-1] == 4)
					{
						SendBackVeh();
						g_u8VehQueIndex = MAXVEHQUE-1;
						g_u8VehQue[g_u8VehQueIndex] = 0;
					}
				}
			}
			
			ReportForceVeh(&g_VehicleSet,JudgeType);
			
			//强制收尾车辆信息添加到上传车型数组20150519
			g_u8VehQue[g_u8VehQueIndex] = JudgeType; //强制收尾的方向为0，直接向下位机队列中添加车辆信息
			g_u8VehQueIndex = (g_u8VehQueIndex + 1) % MAXVEHQUE; //更新队列索引
		}
		else
		{
			//清空当前车辆记录
			memset(&g_VehicleSet,0,sizeof(g_VehicleSet));
		}	
	}
}

/************************************************************************/
/* 强制出车车型判断	                                                    */
/************************************************************************/
uint8  ForceVehJudge(VehicleSet *pVehicle)
{
	uint8 l_u8Retalue = 0;
	uint8 l_u8count = 0;
	
	if (pVehicle == NULL || pVehicle->AxleSet.AxleSum < THREE_AXLE)
		return 0;
	
	pVehicle->AxleSet.AxleSum = SIX_AXLE;
	for(l_u8count = 2; l_u8count < 6; l_u8count ++ )
	{
		pVehicle->AxleSet.Axle[l_u8count].AxleType = 0x02;
	}
	l_u8Retalue = FIFTH_HUOCHE;
	return l_u8Retalue;
}

/************************************************************************/
/* 发送强制出车车辆			                                            */
/************************************************************************/
void ReportForceVeh(VehicleSet *pVehicle,uint8 ForcType)
{
	uint8 UpBuff[100]={0};
	uint8 TmpI=0;
	uint8 u8Index=0;
	uint8 u8TyreCnt=0;

	//调试口出车
	//调试口上传车辆	
	//帧头命令号
	TmpI=0;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=CMD_UPLOADVEHS;
	
	//上传内容
	TmpI=5;
	
	//车序号
	g_VehIndex.u32Index++;//全局计数++；每天从0开始
	SaveVehIndex();
	UpBuff[TmpI++]=((g_VehIndex.u32Index>>16)&0xFF);
	UpBuff[TmpI++]=((g_VehIndex.u32Index>>8)&0xFF);
	UpBuff[TmpI++]=(g_VehIndex.u32Index&0xFF);
	
	UpBuff[TmpI++]=(YEAR>>8)&0xFF;
	UpBuff[TmpI++]=YEAR&0xFF;
	UpBuff[TmpI++]=MONTH;
	UpBuff[TmpI++]=DAY;
	
	UpBuff[TmpI++]=HOUR;
	UpBuff[TmpI++]=MIN;
	UpBuff[TmpI++]=SEC;
	
	//车型
	UpBuff[TmpI++]=ForcType;
	UpBuff[TmpI++]=MOVE_ABNO;
	//轴数
	UpBuff[TmpI++]=pVehicle->AxleSet.AxleSum;
	//轴型
	for(u8Index=0;u8Index<pVehicle->AxleSet.AxleSum;u8Index++)
	{
		UpBuff[TmpI++]=pVehicle->AxleSet.Axle[u8Index].AxleType;
	}
	
	//判定准则
	UpBuff[TmpI++]=(pVehicle->JudgeSet.u8JudgeRule&0xFF);
	//调整准则
	UpBuff[TmpI++]=(pVehicle->JudgeSet.u8ModifyRule&0xFF);
	
	//专车码
	UpBuff[TmpI++]=(pVehicle->JudgeSet.u8SpeVehCode&0xFF);
	
	//底盘高
	UpBuff[TmpI++]=((pVehicle->Dp.DpHeight1_2>>8)&0xFF);
	UpBuff[TmpI++]=(pVehicle->Dp.DpHeight1_2&0xFF);
	//车头高
	UpBuff[TmpI++]=((pVehicle->EdgeSet.u16HeadHeight>>8)&0xFF);
	UpBuff[TmpI++]=(pVehicle->EdgeSet.u16HeadHeight&0xFF);
	//车斗
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsHopper&0xFF);
	
	//头跳变
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsHeadJump&0xFF);
	
	//尾跳变
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsTailJump&0xFF);
	
	//载货
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsCargo&0xFF);
	
	//货箱
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsContainer&0xFF);
	
	//货车尾
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsTruckTail&0xFF);
	
	//投掷到地上
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsCastGrd&0xFF);


	//封闭车窗
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsFillWin&0xFF);	
	
	//车窗
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsGlassWin&0xFF);
	
	//侧面一致
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsSideUiform&0xFF);	
	

	//方向设备计数
	switch(g_SysSetUp.u8DirDevType)
	{
		case DEV_COIL:
			UpBuff[TmpI++]=(pVehicle->DirSet.u8CL_StateHi&0xFF);	
			UpBuff[TmpI++]=(pVehicle->DirSet.u8CL_StateLw&0xFF);
			UpBuff[TmpI++]=(pVehicle->DirSet.u8CL_StateLv&0xFF);
			break;
		case DEV_GRAT:
			UpBuff[TmpI++]=(pVehicle->DirSet.u8GR_StateLS&0xFF);	
			UpBuff[TmpI++]=(pVehicle->DirSet.u8GR_StateLG&0xFF);
			UpBuff[TmpI++]=(pVehicle->DirSet.u8GR_StateGR&0xFF);
			break;	
		case DEV_LASE:
			UpBuff[TmpI++]=0;
			UpBuff[TmpI++]=0;
			UpBuff[TmpI++]=0;
		default:
			UpBuff[TmpI++]=0;
			UpBuff[TmpI++]=0;
			UpBuff[TmpI++]=0;			
			break;
	}	
	
	//预留24 当备注 记录文件夹名称
	memcpy(&UpBuff[TmpI],g_u8ItemName,24);
	TmpI+=24;


	//长度位
	UpBuff[3]=(((TmpI+2)>>8)&0xFF);
	UpBuff[4]=((TmpI+2)&0xFF);
	
	//校验位
	AddCrc16(UpBuff,TmpI);
	SendBuf_full(UpBuff,TmpI + 2,g_UartSet.TS_CHCOM);	

	if(g_DllAtState.DllOpened)
	{
		//上报口上传车辆
		TmpI=0;
		memset(UpBuff,0,100);
		//帧头+命令号
		UpBuff[TmpI++]=0xFF;
		UpBuff[TmpI++]=0xFF;
		UpBuff[TmpI++]=DLL_VHCHECKINFO;
		
		//上传内容
		TmpI=5;
		UpBuff[TmpI++]=g_VehIndex.u32Index&0xFF;//车序号
		UpBuff[TmpI++]=0;//预留1位
		UpBuff[TmpI++]=pVehicle->AxleSet.AxleSum;//轴数
		UpBuff[TmpI++]=pVehicle->JudgeSet.u8JudgePattern;//车型
		UpBuff[TmpI++]=0;//预留1位
		if(pVehicle->AxleSet.Axle[1].AxleType== 0x01 )
			u8TyreCnt = 20;	 //轮数
		else
			u8TyreCnt = 22;	 //轮数		
		UpBuff[TmpI++]=u8TyreCnt;//轮胎个数
		UpBuff[TmpI++]=0;//车速度
		UpBuff[TmpI++]=0;//行进方向
		UpBuff[TmpI++]=0;//专车类型
		
		//长度位
		UpBuff[3]=(((TmpI+2)>>8)&0xFF);
		UpBuff[4]=((TmpI+2)&0xFF);
		
		//校验位
		AddCrc16(UpBuff,TmpI);
		SendBuf_full(UpBuff,TmpI + 2,g_UartSet.RM_CHCOM);	
	}
}

/************************************************************************/
/* //删除队列头一辆车		                                            */
/************************************************************************/
void SendHeadVeh(void) 
{
	uint8 UpBuff[100]={0};
	uint8 TmpI=0;
	uint8 u8Index=0;
	
	//调试口上传车辆	
	//帧头命令号
	TmpI=0;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=CMD_UPLOADVEHS;
	
	//上传内容
	TmpI=5;
	
	//车序号
	UpBuff[TmpI++]=((g_VehIndex.u32Index>>16)&0xFF);
	UpBuff[TmpI++]=((g_VehIndex.u32Index>>8)&0xFF);
	UpBuff[TmpI++]=(g_VehIndex.u32Index&0xFF);
	
	UpBuff[TmpI++]=(YEAR>>8)&0xFF;
	UpBuff[TmpI++]=YEAR&0xFF;
	UpBuff[TmpI++]=MONTH;
	UpBuff[TmpI++]=DAY;
	
	UpBuff[TmpI++]=HOUR;
	UpBuff[TmpI++]=MIN;
	UpBuff[TmpI++]=SEC;
	
	//车型
	UpBuff[TmpI++]=1;
	//方向
	UpBuff[TmpI++]=6;
	//轴数
	UpBuff[TmpI++]=2;
	//轴型
	for(u8Index=0;u8Index<2;u8Index++)
	{
		UpBuff[TmpI++]=1;
	}
	
	//判定准则
	UpBuff[TmpI++]=0;
	//调整准则
	UpBuff[TmpI++]=0;
	
	//专车码
	UpBuff[TmpI++]=0;
	
	//底盘高
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	//车头高
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	//车斗
	UpBuff[TmpI++]=0;
	
	//头跳变
	UpBuff[TmpI++]=0;
	
	//尾跳变
	UpBuff[TmpI++]=0;
	
	//载货
	UpBuff[TmpI++]=0;
	
	//货箱
	UpBuff[TmpI++]=0;
	
	//货车尾
	UpBuff[TmpI++]=0;
	
	//投掷到地上
	UpBuff[TmpI++]=0;

	//封闭车窗
	UpBuff[TmpI++]=0;	
	
	//车窗
	UpBuff[TmpI++]=0;
	
	//侧面一致
	UpBuff[TmpI++]=0;	
	
	//方向设备计数
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;	
	
	//预留24 当备注 记录文件夹名称
	memcpy(&UpBuff[TmpI],g_u8ItemName,24);
	TmpI+=24;



	//长度位
	UpBuff[3]=(((TmpI+2)>>8)&0xFF);
	UpBuff[4]=((TmpI+2)&0xFF);
	
	//校验位
	AddCrc16(UpBuff,TmpI);
	SendBuf_full(UpBuff,TmpI + 2,g_UartSet.TS_CHCOM);
	
	
	//上传口出车 
	if(g_DllAtState.DllOpened)
	{
		memset(UpBuff,0,100);
		TmpI=0;
		UpBuff[TmpI++] = 0xFF;
		UpBuff[TmpI++] = 0xFF;
		UpBuff[TmpI++]= DLL_VHCHECKINFO; 
		
		TmpI=5;
		UpBuff[TmpI++] = g_VehIndex.u32Index &0xFF;	  //车辆ID
		UpBuff[TmpI++] = 0;  //轴组数         
		UpBuff[TmpI++] = 2;	 //轴数
		UpBuff[TmpI++] = 1; //车型
		UpBuff[TmpI++] = 0;	 //轴组重
		UpBuff[TmpI++] = 0;	 //轴型
		UpBuff[TmpI++] = 0;	 //车速度
		UpBuff[TmpI++] = 6;	 //删车
		UpBuff[TmpI++]=0;//专车类型
		
		//长度位
		UpBuff[3]=(((TmpI+2)>>8)&0xFF);
		UpBuff[4]=((TmpI+2)&0xFF);
		
		AddCrc16(UpBuff,TmpI); 
		SendBuf_full(UpBuff,TmpI + 2,g_UartSet.RM_CHCOM);	
	}		
}

/************************************************************************/
/* //增加一辆车				                                            */
/************************************************************************/
void SendAddVeh(uint8 u8AddType)
{
	uint8 UpBuff[100]={0};
	uint8 TmpI=0;
	uint8 u8Index=0;
	uint8 u8AxleCnt=0;
	
	//通过车型定轴数
	switch(u8AddType)
	{
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
			u8AxleCnt = 2;           //轮轴数
		  break;
		case 7:
			u8AxleCnt = 3;           //轮轴数
		  break;
		case 8:
			u8AxleCnt = 4;           //轮轴数
		  break;
		case 9:
			u8AxleCnt = 5;           //轮轴数
		  break;
		default:
			u8AxleCnt = 2;           //轮轴数
		  break;
	}
	
	
	//调试口上传车辆	
	//帧头命令号
	TmpI=0;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=CMD_UPLOADVEHS;
	
	//上传内容
	TmpI=5;
	
	//车序号
	UpBuff[TmpI++]=((g_VehIndex.u32Index>>16)&0xFF);
	UpBuff[TmpI++]=((g_VehIndex.u32Index>>8)&0xFF);
	UpBuff[TmpI++]=(g_VehIndex.u32Index&0xFF);
	
	UpBuff[TmpI++]=(YEAR>>8)&0xFF;
	UpBuff[TmpI++]=YEAR&0xFF;
	UpBuff[TmpI++]=MONTH;
	UpBuff[TmpI++]=DAY;
	
	UpBuff[TmpI++]=HOUR;
	UpBuff[TmpI++]=MIN;
	UpBuff[TmpI++]=SEC;
	
	//车型
	UpBuff[TmpI++]=u8AddType;
	//方向
	UpBuff[TmpI++]=MOVE_FRNT;
	//轴数
	UpBuff[TmpI++]=u8AxleCnt;
	//轴型
	for(u8Index=0;u8Index<2;u8Index++)
	{
		UpBuff[TmpI++]=1;
	}
	
	//判定准则
	UpBuff[TmpI++]=0;
	//调整准则
	UpBuff[TmpI++]=0;
	
	//专车码
	UpBuff[TmpI++]=0;
	
	//底盘高
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	//车头高
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	//车斗
	UpBuff[TmpI++]=0;
	
	//头跳变
	UpBuff[TmpI++]=0;
	
	//尾跳变
	UpBuff[TmpI++]=0;
	
	//载货
	UpBuff[TmpI++]=0;
	
	//货箱
	UpBuff[TmpI++]=0;
	
	//货车尾
	UpBuff[TmpI++]=0;
	
	//投掷到地上
	UpBuff[TmpI++]=0;

	//封闭车窗
	UpBuff[TmpI++]=0;	
	
	//车窗
	UpBuff[TmpI++]=0;
	
	//侧面一致
	UpBuff[TmpI++]=0;	
	
	//方向计数
	UpBuff[TmpI++]=0;	
	UpBuff[TmpI++]=0;	
	UpBuff[TmpI++]=0;	
	
	//预留24 当备注 记录文件夹名称
	memcpy(&UpBuff[TmpI],g_u8ItemName,24);
	TmpI+=24;



	//长度位
	UpBuff[3]=(((TmpI+2)>>8)&0xFF);
	UpBuff[4]=((TmpI+2)&0xFF);
	
	//校验位
	AddCrc16(UpBuff,TmpI);
	SendBuf_full(UpBuff,TmpI + 2,g_UartSet.TS_CHCOM);
	
	
	//上传口出车 
	if(g_DllAtState.DllOpened)
	{
		memset(UpBuff,0,100);
		TmpI=0;
		UpBuff[TmpI++] = 0xFF;
		UpBuff[TmpI++] = 0xFF;
		UpBuff[TmpI++]= DLL_VHCHECKINFO; 
		
		TmpI=5;
		UpBuff[TmpI++] = g_VehIndex.u32Index &0xFF;	  //车辆ID
		UpBuff[TmpI++] = 0;  //轴组数         
		UpBuff[TmpI++] = u8AxleCnt;	 //轴数
		UpBuff[TmpI++] = u8AddType; //车型
		UpBuff[TmpI++] = 0;	 //轴组重
		UpBuff[TmpI++] = 0;	 //轴型
		UpBuff[TmpI++] = 0;	 //车速度
		UpBuff[TmpI++] = 1;	 //进车
		UpBuff[TmpI++]=0;//专车类型
		
		//长度位
		UpBuff[3]=(((TmpI+2)>>8)&0xFF);
		UpBuff[4]=((TmpI+2)&0xFF);
		
		AddCrc16(UpBuff,TmpI); 
		SendBuf_full(UpBuff,TmpI + 2,g_UartSet.RM_CHCOM);	
	}		
	g_u8VehQue[g_u8VehQueIndex] = u8AddType;
	g_u8VehQueIndex = (g_u8VehQueIndex+1)%MAXVEHQUE;
	
}

/************************************************************************/
/* 发送Back车辆		                                                    */
/************************************************************************/
void SendBackVeh(void)
{
	uint8 UpBuff[100]={0};
	uint8 TmpI=0;
	uint8 u8Index=0;
	
	//调试口上传车辆	
	//帧头命令号
	TmpI=0;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=CMD_UPLOADVEHS;
	
	//上传内容
	TmpI=5;
	
	//车序号
	UpBuff[TmpI++]=((g_VehIndex.u32Index>>16)&0xFF);
	UpBuff[TmpI++]=((g_VehIndex.u32Index>>8)&0xFF);
	UpBuff[TmpI++]=(g_VehIndex.u32Index&0xFF);
	
	UpBuff[TmpI++]=(YEAR>>8)&0xFF;
	UpBuff[TmpI++]=YEAR&0xFF;
	UpBuff[TmpI++]=MONTH;
	UpBuff[TmpI++]=DAY;
	
	UpBuff[TmpI++]=HOUR;
	UpBuff[TmpI++]=MIN;
	UpBuff[TmpI++]=SEC;
	
	//车型
	UpBuff[TmpI++]=1;
	//方向
	UpBuff[TmpI++]=MOVE_BACK;
	//轴数
	UpBuff[TmpI++]=2;
	//轴型
	for(u8Index=0;u8Index<2;u8Index++)
	{
		UpBuff[TmpI++]=1;
	}
	
	//判定准则
	UpBuff[TmpI++]=0;
	//调整准则
	UpBuff[TmpI++]=0;
	
	//专车码
	UpBuff[TmpI++]=0;
	
	//底盘高
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	//车头高
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	//车斗
	UpBuff[TmpI++]=0;
	
	//头跳变
	UpBuff[TmpI++]=0;
	
	//尾跳变
	UpBuff[TmpI++]=0;
	
	//载货
	UpBuff[TmpI++]=0;
	
	//货箱
	UpBuff[TmpI++]=0;
	
	//货车尾
	UpBuff[TmpI++]=0;
	
	//投掷到地上
	UpBuff[TmpI++]=0;


	//封闭车窗
	UpBuff[TmpI++]=0;	
	
	//车窗
	UpBuff[TmpI++]=0;
	
	//侧面一致
	UpBuff[TmpI++]=0;	
	
	//方向设备计数
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;	
	
	//预留24 当备注 记录文件夹名称
	memcpy(&UpBuff[TmpI],g_u8ItemName,24);
	TmpI+=24;

	//长度位
	UpBuff[3]=(((TmpI+2)>>8)&0xFF);
	UpBuff[4]=((TmpI+2)&0xFF);
	
	//校验位
	AddCrc16(UpBuff,TmpI);
	SendBuf_full(UpBuff,TmpI + 2,g_UartSet.TS_CHCOM);
	
	
	//上传口出车 
	if(g_DllAtState.DllOpened)
	{
		memset(UpBuff,0,100);
		TmpI=0;
		UpBuff[TmpI++] = 0xFF;
		UpBuff[TmpI++] = 0xFF;
		UpBuff[TmpI++]= DLL_VHCHECKINFO; 
		
		TmpI=5;
		UpBuff[TmpI++] = g_VehIndex.u32Index &0xFF;	  //车辆ID
		UpBuff[TmpI++] = 0;  //轴组数         
		UpBuff[TmpI++] = 2;	 //轴数
		UpBuff[TmpI++] = 1; //车型
		UpBuff[TmpI++] = 0;	 //轴组重
		UpBuff[TmpI++] = 0;	 //轴型
		UpBuff[TmpI++] = 0;	 //车速度
		UpBuff[TmpI++] = 2;	 //倒车
		UpBuff[TmpI++]=0;//专车类型
		
		//长度位
		UpBuff[3]=(((TmpI+2)>>8)&0xFF);
		UpBuff[4]=((TmpI+2)&0xFF);
		
		AddCrc16(UpBuff,TmpI); 
		SendBuf_full(UpBuff,TmpI + 2,g_UartSet.RM_CHCOM);	
	}	
}

/************************************************************************/
/* 发送队列信息				                                            */
/************************************************************************/
void SendQueNum(uint8 CH,uint8 PRTL)
{
	uint16 tempindex=0;
	uint8 TmpBuff[512]={0};
	
	if(PRTL==PTRL_FF)
	{
		g_DllAtState.DllOpened=0;
		
		tempindex = 5;
		TmpBuff[tempindex++] = g_u8VehQueNum;
		TmpBuff[tempindex++] = 0x00;
		TmpBuff[tempindex++] = 0x00;
		TmpBuff[tempindex++] = 0x00;	

		TmpBuff[0] = 0xFF;
		TmpBuff[1] = 0xFF;
		TmpBuff[2] = DLL_UPLDQUEINFO;					
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//协议帧字节数
		TmpBuff[4] = (tempindex + 2)&0xFF;

		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);	
	}
	else
	{
		
	}	
}


/************************************************************************/
/* 检查队列状态				                                            */
/************************************************************************/
void VehQueCheck(void)
{
	uint8 i;
	uint8 Index = 0;
	uint8 Ke1Cnt = 0;
	uint8 Huo1Cnt = 0;
	uint8 Flag = 0;
	uint8 Ke3Cnt = 0;
	uint8 Huo3Cnt = 0;
	if(g_u8VehQueNum == 2) //检查队列数为2的情况
	{
		Ke3Cnt = 0;
		Huo3Cnt = 0;
		for(i=1;i<=2;i++)
		{
		
			Index = (g_u8VehQueIndex >= i) ? (g_u8VehQueIndex-i):(g_u8VehQueIndex + MAXVEHQUE - i); //获取当前车辆信息索引
//			if(g_u8VehQue[Index] >= 7 || //队列中有一辆车为货3，货4，货5类型
//				 g_u8VehQue[Index] == 3 || g_u8VehQue[Index] == 4) // 或者为客3、客4类型  屏蔽
			if(g_u8VehQue[Index] >= 8 )
			{
				Flag = 1;
			}
			else if(g_u8VehQue[Index] >= 7)
				Huo3Cnt ++;
			else if( g_u8VehQue[Index] == 3 || g_u8VehQue[Index] == 4)
				Ke3Cnt ++;
			if(Flag ==1 || Ke3Cnt >= 2 )
				Flag = 1;
		}
		if(Flag) //队列为两辆车但是出现了大车，表示队列多车
		{
			SendHeadVeh(); //删除队列头一辆车，下位机队列数组不需进行操作
//			SendBackVeh(); //删除一辆车		
//			g_u8VehQueIndex = (g_u8VehQueIndex >= 1) ? (g_u8VehQueIndex - 1):(g_u8VehQueIndex + MAXVEHQUE - 1);
//			g_u8VehQue[g_u8VehQueIndex] = 0;
		}
	}
	else if(g_u8VehQueNum == 3) //检查队列数为3的情况
	{
		Ke1Cnt = 0;
		Huo1Cnt = 0;
		Huo3Cnt = 0;
		Ke3Cnt = 0;
		Flag = 0;
		for(i=1;i<=3;i++)
		{
			Index = (g_u8VehQueIndex >= i) ? (g_u8VehQueIndex-i):(g_u8VehQueIndex + MAXVEHQUE - i); //获取当前车辆信息索引
			if(g_u8VehQue[Index] == 1) //计算这3辆车中客1的数
				Ke1Cnt++;
			else if(g_u8VehQue[Index] == 5) //计算这3辆车中货1的数
				Huo1Cnt++; 
			else if(g_u8VehQue[Index] >= 8 )
				Huo3Cnt ++;
			else if(g_u8VehQue[Index] == 3 || g_u8VehQue[Index] == 4)
				Ke3Cnt ++;
			else
				;
//				Flag = 1; //出现了非客1和货1的车
		}
		if(Flag || (Huo3Cnt >= 1 ) || (Ke3Cnt >= 1) ) //队列多车，删一辆车
		{
			SendBackVeh(); //删除一辆车
			g_u8VehQueIndex = (g_u8VehQueIndex >= 1) ? (g_u8VehQueIndex - 1):(g_u8VehQueIndex + MAXVEHQUE - 1);
			g_u8VehQue[g_u8VehQueIndex] = 0;
		}
	}
	else if(g_u8VehQueNum == 4)
	{
			Ke1Cnt = 0;
			Huo1Cnt = 0;
			Huo3Cnt = 0;
			Ke3Cnt = 0;
			Flag = 0;
			for(i=1;i<=4;i++)
		{
			Index = (g_u8VehQueIndex >= i) ? (g_u8VehQueIndex-i):(g_u8VehQueIndex + MAXVEHQUE - i); //获取当前车辆信息索引
			if( g_u8VehQue[Index] ==2 || g_u8VehQue[Index] == 3 || g_u8VehQue[Index] == 4 )
					Ke1Cnt++;
			else if(g_u8VehQue[Index] == 5)
				Huo1Cnt++;
			else if(g_u8VehQue[Index] >=6 )
				Huo3Cnt++;
				
//				Flag = 1;

		}
		if(Flag || Huo3Cnt >= 1 || Ke1Cnt >= 1)
			{
				SendBackVeh(); //删除一辆车
				g_u8VehQueIndex = (g_u8VehQueIndex >= 1) ? (g_u8VehQueIndex - 1):(g_u8VehQueIndex + MAXVEHQUE - 1);
				g_u8VehQue[g_u8VehQueIndex] = 0;
			}
	}
	else if(g_u8VehQueNum > 4) //队列数大于3，必有多车
	{
//		SendBackVeh(); //删除一辆车
//		g_u8VehQueIndex = (g_u8VehQueIndex >= 1) ? (g_u8VehQueIndex - 1):(g_u8VehQueIndex + MAXVEHQUE - 1);
//		g_u8VehQue[g_u8VehQueIndex] = 0;
		SendHeadVeh(); //删除队列头一辆车，下位机队列数组不需进行操作
	}
		
}

#endif

/************************************************************************/
/* 出错处理                                                             */
/************************************************************************/
void ClearVehicleErr()
{

}

/************************************************************************/
/* 更新车辆区域信息                                                     */
/************************************************************************/
void UpdateVehRegion(VehicleSet *pVehicle)
{
	//拷贝数据
	uint16 l_u16PtCnt=0;
	memcpy(&pVehicle->CurFrmInfo,&CurFramInfo,sizeof(CurFramInfo));
	memset(&pVehicle->CurRegion,0,sizeof(RegionSet));


	if (CurFramInfo.u16EndPos-CurFramInfo.u16StaPos+1>FRAME_BUFLEN)
	{
		l_u16PtCnt=FRAME_BUFLEN;
	}
	else
	{
		l_u16PtCnt=CurFramInfo.u16EndPos-CurFramInfo.u16StaPos+1;
	}

	//更新最后一帧数据
	memcpy(&pVehicle->CurRegion.xdata[1],&g_FiltX[CurFramInfo.u16StaPos],l_u16PtCnt*sizeof(int));
	pVehicle->CurRegion.xdata[0]=l_u16PtCnt;
	memcpy(&pVehicle->CurRegion.zdata[1],&g_FiltZ[CurFramInfo.u16StaPos],l_u16PtCnt*sizeof(int));
	pVehicle->CurRegion.zdata[0]=l_u16PtCnt;
	pVehicle->CurRegion.xMax=CurFramInfo.n32xMax;
	pVehicle->CurRegion.xMin=CurFramInfo.n32xMin;
	pVehicle->CurRegion.zMax=CurFramInfo.u16MaxHt;
	pVehicle->EdgeSet.u16MaxHeight[pVehicle->u16FrameCnt]=CurFramInfo.u16MaxHt;
	pVehicle->EdgeSet.u16SecHeight[pVehicle->u16FrameCnt]=CurFramInfo.u16SecHt;
	pVehicle->EdgeSet.u16MinXEdge[pVehicle->u16FrameCnt]=CurFramInfo.n32xMin;
	
	if (CurFramInfo.n32xMin+800<CurFramInfo.u16AvgX)
	{
		//防止散点 造成边界投影异常 20170725
		if (pVehicle->u16FrameCnt<1)
		{
			pVehicle->CurRegion.xMin=CurFramInfo.u16AvgX;
			pVehicle->EdgeSet.u16MinXEdge[pVehicle->u16FrameCnt]=CurFramInfo.u16AvgX;
		}
		else
		{
			pVehicle->CurRegion.xMin=pVehicle->EdgeSet.u16MinXEdge[pVehicle->u16FrameCnt-1];
			pVehicle->EdgeSet.u16MinXEdge[pVehicle->u16FrameCnt]=pVehicle->EdgeSet.u16MinXEdge[pVehicle->u16FrameCnt-1];
		}
	}

	if (pVehicle->EdgeSet.u16MinX == 0)
	{
		pVehicle->EdgeSet.u16MinX=CurFramInfo.n32xMin;
		pVehicle->EdgeSet.u16MinXIndex=pVehicle->u16FrameCnt;
	}
	else if (pVehicle->EdgeSet.u16MinX>CurFramInfo.n32xMin)
	{
		pVehicle->EdgeSet.u16MinX=CurFramInfo.n32xMin;
		pVehicle->EdgeSet.u16MinXIndex=pVehicle->u16FrameCnt;
	}

	if (pVehicle->EdgeSet.u16Width == 0)
	{
		pVehicle->EdgeSet.u16Width=CurFramInfo.u16Width;
	}
	else if (pVehicle->EdgeSet.u16Width<CurFramInfo.u16Width)
	{
		pVehicle->EdgeSet.u16Width=CurFramInfo.u16Width;
	}
	
#ifdef _DEBUG
	//拷贝进点云数据中
	if (pVehicle->u16FrameCnt < FRAME_MASK )
	{
		memcpy(&pVehicle->PC[pVehicle->u16FrameCnt],&pVehicle->CurRegion,sizeof(RegionSet));
	}
	else
	{
		memcpy(&pVehicle->PC[FRAME_MAXCNT],&pVehicle->CurRegion,sizeof(RegionSet));
	}
#endif
	pVehicle->u16FrameCnt++;
	pVehicle->EmptyFrame=0;

	//进行特征提取
	FeatureCal(pVehicle);
}


/************************************************************************/
/* 检查车辆状态                                                         */
/************************************************************************/
void CheckVehState()
{
	if(g_VehicleSet.u8State != NO_USED)
	{
#ifdef _KeilEnVi
		Catch_Frame_RT(&g_VehicleSet,1);
#endif
		if(g_VehicleSet.EmptyFrame + 1> g_VehOutSet.u8EmptyFrCnt && g_VehicleSet.u16FrameCnt)
		{ 		
			g_VehicleSet.u8State = PASSED_USED;  //已结束，可收尾的车	
		}
#ifndef _KeilEnVi
		if (g_ForceOut)
		{
			g_VehicleSet.u8State = PASSED_USED;//强制收尾，出车
		}
#endif

		if(g_VehicleSet.u8State == PASSED_USED) 
		{  
			//收尾处理 进行车型识别	
			OutPutVeh(&g_VehicleSet);
#ifdef _KeilEnVi				
			Catch_Frame_RT(&g_VehicleSet,2);
			if(g_WeigthAt.u8wCarClassify)
			{
				//给IO触发 
			}
#endif				
			memset(&g_VehicleSet,0,sizeof(VehicleSet));
			g_VehicleSet.u8State = NO_USED;
		}
		else
		{
#ifdef _KeilEnVi
			if(g_VehOutSet.u8Ax3Force && g_VehicleSet.u8VoutFlag<1)
			{
				//开启了超3轴强制出车
				if(g_VehicleSet.DirSet.u8DirTrigged&&g_VehicleSet.AxleSet.AxleSum>3)
				{
					g_DllAtState.ForceVout=1;
				}
			}
#endif			
		}
		g_VehicleSet.EmptyFrame++;
	} 	
}

/************************************************************************/
/*IsVehCheck 检测是否为车辆                                             */
/*1-是车辆；0-不是车辆		                                            */
/************************************************************************/
uint8 IsVehCheck(VehicleSet *pVehicle)
{
	if (pVehicle->u16FrameCnt<2||pVehicle->EdgeSet.u16VehHeight<500)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}


/************************************************************************/
/* 出车处理                                                             */
/************************************************************************/
void OutPutVeh(VehicleSet *pVehicle)
{
	VehIdentifier(pVehicle);

	AxleClearGlobal();

	if (IsVehCheck(pVehicle))
	{
#ifndef _KeilEnVi
		Write2TxtFile(pVehicle);
#else
		
		//上传车型前进行队列检查
		if(g_u8VehQueNum && pVehicle->DirSet.u8VehDir != MOVE_BACK) //队列中有车辆,且当前车辆方向不为2
		{
			if(g_u8VehQueIndex>=1)
			{
				if(g_u8VehQue[g_u8VehQueIndex-1] >= 8 || g_u8VehQue[g_u8VehQueIndex-1] == 9 )
				{
					SendBackVeh();
					g_u8VehQueIndex = g_u8VehQueIndex - 1;
					g_u8VehQue[g_u8VehQueIndex] = 0;
				}	
			}  
			else 
			{
				if(g_u8VehQue[MAXVEHQUE-1] >= 8 )
				{
					SendBackVeh();
					g_u8VehQueIndex = MAXVEHQUE-1;
					g_u8VehQue[g_u8VehQueIndex] = 0;
				}
			}
		}		
	
		if(pVehicle->u8VoutFlag<1)
		{
			ReportNormalVeh(pVehicle);
			
			//将车辆信息添加到队列
			if(pVehicle->DirSet.u8VehDir != 2 && pVehicle->DirSet.u8VehDir != 3) //方向为2和3的车不加入队列
			{
				g_u8VehQue[g_u8VehQueIndex] = pVehicle->JudgeSet.u8JudgePattern; //车辆信息的添加
				g_u8VehQueIndex = (g_u8VehQueIndex + 1) % MAXVEHQUE; //更新队列索引
			}
			else if(pVehicle->DirSet.u8VehDir == 2) //方向为2的进行减车处理
			{
				if(g_u8VehQueIndex >= 1)
					g_u8VehQueIndex = g_u8VehQueIndex - 1;
				else
					g_u8VehQueIndex = MAXVEHQUE - 1;

					g_u8VehQue[g_u8VehQueIndex] = 0; //将该队列尾车辆信息清除
			}
		}
		else
		{
			if(pVehicle->AxleSet.AxleSum > SIX_AXLE && g_u8VehQueNum == 1 &&
			pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum - 1].AxleType == 1) //强制按键出车后，车辆又倒车，队列需要将强制出的车删除
			{ 
				//进行队列的删车操作
				SendBackVeh();

				//方向为2，进行队列尾车辆信息清除
				if(g_u8VehQueIndex >= 1)
					g_u8VehQueIndex = g_u8VehQueIndex - 1;
				else
					g_u8VehQueIndex = MAXVEHQUE - 1;

				g_u8VehQue[g_u8VehQueIndex] = 0; //将该队列尾车辆信息清除

			}						
		}
#endif
	}
	
}

#ifndef _KeilEnVi
/************************************************************************/
/* 写入结果至txt文本                                                    */
/************************************************************************/
void Write2TxtFile(VehicleSet *pVehicle)
{
	FILE *l_pFile = NULL;
	char l_acVehicleInfo[512]={0};
	uint16 index=0;
	uint16 u16Idex=0;

	g_VehIndex.u32Index++;

	itoa(g_VehIndex.u32Index, l_acVehicleInfo, 10); 
	index+=6;

	memcpy(&l_acVehicleInfo[index],g_ItemName,strlen(g_ItemName));
	index=index+strlen(g_ItemName)+4;

	//总帧数
	itoa(pVehicle->u16FrameCnt, &l_acVehicleInfo[index], 10); 
	index+=6;


	//起始帧
	itoa(pVehicle->StaWaveIndex, &l_acVehicleInfo[index], 10); 
	index+=6;

	//终止帧
	itoa(pVehicle->EndWaveIndex, &l_acVehicleInfo[index], 10); 
	index+=6;


	//轴数
	itoa(pVehicle->AxleSet.AxleSum, &l_acVehicleInfo[index], 10); 
	index+=4;

	//轴型
	for (u16Idex=0;u16Idex<pVehicle->AxleSet.AxleSum;u16Idex++)
	{
		itoa(pVehicle->AxleSet.Axle[u16Idex].AxleType, &l_acVehicleInfo[index], 10); 
		index+=1;
	}

	index=index+8-pVehicle->AxleSet.AxleSum;
	

	//车型
	itoa(pVehicle->JudgeSet.u8JudgePattern, &l_acVehicleInfo[index], 10); 
	index+=4;

	//判定准则
	itoa(pVehicle->JudgeSet.u8JudgeRule, &l_acVehicleInfo[index], 10); 
	index+=4;

	//调整准则
	itoa(pVehicle->JudgeSet.u8ModifyRule, &l_acVehicleInfo[index], 10); 
	index+=4;

	//专车代码
	itoa(pVehicle->JudgeSet.u8SpeVehCode, &l_acVehicleInfo[index], 10);
	index+=2;


	//车头高
	itoa(pVehicle->EdgeSet.u16HeadHeight, &l_acVehicleInfo[index], 10); 
	index+=6;

	//底盘高
	itoa(pVehicle->Dp.DpHeight1_2, &l_acVehicleInfo[index], 10); 
	index+=6;

	//车斗
	itoa(pVehicle->FeatureSet.IsHopper, &l_acVehicleInfo[index], 10); 
	index+=2;


	//头跳变
	itoa(pVehicle->FeatureSet.IsHeadJump, &l_acVehicleInfo[index], 10); 
	index+=2;
	

	//尾跳变
	itoa(pVehicle->FeatureSet.IsTailJump, &l_acVehicleInfo[index], 10); 
	index+=2;
	
	//载货
	itoa(pVehicle->FeatureSet.IsCargo, &l_acVehicleInfo[index], 10); 
	index+=2;


	//货箱
	itoa(pVehicle->FeatureSet.IsContainer, &l_acVehicleInfo[index], 10); 
	index+=2;

	//货车尾
	itoa(pVehicle->FeatureSet.IsTruckTail, &l_acVehicleInfo[index], 10); 
	index+=2;

	//投掷到地上
	itoa(pVehicle->FeatureSet.IsCastGrd, &l_acVehicleInfo[index], 10); 
	index+=2;


	//封闭车窗
	itoa(pVehicle->FeatureSet.IsFillWin, &l_acVehicleInfo[index], 10); 
	index+=2;

	//车窗
	itoa(pVehicle->FeatureSet.IsGlassWin, &l_acVehicleInfo[index], 10); 
	index+=2;

	//侧面一致
	itoa(pVehicle->FeatureSet.IsSideUiform, &l_acVehicleInfo[index], 10); 
	index+=2;

	l_acVehicleInfo[index] = '\r';
	index++;
	l_acVehicleInfo[index] = '\n';

	if ((l_pFile = fopen("avi.txt","a+")) != NULL) //打开成功
	{
		fwrite(l_acVehicleInfo,index+1,1,l_pFile);
	}

	fclose(l_pFile);
}
#else 
/************************************************************************/
/* 上传单车信息  		                                                */
/************************************************************************/
void ReportNormalVeh(VehicleSet *pVehicle)
{
	uint8 UpBuff[100]={0};
	uint8 TmpI=0;
	uint8 u8Index=0;
	uint8 u8TyreCnt=0;
	
	//调试口上传车辆	
	//帧头命令号
	TmpI=0;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=CMD_UPLOADVEHS;
	
	//上传内容
	TmpI=5;
	
	//车序号
	g_VehIndex.u32Index++;//全局计数++；每天从0开始
	SaveVehIndex();
	UpBuff[TmpI++]=((g_VehIndex.u32Index>>16)&0xFF);
	UpBuff[TmpI++]=((g_VehIndex.u32Index>>8)&0xFF);
	UpBuff[TmpI++]=(g_VehIndex.u32Index&0xFF);
	
	UpBuff[TmpI++]=(YEAR>>8)&0xFF;
	UpBuff[TmpI++]=YEAR&0xFF;
	UpBuff[TmpI++]=MONTH;
	UpBuff[TmpI++]=DAY;
	
	UpBuff[TmpI++]=HOUR;
	UpBuff[TmpI++]=MIN;
	UpBuff[TmpI++]=SEC;
	
	//车型
	UpBuff[TmpI++]=pVehicle->JudgeSet.u8JudgePattern;
	//方向
	UpBuff[TmpI++]=pVehicle->DirSet.u8VehDir;
	
	//轴数
	UpBuff[TmpI++]=pVehicle->AxleSet.AxleSum;
	//轴型
	for(u8Index=0;u8Index<pVehicle->AxleSet.AxleSum;u8Index++)
	{
		UpBuff[TmpI++]=pVehicle->AxleSet.Axle[u8Index].AxleType;
	}
	
	//判定准则
	UpBuff[TmpI++]=(pVehicle->JudgeSet.u8JudgeRule&0xFF);
	//调整准则
	UpBuff[TmpI++]=(pVehicle->JudgeSet.u8ModifyRule&0xFF);
	
	//专车码
	UpBuff[TmpI++]=(pVehicle->JudgeSet.u8SpeVehCode&0xFF);
	
	//底盘高
	UpBuff[TmpI++]=((pVehicle->Dp.DpHeight1_2>>8)&0xFF);
	UpBuff[TmpI++]=(pVehicle->Dp.DpHeight1_2&0xFF);
	//车头高
	UpBuff[TmpI++]=((pVehicle->EdgeSet.u16HeadHeight>>8)&0xFF);
	UpBuff[TmpI++]=(pVehicle->EdgeSet.u16HeadHeight&0xFF);
	//车斗
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsHopper&0xFF);
	
	//头跳变
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsHeadJump&0xFF);
	
	//尾跳变
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsTailJump&0xFF);
	
	//载货
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsCargo&0xFF);
	
	//货箱
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsContainer&0xFF);
	
	//货车尾
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsTruckTail&0xFF);
	
	//投掷到地上
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsCastGrd&0xFF);


	//封闭车窗
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsFillWin&0xFF);	
	
	//车窗
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsGlassWin&0xFF);
	
	//侧面一致
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsSideUiform&0xFF);
	
	//方向设备计数
	switch(g_SysSetUp.u8DirDevType)
	{
		case DEV_COIL:
			UpBuff[TmpI++]=(pVehicle->DirSet.u8CL_StateHi&0xFF);	
			UpBuff[TmpI++]=(pVehicle->DirSet.u8CL_StateLw&0xFF);
			UpBuff[TmpI++]=(pVehicle->DirSet.u8CL_StateLv&0xFF);
			break;
		case DEV_GRAT:
			UpBuff[TmpI++]=(pVehicle->DirSet.u8GR_StateLS&0xFF);	
			UpBuff[TmpI++]=(pVehicle->DirSet.u8GR_StateLG&0xFF);
			UpBuff[TmpI++]=(pVehicle->DirSet.u8GR_StateGR&0xFF);
			break;	
		case DEV_LASE:
			UpBuff[TmpI++]=0;
			UpBuff[TmpI++]=0;
			UpBuff[TmpI++]=0;
		default:
			UpBuff[TmpI++]=0;
			UpBuff[TmpI++]=0;
			UpBuff[TmpI++]=0;			
			break;
	}	
	
	//预留24 当备注 记录文件夹名称
	memcpy(&UpBuff[TmpI],g_u8ItemName,24);
	TmpI+=24;

	

	//长度位
	UpBuff[3]=(((TmpI+2)>>8)&0xFF);
	UpBuff[4]=((TmpI+2)&0xFF);
	
	//校验位
	AddCrc16(UpBuff,TmpI);
	SendBuf_full(UpBuff,TmpI + 2,g_UartSet.TS_CHCOM);

	if(g_DllAtState.DllOpened
		&& pVehicle->DirSet.u8DirTrigged)
	{
		//上报口上传车辆
		TmpI=0;
		memset(UpBuff,0,100);
		//帧头+命令号
		UpBuff[TmpI++]=0xFF;
		UpBuff[TmpI++]=0xFF;
		UpBuff[TmpI++]=DLL_VHCHECKINFO;
		
		//上传内容
		TmpI=5;
		UpBuff[TmpI++]=g_VehIndex.u32Index&0xFF;//车序号
		UpBuff[TmpI++]=0;//预留1位
		UpBuff[TmpI++]=pVehicle->AxleSet.AxleSum;//轴数
		UpBuff[TmpI++]=pVehicle->JudgeSet.u8JudgePattern;//车型
		UpBuff[TmpI++]=0;//预留1位
		for(u8Index=0;u8Index<pVehicle->AxleSet.AxleSum;u8Index++)
		{
			u8TyreCnt+=pVehicle->AxleSet.Axle[u8Index].AxleType*2;//左右两边轮子总数
		}
		UpBuff[TmpI++]=u8TyreCnt;//轮胎个数
		UpBuff[TmpI++]=0;//车速度
		UpBuff[TmpI++]=pVehicle->DirSet.u8VehDir;//行进方向
		UpBuff[TmpI++]=pVehicle->JudgeSet.u8SpeVehCode;//专车类型
		
		//长度位
		UpBuff[3]=(((TmpI+2)>>8)&0xFF);
		UpBuff[4]=((TmpI+2)&0xFF);
		
		//校验位
		AddCrc16(UpBuff,TmpI);
		SendBuf_full(UpBuff,TmpI + 2,g_UartSet.RM_CHCOM);	
	}
}

#endif


