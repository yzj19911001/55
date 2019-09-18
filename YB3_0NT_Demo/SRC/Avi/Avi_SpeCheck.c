#include "stdafx.h"
#include "Avi_SpeCheck.h"
#include "Avi_MathProc.h"
#include <math.h>


/************************************************************************/
/* 特殊车辆检测算子                                                     */
/************************************************************************/
void SpecialVehCal(VehicleSet *pVehicle)
{
	//皮卡检测算子
	PickUpFramCal(pVehicle);
	//封闭车窗检测算子
	EnCloseWinCal(pVehicle);
	EnCloseWinCalEx(pVehicle);

}


/************************************************************************/
/* 特殊车辆检测                                                         */
/************************************************************************/
void SpecialVehCheck(VehicleSet *pVehicle)
{
	//皮卡检测
	PickUpCheck(pVehicle);

	//PickUpTailCheck(pVehicle); //20170803 该方法效果不佳
	//货车尾部检测
	TruckTailCheck(pVehicle); 

	//金杯检测
	JinBeiCheck(pVehicle);
	//封闭客车检测
	EnCloseWinCheck(pVehicle);
	EnCloseWinCheckEx(pVehicle);

	//Axle2WinCheckEx(pVehicle);//20170803 该方法效果不佳
}


/************************************************************************/
/* IsJinBei 判断小客车是否是金杯车                                      */
/* 2表示金杯，1表示客1													*/
/************************************************************************/
void  JinBeiCheck(VehicleSet *pVehicle)
{
//	uint8     l_u8RetValue = FIRST_KECHE;  //默认客1

	if (pVehicle  ==  NULL) //数据异常
	{
		return ;
	}

	//车辆高度在1800—2000之间
	if (pVehicle->EdgeSet.u16VehHeight >= 1800  &&  pVehicle->EdgeSet.u16VehHeight <= 2000)
	{
		//如果输出的是2轴，车头高与车高相差很小
		if (pVehicle->AxleSet.AxleSum  ==  TWO_AXLE)
		{
			if (pVehicle->EdgeSet.u16VehHeight >= pVehicle->EdgeSet.u16HeadHeight  &&  
				pVehicle->EdgeSet.u16VehHeight - pVehicle->EdgeSet.u16HeadHeight <= 200 )
			{
				pVehicle->FeatureSet.IsJinBei=1;
			}
		}
	}
}

/************************************************************************/
/* 货车检测                                                             */
/* 0--不是货车;1--是货车                                                */
/************************************************************************/
void TruckTailCheck(VehicleSet *pVehicle)
{
//	uint8   l_u8RetValue = 0;
//	uint8   l_u8Index   = 0;
	uint16  l_u16Index  = 0;
	uint16  l_u16IndexTmp = 0;
	int32   l_n32MaxDiff  = 0;

	if (pVehicle  ==  NULL || pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 1||pVehicle->AxleSet.AxleSum<2)
		return ;

	if (pVehicle->EdgeSet.u16VehHeight > pVehicle->EdgeSet.u16RearHeight + 400  &&  //车高与车尾高高度变化大
		pVehicle->EdgeSet.u16Axle2FrCnt > 1  &&  pVehicle->EdgeSet.u16Axle3FrCnt < 1) //正常检测出车尾 
	{
		for (l_u16Index = pVehicle->u16FrameCnt-1; l_u16Index > 0; l_u16Index--)
		{

				for (l_u16IndexTmp = pVehicle->AxleSet.Axle[1].AxleOriEndI; l_u16IndexTmp > pVehicle->AxleSet.Axle[1].AxleOriStaI; l_u16IndexTmp--) //相邻帧高度变化差
				{
					l_n32MaxDiff = (pVehicle->EdgeSet.u16MaxHeight[l_u16IndexTmp-1] - pVehicle->EdgeSet.u16MaxHeight[l_u16IndexTmp] > l_n32MaxDiff) ? 
						pVehicle->EdgeSet.u16MaxHeight[l_u16IndexTmp-1] - pVehicle->EdgeSet.u16MaxHeight[l_u16IndexTmp] : l_n32MaxDiff;
					if (pVehicle->EdgeSet.u16VehHeight  ==  pVehicle->EdgeSet.u16MaxHeight[l_u16IndexTmp]  &&  l_n32MaxDiff > 300)
					{
//						l_u8RetValue = 1;  
						pVehicle->FeatureSet.IsTruckTail=pVehicle->FeatureSet.IsTruckTail|1;
						return ;
					}
				}
				break;
		}
	}
}

/************************************************************************/
/* 皮卡尾部检测 针对无明显高度降的皮卡20170729新加方法待测试            */
/************************************************************************/
void PickUpTailCheck(VehicleSet *pVehicle)
{
	int16 i16Index=0;
	int16 StaI=0;
//	int16 EndI=0;
	uint32 SumH=0;
	uint16 AvgH=0;
	uint16 FitCnt=0;
	uint16 ThreshH=1000;
	uint8 FistFlag=0;
	uint8 MinFrCnt=5;

	//异常情况直接退出 停车直接退出
	if (pVehicle==NULL||pVehicle->u16FrameCnt<8||pVehicle->u16FrameCnt+1>MAX_CUT/2)
	{
		return;
	}


	//开始倒着找车斗 要求车高为1000以上
	for (i16Index=pVehicle->u16FrameCnt-1;i16Index>pVehicle->u16FrameCnt/2;i16Index--)
	{
		//先找到第一个大于1000的帧
		if (pVehicle->EdgeSet.u16MaxHeight[i16Index]>ThreshH&&FistFlag<1)
		{
			FistFlag=1;
			StaI=i16Index;
		}

		//如果遇到当前帧高等于车高 直接终止 即使最大高出现在车斗载货段
		if (pVehicle->EdgeSet.u16MaxHeight[i16Index]+1>pVehicle->EdgeSet.u16VehHeight)
		{
			break;
		}


		//然后寻找平坦的斗 并与车高比较 判断是否是斗
		if (FistFlag>0)
		{
			if ((pVehicle->EdgeSet.u16MaxHeight[i16Index]<pVehicle->EdgeSet.u16MaxHeight[StaI]+1
			&&pVehicle->EdgeSet.u16MaxHeight[i16Index]+150>pVehicle->EdgeSet.u16MaxHeight[StaI])
			||(pVehicle->EdgeSet.u16MaxHeight[StaI]<pVehicle->EdgeSet.u16MaxHeight[i16Index]
			&&pVehicle->EdgeSet.u16MaxHeight[StaI]+150>pVehicle->EdgeSet.u16MaxHeight[i16Index]))
			{
				SumH+=pVehicle->EdgeSet.u16MaxHeight[i16Index];
				FitCnt++;
			}
			else
			{
				//小于5帧 直接给清掉 继续往前找
				if (FitCnt<MinFrCnt)
				{
					SumH=0;
					AvgH=0;
					FitCnt=0;
					StaI=i16Index;
				}
				else
				{
					//超过5帧需要判断是否平坦 且前方存在高度降
					if (pVehicle->EdgeSet.u16MaxHeight[i16Index]-250>pVehicle->EdgeSet.u16MaxHeight[StaI])
					{
						AvgH=SumH/FitCnt;
						if (pVehicle->EdgeSet.u16MaxHeight[i16Index]-350>AvgH)
						{
							pVehicle->FeatureSet.IsTruckTail=pVehicle->FeatureSet.IsTruckTail|8;
							break;
						}
						else if (FitCnt>2*MinFrCnt&&(pVehicle->EdgeSet.u16MaxHeight[StaI]<AvgH
							&&pVehicle->EdgeSet.u16MaxHeight[StaI]+50>AvgH))
						{
							pVehicle->FeatureSet.IsTruckTail=pVehicle->FeatureSet.IsTruckTail|8;
							break;
						}
						else if (FitCnt>2*MinFrCnt&&(pVehicle->EdgeSet.u16MaxHeight[StaI]>AvgH
							&&pVehicle->EdgeSet.u16MaxHeight[StaI]-50<AvgH))
						{
							pVehicle->FeatureSet.IsTruckTail=pVehicle->FeatureSet.IsTruckTail|8;
							break;
						}
						else
						{
							SumH=0;
							AvgH=0;
							FitCnt=0;
							StaI=i16Index;
						}

					}
					else
					{
						SumH=0;
						AvgH=0;
						FitCnt=0;
						StaI=i16Index;
					}
				}
			}
		}
	}

}

/************************************************************************/
/* 皮卡检测帧检测算子                                                   */
/************************************************************************/
void PickUpFramCal(VehicleSet *pVehicle)
{
	uint8  u8BreakPtFlag=0;
	int16  i16TmpIndex1   = 0;
	int16  i16TmpIndex2   = 0;
	int16 i16ThreshI=0;
	int16 i16LeftI=0;
//	int16 i16RightI=0;
	int16 i16BreakI=0;
	int16 i16BreakX=0;
	int16 i16DouCnt=0;
	int16 i16DouHt=0;
	int16 i16ErrPt=0;
	
	int32   l_n32LeftZData     = 0;  //左边的最大高度值
	int32   l_n32LeftXData     = 0;  //左边与最大高度值相对应的X值
	int32   l_n32BaseXData     = 0;  //基准点的X值（）
	int32   l_n32RightZData    = 0;
	int32   l_n32RightXData    = 0;


	uint16 CurFramI=0;
	uint16 PtSum=0;

	if (pVehicle == NULL || pVehicle->u16FrameCnt > FRAME_MAXCNT||pVehicle->u16FrameCnt<8)
		return ;

	CurFramI=pVehicle->u16FrameCnt-1;
	PtSum=pVehicle->CurRegion.zdata[0];

	if (PtSum +1 > FRAME_BUFLEN ) //点数不正常
	{
		return;
	}
		
	if (pVehicle->EdgeSet.u16MaxHeight[CurFramI]< 900) //最大高度小于900
	{
		return;
	}

	//第一步 从下往上寻找第一个大于900的点设为左侧的挡板基点
	for (i16TmpIndex1 = PtSum; i16TmpIndex1 >0; i16TmpIndex1--)
	{
		if (pVehicle->CurRegion.zdata[i16TmpIndex1]>900)
		{
			l_n32BaseXData=pVehicle->CurRegion.xdata[i16TmpIndex1];
			l_n32LeftZData=pVehicle->CurRegion.zdata[i16TmpIndex1];
			i16ThreshI=i16TmpIndex1;
			break;
		}
	}
	//第二步 继续搜寻是否存在相邻两点x跨度相差500的点 同时对高度进行检验
	u8BreakPtFlag=0;
	for (i16TmpIndex1=i16ThreshI-1; i16TmpIndex1 >0; i16TmpIndex1--)
	{
		if (pVehicle->CurRegion.xdata[i16TmpIndex1]>0)
		{
			for (i16TmpIndex2=i16TmpIndex1-1;i16TmpIndex2 >0;i16TmpIndex2--)
			{
				if (pVehicle->CurRegion.xdata[i16TmpIndex2]>0)
				{
					if (pVehicle->CurRegion.xdata[i16TmpIndex2]-pVehicle->CurRegion.xdata[i16TmpIndex1]>500
						&&pVehicle->CurRegion.xdata[i16TmpIndex2]-l_n32BaseXData>500)
					{
						u8BreakPtFlag=1;
						i16BreakI=i16TmpIndex2;
						i16BreakX=pVehicle->CurRegion.xdata[i16TmpIndex2];
						i16LeftI=i16TmpIndex1;
					}
					break;
				}
			}
		}
		
		if (i16TmpIndex2<1||u8BreakPtFlag>0)
		{
			break;
		}
	}

	//如果没有分界点则退出
	if (u8BreakPtFlag<1)
	{
		return;
	}


	for (i16TmpIndex1=i16ThreshI;i16TmpIndex1>=i16LeftI;i16TmpIndex1--)
	{
		if (l_n32LeftZData<pVehicle->CurRegion.zdata[i16TmpIndex1])
		{
			l_n32LeftXData=pVehicle->CurRegion.xdata[i16TmpIndex1];
			l_n32LeftZData=pVehicle->CurRegion.zdata[i16TmpIndex1];
		}
	}
	//第三步 寻找右侧挡板的基点 及  斗中点
	if (pVehicle->CurRegion.xMax-l_n32LeftXData<1000)
	{
		//如果两侧挡板间距<1000直接跳出判断
		return;
	}

	for (i16TmpIndex1=1;i16TmpIndex1<i16BreakI;i16TmpIndex1++)
	{
		if (pVehicle->CurRegion.xdata[i16TmpIndex1]<1)
		{
			//过滤打飞点
			continue;
		}

		if (pVehicle->CurRegion.xdata[i16TmpIndex1]+150>pVehicle->CurRegion.xMax)
		{
			if (l_n32RightZData<pVehicle->CurRegion.zdata[i16TmpIndex1])
			{
				l_n32RightZData=pVehicle->CurRegion.zdata[i16TmpIndex1];
				l_n32RightXData=pVehicle->CurRegion.xdata[i16TmpIndex1];
//				i16RightI=i16TmpIndex1;
			}
		}
		else
		{
			if (pVehicle->CurRegion.zdata[i16TmpIndex1]<l_n32LeftZData)
			{
				i16DouCnt++;
			}
			if (i16DouHt==0)
			{
				i16DouHt=pVehicle->CurRegion.zdata[i16TmpIndex1];
			}
			else if (i16DouHt>pVehicle->CurRegion.zdata[i16TmpIndex1])
			{
				i16DouHt=pVehicle->CurRegion.zdata[i16TmpIndex1];
			}


			//如果从右侧遍历过来的点处于分界点左侧 则为左侧驾驶室打穿情况
			if (pVehicle->CurRegion.xdata[i16TmpIndex1]<l_n32LeftXData+500
				&&pVehicle->CurRegion.xdata[i16TmpIndex1]>l_n32LeftZData)
			{
				i16ErrPt++;
			}
		}
	}

	//第四步 判断两侧挡板X间隔，挡板直接是否存在斗深  20170804增加斗深的最低高上限限制
	if ((l_n32RightXData - l_n32LeftXData > 1000) && i16BreakX < l_n32RightXData
		&&i16DouCnt > 0 && (l_n32RightZData-i16DouHt > 250) && i16DouHt > 500 && i16DouHt <1500&& i16ErrPt<5) //
	{
		pVehicle->SpeVSet.u8PickUpFlag[CurFramI]=1;
	}
}

/************************************************************************/
/* 皮卡检测                                                             */
/************************************************************************/
void PickUpCheck(VehicleSet *pVehicle)
{
	uint16 PickFrCnt=0;
	uint16 u16Index=0;
	uint16 StaI=0;

	if (pVehicle == NULL || pVehicle->u16FrameCnt > FRAME_MAXCNT||pVehicle->u16FrameCnt<8)
		return ;

	
	for (u16Index=0;u16Index<pVehicle->u16FrameCnt;u16Index++)
	{
		if (StaI<1)
		{
			if (pVehicle->EdgeSet.u16MaxHeight[u16Index]+1>pVehicle->EdgeSet.u16VehHeight)
			{
				StaI=u16Index;
			}
			else if (u16Index>pVehicle->u16FrameCnt/2)
			{
				StaI=u16Index;
			}
		}
		else
		{
			if (pVehicle->SpeVSet.u8PickUpFlag[u16Index]>0&&pVehicle->EdgeSet.u16MaxHeight[u16Index]>1000)
			{
				PickFrCnt++;
			}
		}
	}

	//20170729 修改皮卡斗帧为至少1帧(最初定为2帧)
	if (PickFrCnt>0 && pVehicle->FrShape.LoDaFeiR<60)
	{
		if(PickFrCnt<5&&pVehicle->FrShape.LoDaFeiR<50)
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|1;
		else 
		{
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|1;
		}
	}
}

/************************************************************************/
/* 封闭帧检测                                                           */
/************************************************************************/
void EnCloseWinCal(VehicleSet *pVehicle)
{
	int16 i16Index=0;
	int16 StaI=0;
	int16 EndI=0;
	uint16 ThreshH=1500;
	uint16 ThreshL=1000;
	uint16 DeltXZ=50;
	uint16 CastX=0;

	uint16 CurFramI=0;
	uint16 PtSum=0;

	uint8  FitFlag=1;
	uint8  DafeiCnt=0;
//	int16 MaxX=0;
//	int16 MinX=9999;

	if (pVehicle->AxleSet.AxleSum<1||pVehicle->u16FrameCnt<8)
	{
		return;
	}

	PtSum=pVehicle->CurRegion.xdata[0];
	CurFramI=pVehicle->u16FrameCnt-1;
	CastX=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleCastX;

	if (pVehicle->EdgeSet.u16MaxHeight[CurFramI]<1500)
	{
		return;
	}

	if (pVehicle->EdgeSet.u16MaxHeight[CurFramI]<1800)
	{
		ThreshH=pVehicle->EdgeSet.u16MaxHeight[CurFramI]-100;
	}
	else
	{
		ThreshH=pVehicle->EdgeSet.u16MaxHeight[CurFramI]-500;
	}


	//找到检测的起止段
	for (i16Index=0;i16Index<PtSum;i16Index++)
	{
		if (pVehicle->CurRegion.zdata[PtSum-i16Index]>0 && pVehicle->CurRegion.xdata[PtSum-i16Index]<CastX+300)
		{
			if (pVehicle->CurRegion.zdata[PtSum-i16Index]>ThreshL && StaI == 0)
			{
				StaI=PtSum-i16Index;
			}
		}


		if (pVehicle->CurRegion.zdata[1+i16Index]>0 && pVehicle->CurRegion.xdata[1+i16Index]<CastX+300)
		{
			if (pVehicle->CurRegion.zdata[1+i16Index]<ThreshH && EndI == 0)
			{
				EndI=1+i16Index;
			}
		}


		if (StaI>1 && EndI>1)
		{
			break;
		}
	}

	//异常判断
	if (EndI>StaI||EndI<2||StaI<2)
	{
		return;
	}

	//开始进行封闭检测
	for (i16Index=StaI;i16Index>EndI;i16Index--)
	{
		if (pVehicle->CurRegion.xdata[i16Index]>0)
		{
			DafeiCnt=0;
			if (ABS(pVehicle->CurRegion.xdata[i16Index]-pVehicle->CurRegion.xdata[i16Index-1])>DeltXZ
				||ABS(pVehicle->CurRegion.zdata[i16Index]-pVehicle->CurRegion.zdata[i16Index-1])>DeltXZ)
			{
				FitFlag=0;
				break;
			}
		}
		else
		{
			DafeiCnt++;
			if (DafeiCnt>DAFEIMAXPT)
			{
				FitFlag=0;
				break;
			}
		}

	}

	//不满足要求 直接返回
	if (FitFlag>0)
	{
		pVehicle->SideSet.SideFlag[CurFramI]=pVehicle->SideSet.SideFlag[CurFramI]|2;
	}
}


/************************************************************************/
/* 封闭车窗检测算子改进版                                               */
/************************************************************************/
void EnCloseWinCalEx(VehicleSet *pVehicle)
{
	int16 i16Index=0;
	int16 StaI=0;
	int16 EndI=0;
//	uint16 ThreshH=1500;
	uint16 ThreshL=1000;
	uint16 DeltXZ=50;
	uint16 CastX=0;

	uint16 CurFramI=0;
	uint16 PtSum=0;

//	uint8  FitFlag=1;
	uint8  DafeiCnt=0;
//	int16 MaxX=0;
//	int16 MinX=9999;

	if (pVehicle->AxleSet.AxleSum<1||pVehicle->u16FrameCnt<8)
	{
		return;
	}

	PtSum=pVehicle->CurRegion.xdata[0];
	CurFramI=pVehicle->u16FrameCnt-1;
	CastX=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleCastX;

	if (pVehicle->EdgeSet.u16MaxHeight[CurFramI]<1200)
	{
		return;
	}


	//找到检测的起止段
	for (i16Index=0;i16Index<PtSum;i16Index++)
	{
		if (pVehicle->CurRegion.zdata[PtSum-i16Index]>0 && pVehicle->CurRegion.xdata[PtSum-i16Index]<CastX+300)
		{
			if (pVehicle->CurRegion.zdata[PtSum-i16Index]>ThreshL && StaI == 0)
			{
				StaI=PtSum-i16Index;
			}
		}

		if (StaI>1)
		{
			break;
		}
	}

	//异常判断
	EndI=2;

	if (EndI>StaI||EndI<2||StaI<2)
	{
		return;
	}

	//开始进行封闭检测
	for (i16Index=StaI;i16Index>EndI;i16Index--)
	{
		if (pVehicle->CurRegion.xdata[i16Index]>0)
		{
			DafeiCnt=0;
			if (ABS(pVehicle->CurRegion.xdata[i16Index]-pVehicle->CurRegion.xdata[i16Index-1])>DeltXZ
				||ABS(pVehicle->CurRegion.zdata[i16Index]-pVehicle->CurRegion.zdata[i16Index-1])>DeltXZ)
			{
//				FitFlag=0;
				pVehicle->SpeVSet.u16CloseWin[CurFramI]=pVehicle->CurRegion.zdata[i16Index];
				break;
			}
			else
			{
				pVehicle->SpeVSet.u16CloseWin[CurFramI]=pVehicle->CurRegion.zdata[i16Index];
			}
		}
		else
		{
			DafeiCnt++;
			if (DafeiCnt>DAFEIMAXPT)
			{
//				FitFlag=0;
				break;
			}
		}

	}

	////不满足要求 直接返回
	//if (FitFlag>0)
	//{
	//	pVehicle->SideSet.SideFlag[CurFramI]=pVehicle->SideSet.SideFlag[CurFramI]|2;
	//}
}

/************************************************************************/
/* 封闭客车检测判为货车                                                 */
/************************************************************************/
void EnCloseWinCheck(VehicleSet *pVehicle)
{
	int16 i16Index=0;
	int16 StaI=0;
	int16 EndI=0;
	int16 CheckCnt=0;
	int16 DisFitCnt=0;
	int16 FitCnt=0;

	if (pVehicle->AxleSet.AxleSum<1||pVehicle->EdgeSet.u16VehHeight<1700||pVehicle->u16FrameCnt+1>FRAME_MAXCNT)
	{
		return;
	}

	if (pVehicle->AxleSet.AxleSum<2)
	{
		if (pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI>pVehicle->u16FrameCnt/2)
		{
			StaI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI;
			EndI=MIN(3*pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI/2-StaI/2,pVehicle->u16FrameCnt-3);//判断终点往后延半个轮轴
		}
		else
		{
			StaI=pVehicle->u16FrameCnt/2;
			EndI=pVehicle->u16FrameCnt*4/5;
		}
	}
	else if (pVehicle->AxleSet.AxleSum<3)
	{
		StaI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI;
		EndI=MIN(3*pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI/2-StaI/2,pVehicle->u16FrameCnt-3);//判断终点往后延半个轮轴
	}



	for (i16Index=StaI;i16Index<EndI;i16Index++)
	{
		if (pVehicle->EdgeSet.u16MaxHeight[i16Index]+150>pVehicle->EdgeSet.u16VehHeight)
		{
			CheckCnt++;
			if (pVehicle->SideSet.SideFlag[i16Index]&2&&pVehicle->WindowsSet.GlassWinFlag[i16Index]<1)
			{
				FitCnt++;
			}
		}
		else
		{
			DisFitCnt++;
		}
	}

	//判断是否封闭 20170727 修改阈值95%为90%
	if (CheckCnt && FitCnt*100/CheckCnt>90 && DisFitCnt*100/CheckCnt<20)
	{
		pVehicle->FeatureSet.IsFillWin=1;
	}
}


/************************************************************************/
/* 封闭客车检测判为货车 改进版	                                        */
/************************************************************************/
void EnCloseWinCheckEx(VehicleSet *pVehicle)
{
	int16 i16Index=0;
	int16 StaI=0;
	int16 EndI=0;
//	int16 CheckCnt=0;
	int16 DisFitCnt=0;
//	int16 FitCnt=0;
	uint32 SumH=0;
	uint16 AvgH=0;
	uint32 MaxH=0;
	uint32 MinH=0;
	uint16 FrCnt=0;
	int16 DisFitI=0;

	if (pVehicle->AxleSet.AxleSum<1||pVehicle->EdgeSet.u16VehHeight<1650||pVehicle->u16FrameCnt+1>FRAME_MAXCNT)
	{
		return;
	}

	//根据轮轴信息确定封闭车窗的检测段
	if (pVehicle->AxleSet.AxleSum<2)
	{
		if (pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI>pVehicle->u16FrameCnt/2)
		{
			StaI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI;
			EndI=MIN(3*pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI/2-StaI/2,pVehicle->u16FrameCnt-3);//判断终点往后延半个轮轴
		}
		else
		{
			StaI=pVehicle->u16FrameCnt/2;
			EndI=pVehicle->u16FrameCnt*4/5;
		}
	}
	else if (pVehicle->AxleSet.AxleSum<3)
	{
		StaI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI;
		EndI=MIN(3*pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI/2-StaI/2,pVehicle->u16FrameCnt-3);//判断终点往后延半个轮轴
	}


	//统计该段第一个不封闭的点高度 可能是玻璃穿透 可能是顶部分界点
	for (i16Index=StaI;i16Index<EndI;i16Index++)
	{
		if (pVehicle->SpeVSet.u16CloseWin[i16Index]>MaxH)
		{
			MaxH=pVehicle->SpeVSet.u16CloseWin[i16Index];
		}


		if (MinH==0)
		{
			MinH=pVehicle->SpeVSet.u16CloseWin[i16Index];
		}
		else if (pVehicle->SpeVSet.u16CloseWin[i16Index]<MinH)
		{
			MinH=pVehicle->SpeVSet.u16CloseWin[i16Index];
		}

		SumH+=pVehicle->SpeVSet.u16CloseWin[i16Index];

		FrCnt++;
	}

	if (FrCnt>0)
	{
		AvgH=SumH/FrCnt;
	}

	//如果该高度值不处在同一范围则不满足封闭客舱的要求
	if (MaxH-100>AvgH||AvgH-100>MinH)
	{
		return;
	}

	//如果最大高、最小高 、 均值高处于同一水平 继续判断是否处在预期的玻璃段
	if (pVehicle->EdgeSet.u16VehHeight-AvgH>300)
	{
		return;
	}
	
	//往前寻找与该均高处于同一水平的帧，如果中间出现较低的帧 扩大检测范围 确认不满足要求
	for (i16Index=StaI-1;i16Index>2;i16Index--)
	{
		if (pVehicle->SpeVSet.u16CloseWin[i16Index]<AvgH&&pVehicle->SpeVSet.u16CloseWin[i16Index]+100>AvgH)
		{
			if (DisFitCnt>0)
			{
				DisFitI=i16Index;
				break;
			}
			else
			{
				//扩展范围
				StaI=i16Index;
			}
		}
		else if (pVehicle->SpeVSet.u16CloseWin[i16Index]>AvgH&&pVehicle->SpeVSet.u16CloseWin[i16Index]-100<AvgH)
		{
			if (DisFitCnt>0)
			{
				DisFitI=i16Index;
				break;
			}			
			else
			{
				//扩展范围
				StaI=i16Index;
			}
		}
		else
		{
			DisFitCnt++;
		}
	}

	//判断是否封闭 20170727 修改阈值95%为90%
	if (DisFitCnt>1&&(StaI-DisFitI)<(EndI-StaI)/2)
	{
		pVehicle->FeatureSet.IsFillWin=0;
	}
	else
	{
		pVehicle->FeatureSet.IsFillWin=1;
	}
}

/************************************************************************/
/* 第二轴车窗检测 判为客车 改进版	                                    */
/************************************************************************/
void Axle2WinCheckEx(VehicleSet *pVehicle)
{
	int16 i16Index=0;
	int16 StaI=0;
	int16 EndI=0;
//	int16 CheckCnt=0;
//	int16 DisFitCnt=0;
	int16 FitCnt=0;
	uint32 SumH=0;
	uint16 AvgH=0;
	uint32 MaxH=0;
	uint32 MinH=0;
	uint16 FrCnt=0;
//	int16 DisFitI=0;

	if (pVehicle->AxleSet.AxleSum<1||pVehicle->EdgeSet.u16VehHeight<1650||pVehicle->u16FrameCnt+1>FRAME_MAXCNT)
	{
		return;
	}

	//根据轮轴信息确定第二轴车窗的检测段
	if (pVehicle->AxleSet.AxleSum<2)
	{
		if (pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI>pVehicle->u16FrameCnt/2)
		{
			StaI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI;
			EndI=MIN(pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI,pVehicle->u16FrameCnt-3);//判断终点往后延半个轮轴
		}
		else
		{
			StaI=pVehicle->u16FrameCnt/2;
			EndI=pVehicle->u16FrameCnt*4/5;
		}
	}
	else if (pVehicle->AxleSet.AxleSum<3)
	{
		StaI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI;
		EndI=MIN(pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI,pVehicle->u16FrameCnt-3);//判断终点往后延半个轮轴
	}


	//统计该段第一个不封闭的点高度 可能是玻璃穿透 可能是顶部分界点
	for (i16Index=StaI;i16Index<EndI;i16Index++)
	{
		if (pVehicle->EdgeSet.u16VehHeight-pVehicle->EdgeSet.u16MaxHeight[i16Index]<150)
		{
			FitCnt++;
		}
		
		if (pVehicle->SpeVSet.u16CloseWin[i16Index]>MaxH)
		{
			MaxH=pVehicle->SpeVSet.u16CloseWin[i16Index];
		}


		if (pVehicle->SpeVSet.u16CloseWin[i16Index]>0
			&& pVehicle->EdgeSet.u16VehHeight-pVehicle->SpeVSet.u16CloseWin[i16Index]>300)
		{
			if (MinH==0)
			{
				MinH=pVehicle->SpeVSet.u16CloseWin[i16Index];
			}
			else if (pVehicle->SpeVSet.u16CloseWin[i16Index]<MinH)
			{
				MinH=pVehicle->SpeVSet.u16CloseWin[i16Index];
			}

			SumH+=pVehicle->SpeVSet.u16CloseWin[i16Index];

			FrCnt++;
		}
	}
	

	//找到乖点高度与车高差大于300以上的帧
	if (FrCnt>0)
	{
		AvgH=SumH/FrCnt;

		//如果该高度值不处在同一范围则不满足封闭客舱的要求
		if (MaxH-100>AvgH || AvgH-100>MinH)
		{
			if (FitCnt*100/FrCnt<30 && MaxH-MinH>300 
				&& pVehicle->EdgeSet.u16VehHeight-MaxH > 300)
			{
				//拐点高度不平齐 呈现二级降
				pVehicle->FeatureSet.IsTruckTail=pVehicle->FeatureSet.IsTruckTail|16;
			}
		}
		else
		{
			////判断是否为有窗客车 拐点高度一直 且与车高差距150mm以内
			//if (FrCnt*100>(EndI-StaI)*70 && FitCnt*100/FrCnt>70 
			//	&& pVehicle->EdgeSet.u16VehHeight-MaxH<800)
			//{
			//	//仅识别金杯 SUV 尾部客车
			//	pVehicle->FeatureSet.IsCar=1;
			//}
			//else 
				if (FitCnt*100/FrCnt>70 && pVehicle->EdgeSet.u16VehHeight-MaxH>1000)
			{
				pVehicle->FeatureSet.IsTruckTail=pVehicle->FeatureSet.IsTruckTail|32;
			}
			else if (FitCnt*100/FrCnt<25 && pVehicle->EdgeSet.u16VehHeight-MaxH>300)
			{
				//尾部拐点高度同高 且不与车高相同 与最大高差超300
				pVehicle->FeatureSet.IsTruckTail=pVehicle->FeatureSet.IsTruckTail|32;
			}
		}
	}
}
