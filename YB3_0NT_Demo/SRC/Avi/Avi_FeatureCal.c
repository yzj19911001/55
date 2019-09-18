#include "stdafx.h"
#include "Avi_FeatureCal.h"
#include "Avi_MathProc.h"
#include "Avi_SpeCheck.h"
#include "Avi_AxleCheck.h"

/************************************************************************/
/* 获取最终底盘高特征值                                                 */
/************************************************************************/
void DpHtCheck(VehicleSet *pVehicle)
{
	uint8 TmpI=0;
	uint32 TmpSum=0;
	uint32 TmpCnt=0;
	//计算底盘高
	if (pVehicle->AxleSet.AxleSum<= TWO_AXLE)
	{
		if (pVehicle->Dp.DpFrSum1_2 > 4)
		{
			TmpSum=0;
			TmpCnt=0;
			for (TmpI=0;TmpI<5;TmpI++)
			{
				if (pVehicle->Dp.DpHtHist1_2[TmpI][0]*10/pVehicle->Dp.DpFrSum1_2>6)
				{
					TmpSum=pVehicle->Dp.DpHtHist1_2[TmpI][1];
					TmpCnt=pVehicle->Dp.DpHtHist1_2[TmpI][0];
					break;
				}

				if (pVehicle->Dp.DpHtHist1_2[TmpI][0]*10/pVehicle->Dp.DpFrSum1_2>1)
				{
					TmpSum+=pVehicle->Dp.DpHtHist1_2[TmpI][1];
					TmpCnt+=pVehicle->Dp.DpHtHist1_2[TmpI][0];
				}
			}

			//20170720采用加权底盘高 代替平均底盘高
			//pVehicle->Dp.DpHeight1_2=(pVehicle->Dp.DpHtTop1_2[0]*5+pVehicle->Dp.DpHtTop1_2[0]*3+pVehicle->Dp.DpHtTop1_2[2])/10;
			//pVehicle->Dp.DpHeight1_2 = pVehicle->Dp.DpHeight1_2/pVehicle->Dp.DpFrSum1_2 ;
			if (TmpCnt)
			{
				pVehicle->Dp.DpHeight1_2=TmpSum/TmpCnt;
			}
			else if (pVehicle->Dp.DpFrSum1_2)
			{
				pVehicle->Dp.DpHeight1_2=pVehicle->Dp.DpHeight1_2/pVehicle->Dp.DpFrSum1_2;
			}
		}
		else
			pVehicle->Dp.DpHeight1_2 = 0;

		if (pVehicle->AxleSet.AxleSum  ==  ONE_AXLE  &&  pVehicle->Dp.DpHeight1_2 >= 430)
			pVehicle->Dp.DpHeight1_2 -= 50;

	}
	else if (pVehicle->Dp.DpFrSum1_2 > 0)
	{

		TmpSum=0;
		TmpCnt=0;
		for (TmpI=0;TmpI<5;TmpI++)
		{
			if (pVehicle->Dp.DpHtHist1_2[TmpI][0]*10/pVehicle->Dp.DpFrSum1_2>6)
			{
				TmpSum=pVehicle->Dp.DpHtHist1_2[TmpI][1];
				TmpCnt=pVehicle->Dp.DpHtHist1_2[TmpI][0];
				break;
			}

			if (pVehicle->Dp.DpHtHist1_2[TmpI][0]*10/pVehicle->Dp.DpFrSum1_2>1)
			{
				TmpSum+=pVehicle->Dp.DpHtHist1_2[TmpI][1];
				TmpCnt+=pVehicle->Dp.DpHtHist1_2[TmpI][0];
			}
		}

		//20170720采用加权底盘高 代替平均底盘高
		//pVehicle->Dp.DpHeight1_2=(pVehicle->Dp.DpHtTop1_2[0]*5+pVehicle->Dp.DpHtTop1_2[0]*3+pVehicle->Dp.DpHtTop1_2[2])/10;
		//pVehicle->Dp.DpHeight1_2 = pVehicle->Dp.DpHeight1_2/pVehicle->Dp.DpFrSum1_2 ;
		if (TmpCnt)
		{
			pVehicle->Dp.DpHeight1_2=TmpSum/TmpCnt;
		}
		else if (pVehicle->Dp.DpFrSum1_2)
		{
			pVehicle->Dp.DpHeight1_2=pVehicle->Dp.DpHeight1_2/pVehicle->Dp.DpFrSum1_2;
		}
		//pVehicle->Dp.DpHeight1_2 = pVehicle->Dp.DpHeight1_2/pVehicle->Dp.DpFrSum1_2;
	}

	if (pVehicle->Dp.DpFrSum2_3  > 0)
	{
		//pVehicle->Dp.DpHeight2_3=(pVehicle->Dp.DpHtTop2_3[0]*5+pVehicle->Dp.DpHtTop2_3[0]*3+pVehicle->Dp.DpHtTop2_3[2])/10;
		//pVehicle->Dp.DpHeight2_3 = pVehicle->Dp.DpHeight2_3/pVehicle->Dp.DpFrSum2_3;	
		TmpSum=0;
		TmpCnt=0;
		for (TmpI=0;TmpI<5;TmpI++)
		{
			if (pVehicle->Dp.DpHtHist2_3[TmpI][0]*10/pVehicle->Dp.DpFrSum2_3>6)
			{
				TmpSum=pVehicle->Dp.DpHtHist2_3[TmpI][1];
				TmpCnt=pVehicle->Dp.DpHtHist2_3[TmpI][0];
				break;
			}

			if (pVehicle->Dp.DpHtHist2_3[TmpI][0]*10/pVehicle->Dp.DpFrSum2_3>1)
			{
				TmpSum+=pVehicle->Dp.DpHtHist2_3[TmpI][1];
				TmpCnt+=pVehicle->Dp.DpHtHist2_3[TmpI][0];
			}
		}

		//20170720采用加权底盘高 代替平均底盘高
		//pVehicle->Dp.DpHeight1_2=(pVehicle->Dp.DpHtTop1_2[0]*5+pVehicle->Dp.DpHtTop1_2[0]*3+pVehicle->Dp.DpHtTop1_2[2])/10;
		//pVehicle->Dp.DpHeight1_2 = pVehicle->Dp.DpHeight1_2/pVehicle->Dp.DpFrSum1_2 ;
		if (TmpCnt)
		{
			pVehicle->Dp.DpHeight2_3=TmpSum/TmpCnt;
		}
		else if (pVehicle->Dp.DpFrSum2_3)
		{
			pVehicle->Dp.DpHeight2_3=pVehicle->Dp.DpHeight2_3/pVehicle->Dp.DpFrSum2_3;
		}
	}

}

/************************************************************************/
/*获取对应帧底盘高                                                      */
/************************************************************************/
void  DpHeightCal(VehicleSet *pVehicle)
{
	uint16   l_u16DaFeiPtCnt  = 0;
	uint16   l_u16LeftPoint = 0;
	uint16   l_u16RightPoint = 0;
	uint16   l_u16Index = 0;
//	uint16   l_u16TmpIndex = 0;
	uint16   l_u16MaxHeightThres = 800;  //高度阈值上线
	uint16   l_u16MinHeightThres = 300;  //高度阈值下线
	uint16   l_u16WideThres   = 150;  //宽度相差阈值
	uint16   l_u16Count       = 0;
	uint16   l_u16Index2      = 0;
	uint16   l_AxleDp1Ht=0;
	uint16   l_Axle2DpHt=0;
	int16    l_DpIndex=0;



	if (pVehicle  ==  NULL||pVehicle->AxleSet.AxleSum<1||pVehicle->AxleSet.AxleSum>2)
		return;
	l_u16LeftPoint =pVehicle->CurFrmInfo.u16StaPos;
	l_u16RightPoint = pVehicle->CurFrmInfo.u16EndPos;
	
	
	

	if (l_u16LeftPoint >= POINT_SUM || l_u16RightPoint>= POINT_SUM || l_u16LeftPoint >= l_u16RightPoint||l_u16RightPoint<8)
		return;

	//20150522 修改
	if (pVehicle->CurFrmInfo.u16MaxHt <= l_u16MaxHeightThres + 200)
		l_u16MaxHeightThres = 600;
	//20150522

	//大点方向是轴
	//if (g_FiltZ[l_u16RightPoint] > l_u16MaxHeightThres) //最大点大于阈值
	//{
	//	//20150427 增加
	//	//直接判断底盘处是否打飞
	//	if (g_FiltZ[l_u16RightPoint] > CHASSISHEIGHTTHRES  &&  (g_FiltZ[l_u16RightPoint]  ==  g_FiltZ[l_u16RightPoint-1]  && 
	//		g_FiltZ[l_u16RightPoint-1]  ==  g_FiltZ[l_u16RightPoint-2]))
	//		return;
	//	//20150427 增加完成
	//	if (pVehicle->AxleSet.AxleSum  ==  ONE_AXLE)//存储第1、2轴之间底盘高
	//	{
	//		pVehicle->Dp.DpHeight1_2 += g_FiltZ[l_u16RightPoint];
	//		pVehicle->Dp.DpFrSum1_2++;		
	//	}
	//	else if (pVehicle->AxleSet.AxleSum  ==  TWO_AXLE) //存储第2、3轴之间底盘高
	//	{
	//		pVehicle->Dp.DpHeight2_3 += g_FiltZ[l_u16RightPoint];
	//		pVehicle->Dp.DpFrSum2_3++;		
	//	}
	//	return;
	//}

	//直接判断底盘处是否打飞
	if (g_FiltZ[l_u16RightPoint] > CHASSISHEIGHTTHRES  &&  (g_FiltZ[l_u16RightPoint-1] == 0  && 
		g_FiltZ[l_u16RightPoint-2] == 0 && g_FiltZ[l_u16RightPoint-3] == 0))
		return;
	//先寻找到高度阈值下的第1个点,然后开始找底盘高
	for (l_u16Index = l_u16RightPoint; l_u16Index > l_u16LeftPoint; l_u16Index--)
	{
		//判断打飞
		if (0  ==  g_FiltZ[l_u16Index])
		{
			l_u16DaFeiPtCnt++;
			if (l_u16DaFeiPtCnt>2*DAFEIMAXPT)
			{
				//连续打飞超过8个点直接退出
				return;
			}
		}
		else if (g_FiltZ[l_u16Index] > l_u16MaxHeightThres) //跳出
		{
			break;
		}
		else if (g_FiltZ[l_u16Index]<l_u16MaxHeightThres+1)
		{
			l_u16DaFeiPtCnt = 0;
		}
	}

	if (l_u16DaFeiPtCnt > 0)
		l_u16Index += (l_u16DaFeiPtCnt+1);
	else
		l_u16Index++;

	
	l_DpIndex=l_u16Index;//第一次找到的底盘点
	l_u16DaFeiPtCnt = 0;

	//寻找底盘高所在点 20170721  底盘点继续往下收缩及第二次搜寻的底盘点
	for (l_u16Index=l_DpIndex;l_u16Index<l_u16RightPoint+1;l_u16Index++)
	{
		//排除打飞造成的宽度影响
		if (0  ==  g_FiltZ[l_u16Index])
		{
			l_u16DaFeiPtCnt++;
			if (l_u16DaFeiPtCnt>DAFEIMAXPT)
			{
				return;
			}
			continue;
		}
		else
		{
			l_u16DaFeiPtCnt = 0;

			if (g_FiltZ[l_u16Index] < l_u16MinHeightThres)
			{
				break;
			}
			else if (ABS(g_FiltX[l_u16Index]-g_FiltX[l_DpIndex]) > l_u16WideThres)
			{
				l_DpIndex=l_u16Index;
				break;
			}
			else
			{
				l_DpIndex=l_u16Index;
			}

		}
	}

	//在第二次搜寻到底盘点基础上做出判断 
	//判断是否打飞
	if (l_u16DaFeiPtCnt > 1  &&  g_FiltZ[l_DpIndex-1] > CHASSISHEIGHTTHRES  &&  g_FiltZ[l_DpIndex] < l_u16MinHeightThres  &&  l_DpIndex <= l_u16RightPoint)
	{
		//前一个点高于底盘阈值，当前底盘点高于300，且存在打飞点
		return;
	}
	else if (g_FiltZ[l_DpIndex] > CHASSISHEIGHTTHRES  &&  g_FiltZ[l_DpIndex-1]  == 0  &&  g_FiltZ[l_DpIndex-2] == 0  &&  g_FiltZ[l_DpIndex-3] == 0)
	{
		//当前底盘点高度大于底盘高阈值，前三个点打飞
		return;
	}
	else if (l_DpIndex < l_u16RightPoint  &&  g_FiltZ[l_DpIndex] > CHASSISHEIGHTTHRES)
	{
		//第二次底盘点下方是否存在打飞 打飞这退出计算底盘高
		for (l_u16Index2 = l_DpIndex; l_u16Index2 < l_u16RightPoint; l_u16Index2++)
		{
			if (g_FiltZ[l_u16Index2] > CHASSISHEIGHTTHRES)
			{
				if (g_FiltZ[l_u16Index2+1]  ==  0)
				{ 
					//打飞
					l_u16Count++;
				}
			}
			else if(g_FiltZ[l_u16Index2] < l_u16MinHeightThres)
			{
				//低于底盘阈值 直接退出
				return;
			}
			else 
			{
				l_DpIndex=l_u16Index2;
			}

			if (l_u16Count > 0  &&  g_FiltZ[l_u16Index2]  ==  0) //打飞
			{
				//连续打飞
				l_u16Count++;
			}
			if (l_u16Count>1)
			{
				//连续打飞两个点
				return;
			}
		}
	}
	else if (l_DpIndex<POINT_SUM-3)
	{
		//当前底盘点后面三个点打飞
		if (g_FiltZ[l_DpIndex] > CHASSISHEIGHTTHRES  &&  g_FiltZ[l_DpIndex+1]  == 0 
			&&  g_FiltZ[l_DpIndex+2] == 0 && g_FiltZ[l_DpIndex+3] == 0)
			return;
	}

	if (pVehicle->AxleSet.AxleSum  ==  ONE_AXLE && g_FiltZ[l_DpIndex] > l_u16MinHeightThres)//存储第1、2轴之间底盘高
	{
		l_AxleDp1Ht=g_FiltZ[l_DpIndex];
		pVehicle->Dp.DpHeight1_2 += l_AxleDp1Ht;
		pVehicle->Dp.DpFrSum1_2++;	
		if (l_AxleDp1Ht>CHASSISHEIGHTTHRES && l_AxleDp1Ht<530)
		{
			pVehicle->Dp.DpHtHist1_2[(l_AxleDp1Ht-380)/50+1][1]+=l_AxleDp1Ht;
			pVehicle->Dp.DpHtHist1_2[(l_AxleDp1Ht-380)/50+1][0]++;
		}
		else if (l_AxleDp1Ht>529)
		{
			pVehicle->Dp.DpHtHist1_2[4][1]+=l_AxleDp1Ht;
			pVehicle->Dp.DpHtHist1_2[4][0]++;
		}
		else if (l_AxleDp1Ht<CHASSISHEIGHTTHRES)
		{
			pVehicle->Dp.DpHtHist1_2[0][1]+=l_AxleDp1Ht;
			pVehicle->Dp.DpHtHist1_2[0][0]++;
		}
	}
	else if (pVehicle->AxleSet.AxleSum  ==  TWO_AXLE  && g_FiltZ[l_DpIndex] > l_u16MinHeightThres ) //存储第2、3轴之间底盘高
	{
		
		l_Axle2DpHt=g_FiltZ[l_DpIndex];

		pVehicle->Dp.DpHeight2_3 += l_Axle2DpHt;
		pVehicle->Dp.DpFrSum2_3++;		

		if (l_Axle2DpHt>CHASSISHEIGHTTHRES && l_Axle2DpHt<530)
		{
			pVehicle->Dp.DpHtHist2_3[(g_FiltZ[l_DpIndex]-380)/50+1][1]+=l_Axle2DpHt;
			pVehicle->Dp.DpHtHist2_3[(g_FiltZ[l_DpIndex]-380)/50+1][0]++;
		}
		else if (l_Axle2DpHt>529)
		{
			pVehicle->Dp.DpHtHist2_3[4][1]+=l_Axle2DpHt;
			pVehicle->Dp.DpHtHist2_3[4][0]++;
		}
		else if (l_Axle2DpHt<CHASSISHEIGHTTHRES)
		{
			pVehicle->Dp.DpHtHist2_3[0][1]+=l_Axle2DpHt;
			pVehicle->Dp.DpHtHist2_3[0][0]++;
		}
	}
}


/************************************************************************/
/*侧面检测 计算当前帧1米以下侧面的一致性，用来区分客货					*/
/*输入：pVehicle 车辆信息结构体											*/
/*返回值：无                                                            */
/************************************************************************/
void SideUniformCal(VehicleSet *pVehicle)
{
	uint8   l_u8PtCount  = 0;  //在高度阈值内的点数
	uint16  l_u16Index   = 0;
	uint16  l_u16CurFramI = 0;
	int32   l_n32HeightThres = 1000;  //高度阈值1000
	int32   l_n32HeightMinThres = 350; //高度阈值
	int32   l_n32WideThres   = 150;   //宽度阈值
	int32   l_n32Max_X   = -99999;
	int32   l_n32Min_X   = 99999;
	int32   l_n32Min_Z   = 9999;

	if (pVehicle  ==  NULL || pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 1)
		return;
	l_u16CurFramI = pVehicle->u16FrameCnt - 1;

	if (pVehicle->CurRegion.xdata[0] > FRAME_BUFLEN-1 || //点数异常
		pVehicle->CurRegion.xdata[0] < 8) 
		return;

	//右边是扫描到轮胎的点
	if (pVehicle->CurRegion.zdata[pVehicle->CurRegion.xdata[0]] >= l_n32HeightThres) //最低高度大于阈值
		return;

	//进入一致性检测的帧数
	pVehicle->SideSet.SideCheckCnt++;

	for (l_u16Index = pVehicle->CurRegion.zdata[0]; l_u16Index > 0; l_u16Index--)
	{
		if ( pVehicle->CurRegion.zdata[l_u16Index] <= l_n32HeightMinThres)
			continue;
		else if (pVehicle->CurRegion.zdata[l_u16Index] < l_n32HeightThres  && 
			pVehicle->CurRegion.zdata[l_u16Index]> l_n32HeightMinThres) //高度1m以下
		{
			//寻找最大、最小X坐标值
			//20150422 修改
			l_n32Min_Z = l_n32Min_Z > pVehicle->CurRegion.zdata[l_u16Index] ? pVehicle->CurRegion.zdata[l_u16Index] : l_n32Min_Z;
			//20150422 完成
			l_u8PtCount++;
			l_n32Max_X = (l_n32Max_X < pVehicle->CurRegion.xdata[l_u16Index]) ? pVehicle->CurRegion.xdata[l_u16Index] : l_n32Max_X;
			l_n32Min_X = (l_n32Min_X > pVehicle->CurRegion.xdata[l_u16Index]) ? pVehicle->CurRegion.xdata[l_u16Index] : l_n32Min_X;
		}
		else
			break;
	}

	//20150422 修改
	if ((l_n32Max_X > l_n32Min_X)  &&  (l_n32Max_X - l_n32Min_X < l_n32WideThres)  &&  (l_u8PtCount >= 3)   &&  (l_n32Min_Z < 550))//20150422 完成
	{
		pVehicle->SideSet.SideFitCnt++;
		pVehicle->SideSet.SideFlag[l_u16CurFramI]=pVehicle->SideSet.SideFlag[l_u16CurFramI]|1;  //侧面具有一致性
	}
}

/************************************************************************/
/*CheckSide 侧面检测 检测1米以下侧面的一致性，用来区分客货				*/
/*输入：pVehicle 车辆信息结构体											*/
/*返回值： 1表示侧面一致，0其它											*/
/************************************************************************/
void SideUinformCheck(VehicleSet *pVehicle)
{
//	uint8   l_u8RetValue  = 0;
	uint16  l_u16Index    = 0;
	uint16  l_u16SideTotal = 0;
	uint16  l_u16TmpCnt    = 0;
	uint16  l_u16MaxCnt   = 5;  //最大连续侧面标识非1的个数
	uint16  l_u16CheckCnt=0;

	if (pVehicle  ==  NULL || pVehicle->u16FrameCnt > FRAME_MAXCNT)
		return ;


	if ((pVehicle->EdgeSet.u16RearHeight<1000 && pVehicle->EdgeSet.u16RearHeight > 0) 
		|| pVehicle->u16FrameCnt >= FRAME_MAXCNT) // 20150422 增加帧数达到最大帧数不判断一致性
	{
		return ;
	}

	if (pVehicle->u16FrameCnt < 5) //最少5帧
		return ;

	for (l_u16Index = pVehicle->AxleSet.Axle[0].AxleOriEndI; l_u16Index < pVehicle->u16FrameCnt-1; l_u16Index++)
	{
		if (pVehicle->EdgeSet.u16MaxHeight[l_u16Index] > 1000  &&  (pVehicle->SideSet.SideFlag[l_u16Index]&1)>0)
		{
			l_u16SideTotal += pVehicle->SideSet.SideFlag[l_u16Index];
			l_u16TmpCnt = 0;
		}
		else
		{
			l_u16TmpCnt++;
			if (l_u16TmpCnt>5)
			{
				l_u16TmpCnt=0;
				l_u16MaxCnt++;
			}
		}
		l_u16CheckCnt++;
	}

	if (l_u16MaxCnt <= l_u16SideTotal/8  &&   //最大连续非1的个数不大于3  
		l_u16SideTotal*5 >= l_u16CheckCnt*4)  //计算帧数的4/5
	{
		pVehicle->FeatureSet.IsSideUiform=pVehicle->FeatureSet.IsSideUiform|1;
	}
}

/************************************************************************/
/* IsJiZhuangXiang();  //判断取得的某帧是否是集装箱                     */
/* 输入pFrameInfo 垂直激光器车辆帧信息结构体							*/
/*返回值：0表示不是集装箱，1表示是集装箱                                */
/************************************************************************/
uint8  IsJiZhuangXiang(FramInfo *pFrameInfo)
{
	uint8   l_u8ReturnValue     = 0;   
	uint16  l_u16Index          = 0;
	int32   l_n32MinDistance    = 0;  //最小水平距离
	int32   l_n32MaxDistance    = 0;  //最大水平距离
	int32   l_n32HeightThres    = 0;  //用于计算的高度阈值

	if (pFrameInfo  ==  NULL || pFrameInfo->u16StaPos >= pFrameInfo->u16EndPos ||
		pFrameInfo->u16StaPos >= POINT_SUM || pFrameInfo->u16EndPos >= POINT_SUM) //数据异常
		return 0;

	if (pFrameInfo->u16MaxHt < MAXHEIGHT_JIZHUANGXIANG)  //该帧高度小于集装箱最低高度 
	{
		return 0;
	}

	if (pFrameInfo->u16MaxHt > MAXHEIGHT_JIZHUANGXIANG + 300) //增加动态改变高度阈值
		l_n32HeightThres = pFrameInfo->u16MaxHt - 300;
	else
		l_n32HeightThres = MAXHEIGHT_JIZHUANGXIANG;

	//大点方向是轮轴(扫描到车辆低的部分)	

	//先从最低处找到最小高度阈值处
	for (l_u16Index = pFrameInfo->u16EndPos; l_u16Index > pFrameInfo->u16StaPos; l_u16Index--)
	{
		if (g_FiltZ[l_u16Index] >= MINHEIGHT_JIZHUANGXIANG)  //不小于阈值，直接跳出
			break;
	}

	//找最小高度阈值和最大高度阈值之间的最大水平距离差
	l_n32MinDistance = g_FiltX[l_u16Index];
	for (; l_u16Index > pFrameInfo->u16StaPos; l_u16Index--)
	{
		if (g_FiltZ[l_u16Index] < l_n32HeightThres)
		{
			if (l_n32MinDistance > g_FiltX[l_u16Index])
			{
				l_n32MinDistance = g_FiltX[l_u16Index];  //取最小距离
			}	
			if (l_n32MaxDistance < g_FiltX[l_u16Index])
			{
				l_n32MaxDistance = g_FiltX[l_u16Index];  //取最大距离
			}
		}
		else
			break;
	}

	if (l_n32MaxDistance > 0  &&  l_n32MinDistance > 0  &&  l_n32MaxDistance >= l_n32MinDistance) //计算最大水平距离差值
	{
		if (l_n32MaxDistance - l_n32MinDistance < MAXDIFFER_JIZHUANGXIANG) //小于最大宽度阈值差值
		{
			l_u8ReturnValue = 1;
		}
	}

	return l_u8ReturnValue;
}

/************************************************************************/
/* 货箱最终判定                                                         */
/************************************************************************/
void ContainerCheck(VehicleSet *pVehicle)
{

//	uint8    l_u8RetValue = 0;
//	uint8    l_u8CarWindowFlag = 0; //车窗标识
//	uint8    l_u8Count = 0;
	uint16   l_u16Index   = 0;
//	uint16   l_u16TmpIndex = 0;
//	uint16   l_u16CurHeight = 0;

	uint16  StaI=0;
	uint16  EndI=0;
	uint16  CheckCnt=0;
	uint32  ContainerHt=0;
	uint32  ContainerCnt=0;
	uint8  TmpI=0;
	uint32  HtHist[3][2]={0};//3段，分别计算均值 确定货箱高度 1--累计高度值;0--点数;
	

	if (pVehicle->EdgeSet.u16VehHeight<2000||pVehicle == NULL)
	{
		return;
	}


	if (pVehicle->AxleSet.AxleSum>1
		 && pVehicle->u16FrameCnt/2-pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI>0)
	{
		StaI=(pVehicle->AxleSet.Axle[0].AxleOriEndI+pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI)/2;
		EndI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI;
	}
	else
	{
		StaI=pVehicle->u16FrameCnt/2;
		EndI=pVehicle->u16FrameCnt-1;
	}

	if (EndI-StaI<1||StaI>EndI)
	{
		return;
	}


	//计算货箱的高度 采用统计规律实现
	for (l_u16Index = StaI; l_u16Index < EndI; l_u16Index++) 
	{
		if (pVehicle->EdgeSet.u16VehHeight < pVehicle->EdgeSet.u16MaxHeight[l_u16Index] + 150)
		{
			HtHist[0][0]++;
			HtHist[0][1]+=pVehicle->EdgeSet.u16MaxHeight[l_u16Index];
		}
		else if (pVehicle->EdgeSet.u16VehHeight < pVehicle->EdgeSet.u16MaxHeight[l_u16Index] + 300)
		{
			HtHist[1][0]++;
			HtHist[1][1]+=pVehicle->EdgeSet.u16MaxHeight[l_u16Index];
		}
		else if (pVehicle->EdgeSet.u16VehHeight < pVehicle->EdgeSet.u16MaxHeight[l_u16Index] + 450)
		{
			HtHist[2][0]++;
			HtHist[2][1]+=pVehicle->EdgeSet.u16MaxHeight[l_u16Index];
		}
	}

	for (TmpI=0;TmpI<3;TmpI++)
	{
		if (HtHist[TmpI][0]*10/(EndI-StaI)>6)
		{
			ContainerHt=HtHist[TmpI][1];
			ContainerCnt=HtHist[TmpI][0];
			break;
		}

		if (HtHist[TmpI][0]*10/(EndI-StaI)>1)
		{
			ContainerHt+=HtHist[TmpI][1];
			ContainerCnt+=HtHist[TmpI][0];
		}
	}

	if (ContainerCnt)
	{
		ContainerHt=ContainerHt/ContainerCnt;
	}
	else 
	{
		ContainerHt=pVehicle->EdgeSet.u16VehHeight;
	}



	for (l_u16Index = StaI; l_u16Index < EndI; l_u16Index++) //检测后一半帧数	
	{
		if (ContainerHt < pVehicle->EdgeSet.u16MaxHeight[l_u16Index] + 150) //高度限定
		{
			CheckCnt++;
			if (pVehicle->ContainerSet.ContainerFlag[l_u16Index]>0)
			{
				pVehicle->ContainerSet.ContainerCnt++;
			}
			
		}
	}

	//20170719 修改求占比的时，为满足高度15cm范围内的帧的占比
	if (pVehicle->ContainerSet.ContainerCnt*3>CheckCnt*2)
	{
		pVehicle->FeatureSet.IsContainer=pVehicle->FeatureSet.IsContainer|1;
	}
}

/************************************************************************/
/*函数名 ContainerCal 货箱检查函数	                                    */
/*输入： *pVehicle 车辆信息结构体					                    */
/*输出：无                                                              */
/************************************************************************/
void ContainerCal(VehicleSet *pVehicle)
{
//	uint8   l_u8Ret       = 0;
	uint8   l_u8Flag      = 0;
	uint16  l_u16Index    = 0;
	uint16  l_u16CurFramI = 0;
	int32   l_n32Max_X    = -99999;
	int32   l_n32Min_X    = 99999;
//	int32   l_n32MaxHeight_Thres = 0;     //最大高度阈值
	int32   l_n32MinHeight_Thres = 1200;  //最小高度阈值
	int32   l_n32MaxWide_Thres   = 100;

	if (pVehicle  ==  NULL || pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 1||
		pVehicle->CurRegion.zdata[0] > FRAME_BUFLEN - 1 ||
		pVehicle->CurRegion.zdata[0] < 6)
		return ;

	l_u16CurFramI = pVehicle->u16FrameCnt-1;

	//20170919 修改当前帧高>2000
	if (pVehicle->AxleSet.AxleSum<1||pVehicle->EdgeSet.u16MaxHeight[l_u16CurFramI]<1700)
	{
		return;
	}

	

	for (l_u16Index = pVehicle->CurRegion.zdata[0]; l_u16Index > 5; l_u16Index--) //最小高度阈值以上所有点
	{
		if (pVehicle->CurRegion.zdata[l_u16Index] <= l_n32MinHeight_Thres  &&  (!l_u8Flag))
		{
			continue;
		}
		else  //在最大高度阈值以下
		{
			//寻找最大、最小X坐标值
			l_u8Flag = 1;
			l_n32Max_X = (l_n32Max_X < pVehicle->CurRegion.xdata[l_u16Index]) ? pVehicle->CurRegion.xdata[l_u16Index] : l_n32Max_X;
			l_n32Min_X = (l_n32Min_X > pVehicle->CurRegion.zdata[l_u16Index]) ? pVehicle->CurRegion.xdata[l_u16Index] : l_n32Min_X;
		}
	}
	
	
	//左侧边界x极左极右差值处于100mm以内 及 侧面是平坦的
	if (l_n32Max_X - l_n32Min_X < l_n32MaxWide_Thres  &&  l_n32Min_X > Smp.StartX)
	{
		//l_u8Ret = 1;
		if (pVehicle->ContainerSet.ContainerCnt == 0)
		{
			pVehicle->ContainerSet.ContainerStaI=l_u16CurFramI;
		}
		pVehicle->ContainerSet.ContainerEndI=l_u16CurFramI;
		pVehicle->ContainerSet.ContainerFlag[l_u16CurFramI]=1;
	}
	else
	{
		//l_u8Ret = 0;
	}
} 


/************************************************************************/
/*MinHeightCheck //检测最低高度 主要针对小车 检测第1个轴后的帧数        */
/*输入:车辆信息结构体 pVehicle											*/
/*输出：                                                                */
/************************************************************************/
void Axle1MinHeightCal(VehicleSet *pVehicle)
{
	uint16   l_u16PtIndex = 0;
	int32    l_n32DiffThres = 100;
	int32    l_n32HeightThres  = 500;
	int32    l_n32Height = 0;
	uint16   l_u16HeightIndex = 0;

	if (pVehicle  ==  NULL || pVehicle->CurFrmInfo.u16StaPos >= pVehicle->CurFrmInfo.u16EndPos 
		||pVehicle->AxleSet.AxleSum<1||pVehicle->AxleSet.AxleSum>1) //合法性判断
	{
		return;
	}

	l_u16PtIndex = pVehicle->CurFrmInfo.u16EndPos;
	//大点方向扫描到车辆下方的点数

	if (g_FiltZ[l_u16PtIndex-1] - g_FiltZ[l_u16PtIndex] > l_n32DiffThres || //最低点之间变化较大
		(g_FiltZ[l_u16PtIndex-2] - g_FiltZ[l_u16PtIndex-1])*2 > g_FiltZ[l_u16PtIndex-1] - g_FiltZ[l_u16PtIndex])//最低3个点差值比较
	{
		l_n32Height = g_FiltZ[l_u16PtIndex-1];
		l_u16HeightIndex = l_u16PtIndex-1;
	}
	else
	{
		l_n32Height = g_FiltZ[l_u16PtIndex];
		l_u16HeightIndex = l_u16PtIndex;
	}

	if (g_FiltZ[l_u16HeightIndex]  ==  g_FiltZ[l_u16HeightIndex-1]  && 
		g_FiltZ[l_u16HeightIndex-1]  ==  g_FiltZ[l_u16HeightIndex-2]) //认为打飞，直接返回
		return;

	if (l_n32Height > l_n32HeightThres)
	{
		pVehicle->EdgeSet.u16A1MinHtCnt[0]++;
		pVehicle->EdgeSet.u16A1MinHeight[0] += l_n32Height;
	}
	else
	{
		pVehicle->EdgeSet.u16A1MinHtCnt[1]++;
		pVehicle->EdgeSet.u16A1MinHeight[1] += l_n32Height;
	}
}

/************************************************************************/
/* 计算第一轴的相应车高作为车头高                                       */
/************************************************************************/
void VehHeadHtCheck(VehicleSet *pVehicle)
{
//	uint8    l_u8Flag       = 0;
	uint8    l_u8Count      = 0;
//	uint8    l_u8DouPengFlag = 0;
	uint16   l_u16RetHeight = 0;
	uint16   l_u16Index     = 0;
//	uint16   l_u16Index1    = 0;
//	uint16   l_u16PointCnt  = 0;  //用于计算车头高的点数
	uint16   l_uFrameCnt    = 0;
//	uint16   l_u16MaxHeight = 0;
//	uint16   l_u16MinHeight = 0; 
//	uint16   l_u16PreHeight = 0; //前一帧的高度值
	uint32   l_u32Height    = 0;

	if (pVehicle  ==  NULL || pVehicle->AxleSet.AxleSum<1)
		return ;


	//20150423 修改 直接采用车头前3帧平均高为车头高
	l_uFrameCnt=MIN(pVehicle->AxleSet.Axle[0].AxleOriEndI-pVehicle->AxleSet.Axle[0].AxleOriStaI+1,3);

	if (pVehicle->AxleSet.AxleSum > 0)
	{
		for (l_u16Index = 0; l_u16Index < l_uFrameCnt; l_u16Index++)
		{
			if (pVehicle->EdgeSet.u16MaxHeight[pVehicle->AxleSet.Axle[0].AxleOriStaI+l_u16Index] > 0)
			{
				l_u32Height += pVehicle->EdgeSet.u16MaxHeight[pVehicle->AxleSet.Axle[0].AxleOriStaI+l_u16Index];
				l_u8Count++;
			}
		}

		if (l_u8Count > 0)
			l_u16RetHeight = l_u32Height/l_u8Count;

	}

	pVehicle->EdgeSet.u16HeadHeight=l_u16RetHeight;
}

/************************************************************************/
/*第二轴相对应的车高                                                    */
/************************************************************************/
void VehRearHtCheck(VehicleSet *pVehicle)
{
	uint8  l_u8Index = 0;
//	uint16 l_u16RetHeight = 0;
	int32  l_n32Height = 0;
	uint32 l_u32FrCnt=0;

	if (pVehicle  ==  NULL||pVehicle->AxleSet.AxleSum<2)
		return ;

	//第二个轴 帧数与对应高度
	if (pVehicle->AxleSet.Axle[1].AxleOriEndI-pVehicle->AxleSet.Axle[1].AxleOriStaI>0)
	{
		l_n32Height=0;

		pVehicle->EdgeSet.u16Axle2FrCnt=pVehicle->AxleSet.Axle[1].AxleOriEndI-pVehicle->AxleSet.Axle[1].AxleOriStaI+1;

		l_u32FrCnt=MIN(pVehicle->EdgeSet.u16Axle2FrCnt,32);

		for (l_u8Index =0 ; l_u8Index < l_u32FrCnt; l_u8Index++)
			l_n32Height += pVehicle->EdgeSet.u16MaxHeight[pVehicle->AxleSet.Axle[1].AxleOriStaI+l_u8Index];

		if (l_u32FrCnt)
		{
			pVehicle->EdgeSet.u16Axle2Ht = l_n32Height/l_u32FrCnt;
		}
		else
		{
			pVehicle->EdgeSet.u16Axle2Ht = 0;
		}
	}

	
	

	//第三个轴 轴数与对应高度
	if (pVehicle->AxleSet.Axle[2].AxleOriEndI-pVehicle->AxleSet.Axle[2].AxleOriStaI>0)
	{
		l_n32Height=0;

		pVehicle->EdgeSet.u16Axle3FrCnt=pVehicle->AxleSet.Axle[2].AxleOriEndI-pVehicle->AxleSet.Axle[2].AxleOriStaI+1;

		l_u32FrCnt=MIN(pVehicle->EdgeSet.u16Axle3FrCnt,32);

		for (l_u8Index =0 ; l_u8Index < l_u32FrCnt; l_u8Index++)
			l_n32Height += pVehicle->EdgeSet.u16MaxHeight[pVehicle->AxleSet.Axle[2].AxleOriStaI+l_u8Index];

		if (l_u32FrCnt)
		{
			pVehicle->EdgeSet.u16Axle3Ht = l_n32Height/l_u32FrCnt;
		}
		else
		{
			pVehicle->EdgeSet.u16Axle3Ht = 0;
		}
	}
	

	if (pVehicle->AxleSet.AxleSum == TWO_AXLE)
	{
		pVehicle->EdgeSet.u16RearHeight=pVehicle->EdgeSet.u16Axle2Ht;
	}
	else if (pVehicle->AxleSet.AxleSum == THREE_AXLE)
	{
		pVehicle->EdgeSet.u16RearHeight=pVehicle->EdgeSet.u16Axle3Ht;
	}
}

/************************************************************************/
/*计算车辆高度                                                          */
/************************************************************************/
void VehHeightCheck(VehicleSet *pVehicle)
{
//	uint16    l_u16ReturnHeight = 0;
//	uint16    l_u16Index         = 0;
//	uint16    l_u16MaxHeight       = 0;
//	uint16    l_u16SecHeight       = 0;
//	uint16    l_u16ThdHeight       = 0;
//	uint16    l_u16MaxHtIndex=0;
//	uint16    l_u16SecHtIndex=0;
//	uint16    l_u16ThdHtIndex=0;

	if ( pVehicle  ==  NULL || pVehicle->u16FrameCnt > FRAME_MAXCNT)
		return ;

	CalcVehHeight(pVehicle->EdgeSet.u16MaxHeight,pVehicle->u16FrameCnt,&pVehicle->EdgeSet.u16VehHeight,&pVehicle->EdgeSet.u16VehHtIndex);
	
}

/************************************************************************/
/* 高度平整度检测                                                       */
/* 0默认值，1表示检测高度平整 2表示车尾与车高有较大高度差               */
/************************************************************************/
void HtRoughnessCheck(VehicleSet *pVehicle)
{
	uint8    l_u8RetValue = 0;
	uint8    l_u8HeightFlag = 0;  //表示车头第1个接近车高的标识
	uint16   l_u16Count   = 0;
	uint16   l_u16Index   = 0;
	// 	uint16   l_u16MinHeight = 0;
	// 	uint16   l_u16MaxHeight = 0;
	uint32   l_u32HeightSum = 0;

	if (pVehicle  ==  NULL)
		return ;

	if (pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 8) //小于10帧或帧数异常
		return ;

	for (l_u16Index = 0; l_u16Index < pVehicle->u16FrameCnt/3; l_u16Index++) //寻找前1/3帧数中第1个接近高度值的索引
	{
		if (pVehicle->EdgeSet.u16VehHeight < pVehicle->EdgeSet.u16MaxHeight[l_u16Index] + 150)
		{
			l_u8HeightFlag = 1;
			break;
		}
	}

	if (l_u8HeightFlag) //检测车辆高度平整度 指一致平整
	{
		for (; l_u16Index < pVehicle->u16FrameCnt*4/5; l_u16Index++)
		{
			if (l_u16Index > pVehicle->u16FrameCnt - 8)
				break;
			else if (pVehicle->EdgeSet.u16VehHeight < pVehicle->EdgeSet.u16MaxHeight[l_u16Index] + 150) //具有一致平整性
				continue;
			else
			{
				l_u8HeightFlag = 2;  //不具有一致平整性
				break;
			}

		}

		if (l_u8HeightFlag < 2)
			l_u8RetValue = 1;   //高度平整
	}

	if (2  ==  l_u8HeightFlag)
	{
		//寻找后1/3帧数的平均高度值
		for (l_u16Index = pVehicle->u16FrameCnt*2/3; l_u16Index < pVehicle->u16FrameCnt-8; l_u16Index++)	
		{
			//检测车尾高度
			l_u32HeightSum += pVehicle->EdgeSet.u16MaxHeight[l_u16Index];
			l_u16Count++;
		}	

		if (l_u16Count > 4) //至少需要5帧
		{
			l_u32HeightSum /= l_u16Count;

			if (pVehicle->EdgeSet.u16VehHeight > l_u32HeightSum + 500) //有高度差
			{
				l_u8RetValue = 2; //表示有高度差
			}
		}	
	}

	if (l_u8RetValue == 1)
	{
		pVehicle->FeatureSet.IsHtRough=1;
	}
	else
	{
		pVehicle->FeatureSet.IsHtRough=0;
	}
}


/************************************************************************/
/* 每帧高度检测与调整，主要防止丢弃点导致高度不准                       */
/************************************************************************/
void VehHeightModify(VehicleSet *pVehicle)
{
	uint16  l_u16Index  = 0;
	uint16  l_u16Height = 100;
	uint16  l_u16Count  = 0;
	uint16  l_u16IndexTmp = 0;
	uint16  l_u16StartIndex = 0;
	if (pVehicle  ==  NULL)
		return;

	if (pVehicle->u16FrameCnt > FRAME_MAXCNT)
		return;

	for (l_u16Index = 1; l_u16Index < pVehicle->u16FrameCnt; l_u16Index++)
	{
		if (pVehicle->EdgeSet.u16MaxHeight[l_u16Index] != pVehicle->EdgeSet.u16SecHeight[l_u16Index]) //存储的高度不等
		{
			if (pVehicle->EdgeSet.u16MaxHeight[l_u16Index]>pVehicle->EdgeSet.u16MaxHeight[l_u16Index-1]+1000
				 && ABS(pVehicle->EdgeSet.u16SecHeight[l_u16Index]-pVehicle->EdgeSet.u16MaxHeight[l_u16Index-1])<500)
			{
				pVehicle->EdgeSet.u16MaxHeight[l_u16Index]=pVehicle->EdgeSet.u16SecHeight[l_u16Index];//当前帧第二高与前一帧最大高接近 该帧高度异常 取第二高
			}
			else if (pVehicle->EdgeSet.u16MaxHeight[l_u16Index-1] > pVehicle->EdgeSet.u16MaxHeight[l_u16Index] + l_u16Height)
			{//前一帧高度比当前高度高
				l_u16Count++;
				l_u16StartIndex = l_u16StartIndex < 1 ? l_u16Index : l_u16StartIndex; //记录开始索引
			}
			else if (l_u16Count > 0)
			{
				l_u16Count++;
			}
		}
		else
		{
			//高度相等
			if (l_u16Count > 0  &&  pVehicle->EdgeSet.u16MaxHeight[l_u16Index] > pVehicle->EdgeSet.u16MaxHeight[l_u16Index-1] + l_u16Height) //说明有凹陷处存在
			{//凹陷明显
				for(l_u16IndexTmp = l_u16StartIndex; l_u16IndexTmp < l_u16Index; l_u16IndexTmp++) //修改高度值
				{
					pVehicle->EdgeSet.u16MaxHeight[l_u16IndexTmp] = pVehicle->EdgeSet.u16SecHeight[l_u16IndexTmp];
					pVehicle->EdgeSet.u8HeightAdjFlag[l_u16IndexTmp] = 1; //修改高度标识
				}
			}
			else if (l_u16Count > 0)
			{//找到的开始地方，结束地方前后帧高度没有显著差异
				for (l_u16IndexTmp = l_u16Index; l_u16IndexTmp >= l_u16StartIndex; l_u16IndexTmp--)
				{
					if (pVehicle->EdgeSet.u16MaxHeight[l_u16IndexTmp] <= pVehicle->EdgeSet.u16MaxHeight[l_u16IndexTmp-1] + l_u16Height)
						continue;
					else
						break;
				}
				for(; l_u16IndexTmp >= l_u16StartIndex; l_u16IndexTmp--)
				{
					pVehicle->EdgeSet.u16MaxHeight[l_u16IndexTmp] = pVehicle->EdgeSet.u16SecHeight[l_u16IndexTmp];
					pVehicle->EdgeSet.u8HeightAdjFlag[l_u16IndexTmp] = 1; //修改高度标识
				}
			}
			l_u16Count = 0;
			l_u16StartIndex = 0;
		}
	}


	//单独检查第一帧 20170714
	if (pVehicle->EdgeSet.u16MaxHeight[0] != pVehicle->EdgeSet.u16SecHeight[0]
	 && pVehicle->EdgeSet.u16MaxHeight[1]+1000<pVehicle->EdgeSet.u16MaxHeight[0])
	{
		pVehicle->EdgeSet.u16MaxHeight[0]=pVehicle->EdgeSet.u16MaxHeight[1];
		pVehicle->EdgeSet.u8HeightAdjFlag[0] = 1; //修改高度标识
	}


}

/************************************************************************/
/*轮轴调整		                                                        */
/************************************************************************/
void AxleAdjustMent(VehicleSet *pVehicle)
{
	//超6轴 出6轴
	if (pVehicle->AxleSet.AxleSum> SIX_AXLE)
		pVehicle->AxleSet.AxleSum = SIX_AXLE;

	//车高<2200的纠偏0轴和1轴的车辆
	if (pVehicle->EdgeSet.u16VehHeight< 2200)
	{
		if (pVehicle->AxleSet.AxleSum < ONE_AXLE)
		{
			pVehicle->AxleSet.AxleSum = TWO_AXLE;  //2轴
			pVehicle->AxleSet.Axle[0].AxleType=1;
			pVehicle->AxleSet.Axle[1].AxleType=1;
		}
		else if (ONE_AXLE  ==  pVehicle->AxleSet.AxleSum) //扫描到1个轴
		{
			pVehicle->AxleSet.AxleSum = TWO_AXLE;  //2轴
			pVehicle->AxleSet.Axle[0].AxleType=1;
			pVehicle->AxleSet.Axle[1].AxleType=1;
		}
		else
		{
			pVehicle->AxleSet.AxleSum = TWO_AXLE;
		}	
	}
	else if (pVehicle->EdgeSet.u16VehHeight<3200)
	{
		if (pVehicle->AxleSet.AxleSum<=ONE_AXLE)
		{
			pVehicle->AxleSet.AxleSum = TWO_AXLE;  //2轴
			pVehicle->AxleSet.Axle[0].AxleType=1;
			pVehicle->AxleSet.Axle[1].AxleType=1;
		}
	}
}

/************************************************************************/
/* 高度变化率聚类方法                                                   */
/************************************************************************/
void HtClusterCheck(VehicleSet *pVehicle)
{
	int16 i=0;
	int16 j=0;
	int16 iTmpI=0;
	uint16 ThreshPt=5;
	uint16 u16Index=0;
	uint16 PtSum=0;//满足阈值点数
	uint16 uFilterH[MAX_CUT]={0};
	int32  iChangeH[MAX_CUT]={0};
	int32  iDifferH[MAX_CUT]={0};
	int32  iSignH[MAX_CUT]={0};
	uint16 u16FramCnt=pVehicle->u16FrameCnt;
	uint8 u8Offset=0;
	int16 iRealCnt=0;
	uint8 u8Index=0;


	if (u16FramCnt>MAX_CUT||pVehicle->AxleSet.AxleSum>2)
	{
		pVehicle->Cluster.IsOk=0;
		return;
	}


	ThreshPt=MIN(u16FramCnt/30+1,5);
	
	//对中轴线进行5邻域均值滤波  及 搜寻极值点位置
	for (u16Index=0;u16Index<u16FramCnt;u16Index++)
	{
		//5邻域均值滤波
		if (u16Index+1>ThreshPt/2 && u16Index<u16FramCnt-ThreshPt/2)
		{
			uFilterH[PtSum]=MeanFilterU(&pVehicle->EdgeSet.u16MaxHeight[u16Index-ThreshPt/2],ThreshPt);
			PtSum++;
		}

		//极大值
		if (pVehicle->EdgeSet.u16MaxHeight[u16Index]>pVehicle->Cluster.Hmax)
		{
			pVehicle->Cluster.Hmax=pVehicle->EdgeSet.u16MaxHeight[u16Index];
			pVehicle->Cluster.HmaxIdx=u16Index;
		}

		//极小值
		if (pVehicle->Cluster.Hmin == 0 && pVehicle->EdgeSet.u16MaxHeight[u16Index]>0)
		{
			pVehicle->Cluster.Hmin=pVehicle->EdgeSet.u16MaxHeight[u16Index];
		}
		else if (pVehicle->EdgeSet.u16MaxHeight[u16Index]<pVehicle->Cluster.Hmin)
		{
			pVehicle->Cluster.Hmin=pVehicle->EdgeSet.u16MaxHeight[u16Index];
		}

	}
	u8Offset=u8Offset+ThreshPt/2;


	//求高度密度曲线
	for (u16Index=0;u16Index<PtSum;u16Index++)
	{
		if (u16Index == 0)
		{
			iChangeH[u16Index]=uFilterH[u16Index]*100;
		}
		else
		{
			iChangeH[u16Index]=(iChangeH[u16Index-1]*u16Index/100+uFilterH[u16Index])*100/(u16Index+1);
		}
	}
	
	//求高度密度曲线差分曲线
	for (u16Index=1;u16Index<PtSum;u16Index++)
	{
		iDifferH[u16Index-1]=(iChangeH[u16Index]-iChangeH[u16Index-1]);
	}
	u8Offset=u8Offset+1;

	//标记差分曲线变化一致性段
	IndicateTrends(iDifferH,iSignH,PtSum-1);
	
	//按顺序依次聚类 分成几个部分
	iRealCnt=u16FramCnt-u8Offset-ThreshPt/2;
	for (i=u8Offset;i<iRealCnt;i++)
	{
		u8Index=pVehicle->Cluster.ClNum;
		pVehicle->Cluster.CI[u8Index].Sta=i;
		pVehicle->Cluster.CI[u8Index].End=i;
		pVehicle->Cluster.CI[u8Index].Num=1;
		pVehicle->Cluster.CI[u8Index].MinZ=uFilterH[i];
		pVehicle->Cluster.CI[u8Index].MinZI=i;
		pVehicle->Cluster.CI[u8Index].MaxZ=uFilterH[i];
		pVehicle->Cluster.CI[u8Index].MaxZI=i;
		pVehicle->Cluster.CI[u8Index].SumH+=uFilterH[i];
		pVehicle->Cluster.CI[u8Index].MaxX=pVehicle->EdgeSet.u16MinXEdge[i];
		pVehicle->Cluster.CI[u8Index].MinX=pVehicle->EdgeSet.u16MinXEdge[i];
		pVehicle->Cluster.CI[u8Index].MinXI=i;
		pVehicle->Cluster.CI[u8Index].MaxXI=i;
		pVehicle->Cluster.CI[u8Index].SumX+=pVehicle->EdgeSet.u16MinXEdge[i];
		for (j=i+1;j<iRealCnt+1;j++)//多加一位 填充0 确保最后类计入统计
		{
			if ((iSignH[j-u8Offset] == iSignH[i-u8Offset]||iSignH[j+1-u8Offset] == iSignH[i-u8Offset])
			 && ((iDifferH[i]*iDifferH[j]>0||iDifferH[i]*iDifferH[j+1]>0)))
			{
				pVehicle->Cluster.CI[u8Index].End=j;
				pVehicle->Cluster.CI[u8Index].Num++;
				pVehicle->Cluster.CI[u8Index].SumH+=uFilterH[j];
				pVehicle->Cluster.CI[u8Index].SumX+=pVehicle->EdgeSet.u16MinXEdge[j];
				if (pVehicle->Cluster.CI[u8Index].MaxZ<uFilterH[j])
				{
					pVehicle->Cluster.CI[u8Index].MaxZ=uFilterH[j];
					pVehicle->Cluster.CI[u8Index].MaxZI=j;
				}

				if (pVehicle->Cluster.CI[u8Index].MinZ>uFilterH[j])
				{
					pVehicle->Cluster.CI[u8Index].MinZ=uFilterH[j];
					pVehicle->Cluster.CI[u8Index].MinZI=j;
				}

				if (pVehicle->Cluster.CI[u8Index].MaxX<pVehicle->EdgeSet.u16MinXEdge[j])
				{
					pVehicle->Cluster.CI[u8Index].MaxX=pVehicle->EdgeSet.u16MinXEdge[j];
					pVehicle->Cluster.CI[u8Index].MaxXI=j;
				}

				if (pVehicle->Cluster.CI[u8Index].MinX>pVehicle->EdgeSet.u16MinXEdge[j])
				{
					pVehicle->Cluster.CI[u8Index].MinX=pVehicle->EdgeSet.u16MinXEdge[j];
					pVehicle->Cluster.CI[u8Index].MinXI=j;
				}
			}
			else
			{
				i=j-1;
				pVehicle->Cluster.CI[u8Index].AvgH=pVehicle->Cluster.CI[u8Index].SumH/pVehicle->Cluster.CI[u8Index].Num;
				pVehicle->Cluster.CI[u8Index].AvgX=pVehicle->Cluster.CI[u8Index].SumX/pVehicle->Cluster.CI[u8Index].Num;
				pVehicle->Cluster.ClNum++;
				break;
			}
		}
		//车身类太多 则为起伏变化太大 实际上为货车 但作为异常扔掉 
		if (pVehicle->Cluster.ClNum>19)
		{
			pVehicle->Cluster.IsOk=0;
			return;
		}
	}

	//分析每个聚类段的线型
	for (i=0;i<pVehicle->Cluster.ClNum;i++)
	{
		if (pVehicle->Cluster.CI[i].Num)
		{
			ShapeAnalysisEx(uFilterH,&pVehicle->Cluster.CI[i]);
		}
	}

	//聚类段的合并
	iTmpI=0;
	for (i=pVehicle->Cluster.ClNum;i>0;i--)
	{
		if (pVehicle->Cluster.CI[i].Style == pVehicle->Cluster.CI[i-1].Style && 
			(ABS(pVehicle->Cluster.CI[i].AvgH-pVehicle->Cluster.CI[i-1].AvgH)<50)
			||(pVehicle->Cluster.CI[i].MaxZ>pVehicle->Cluster.CI[i-1].MaxZ && ABS(pVehicle->Cluster.CI[i].MinZ-pVehicle->Cluster.CI[i-1].MinZ)<100))

		{
			if (pVehicle->Cluster.CI[i].MaxZ>pVehicle->Cluster.CI[i-1].MaxZ)
			{
				pVehicle->Cluster.CI[i-1].MaxZ=pVehicle->Cluster.CI[i].MaxZ;
				pVehicle->Cluster.CI[i-1].MaxZI=pVehicle->Cluster.CI[i].MaxZI;
			}

			if (pVehicle->Cluster.CI[i].MinZ<pVehicle->Cluster.CI[i-1].MinZ)
			{
				pVehicle->Cluster.CI[i-1].MinZ=pVehicle->Cluster.CI[i].MinZ;
				pVehicle->Cluster.CI[i-1].MinZI=pVehicle->Cluster.CI[i].MinZI;
			}

			if (pVehicle->Cluster.CI[i].MaxX>pVehicle->Cluster.CI[i-1].MaxX)
			{
				pVehicle->Cluster.CI[i-1].MaxX=pVehicle->Cluster.CI[i].MaxX;
				pVehicle->Cluster.CI[i-1].MaxXI=pVehicle->Cluster.CI[i].MaxXI;
			}

			if (pVehicle->Cluster.CI[i].MinX<pVehicle->Cluster.CI[i-1].MinX)
			{
				pVehicle->Cluster.CI[i-1].MinX=pVehicle->Cluster.CI[i].MinX;
				pVehicle->Cluster.CI[i-1].MinXI=pVehicle->Cluster.CI[i].MinXI;
			}

			pVehicle->Cluster.CI[i-1].End=pVehicle->Cluster.CI[i].End;
			pVehicle->Cluster.CI[i-1].Num=pVehicle->Cluster.CI[i-1].Num+pVehicle->Cluster.CI[i].Num;
			pVehicle->Cluster.CI[i-1].SumH=pVehicle->Cluster.CI[i-1].SumH+pVehicle->Cluster.CI[i].SumH;
			pVehicle->Cluster.CI[i-1].AvgH=pVehicle->Cluster.CI[i-1].SumH/pVehicle->Cluster.CI[i-1].Num;
			pVehicle->Cluster.CI[i-1].SumX=pVehicle->Cluster.CI[i-1].SumX+pVehicle->Cluster.CI[i].SumX;
			pVehicle->Cluster.CI[i-1].AvgX=pVehicle->Cluster.CI[i-1].SumX/pVehicle->Cluster.CI[i-1].Num;
			for (iTmpI=i+1;iTmpI<pVehicle->Cluster.ClNum;iTmpI++)
			{
				memcpy(&pVehicle->Cluster.CI[iTmpI-1],&pVehicle->Cluster.CI[iTmpI],sizeof(ClusterInfo));
			}
			memset(&pVehicle->Cluster.CI[pVehicle->Cluster.ClNum-1],0,sizeof(ClusterInfo));
			pVehicle->Cluster.ClNum--;
		}
	}

	//统计每个段内连续打飞的帧数
	for (i=0;i<pVehicle->Cluster.ClNum;i++)
	{
		for (j=pVehicle->Cluster.CI[i].Sta;j<pVehicle->Cluster.CI[i].End;j++)
		{
			if (pVehicle->FrShape.u8DaFeiCnt[j]+1>DAFEIMAXPT&&pVehicle->FrShape.u8DaFeiCnt[j+1]+1>DAFEIMAXPT)
			{
				pVehicle->Cluster.CI[i].DaFeiCnt++;
			}
		}
	}
	pVehicle->Cluster.IsOk=1;

}


/************************************************************************/
/* 聚类法车斗检测                                                       */
/************************************************************************/
void ClusterHopperCheck(VehicleSet *pVehicle)
{
	int i=0;
//	uint8 l_u8retValue=0;
	int HopperIdx=-1;//货斗类的索引
	uint16 l_u16FramCnt=0;

	if (pVehicle->u16FrameCnt<8||pVehicle->EdgeSet.u16VehHeight<1450)
	{
		return ;
	}

	l_u16FramCnt=pVehicle->u16FrameCnt;


	if (!pVehicle->Cluster.IsOk)
	{
		return ;
	}

	//过滤掉小客车
	if (pVehicle->Cluster.CI[0].Num>1 && pVehicle->Cluster.CI[0].AvgH<1100
		 && pVehicle->Cluster.CI[pVehicle->Cluster.ClNum-1].AvgH<1000)
	{
		return ;
	}

	//寻找车斗所在的类
	for (i=0;i<pVehicle->Cluster.ClNum;i++)
	{
		if (pVehicle->Cluster.CI[i].Sta>pVehicle->Cluster.HmaxIdx && pVehicle->Cluster.CI[i].End>l_u16FramCnt/2
			 &&  pVehicle->Cluster.CI[i].AvgH<2500  &&  pVehicle->Cluster.Hmax-pVehicle->Cluster.CI[i].AvgH>300)
		{
			if (pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleCastX-50>pVehicle->Cluster.CI[i].AvgX
				 && pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleCastX>pVehicle->Cluster.CI[i].MaxX)
			{
				if (pVehicle->Cluster.CI[i].Num>5 && pVehicle->Cluster.CI[i].Style == 2)
				{
					HopperIdx=i;
					break;
				}
				else if (pVehicle->Cluster.CI[i].Num>4 && pVehicle->Cluster.CI[i].Style == 1 && 
					(pVehicle->Cluster.CI[i].MaxZ+pVehicle->Cluster.CI[i].MinZ)/2>pVehicle->Cluster.CI[i].AvgH
					 && pVehicle->Cluster.CI[i].MaxZ-pVehicle->Cluster.CI[i].AvgH>400)
				{
					HopperIdx=i;
					break;
				}
			}

		}
	}
	if (HopperIdx!=-1)
	{
		//加入连续打飞帧的占比限制 防止小轿车尾部全部飞了 引起误判20170729
		if (pVehicle->Cluster.CI[HopperIdx].DaFeiCnt*100/pVehicle->Cluster.CI[HopperIdx].Num<50)
		{
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|2;
		}
	}
	
}

/************************************************************************/
/* 通过外形特性判断是否载货                                             */
/************************************************************************/
void FramStyleCheck(VehicleSet *pVehicle)
{
	int16 iIndex=0;
	int16 PtNum=0;
	int16 StaIndex=0;//第二轴后第一个一个大于1000的点序号
	int16 EndIndex=0;//第二轴后最后一个大于1000的点序号

	uint16 HopperTh=1000;//车斗高度下限


	//不是两轴 或车高＜1450 或 帧数＜8 直接跳出不判断
	if (pVehicle->AxleSet.AxleSum!=TWO_AXLE||
		pVehicle->EdgeSet.u16VehHeight>2200||pVehicle->u16FrameCnt<8
		&&pVehicle->EdgeSet.u16HeadHeight>2200)
	{
		return;
	}

	//轮轴之后的帧 线形分析

	//①寻找类似车斗的部分段
	PtNum=pVehicle->u16FrameCnt-pVehicle->AxleSet.Axle[1].AxleOriStaI+1;
	for (iIndex=0;iIndex<PtNum;iIndex++)
	{
		if (StaIndex == 0 && pVehicle->EdgeSet.u16MaxHeight[iIndex+pVehicle->AxleSet.Axle[1].AxleOriStaI]>HopperTh)
		{
			StaIndex=pVehicle->AxleSet.Axle[1].AxleOriStaI+iIndex;
		}

		if (EndIndex == 0 && pVehicle->EdgeSet.u16MaxHeight[pVehicle->u16FrameCnt-iIndex]>HopperTh)
		{
			EndIndex=pVehicle->u16FrameCnt-iIndex;
		}

		if (StaIndex!=0 && EndIndex!=0)
		{
			break;
		}
	}

	//左高右低，不打飞，两线高差大300
	for (iIndex=StaIndex;iIndex<EndIndex-5;iIndex++)
	{
		if (pVehicle->FrShape.u16LeftMaxHt[iIndex]>0&&pVehicle->FrShape.u8DaFeiCnt[iIndex]<DAFEIMAXPT
			&&ABS(pVehicle->FrShape.u16LeftMaxHt[iIndex]-pVehicle->EdgeSet.u16MaxHeight[iIndex])>300
			&&pVehicle->FrShape.u16FrShape[iIndex]==3)
		{
			//pVehicle->FeatureSet.IsCargo=pVehicle->FeatureSet.IsCargo|2;
		}
	}

}


/************************************************************************/
/* 轮轴定位车斗检测法                                                   */
/************************************************************************/
void AxleHopperCheckEx(VehicleSet *pVehicle,uint16 *pH)
{
	int16 iIndex=0;
	int16 PtNum=0;
	int16 StaIndex=0;//第二轴后第一个一个大于1000的点序号
	int16 EndIndex=0;//第二轴后最后一个大于1000的点序号

	uint16 HopperTh=1000;//车斗高度下限

	ClusterInfo HopperCI;//车斗聚类段
	ClusterInfo CrashCI;//防撞栏聚类段

	memset(&HopperCI,0,sizeof(HopperCI));
	memset(&CrashCI,0,sizeof(CrashCI));

	//不是两轴 或车高＜1450 或 帧数＜8 直接跳出不判断
	if (pVehicle->AxleSet.AxleSum!=TWO_AXLE||
		pVehicle->EdgeSet.u16VehHeight<1450||pVehicle->u16FrameCnt<8)
	{
		return;
	}

	//轮轴之后的帧 线形分析

	//①寻找类似车斗的部分段
	PtNum=pVehicle->u16FrameCnt-pVehicle->AxleSet.Axle[1].AxleOriStaI+1;
	for (iIndex=0;iIndex<PtNum;iIndex++)
	{
		if (StaIndex == 0 && pH[iIndex+pVehicle->AxleSet.Axle[1].AxleOriStaI]>HopperTh)
		{
			StaIndex=pVehicle->AxleSet.Axle[1].AxleOriStaI+iIndex;
		}

		if (EndIndex == 0 && pH[pVehicle->u16FrameCnt-iIndex]>HopperTh)
		{
			EndIndex=pVehicle->u16FrameCnt-iIndex;
		}

		if (StaIndex!=0 && EndIndex!=0)
		{
			break;
		}
	}

	//②评价该段的线形
	if (EndIndex-StaIndex>3)
	{
		HopperCI.Sta=StaIndex;
		HopperCI.End=EndIndex;
		HopperCI.Num=EndIndex-StaIndex+1;
		HopperCI.MaxZ=pH[StaIndex];
		HopperCI.MaxZI=StaIndex;
		HopperCI.MinZ=pH[StaIndex];
		HopperCI.MinZI=StaIndex;
		HopperCI.MaxX=pVehicle->EdgeSet.u16MinXEdge[StaIndex];
		HopperCI.MinX=pVehicle->EdgeSet.u16MinXEdge[StaIndex];
		HopperCI.MaxXI=StaIndex;
		HopperCI.MinXI=StaIndex;
		HopperCI.SumX+=pVehicle->EdgeSet.u16MinXEdge[StaIndex];
	}
	else
	{
		return;
	}

	//定位车斗段的极大极小位置
	for (iIndex=StaIndex;iIndex<EndIndex+1;iIndex++)
	{
		if (pH[iIndex]>0)
		{
			HopperCI.SumH+=pH[iIndex];
			HopperCI.SumX+=pVehicle->EdgeSet.u16MinXEdge[iIndex];
			if (pH[iIndex]>HopperCI.MaxZ)
			{
				HopperCI.MaxZ=pH[iIndex];
				HopperCI.MaxZI=iIndex;
			}

			if (pH[iIndex]<HopperCI.MinZ)
			{
				HopperCI.MinZ=pH[iIndex];
				HopperCI.MinZI=iIndex;
			}

			if (pVehicle->EdgeSet.u16MinXEdge[iIndex]>HopperCI.MaxX)
			{
				HopperCI.MaxX=pVehicle->EdgeSet.u16MinXEdge[iIndex];
				HopperCI.MaxXI=iIndex;
			}

			if (pVehicle->EdgeSet.u16MinXEdge[iIndex]<HopperCI.MinX)
			{
				HopperCI.MinX=pVehicle->EdgeSet.u16MinXEdge[iIndex];
				HopperCI.MinXI=iIndex;
			}
		}
	}

	//计算车斗段的均值高度
	HopperCI.AvgH=HopperCI.SumH/HopperCI.Num;
	HopperCI.AvgX=HopperCI.SumX/HopperCI.Num;

	ShapeAnalysisEx(pH,&HopperCI);


	//③斗的判定 x方向与轴的位置偏差不大 存在高度降低或高度陡降
	if (pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleCastX+150>HopperCI.AvgX)
	{
		if (HopperCI.Style == 2 && HopperCI.FlatR>70 && HopperCI.AvgH+300<pVehicle->EdgeSet.u16VehHeight)
		{
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|4;
			return;
		}
		else if (HopperCI.Style == 2 && HopperCI.FlatR>95 && HopperCI.AvgH+240<pVehicle->EdgeSet.u16VehHeight
			&& HopperCI.Sta>pVehicle->EdgeSet.u16VehHtIndex)
		{
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|4;
			return;
		}
		else if (HopperCI.Style == 1 && pVehicle->EdgeSet.u16VehHtIndex<HopperCI.Sta && 
			HopperCI.AvgH+400<pVehicle->EdgeSet.u16VehHeight && HopperCI.MaxZ+HopperCI.MinZ>HopperCI.AvgH*2
			&& HopperCI.MaxZ-HopperCI.MinZ>500 && HopperCI.MaxZI>HopperCI.MinZI)
		{
			//最大高出现在第二轴前 且第二轴上高度与车高有高差
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|4;
			return;
		}
		else if (pVehicle->EdgeSet.u16VehHtIndex>HopperCI.Sta && HopperCI.Style == 1
			&& HopperCI.MaxZ-HopperCI.AvgH>400 && HopperCI.AvgH-HopperCI.MinZ<100)
		{
			//最大高出现在第二轴上，且呈现下降趋势，整体高度落差有400 
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|4;
			return;
		}
		else if (HopperCI.Style == 2 && HopperCI.Sta>pVehicle->EdgeSet.u16VehHtIndex && HopperCI.AvgH+180<pVehicle->EdgeSet.u16VehHeight)
		{
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|4;
			return;
		}

	}
}




/************************************************************************/
/* 轮轴定位车斗检测法                                                   */
/************************************************************************/
void AxleHopperCheck(VehicleSet *pVehicle)
{
	int16 iIndex=0;
	int16 PtNum=0;
	int16 StaIndex=0;//第二轴后第一个一个大于1000的点序号
	int16 EndIndex=0;//第二轴后最后一个大于1000的点序号

	uint16 HopperTh=1000;//车斗高度下限

	ClusterInfo HopperCI;//车斗聚类段
	ClusterInfo CrashCI;//防撞栏聚类段

	memset(&HopperCI,0,sizeof(HopperCI));
	memset(&CrashCI,0,sizeof(CrashCI));

	//不是两轴 或车高＜1450 或 帧数＜8 直接跳出不判断
	if (pVehicle->AxleSet.AxleSum!=TWO_AXLE||
		pVehicle->EdgeSet.u16VehHeight<1450||pVehicle->u16FrameCnt<8)
	{
		return;
	}

	//轮轴之后的帧 线形分析

	//①寻找类似车斗的部分段
	PtNum=pVehicle->u16FrameCnt-pVehicle->AxleSet.Axle[1].AxleOriStaI+1;
	for (iIndex=0;iIndex<PtNum;iIndex++)
	{
		if (StaIndex == 0 && pVehicle->EdgeSet.u16MaxHeight[iIndex+pVehicle->AxleSet.Axle[1].AxleOriStaI]>HopperTh)
		{
			StaIndex=pVehicle->AxleSet.Axle[1].AxleOriStaI+iIndex;
		}

		if (EndIndex == 0 && pVehicle->EdgeSet.u16MaxHeight[pVehicle->u16FrameCnt-iIndex]>HopperTh)
		{
			EndIndex=pVehicle->u16FrameCnt-iIndex;
		}

		if (StaIndex!=0 && EndIndex!=0)
		{
			break;
		}
	}

	//②评价该段的线形
	if (EndIndex-StaIndex>3)
	{
		HopperCI.Sta=StaIndex;
		HopperCI.End=EndIndex;
		HopperCI.Num=EndIndex-StaIndex+1;
		HopperCI.MaxZ=pVehicle->EdgeSet.u16MaxHeight[StaIndex];
		HopperCI.MaxZI=StaIndex;
		HopperCI.MinZ=pVehicle->EdgeSet.u16MaxHeight[StaIndex];
		HopperCI.MinZI=StaIndex;
		HopperCI.MaxX=pVehicle->EdgeSet.u16MinXEdge[StaIndex];
		HopperCI.MinX=pVehicle->EdgeSet.u16MinXEdge[StaIndex];
		HopperCI.MaxXI=StaIndex;
		HopperCI.MinXI=StaIndex;
		HopperCI.SumX+=pVehicle->EdgeSet.u16MinXEdge[StaIndex];
	}
	else
	{
		return;
	}
	
	//定位车斗段的极大极小位置
	for (iIndex=StaIndex;iIndex<EndIndex+1;iIndex++)
	{
		HopperCI.SumH+=pVehicle->EdgeSet.u16MaxHeight[iIndex];
		HopperCI.SumX+=pVehicle->EdgeSet.u16MinXEdge[iIndex];
		if (pVehicle->EdgeSet.u16MaxHeight[iIndex]>HopperCI.MaxZ)
		{
			HopperCI.MaxZ=pVehicle->EdgeSet.u16MaxHeight[iIndex];
			HopperCI.MaxZI=iIndex;
		}

		if (pVehicle->EdgeSet.u16MaxHeight[iIndex]<HopperCI.MinZ)
		{
			HopperCI.MinZ=pVehicle->EdgeSet.u16MaxHeight[iIndex];
			HopperCI.MinZI=iIndex;
		}

		if (pVehicle->EdgeSet.u16MinXEdge[iIndex]>HopperCI.MaxX)
		{
			HopperCI.MaxX=pVehicle->EdgeSet.u16MinXEdge[iIndex];
			HopperCI.MaxXI=iIndex;
		}

		if (pVehicle->EdgeSet.u16MinXEdge[iIndex]<HopperCI.MinX)
		{
			HopperCI.MinX=pVehicle->EdgeSet.u16MinXEdge[iIndex];
			HopperCI.MinXI=iIndex;
		}


	}

	//计算车斗段的均值高度
	HopperCI.AvgH=HopperCI.SumH/HopperCI.Num;
	HopperCI.AvgX=HopperCI.SumX/HopperCI.Num;

	ShapeAnalysisEx(pVehicle->EdgeSet.u16MaxHeight,&HopperCI);


	//③斗的判定 x方向与轴的位置偏差不大 存在高度降低或高度陡降
	if (pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleCastX+150>HopperCI.AvgX)
	{
		if (HopperCI.Style == 2 && HopperCI.FlatR>70 && HopperCI.AvgH+300<pVehicle->EdgeSet.u16VehHeight)
		{
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|4;
			return;
		}
		else if (HopperCI.Style == 2 && HopperCI.FlatR>95 && HopperCI.AvgH+240<pVehicle->EdgeSet.u16VehHeight
			 && HopperCI.Sta>pVehicle->EdgeSet.u16VehHtIndex)
		{
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|4;
			return;
		}
		else if (HopperCI.Style == 1 && pVehicle->EdgeSet.u16VehHtIndex<HopperCI.Sta && 
			HopperCI.AvgH+400<pVehicle->EdgeSet.u16VehHeight && HopperCI.MaxZ+HopperCI.MinZ>HopperCI.AvgH*2
			 && HopperCI.MaxZ-HopperCI.MinZ>500 && HopperCI.MaxZI>HopperCI.MinZI)
		{
			//最大高出现在第二轴前 且第二轴上高度与车高有高差
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|4;
			return;
		}
		else if (pVehicle->EdgeSet.u16VehHtIndex>HopperCI.Sta && HopperCI.Style == 1
			 && HopperCI.MaxZ-HopperCI.AvgH>400 && HopperCI.AvgH-HopperCI.MinZ<100)
		{
			//最大高出现在第二轴上，且呈现下降趋势，整体高度落差有400 
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|4;
			return;
		}
		else if (HopperCI.Style == 2 && HopperCI.Sta>pVehicle->EdgeSet.u16VehHtIndex && HopperCI.AvgH+180<pVehicle->EdgeSet.u16VehHeight)
		{
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|4;
			return;
		}

	}
}


//基于斜率的车窗算子
void SlopeGlassCal(VehicleSet *pVehicle)
{
	int16 ThreshSt=0;//大于阈值高度的第一个点
	int16 WinStaPt=0;
	int16 WinStaX=0;
	int32 AvgIntoX=0;
	int32 GlassPtCnt=0;
	int32 GlassOver=0;
//	int16 WinEndPt=0;
//	int16 WinStaHt=0;
//	int16 WinEndHt=0;

	int16 MaxX=0;
	int16 MinX=9999;


	int16 i16Index=0;
	int16 i16Index1=0;
	
	uint16 CurFramI=0;
	uint16 l_u16PtSum=0;
	uint16 l_u16ThreshH=1000;
	uint16 ThreshX=200;
	uint16 ThreshZ=260;

	int16 K1=0;
	int16 K2=0;
	int16 K12=0;


	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT 
		|| pVehicle->u16FrameCnt < 8||pVehicle->AxleSet.AxleSum<1||pVehicle->CurFrmInfo.u16MaxHt<1000
		||pVehicle->CurRegion.zdata[0]>= FRAME_BUFLEN)
		return ;

	CurFramI=pVehicle->u16FrameCnt-1;
	l_u16PtSum=pVehicle->CurRegion.zdata[0];
	l_u16ThreshH = pVehicle->CurFrmInfo.u16MaxHt/2 > 1000 ? pVehicle->CurFrmInfo.u16MaxHt/2 : 1000;   //最低1000

	//第一步找到高度大于1000的第一个点
	for (i16Index=l_u16PtSum;i16Index>1;i16Index--)
	{
		if (pVehicle->CurRegion.zdata[i16Index]>l_u16ThreshH)
		{
			ThreshSt=i16Index;
			break;
		}
	}
    //第二步通过斜率识别穿透特征
	for (i16Index=ThreshSt;i16Index>2;i16Index--)
	{
		
		if (pVehicle->CurRegion.zdata[i16Index] == 0||pVehicle->CurRegion.zdata[i16Index-1] == 0||pVehicle->CurRegion.zdata[i16Index]<1200
			||pVehicle->CurRegion.xdata[i16Index]-pVehicle->CurRegion.xdata[i16Index-1]  == 0)
		{
			//打飞直接跳过
			continue;
		}
		else
		{
			if ((ABS(pVehicle->CurRegion.xdata[i16Index]-pVehicle->CurRegion.xdata[i16Index-1])>ThreshX
				 && ABS(pVehicle->CurRegion.zdata[i16Index]-pVehicle->CurRegion.zdata[i16Index-1])>ThreshX/10)
				||ABS(pVehicle->CurRegion.zdata[i16Index]-pVehicle->CurRegion.zdata[i16Index-1])>ThreshZ)
			{

				//分别求激光到车身上点的斜率 k1 k2,与车身上相邻点的斜率k12;
				K1 = 1000* ( pVehicle->CurRegion.zdata[i16Index] -Smp.JGHeight ) / (pVehicle->CurRegion.xdata[i16Index] );//斜率 y/x,y需要转换为到激光的高度 
				K2 = 1000 * (  pVehicle->CurRegion.zdata[i16Index-1]-Smp.JGHeight) / (pVehicle->CurRegion.xdata[i16Index-1]);

				K12 = 1000 * (  pVehicle->CurRegion.zdata[i16Index]  - pVehicle->CurRegion.zdata[i16Index-1] ) / ( pVehicle->CurRegion.xdata[i16Index]  -  pVehicle->CurRegion.xdata[i16Index-1] )  ; 

				
				if (K1*K12 > 0  &&  K1*K2>0   &&   K12 > K2  &&  K2 > K1 )
				{
					//判断点的关系，车窗以上的点，车顶的点，应大于车窗边沿点（穿透点）
					WinStaPt =  i16Index-1;//车窗起始点
					WinStaX =pVehicle->CurRegion.xdata[i16Index+2];
					GlassPtCnt=0;
					//跳过车窗 判断车窗上部平整度
					for (i16Index1=WinStaPt;i16Index1>1;i16Index1--)
					{
					
						if (pVehicle->CurRegion.xdata[i16Index1]>0)
						{
							if (GlassOver<1)
							{
								if (pVehicle->CurRegion.xdata[i16Index1]<WinStaX && GlassPtCnt>2)
								{
									GlassOver=1;
								}
								else
								{
									AvgIntoX+=pVehicle->CurRegion.xdata[i16Index1]-WinStaX;
//									WinEndPt=i16Index1;
									GlassPtCnt++;
								}
							}
							else if (GlassOver>0)
							{
								if (pVehicle->CurRegion.xdata[i16Index1]<MinX)
								{
									MinX=pVehicle->CurRegion.xdata[i16Index1];
								}

								if (pVehicle->CurRegion.xdata[i16Index1]>MaxX)
								{
									MaxX=pVehicle->CurRegion.xdata[i16Index1];
								}

							}
						}
					}


					if (GlassPtCnt>0)
					{
						AvgIntoX=AvgIntoX/GlassPtCnt;
					}


					if (MaxX-MinX>0 && MaxX-MinX<AvgIntoX/3 && AvgIntoX>200)
					{
						pVehicle->WindowsSet.GlassWinFlag[CurFramI]|=4;
						break;
					}
				}
			}
		}
	}
}

//整体打飞车窗投影图 车窗算子
void MapGlassCal(VehicleSet *pVehicle)
{
	int16 i16Index=0;
	uint16 ThreshH=900;//高度阈值
	uint16 PtSum=0;//当前帧总点数
	uint16 CurFramI=0;//当前帧序号
	uint16 ThresHSta=0;
	uint16 StaI=0;
	uint16 StaHt=0;
	uint16 EndI=0;
	uint16 EndHt=0;
	uint16 BlockCnt=0;
	uint16 TmpStaH=0;
	uint16 TmpEndH=0;
	uint16 sCoverCnt=0;
	uint16 bCoverCnt=0;
	uint16 TmpIndex=0;

	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT|| pVehicle->u16FrameCnt < 8 ||
		pVehicle->AxleSet.AxleSum<1||pVehicle->CurFrmInfo.u16MaxHt<1000||pVehicle->CurRegion.zdata[0] > FRAME_BUFLEN-1
		|| pVehicle->CurRegion.zdata[0] < 10)
		return ;



	PtSum=pVehicle->CurRegion.zdata[0];

	CurFramI=pVehicle->u16FrameCnt-1;

	ThreshH=pVehicle->CurRegion.zMax/2>900?pVehicle->CurRegion.zMax/2:900;
	//遍历一遍 寻找打飞块
	for (i16Index=PtSum;i16Index>0;i16Index--)
	{
		if (ThresHSta<1)
		{
			if (pVehicle->CurRegion.zdata[i16Index]>ThreshH)
			{
				ThresHSta=i16Index;
			}
		}
		else
		{
			if (StaI<1)
			{
				if (pVehicle->CurRegion.zdata[i16Index] == 0)
				{
					StaI=i16Index+1;
					StaHt=pVehicle->CurRegion.zdata[StaI];
				}
			}
			else
			{
				if (pVehicle->CurRegion.zdata[i16Index]>0)
				{
					EndI=i16Index;
					EndHt=pVehicle->CurRegion.zdata[EndI];

					if (StaI-EndI>3 && EndHt-StaHt>150 && EndHt+300>pVehicle->CurRegion.zMax
						 && (pVehicle->CurRegion.xdata[StaI]-pVehicle->CurRegion.xdata[EndI])<150)
					{
						//确保打飞点数有至少3个，打飞形成的空白高度至少150 打飞处于同一面上
						BlockCnt++;
						break;
					}
				}
			}
		}
	}

	if (BlockCnt>0)
	{
		pVehicle->WindowsSet.BlockSet[pVehicle->WindowsSet.BlockIndex].StaHt=StaHt;
		pVehicle->WindowsSet.BlockSet[pVehicle->WindowsSet.BlockIndex].EndHt=EndHt;
		pVehicle->WindowsSet.BlockSet[pVehicle->WindowsSet.BlockIndex].GlassHt=EndHt-StaHt;
		pVehicle->WindowsSet.BlockSet[pVehicle->WindowsSet.BlockIndex].FramIndex=CurFramI;
		pVehicle->WindowsSet.BlockIndex=pVehicle->WindowsSet.BlockIndex++;
		pVehicle->WindowsSet.BlockIndex=pVehicle->WindowsSet.BlockIndex%MAPBLOCKCNT;

		if (pVehicle->WindowsSet.BlockCnt<MAPBLOCKCNT)
		{
			pVehicle->WindowsSet.BlockCnt++;
		}
		else
		{
			pVehicle->WindowsSet.BlockCnt=MAPBLOCKCNT;
		}
		

		if (pVehicle->WindowsSet.BlockCnt>MAPBLOCKCNT-1)
		{
			for (i16Index=0;i16Index<MAPBLOCKCNT;i16Index++)
			{
				StaI=i16Index;
				EndI=(i16Index+1)%MAPBLOCKCNT;
				TmpStaH=MAX(pVehicle->WindowsSet.BlockSet[StaI].StaHt,pVehicle->WindowsSet.BlockSet[EndI].StaHt);
				TmpEndH=MIN(pVehicle->WindowsSet.BlockSet[StaI].EndHt,pVehicle->WindowsSet.BlockSet[EndI].EndHt);


				if (TmpEndH-TmpStaH>100)
				{
					if (TmpEndH-TmpStaH>500)
					{
						bCoverCnt++;
					}
					sCoverCnt++;
				}
				else
				{
					//如果两帧波形重叠部分不存在 则清除前一帧
					memset(&pVehicle->WindowsSet.BlockSet[StaI],0,sizeof(GlassBlock));
					pVehicle->WindowsSet.BlockCnt=pVehicle->WindowsSet.BlockCnt-1;
					break;
				}
			}

			if (sCoverCnt>2)
			{
				for (i16Index=0;i16Index<MAPBLOCKCNT;i16Index++)
				{
					TmpIndex=pVehicle->WindowsSet.BlockSet[i16Index].FramIndex;
					//小型车的玻璃高度要小
					if (pVehicle->WindowsSet.GlassWinFlag[TmpIndex]&16)
					{

					}
					else
					{
						pVehicle->WindowsSet.GlassWinFlag[TmpIndex]=pVehicle->WindowsSet.GlassWinFlag[TmpIndex]|16;
					}
					//大型车的玻璃要大
					if (bCoverCnt>1)
					{
						if (pVehicle->WindowsSet.GlassWinFlag[TmpIndex]&32)
						{

						}
						else
						{
							pVehicle->WindowsSet.GlassWinFlag[TmpIndex]=pVehicle->WindowsSet.GlassWinFlag[TmpIndex]|32;
						}
					}
				}
			}
			/*else
			{
				memset(&pVehicle->WindowsSet.BlockSet,0,sizeof(GlassBlock));
				pVehicle->WindowsSet.BlockCnt=0;
				pVehicle->WindowsSet.BlockIndex=0;
			}*/
		}

	}
	else
	{
		memset(&pVehicle->WindowsSet.BlockSet,0,sizeof(GlassBlock));
		pVehicle->WindowsSet.BlockCnt=0;
		pVehicle->WindowsSet.BlockIndex=0;
	}
}


//车身穿透算子 新版 
void ThroughGlassCal(VehicleSet *pVehicle)
{

	int16 i16Index=0;
	uint16 PtSum=0;//当前帧总点数
	uint16 CurFramI=0;//当前帧序号

	uint16 xMax=0;//x最大值
	uint16 xMin=0;//x最小值
//	uint16 StaX0=0;//分布起始x
//	uint16 nStep=0;//分布步数
//	uint16 xStep=50;//分布步长
//	uint16 xHist[60]={0};//0-3000m分布
//	uint16 nHistCnt=0;//参与统计的点数
//	uint16 nEdge=0;//车身边界x 分布点
	uint16 xEdge=0;//车身边界x
	uint16 ThreshH=1000;//高度阈值
	int32 CastX=0;//穿透部分x之差
	uint16 CastCnt=0;//穿透点数

	uint16 StaPt=0;
	uint16 EndPt=0;

	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > MAX_CUT 
		|| pVehicle->u16FrameCnt < 8 ||pVehicle->AxleSet.AxleSum<1||pVehicle->CurFrmInfo.u16MaxHt<1000||
		pVehicle->CurRegion.zdata[0] > FRAME_BUFLEN-1 || pVehicle->CurRegion.zdata[0] < 10)
		return ;

	PtSum=pVehicle->CurRegion.zdata[0];

	CurFramI=pVehicle->u16FrameCnt-1;

	ThreshH=pVehicle->CurRegion.zMax/2>800?pVehicle->CurRegion.zMax/2:800;

	//找到边界线
	xEdge=(2*pVehicle->CurRegion.xMin+pVehicle->CurRegion.xMax)/3;//最左往右扩展宽度的1/3作为边界

	//找到玻璃段及玻璃段上方段
	xMin=pVehicle->CurRegion.xMax;
	for (i16Index=PtSum-2;i16Index>2;i16Index--)
	{
		if (pVehicle->CurRegion.zdata[i16Index]>ThreshH)
		{
			if (pVehicle->CurRegion.xdata[i16Index]>xEdge && StaPt<1)
			{
				//车窗起点
				EndPt=i16Index;
				StaPt=i16Index;
			}
			else if (pVehicle->CurRegion.xdata[i16Index]<xEdge && pVehicle->CurRegion.xdata[i16Index-1]<xEdge
				 && StaPt>0 && StaPt-EndPt<1)
			{
				EndPt=i16Index+1;
			}

			//穿透部分
			if (StaPt>0 && StaPt-EndPt<1)
			{
				//穿透部分
				CastX+=pVehicle->CurRegion.xdata[i16Index]-xEdge;
				CastCnt++;
			}

			//车窗上端
			if (StaPt-EndPt>0)
			{
				if (pVehicle->CurRegion.xdata[i16Index]>xMax)
				{
					xMax=pVehicle->CurRegion.xdata[i16Index];
				}

				if (pVehicle->CurRegion.xdata[i16Index]<xMin)
				{
					xMin=pVehicle->CurRegion.xdata[i16Index];
				}
			}
		}
	}

	if (CastCnt>0 && xMax-xMin>0 && 
		xMax-xMin<200 && CastX/CastCnt>300)
	{
		pVehicle->WindowsSet.GlassWinFlag[CurFramI]=pVehicle->WindowsSet.GlassWinFlag[CurFramI]|8;
	}
}



//车身穿透算子
void IntoGlassCal(VehicleSet *pVehicle)
{
	int32 iCoorX[FRAME_BUFLEN]={0};
	int32 iCoorZ[FRAME_BUFLEN]={0};
	int32 iFilterX[FRAME_BUFLEN]={0};
	int32 iFilterZ[FRAME_BUFLEN]={0};

	uint16 FitPoint=0;
	uint16 PointCnt=0;
	uint16 DafeiPt=0;

	//阈值设置  高度下限 滤波点数
	uint16 ThreshH =1000;//阈值
	uint16 ThreshPt=4;
	uint16 ThreshX=300;
//	uint16 ThreshZ=300;


	uint16 MaxXPt=0;//最大x位置
	int32  MaxX=0;//最大X位置
//	uint16 MaxXHt=0;//最大x对应高度

	uint16 MinX1=9999;//上部分最小x值
	uint16 MinX1Pt=0;//上部分最小x值对应的点序
	uint16 MinX2=9999;//下部分最小x值
	uint16 MinX2Pt=0;//下部分最小x值对应的点序

	int32 SumDiffX=0;


//	int32 MaxHtx=0;//最大高x位置
//	int32  MaxHt=0;//高最大值
//	uint16 MinHtPt=0;//高最小值位置
//	int32  MinHt=65535;//高最小值
//	int32 MinHtx=0;//最小高x位置

	uint16 PtSum=0;//满足阈值点数
	uint16 zNearCnt=0;

	uint16 u16Index=0;
	uint16 CurFramI=0;

//	uint16 xHist[35]={0};//x方向直方图
	uint16 xMax=0;
	uint16 xMin=0;


	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > MAX_CUT 
		|| pVehicle->u16FrameCnt < 80||pVehicle->AxleSet.AxleSum<1||pVehicle->CurFrmInfo.u16MaxHt<1000||
		pVehicle->CurRegion.zdata[0] > FRAME_BUFLEN-1 || pVehicle->CurRegion.zdata[0] < 10)
		return ;

	PointCnt=pVehicle->CurRegion.zdata[0];

	CurFramI=pVehicle->u16FrameCnt-1;

	//第一步 找到最左和最右边界
	for (u16Index=1;u16Index<PointCnt+1;u16Index++)
	{
		if (pVehicle->CurRegion.zdata[u16Index]!=0)
		{
			if (xMax<pVehicle->CurRegion.xdata[u16Index])
			{
				xMax=pVehicle->CurRegion.xdata[u16Index];
			}

			if (xMin>pVehicle->CurRegion.xdata[u16Index])
			{
				xMin=pVehicle->CurRegion.xdata[u16Index];
			}
		}
	}


	//筛选数据 过滤打飞情况
	for (u16Index=1;u16Index<PointCnt+1;u16Index++)
	{
		if (pVehicle->CurRegion.zdata[u16Index]>ThreshH)
		{
			iCoorZ[FitPoint]=pVehicle->CurRegion.zdata[u16Index];
			iCoorX[FitPoint]=pVehicle->CurRegion.xdata[u16Index];
			FitPoint++;
		}
		else if (FitPoint>0 && pVehicle->CurRegion.zdata[u16Index] == 0)
		{
			DafeiPt++;
		}
	}


	if (DafeiPt>3)
	{
		return;
	}

	ThreshPt=MAX(FitPoint/10+1,4);


	//进行四邻域均值滤波 点数多邻域扩大
	PtSum=0;
	for (u16Index=ThreshPt/2;u16Index<FitPoint-ThreshPt/2;u16Index++)
	{
		iFilterX[PtSum]=MeanFilterI(&iCoorX[u16Index-ThreshPt/2],ThreshPt);
		iFilterZ[PtSum]=MeanFilterI(&iCoorZ[u16Index-ThreshPt/2],ThreshPt);
		PtSum++;
	}

	//定位最右点位置
	for (u16Index=0;u16Index<PtSum-1;u16Index++)
	{
		if (iFilterX[u16Index]>MaxX)
		{
			MaxXPt=u16Index;
			MaxX=iFilterX[u16Index];
//			MaxXHt=iFilterZ[u16Index];
		}
	}



	//以最右点为分界寻找上下段最下x
	for (u16Index=0;u16Index<PtSum-1;u16Index++)
	{
		if (u16Index<MaxXPt)
		{
			//上部分的最小x值及位置
			if (MinX1>iFilterX[u16Index])
			{
				MinX1=iFilterX[u16Index];
				MinX1Pt=u16Index;
			}

		}
		else
		{
			//下部分的最小x及位置
			if (MinX1>iFilterX[u16Index])
			{
				MinX2=iFilterX[u16Index];
				MinX2Pt=u16Index;
			}
		}
	}


	//没找到极右上下边界直接退出
	if (MinX1Pt<1||MinX2Pt<1)
	{
		return;
	}



	//计算累计求和
	for (u16Index=0;u16Index<PtSum-1;u16Index++)
	{
		if (u16Index>MinX1Pt && u16Index<MaxXPt+1)
		{
			if (iFilterX[u16Index]>MinX1+100)
			{
				SumDiffX+=iFilterX[u16Index+1]-iFilterX[u16Index];
			}

			if (ABS(iFilterZ[u16Index]-iFilterZ[u16Index+1])>100)
			{
				zNearCnt++;
			}
		}

		if (u16Index>MaxXPt-1 && u16Index<MinX2Pt-1)
		{
			if (iFilterX[u16Index]>MinX2+100)
			{
				SumDiffX+=iFilterX[u16Index]-iFilterX[u16Index+1];
			}

			if (ABS(iFilterZ[u16Index]-iFilterZ[u16Index+1])>100)
			{
				zNearCnt++;
			}
		}
	}

	if (SumDiffX/2>ThreshX && zNearCnt<2)
	{
		if (pVehicle->WindowsSet.GlassWinCnt == 0)
		{
			pVehicle->WindowsSet.GlassWinStaI=CurFramI;

		}
		pVehicle->WindowsSet.GlassWinEndI=CurFramI;
		/*pVehicle->WindowsSet.GlassWinCnt++;*/
		pVehicle->WindowsSet.GlassWinFlag[CurFramI]=pVehicle->WindowsSet.GlassWinFlag[CurFramI]|2;
	}

}



/************************************************************************/
/* 车窗检测				                                                */
/************************************************************************/
void GlassWinCheck(VehicleSet *pVehicle)
{
//	uint8    l_u8RetValue = 0;
//	uint8    l_u8CarWindowFlag = 0; //车窗标识
//	uint8    l_u8Count = 0;
	uint16   l_u16Index   = 0;
//	uint16   l_u16TmpIndex = 0;
//	uint16   l_u16CurHeight = 0;
	uint16   TmpCnt=0;
	uint16   MinAxleFrCnt=9999;//轮轴最小帧数


	uint16  StaI=0;
	uint16  EndI=0;

	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 10)
		return ;


	//进行多轴车的简单纠错
	if (pVehicle->AxleSet.AxleSum>2 && pVehicle->EdgeSet.u16VehHeight<2200)
	{
		pVehicle->AxleSet.AxleSum=TWO_AXLE;
	}

	for (l_u16Index=0;l_u16Index<pVehicle->AxleSet.AxleSum;l_u16Index++)
	{
		TmpCnt=pVehicle->AxleSet.Axle[l_u16Index].AxleOriEndI-pVehicle->AxleSet.Axle[l_u16Index].AxleOriStaI;
		if (TmpCnt<MinAxleFrCnt)
		{
			MinAxleFrCnt=TmpCnt;
		}
	}

	


	//确定车窗检测的范围段
	if (pVehicle->AxleSet.AxleSum>1)
	{
		if (pVehicle->u16FrameCnt/2-pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI>0)
		{
			//停车导致最后一个轮轴尾部一直被扫描到 只选取第一胎结束帧与最后一胎结束帧的一半  到 最后一轴结束的位置进入统计分析
			StaI=(pVehicle->AxleSet.Axle[0].AxleOriEndI+pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI)/2;
			EndI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI;
		}
		else if (pVehicle->u16FrameCnt/3-pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI>0
			 && pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI-pVehicle->AxleSet.Axle[0].AxleOriEndI>MinAxleFrCnt/2)
		{
			//停车导致最后一个轮轴开始一直被扫到 选取第一轴结束到最后一个轴的开始段进行统计分析  20170714 加入前后轴间隔限制
			StaI=(pVehicle->AxleSet.Axle[0].AxleOriEndI+pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI)/2;
			EndI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI;

		}
		else if (pVehicle->u16FrameCnt<pVehicle->AxleSet.Axle[0].AxleOriEndI*2
			 && pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI-pVehicle->AxleSet.Axle[0].AxleOriEndI>MinAxleFrCnt)
		{
			//第一个轮轴前方停车 选取第一个轴结束到第二个轴开始位置进行统计 20170714
			StaI=(pVehicle->AxleSet.Axle[0].AxleOriEndI+pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI)/2;
			EndI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI;
		}
		else
		{
			StaI=pVehicle->u16FrameCnt/2;
			EndI=pVehicle->u16FrameCnt-1;
		}
	}
	else
	{
		StaI=pVehicle->u16FrameCnt/2;
		EndI=pVehicle->u16FrameCnt-1;
	}



	for (l_u16Index = StaI; l_u16Index < EndI; l_u16Index++) //检测后一半帧数	
	{
		//20170714 由高度200范围扩大到300范围内的帧进行判断
		if (pVehicle->EdgeSet.u16VehHeight < pVehicle->EdgeSet.u16MaxHeight[l_u16Index] + 300  && 
			ABS(pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleCastX-pVehicle->EdgeSet.u16MinXEdge[l_u16Index])<200) //高度限定
		{
			if (pVehicle->EdgeSet.u16HeadHeight<1800)
			{
				if (pVehicle->WindowsSet.GlassWinFlag[l_u16Index] == 16)
				{
					pVehicle->WindowsSet.GlassWinCnt++;
				}
				else if (pVehicle->WindowsSet.GlassWinFlag[l_u16Index]!=16)
				{
					if (pVehicle->WindowsSet.GlassWinFlag[l_u16Index])
					{
						pVehicle->WindowsSet.GlassWinCnt++;
					}
				}
			}
			else
			{
				if (pVehicle->WindowsSet.GlassWinFlag[l_u16Index]&32)
				{
					pVehicle->WindowsSet.GlassWinCnt++;
				}
				else if ((pVehicle->WindowsSet.GlassWinFlag[l_u16Index]&16)<0)
				{
					if (pVehicle->WindowsSet.GlassWinFlag[l_u16Index])
					{
						pVehicle->WindowsSet.GlassWinCnt++;
					}
				}
			}
		}
	}

	if (pVehicle->EdgeSet.u16HeadHeight<1800)
	{
		if (pVehicle->WindowsSet.GlassWinCnt > 1) //超2帧
		{
			pVehicle->FeatureSet.IsGlassWin=1;
		}
	}
	else
	{
		if (pVehicle->WindowsSet.GlassWinCnt > 5
			 && pVehicle->WindowsSet.GlassWinCnt*3>EndI-StaI) //超2帧
		{
			pVehicle->FeatureSet.IsGlassWin=1;
		}
	}

}
/************************************************************************/
/* 尾部跳变 检测车斗或载货                                              */
/************************************************************************/
void TailJumpCheck(VehicleSet *pVehicle)
{
	int16 i16Index=0;
	uint16 u16StaI=0;
	uint16 u16EndI=0;
	uint16 u16RealIndex=0;
	uint16 u16HeadCastX=0;//车头的x边界值
//	uint16 u16HeadEndI=0;//车头结束帧
	uint16 u16CheckCnt=0;//检查段帧数
	uint16 u16CheckMaxZ=0;//检查段最大高
	uint16 u16CheckMinZ=9999;//检查段最小高
	uint32 n32CheckSumZ=0;//检查段高之和
	uint16 u16CheckAvgZ=0;//检查段高均值
	uint16 u16CheckMaxX=0;//检查段边界x最大
	uint16 u16CheckMinX=9999;//检查段边界x最小
	uint32 n32CheckSumX=0;//检查段边界x之和
//	uint16 u16CheckAvgX=0;//检查段边界x均值
	int32 n32HeadHeight=0;//车头高度
	int32 n32BodyHeight=0;//车身段高度
	int32 n32TailHeight=0;//车位段高度
	uint16 u16CalCnt=0;//临时计算计数


	//至少一个轴
	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT 
		|| pVehicle->u16FrameCnt < 10||pVehicle->AxleSet.AxleSum<1
		||pVehicle->EdgeSet.u16HeadHeight<1800)

		return ;

	if (pVehicle->AxleSet.AxleSum>1)
	{
		//两轴以上情况 直接以轴定位检测跳变
		u16StaI=pVehicle->AxleSet.Axle[0].AxleOriEndI;
		u16EndI=pVehicle->AxleSet.Axle[1].AxleOriStaI;
		n32HeadHeight=pVehicle->EdgeSet.u16HeadHeight;

		if (u16StaI<1||u16EndI<1)
		{
			//防止轴数是纠正得到的
			return;
		}

		u16HeadCastX=pVehicle->AxleSet.Axle[0].AxleCastX;
		u16CheckCnt=MIN(u16EndI-u16StaI,2*(pVehicle->AxleSet.Axle[1].AxleOriEndI-u16EndI));

		if (u16CheckCnt<2)
		{
			return;
		}

		for (i16Index=0;i16Index<u16CheckCnt;i16Index++)
		{
			u16RealIndex=u16EndI-i16Index;
			if (pVehicle->EdgeSet.u16MinXEdge[u16RealIndex]<u16CheckMinX)
			{
				u16CheckMinX=pVehicle->EdgeSet.u16MinXEdge[u16RealIndex];
			}
			if (pVehicle->EdgeSet.u16MinXEdge[u16RealIndex]>u16CheckMaxX)
			{
				u16CheckMaxX=pVehicle->EdgeSet.u16MinXEdge[u16RealIndex];
			}

			n32CheckSumX+=pVehicle->EdgeSet.u16MinXEdge[u16RealIndex];


			if (pVehicle->EdgeSet.u16MaxHeight[u16RealIndex]<u16CheckMinZ)
			{
				u16CheckMinZ=pVehicle->EdgeSet.u16MaxHeight[u16RealIndex];
			}

			if (pVehicle->EdgeSet.u16MaxHeight[u16RealIndex]>u16CheckMaxZ)
			{
				u16CheckMaxZ=pVehicle->EdgeSet.u16MaxHeight[u16RealIndex];
			}

			n32CheckSumZ+=pVehicle->EdgeSet.u16MaxHeight[u16RealIndex];
		}

//		u16CheckAvgX=n32CheckSumX/u16CheckCnt;
		u16CheckAvgZ=n32CheckSumZ/u16CheckCnt;


		if ((n32HeadHeight+300<u16CheckAvgZ||n32HeadHeight+400<u16CheckMaxZ) && u16HeadCastX>u16CheckMinX
			 && u16CheckAvgZ-pVehicle->EdgeSet.u16RearHeight>300 && pVehicle->EdgeSet.u16RearHeight>0)
		{
			pVehicle->FeatureSet.IsTailJump|=8;
		}
		else if ((n32HeadHeight-250>u16CheckAvgZ||n32HeadHeight-200>u16CheckMaxZ)
			 && u16HeadCastX>u16CheckMinX)
		{
			pVehicle->FeatureSet.IsTailJump|=16;
		}
	}
	else
	{
		//第一轴为尾部轴 计算车头高进行判断跳变
		u16StaI=pVehicle->AxleSet.Axle[0].AxleOriStaI;
		u16EndI=pVehicle->AxleSet.Axle[0].AxleOriEndI;
		if (u16StaI<1||u16StaI<pVehicle->u16FrameCnt/3)
		{
			//防止轴数是纠正得到的
			return;
		}

		//计算车头高
		u16CalCnt=MIN(pVehicle->u16FrameCnt/3,u16EndI-u16StaI);
		n32HeadHeight=0;
		i16Index=0;
		for (u16RealIndex=2;u16RealIndex<pVehicle->u16FrameCnt/3;u16RealIndex++)
		{
			if (pVehicle->EdgeSet.u16MaxHeight[u16RealIndex]>1000)
			{
				n32HeadHeight+=pVehicle->EdgeSet.u16MaxHeight[u16RealIndex];
				i16Index++;
			}
			if (i16Index+1>u16CalCnt)
			{
				break;
			}
		}
		u16CalCnt=i16Index;
		if (u16CalCnt<2)
		{
			return;
		}
		n32HeadHeight=n32HeadHeight/u16CalCnt;

		//计算车体中间段高度
		u16CalCnt=MIN(pVehicle->u16FrameCnt/3,u16EndI-u16StaI);
		if (u16CalCnt<2)
		{
			return;
		}
		n32BodyHeight=0;
		for (i16Index=0;i16Index<u16CalCnt;i16Index++)
		{
			u16RealIndex=pVehicle->u16FrameCnt/2-u16CalCnt/2+i16Index;
			n32BodyHeight+=pVehicle->EdgeSet.u16MaxHeight[u16RealIndex];
		}
		n32BodyHeight=n32BodyHeight/u16CalCnt;

		n32TailHeight=pVehicle->EdgeSet.u16HeadHeight;

		
		//已经保证尾部高度>1000
		if (ABS(n32BodyHeight-n32HeadHeight)>300 && ABS(n32TailHeight-n32HeadHeight)>300)
		{
			//车身段与车头高差大于300  车尾段与车头段高度差大于300
			pVehicle->FeatureSet.IsTailJump|=32;
		}
		else if (ABS(n32BodyHeight-n32HeadHeight)>300 && ABS(n32TailHeight-n32BodyHeight)>300)
		{
			//车身段与车头高差大于300  车尾段与车尾段高度差大于300
			pVehicle->FeatureSet.IsTailJump|=64;
		}
	}
}

/************************************************************************/
/* 检测货车驾驶室		                                                */
/************************************************************************/
void TruckCanCheck(VehicleSet *pVehicle)
{
	uint16 u16indx=0;
	uint16 u16TmpI=0;
	int32  shape[MAX_CUT]={0};//中轴线形状
	int32  zlen=0;
	uint8 temp=0;

	uint32 ContainSumH=0;
	uint32 CabSumH=0;

	uint16 CheckFram=0;//检查的帧数


	uint8 ContainRat=0;  //货箱帧数比例
	uint16 ContainCnt=0; //货箱帧数
	uint16 ContainSt=MAX_CUT; //货箱起点索引
	uint16 ContainH=0;  //货箱平均高度
	uint16 CabSt=0; //驾驶室起始索引
	int16 CabEd=0;//驾驶室的终止索引
	uint16 CabCnt=0;  //驾驶室帧数
	int16 CabH=0;  //驾驶室均高
	uint16 CabHmax=0;

	uint16 CabHst=0;//驾驶最前高度
	uint16 CabFlat=0;//驾驶室平坦帧
	uint16 FlatHt=0;
	uint32 CabFrH=0;//驾驶室前一半均高
	uint32 CabBaH=0;//驾驶室后一半均高

	uint8 CabStlye=0;//0-未检测；1--上升；2--平坦；3--下降；



	if (pVehicle  ==  NULL || pVehicle->u16FrameCnt < 6 
		|| pVehicle->EdgeSet.u16VehHeight < 2000||pVehicle->AxleSet.AxleSum>2)
		return ;


	if (pVehicle->u16FrameCnt>MAX_CUT)
	{
		if (pVehicle->AxleSet.AxleSum<1)
		{
			return;
		}
		else if (pVehicle->AxleSet.AxleSum<2)
		{
			//如果是只有第一个轴，后面轴无法定位 直接退出
			if (pVehicle->AxleSet.Axle[0].AxleEndIndex<pVehicle->u16FrameCnt/3)
			{
				return;
			}
			else
			{
				CheckFram=MIN(pVehicle->AxleSet.Axle[0].AxleOriStaI,MAX_CUT);//最后一轴
			}

		}
		else
		{
			if (pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI<pVehicle->u16FrameCnt/2)
			{
				CheckFram=MIN(pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI,MAX_CUT);//最后一轴
			}
			else
			{
				return;
			}
		}
	}
	else
	{
		CheckFram=pVehicle->u16FrameCnt;
	}



	zlen=pVehicle->EdgeSet.u16VehHeight/100-7;
	/************************************************************************/
	/*纵向阶梯化，变为锯齿状                                                */
	/************************************************************************/
	for (u16indx=0;u16indx<CheckFram;u16indx++)//阶梯化
	{
		//阶梯化
		temp=pVehicle->EdgeSet.u16MaxHeight[u16indx]/100;
		if (temp>7)
		{
			shape[u16indx]=temp-7;
		}
		else
		{
			shape[u16indx]=0;
		}
		//货箱帧数与起点
		if (shape[u16indx]+2>=zlen)
		{
			ContainSumH+=shape[u16indx];
			ContainCnt++;
			if (ContainSt>u16indx)
			{
				ContainSt=u16indx;
			}
		}
	}



	//货箱起点微调整
	if (ContainSt > 0  &&  ContainSt < MAX_CUT)
	{
		for (u16indx=ContainSt-1;u16indx>3;u16indx--)
		{
			if (abs(shape[u16indx]-shape[u16indx+1])<=1
				 && shape[u16indx]-shape[u16indx-1]>2
				 && shape[u16indx]>10)
			{
				ContainCnt=ContainCnt+ContainSt-u16indx;
				ContainSt=u16indx;
				ContainSumH+=shape[u16indx];
				break;
			}
		}
	}

	//箱体占比
	ContainRat=ContainCnt*10/CheckFram;
	//货箱均高
	if (ContainCnt > 0)
		ContainH=ContainSumH/ContainCnt;
	//有明显驾驶室货箱小货 往前遍历寻找驾驶室
	if(ContainRat>5 && ContainH>12 && ContainSt>2) 
	{
		//是否存在驾驶室及驾驶室高度
		CabEd=ContainSt-1;//初始化驾驶室终点
		CabSt=CabEd;//初始化驾驶室起点
		for (u16indx=ContainSt-1;u16indx>0;u16indx--)
		{
			if (zlen<9)
			{
				//低于直接退出
				break;
			}

			//寻找驾驶室
			if (zlen-shape[u16indx]>2 && ABS(shape[u16indx]-shape[CabEd])<3)
			{
				CabSumH+=shape[u16indx];
				CabCnt++;
				if (CabSt>u16indx)
				{
					CabSt=u16indx;
				}
				if (CabHmax<shape[u16indx])
				{
					CabHmax=shape[u16indx];
				}
			}
			else
			{
				if (CabHmax>0)
				{
					CabHst=shape[u16indx];

					//判断是否满足驾驶室平坦要求
					if (CabCnt<5)
					{	//重新寻找驾驶室
						CabEd=u16indx;
						CabSt=u16indx;
						CabHmax=0;
						CabSumH=0;
						CabCnt=0;
					}
					else
					{
						//判断是否为平坦驾驶室
						//判断驾驶室的形状
						FlatHt=0;
						CabFlat=0;
						CabFrH=0;
						CabBaH=0;
						CabStlye=0;
						for (u16TmpI=CabSt;u16TmpI<CabEd+1;u16TmpI++)
						{

							if (FlatHt == 0)
							{
								if (shape[u16TmpI] == shape[u16TmpI+1])
								{
									CabFlat++;
									FlatHt=shape[u16TmpI];
								}
							}
							else
							{
								if (shape[u16TmpI] == FlatHt)
								{
									CabFlat++;
								}
								else
								{
									if (shape[u16TmpI] == shape[u16TmpI+1])
									{
										CabFlat++;
										FlatHt=shape[u16TmpI];
									}
									else
									{
										FlatHt=0;
									}
								}
							}


							if (u16indx<(CabSt+CabEd)/2)
							{
								CabFrH+=shape[u16TmpI];
							}
							else if(u16indx>(CabSt+CabEd)/2)
							{
								CabBaH+=shape[u16TmpI];
							}
						}
						if (CabFlat*10/(CabEd-CabSt)>5 && CabFlat>3)
						{
							CabStlye=2;
							break;
						}
						else if (CabFrH<CabBaH)
						{
							CabStlye=1;
							CabEd=u16indx;
							CabSt=u16indx;
							CabHmax=0;
							CabSumH=0;
							CabCnt=0;
						}
						else if (CabFrH>CabBaH)
						{
							CabStlye=3;
							CabEd=u16indx;
							CabSt=u16indx;
							CabHmax=0;
							CabSumH=0;
							CabCnt=0;
						}

					}
				}
			}
		}

		if (CabStlye==2 && CabCnt>4 && CabHst<CabH)
		{
			//驾驶室的高度
			CabH=CabSumH/CabCnt;
			//如果存在且高度满足为货箱小货车 
			if (CabH>12 && CabH<16 && CabSt>1 && ContainH-CabH>3)//驾驶室高度2000-2200之间，车头帧数在6帧，防止客车混入
			{
				pVehicle->FeatureSet.IsContainer=pVehicle->FeatureSet.IsContainer|2;
			}
		
		}
	}
}

/************************************************************************/
/* 倒着搜寻车尾跳变                                                     */
/************************************************************************/
void FindTailJump(VehicleSet *pVehicle,uint8 nMode)
{
	int32 iIndex=0;

	int32 StaIndex=0;//检测的起始帧序号
	int32 EndIndex=0;//检查的终止帧序号
	uint16 HopperTh=1000;//车斗高度下限
	int32 iBreakPt=0;
	uint16 FitCnt=0;
	uint32 FrontHt=0;
	uint16 FallCnt=0;

	uint32 FallHt=0;
	int32 iIndex1=0;

	uint16 *pH=NULL; 
	uint8  nBreaks=0;

	//进行多轴车的简单纠错  20170803 客车车窗检测也有此纠正操作
	if (pVehicle->AxleSet.AxleSum>2 && pVehicle->EdgeSet.u16VehHeight<2200)
	{
		pVehicle->AxleSet.AxleSum=TWO_AXLE;
	}

	//不是两轴 或车高＜1450 或 帧数＜8 直接跳出不判断
	if (pVehicle->AxleSet.AxleSum>TWO_AXLE||
		pVehicle->EdgeSet.u16VehHeight<1450||pVehicle->u16FrameCnt<10)
	{
		return;
	}

	if (nMode==1)
	{
		pH=pVehicle->FrShape.u16LeftMaxHt;
	}
	else
	{
		pH=pVehicle->EdgeSet.u16MaxHeight;
	}


	//定位检查区间的起始位置和终止位置
	if (pVehicle->AxleSet.AxleSum>1)
	{
		StaIndex=MAX(pVehicle->AxleSet.Axle[0].AxleOriEndI+1,pVehicle->u16FrameCnt/3);
		EndIndex=pVehicle->u16FrameCnt-1;
	}
	else 
	{
		StaIndex=pVehicle->u16FrameCnt/3;
		EndIndex=pVehicle->u16FrameCnt-1;
	}


	//倒着寻找第一个大于1000的帧定为检查的终止帧
	for (iIndex=pVehicle->u16FrameCnt-1;iIndex>pVehicle->u16FrameCnt/2;iIndex--)
	{
		if (MIN_N_NEAR(&pH[iIndex],3,0)>HopperTh)
		{
			EndIndex=iIndex;
			break;
		}
	}


	//寻找高度间存在陡变大于350的帧
	for (iIndex=EndIndex;iIndex>StaIndex;iIndex--)
	{
		
		if (iIndex-3<2)
		{
			break;
		}
		else if (iIndex>pVehicle->u16FrameCnt-3)
		{
			//尾部跳过3帧
			continue;
		}

		//防止前部帧高异常 采用前三帧均值进行限制
		FrontHt=(pH[iIndex-1]+pH[iIndex-2]+pH[iIndex-3])/3;
		//if (FrontHt+100>pH[iIndex-1])
		//{
		//	FrontHt=pH[iIndex-1];
		//}

		//开始找高度陡降点
		if ((FrontHt+1>pH[iIndex-1]&&FrontHt-100<pH[iIndex-1])
			||(FrontHt<pH[iIndex-1]&&FrontHt+100>pH[iIndex-1]))
		{
			//前部和尾部均满足要求，则判定陡变350
			if (pVehicle->FrShape.u8DaFeiCnt[iIndex]<DAFEIMAXPT
				&&pH[iIndex]>0&&FrontHt>0&&pH[iIndex]+350<FrontHt)
			{
				iBreakPt=iIndex;
				nBreaks=1;
				break;
			}
			else if (pVehicle->FrShape.u8DaFeiCnt[iIndex]<3*DAFEIMAXPT
				&&pH[iIndex]>0&&FrontHt>0&&pH[iIndex]+500<FrontHt)
			{
				//20170804增加 适应性有待验证
				iBreakPt=iIndex;
				nBreaks=2;
				break;
			}
		}
		else
		{

			//前部多帧呈现由高到底趋势 与车斗缓降和客车尾部缓降类似
			FallHt=0;
			for(iIndex1=iIndex;iIndex1>StaIndex;iIndex1--)
			{
				//多连降 整体降幅达到300 帧帧降幅超100
				if (pH[iIndex1]>1000)
				{
					if (pH[iIndex1]+100<pH[iIndex1-1])
					{
						FallHt=pH[iIndex1-1]-pH[iIndex];
					}
					else
					{
						break;
					}
				}
				else
					break;
				
			}

			//排除鼓励点
			if (FallHt>300&&iIndex-iIndex1>1)
			{	
				iBreakPt=iIndex;
				break;
			}
		}
	}

	//没有找到合适的陡变点
	if (iBreakPt<StaIndex)
	{
		return;
	}

	//往前找5帧 确保不打飞 防止客车打飞造成假斗 误判 20170729
	for (iIndex=iBreakPt-1;iIndex>MAX(iBreakPt-6,6);iIndex--)
	{
		if (pVehicle->FrShape.u8DaFeiCnt[iIndex]>2*DAFEIMAXPT)
		{
			FitCnt++;
		}

		//打飞帧过多 直接退出
		if (FitCnt>2)
		{
			return;
		}
	}

	FitCnt=0;
	//往后寻找5帧看是否存在平坦或整体低于陡变点的帧 不检查会参杂进客车尾部2帧较低帧干扰
	for (iIndex=iBreakPt;iIndex<MIN(iBreakPt+6,pVehicle->u16FrameCnt-2);iIndex++)
	{
	
		if (pVehicle->FrShape.u8DaFeiCnt[iIndex]<DAFEIMAXPT
			&&(pH[iBreakPt]<pH[iIndex]+1&&pH[iBreakPt]+150>pH[iIndex])
			||(pH[iIndex]<pH[iBreakPt]&&pH[iIndex]+150>pH[iBreakPt]))
		{
			FitCnt++;
		}
		
		
		//else
		//{
		//	FitCnt=0;
		//}

		if (pH[iIndex]>pH[iIndex+1])
		{
			FallCnt++;
		}
	}

	//6帧里头有4帧是平坦的 基本是平坦的 + 增加高度逐步下降的判断 排除客车尾部停车(20170804)
	if (FitCnt>3&&FallCnt<5 && nBreaks==1)
	{
		pVehicle->FeatureSet.IsTailJump=pVehicle->FeatureSet.IsTailJump|1;
	}
	else if (FitCnt>1&&FallCnt<5 && nBreaks ==2)
	{
		pVehicle->FeatureSet.IsTailJump=pVehicle->FeatureSet.IsTailJump|1;
	}
}


/************************************************************************/
/* 车头跳变 区分客货                                                    */
/************************************************************************/
void HeadJumpCheckEx(VehicleSet *pVehicle,uint16 *pH)
{
	uint16 u16Index=0;
	uint16 u16StaI=0;
	uint16 u16EndI=0;
	uint16 u16RealIndex=0;
	uint16 u16HeadCastX=0;//车头的x边界值
	uint16 u16HeadEndI=0;//车头结束帧
	uint16 u16CheckCnt=0;//检查段帧数
	uint16 u16CheckMaxZ=0;//检查段最大高
	uint16 u16CheckMinZ=9999;//检查段最小高
	uint32 n32CheckSumZ=0;//检查段高之和
	uint16 u16CheckAvgZ=0;//检查段高均值
	uint16 u16CheckMaxX=0;//检查段边界x最大
	uint16 u16CheckMinX=9999;//检查段边界x最小
	uint32 n32CheckSumX=0;//检查段边界x之和
//	uint16 u16CheckAvgX=0;//检查段边界x均值

	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 10
		||pVehicle->EdgeSet.u16HeadHeight<1800||pVehicle->AxleSet.AxleSum<1)

		return ;


	u16StaI=pVehicle->AxleSet.Axle[0].AxleOriStaI;
	u16EndI=pVehicle->AxleSet.Axle[0].AxleOriEndI;
	u16HeadCastX=pVehicle->AxleSet.Axle[0].AxleCastX;


	if (u16StaI<1||u16StaI>pVehicle->u16FrameCnt/3)
	{
		//防止轴数是纠正得到的
		return;
	}

	for (u16Index=u16StaI;u16Index<u16EndI;u16Index++)
	{
		if (ABS(pH[u16Index]-pVehicle->EdgeSet.u16HeadHeight)<200)
		{
			u16HeadEndI=u16Index;
		}
		else
		{
			break;
		}
	}

	//计算与第一轴等长区间段的均值高度与最大最小高度 判断是否存在跳变
	if (u16EndI-u16StaI<10)
	{
		u16CheckCnt=pVehicle->u16FrameCnt-u16EndI;
	}
	else
	{
		u16CheckCnt=MIN(u16EndI-u16StaI,pVehicle->u16FrameCnt-u16EndI);
	}

	if (u16CheckCnt<2)
	{
		return;
	}

	for (u16Index=0;u16Index<u16CheckCnt;u16Index++)
	{
		u16RealIndex=u16Index+u16HeadEndI;
		if (pVehicle->EdgeSet.u16MinXEdge[u16RealIndex]<u16CheckMinX)
		{
			u16CheckMinX=pVehicle->EdgeSet.u16MinXEdge[u16RealIndex];
		}
		if (pVehicle->EdgeSet.u16MinXEdge[u16RealIndex]>u16CheckMaxX)
		{
			u16CheckMaxX=pVehicle->EdgeSet.u16MinXEdge[u16RealIndex];
		}

		n32CheckSumX+=pVehicle->EdgeSet.u16MinXEdge[u16RealIndex];


		if (pH[u16RealIndex]<u16CheckMinZ)
		{
			u16CheckMinZ=pH[u16RealIndex];
		}

		if (pH[u16RealIndex]>u16CheckMaxZ)
		{
			u16CheckMaxZ=pH[u16RealIndex];
		}

		n32CheckSumZ+=pH[u16RealIndex];
	}

//	u16CheckAvgX=n32CheckSumX/u16CheckCnt;
	u16CheckAvgZ=n32CheckSumZ/u16CheckCnt;


	if ((pVehicle->EdgeSet.u16HeadHeight+200<u16CheckAvgZ||pVehicle->EdgeSet.u16HeadHeight+350<u16CheckMaxZ)
		&& u16HeadCastX>u16CheckMinX)
	{
		pVehicle->FeatureSet.IsHeadJump|=2;
	}
	else if ((pVehicle->EdgeSet.u16HeadHeight-250>u16CheckAvgZ||pVehicle->EdgeSet.u16HeadHeight-200>u16CheckMaxZ)
		&& u16HeadCastX>u16CheckMinX)
	{
		pVehicle->FeatureSet.IsHeadJump|=4;
	}

}


/************************************************************************/
/* 车头跳变检测  区分客货20170803                                       */
/************************************************************************/
void FindHeadJump(VehicleSet *pVehicle,uint8 nMode)
{
	int16 i16Index=0;
	int16 i16IndexJ=0;
//	uint16 u16StaI=0;
//	uint16 u16EndI=0;

	int16 ContainSt=0;
	int16 ContainEd=0;
	int16  CabSt=0;
	int16  CabEd=0;
	uint16 CabHmax=0;
	uint32 CabSumH=0;
	int16 CabCnt=0;
//	int16 u16TmpI=0;


//	uint16 CabHst=0;//驾驶最前高度
//	uint16 CabFlat=0;//驾驶室平坦帧
//	uint16 FlatHt=0;
//	uint32 CabFrH=0;//驾驶室前一半均高
//	uint32 CabBaH=0;//驾驶室后一半均高

	uint16 PassMaxHt=0;//过渡段最大高
	uint16 PassMinHt=0;//过渡段最小高
	uint32 PassAvgHt=0;//过渡段均值
	uint32 PassCnt=0;//过渡段帧数

	uint16 *pH=NULL;
	uint16 u16VehHeight=0;
	uint16 u16MaxIndx=0;

	//准入条件 
	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 10
		||pVehicle->EdgeSet.u16HeadHeight>2500||pVehicle->EdgeSet.u16HeadHeight<1000
		||pVehicle->EdgeSet.u16VehHeight<1800)

		return ;


	//根据输入的不同 自动选取
	if (nMode==1)
	{
		pH=pVehicle->FrShape.u16LeftMaxHt;
		CalcVehHeight(pVehicle->FrShape.u16LeftMaxHt,pVehicle->u16FrameCnt,&u16VehHeight,&u16MaxIndx);
	}
	else
	{
		pH=pVehicle->EdgeSet.u16MaxHeight;
		u16VehHeight=pVehicle->EdgeSet.u16VehHeight;
	}

	if (pH==NULL)
	{
		return;
	}


	//找到车高平坦段
	for (i16Index=pVehicle->u16FrameCnt-1;i16Index>pVehicle->u16FrameCnt/3;i16Index--)
	{
		if (pH[i16Index]+150>u16VehHeight && ContainEd<1)
		{
			ContainEd=i16Index;
		}
		else 
		{
			if (ContainSt<1)
			{
				if (pH[i16Index]+150>u16VehHeight)
				{
					ContainSt=i16Index;
				}
			}
			else
			{
				if (pH[i16Index]+150>u16VehHeight)
				{
					ContainSt=i16Index;
				}
				else
				{
					break;
				}
			}
		}
	}

	//往前寻找高度平坦，且存在高度降的部位--比如驾驶室或较低的货箱或栅栏段
	//是否存在驾驶室及驾驶室高度
	CabEd=ContainSt-1;//初始化驾驶室终点
	CabSt=CabEd;//初始化驾驶室起点
	for (i16Index=CabEd;i16Index>5;i16Index--)
	{
		if (pH[i16Index]<1200)
		{
			//低于直接退出
			break;
		}

		//寻找驾驶室
		if (pH[ContainSt]-pH[i16Index]>200 
			&&((pH[i16Index]+1>pH[CabEd]
		&&pH[CabEd]+150>pH[i16Index])
			||(pH[i16Index]<pH[CabEd]
		&&pH[i16Index]+150>pH[CabEd])))
		{
			//与驾驶室终点高度在150以内
			CabSumH+=pH[i16Index];
			CabCnt++;
			if (CabSt>i16Index)
			{
				CabSt=i16Index;
			}
			if (CabHmax<pH[i16Index])
			{
				CabHmax=pH[i16Index];
			}
		}
		else
		{
			if (CabHmax>0)
			{
				//判断是否满足驾驶室平坦要求
				if (CabCnt<5)
				{	//重新寻找驾驶室
					CabEd=i16Index;
					CabSt=i16Index;
					CabHmax=0;
					CabSumH=0;
					CabCnt=0;
				}
				else
				{
					//判断是否为平坦驾驶室
					if (ContainSt-CabEd>3)
					{
						//存在过渡段 计算最高 最低 均值 判断是否为平坦过渡 判处客车
						PassMaxHt=0;//过渡段最大高
						PassMinHt=0;//过渡段最小高
						PassAvgHt=0;//过渡段均值
						PassCnt=0;//过渡段帧数
						for (i16IndexJ=CabEd+1;i16IndexJ<ContainSt;i16IndexJ++)
						{
							if (pH[i16IndexJ]>PassMaxHt)
							{
								PassMaxHt=pH[i16IndexJ];
							}

							if (PassMinHt==0)
							{
								PassMinHt=pH[i16IndexJ];
							}
							else if (pH[i16IndexJ]<PassMinHt)
							{
								PassMinHt=pH[i16IndexJ];
							}

							PassAvgHt+=pH[i16IndexJ];

							PassCnt++;

						}

						PassAvgHt=PassAvgHt/PassCnt;

						if(PassMaxHt>0&&(PassMaxHt-PassAvgHt<100 || PassAvgHt-PassMinHt<100) && PassAvgHt>2100)
						{
							//过渡段为平坦的 且 均高在2100以上  排除金杯等干扰
							pVehicle->FeatureSet.IsHeadJump=pVehicle->FeatureSet.IsHeadJump|2;
							break;
						}
						else if (PassAvgHt<1700)
						{
							break;
						}
						else
						{
							CabEd=i16Index;
							CabSt=i16Index;
						}
					}
					else
					{
						//不存在过渡段 直接判定
						if (CabSt<pVehicle->u16FrameCnt/2&&CabSumH/CabCnt>1600)
						{
							pVehicle->FeatureSet.IsHeadJump=pVehicle->FeatureSet.IsHeadJump|1;
							break;
						}
					}
				}
			}
			else
			{
				CabEd=i16Index;
				CabSt=i16Index;
			}
		}
	}
}



/************************************************************************/
/* 车头跳变 区分客货                                                    */
/************************************************************************/
void HeadJumpCheck(VehicleSet *pVehicle)
{
	int16 i16Index=0;
	int16 i16IndexJ=0;
//	uint16 u16StaI=0;
//	uint16 u16EndI=0;
	
	int16 ContainSt=0;
	int16 ContainEd=0;
	int16  CabSt=0;
	int16  CabEd=0;
	uint16 CabHmax=0;
	uint32 CabSumH=0;
	int16 CabCnt=0;
//	int16 u16TmpI=0;


//	uint16 CabHst=0;//驾驶最前高度
//	uint16 CabFlat=0;//驾驶室平坦帧
//	uint16 FlatHt=0;
//	uint32 CabFrH=0;//驾驶室前一半均高
//	uint32 CabBaH=0;//驾驶室后一半均高

	uint16 PassMaxHt=0;//过渡段最大高
	uint16 PassMinHt=0;//过渡段最小高
	uint32 PassAvgHt=0;//过渡段均值
	uint32 PassCnt=0;//过渡段帧数


	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 10
		||pVehicle->EdgeSet.u16HeadHeight>2500||pVehicle->EdgeSet.u16HeadHeight<1000||pVehicle->EdgeSet.u16VehHeight<1800)

		return ;

	//找到车高平坦段
	for (i16Index=pVehicle->u16FrameCnt-1;i16Index>pVehicle->u16FrameCnt/3;i16Index--)
	{
		if (pVehicle->EdgeSet.u16MaxHeight[i16Index]+150>pVehicle->EdgeSet.u16VehHeight && ContainEd<1)
		{
			ContainEd=i16Index;
		}
		else 
		{
			if (ContainSt<1)
			{
				if (pVehicle->EdgeSet.u16MaxHeight[i16Index]+150>pVehicle->EdgeSet.u16VehHeight)
				{
					ContainSt=i16Index;
				}
			}
			else
			{
				if (pVehicle->EdgeSet.u16MaxHeight[i16Index]+150>pVehicle->EdgeSet.u16VehHeight)
				{
					ContainSt=i16Index;
				}
				else
				{
					break;
				}
			}
		}
	}

	//往前寻找高度平坦，且存在高度降的部位--比如驾驶室或较低的货箱或栅栏段
	//是否存在驾驶室及驾驶室高度
	CabEd=ContainSt-1;//初始化驾驶室终点
	CabSt=CabEd;//初始化驾驶室起点
	for (i16Index=CabEd;i16Index>5;i16Index--)
	{
		if (pVehicle->EdgeSet.u16MaxHeight[i16Index]<1200)
		{
			//低于直接退出
			break;
		}

		//寻找驾驶室
		if (pVehicle->EdgeSet.u16MaxHeight[ContainSt]-pVehicle->EdgeSet.u16MaxHeight[i16Index]>200 
			&&((pVehicle->EdgeSet.u16MaxHeight[i16Index]+1>pVehicle->EdgeSet.u16MaxHeight[CabEd]
		&&pVehicle->EdgeSet.u16MaxHeight[CabEd]+150>pVehicle->EdgeSet.u16MaxHeight[i16Index])
			||(pVehicle->EdgeSet.u16MaxHeight[i16Index]<pVehicle->EdgeSet.u16MaxHeight[CabEd]
		&&pVehicle->EdgeSet.u16MaxHeight[i16Index]+150>pVehicle->EdgeSet.u16MaxHeight[CabEd])))
		{
			//与驾驶室终点高度在150以内
			CabSumH+=pVehicle->EdgeSet.u16MaxHeight[i16Index];
			CabCnt++;
			if (CabSt>i16Index)
			{
				CabSt=i16Index;
			}
			if (CabHmax<pVehicle->EdgeSet.u16MaxHeight[i16Index])
			{
				CabHmax=pVehicle->EdgeSet.u16MaxHeight[i16Index];
			}
		}
		else
		{
			if (CabHmax>0)
			{
				//判断是否满足驾驶室平坦要求
				if (CabCnt<5)
				{	//重新寻找驾驶室
					CabEd=i16Index;
					CabSt=i16Index;
					CabHmax=0;
					CabSumH=0;
					CabCnt=0;
				}
				else
				{
					//判断是否为平坦驾驶室
					if (ContainSt-CabEd>3)
					{
						//存在过渡段 计算最高 最低 均值 判断是否为平坦过渡 判处客车
						PassMaxHt=0;//过渡段最大高
						PassMinHt=0;//过渡段最小高
						PassAvgHt=0;//过渡段均值
						PassCnt=0;//过渡段帧数
						for (i16IndexJ=CabEd+1;i16IndexJ<ContainSt;i16IndexJ++)
						{
							if (pVehicle->EdgeSet.u16MaxHeight[i16IndexJ]>PassMaxHt)
							{
								PassMaxHt=pVehicle->EdgeSet.u16MaxHeight[i16IndexJ];
							}
							
							if (PassMinHt==0)
							{
								PassMinHt=pVehicle->EdgeSet.u16MaxHeight[i16IndexJ];
							}
							else if (pVehicle->EdgeSet.u16MaxHeight[i16IndexJ]<PassMinHt)
							{
								PassMinHt=pVehicle->EdgeSet.u16MaxHeight[i16IndexJ];
							}

							PassAvgHt+=pVehicle->EdgeSet.u16MaxHeight[i16IndexJ];

							PassCnt++;

						}

						PassAvgHt=PassAvgHt/PassCnt;

						
							if(PassMaxHt>0&&(PassMaxHt-PassAvgHt<100 || PassAvgHt-PassMinHt<100) && PassAvgHt>2100)
						{
							//过渡段为平坦的 且 均高在2100以上  排除金杯等干扰
							pVehicle->FeatureSet.IsHeadJump=pVehicle->FeatureSet.IsHeadJump|2;
							break;
						}
						else if (PassAvgHt<1700)
						{
							break;
						}
						else
						{
							CabEd=i16Index;
							CabSt=i16Index;
						}
					}
					else
					{
						//不存在过渡段 直接判定
						if (CabSt<pVehicle->u16FrameCnt/2&&CabSumH/CabCnt>1600)
						{
							pVehicle->FeatureSet.IsHeadJump=pVehicle->FeatureSet.IsHeadJump|1;
							break;
						}
					}
				}
			}
			else
			{
				CabEd=i16Index;
				CabSt=i16Index;
			}
		}
	}
}

/************************************************************************/
/* 每帧外形类型检测 20170803改进版                                      */
/************************************************************************/
void FramShapeCal(VehicleSet *pVehicle)
{
	uint8  TmpFlag=0;
	uint16 CurFramI=0;
	int16 i16PtSum=0;
	int16 i16Index=0;

//	uint16 ThreshXZ=50;
//	uint16 ThreshHt=800;
//	int16 iOutLinePt=0;//轮廓线点
//	int16 iThreshPt=0;//达到阈值的第一个点
//	int16 FitCnt=0;

//	uint16 BaseX=0;//边界X基

	int16 MaxHt=0;
	int16 MaxHI=0;
	int16 MinHt=9999;
	int16 MinHI=0;
	int16 AvgHt=0;
	int16 AvgCnt=0;

//	int32 EdgeZ[FRAME_BUFLEN]={0};
//	int32 EdgeX[FRAME_BUFLEN]={0};

//	int16 i16IndexJ=0;
//	int16 TmpCnt=0;
//	int32 TmpSum=0;
//	int32 PredictX=0;

	uint16 LoDaFeiCnt=0;
	uint16 HiDaFeiCnt=0;


	uint32 BlockMap[2][15]={0};

	if (pVehicle  ==  NULL)
	{
		return;
	}
	//提取每帧的轮廓值
	CurFramI=pVehicle->u16FrameCnt-1;
	i16PtSum=pVehicle->CurRegion.xdata[0];

	//操作- 检查1m以下和1m以上的连续打飞点个数
	for (i16Index=i16PtSum;i16Index>1;i16Index--)
	{
		if(pVehicle->CurRegion.zdata[i16Index]>1000&&TmpFlag<1)
		{
			TmpFlag=1;
		}
		else
		{
			if (TmpFlag)
			{
				if (pVehicle->CurRegion.zdata[i16Index]<1&&pVehicle->CurRegion.zdata[i16Index]<1)
				{
					HiDaFeiCnt++;
					
				}
			}
			else
			{
				if (pVehicle->CurRegion.zdata[i16Index]<1&&pVehicle->CurRegion.zdata[i16Index]<1)
				{
					LoDaFeiCnt++;
				}
			}
		}
	}
	//记录打飞情况
	if (LoDaFeiCnt>2*DAFEIMAXPT)
	{
		pVehicle->FrShape.FrLoDFCnt++;
	}
	if (HiDaFeiCnt>2*DAFEIMAXPT)
	{
		pVehicle->FrShape.FrHiDFCnt++;
	}
	pVehicle->FrShape.u8DaFeiCnt[CurFramI]=HiDaFeiCnt;
	pVehicle->FrShape.LoDaFeiR=pVehicle->FrShape.FrLoDFCnt*100/pVehicle->u16FrameCnt;
	pVehicle->FrShape.HiDaFeiR=pVehicle->FrShape.FrHiDFCnt*100/pVehicle->u16FrameCnt;

	//操作二 截取侧面20CM以内的点 形成切面
	if (pVehicle->AxleSet.AxleSum>0)
	{
		if (ABS(pVehicle->CurRegion.xMin-pVehicle->FrShape.BaseX)<50)
		{
			pVehicle->FrShape.BaseX=pVehicle->CurRegion.xMin;
		}
	}
	else
	{
		pVehicle->FrShape.BaseX=pVehicle->CurRegion.xMin;
	}

	for (i16Index=i16PtSum;i16Index>0;i16Index--)
	{
		if (pVehicle->CurRegion.zdata[i16Index]<VEHTH_LOW)
		{
			//防止中间投掷到地上拉低高度引起外形特性的误判 该外形是基于外表面轮廓线所形成
			continue;
		}
		if (pVehicle->CurRegion.xdata[i16Index]<pVehicle->FrShape.BaseX+300)
		{
			if (pVehicle->CurRegion.zdata[i16Index]>pVehicle->FrShape.u16LeftMaxHt[CurFramI])
			{
				pVehicle->FrShape.u16LeftMaxHt[CurFramI]=pVehicle->CurRegion.zdata[i16Index];
			}
			BlockMap[0][0]++;
			//由于第一个区 其左侧能扫描到车辆底部 则取最大高代替
			BlockMap[1][0]=pVehicle->FrShape.u16LeftMaxHt[CurFramI]*BlockMap[0][0];
		}
		else 
		{
			BlockMap[0][ABS(pVehicle->CurRegion.xdata[i16Index]-pVehicle->FrShape.BaseX)/300+1]++;
			BlockMap[1][ABS(pVehicle->CurRegion.xdata[i16Index]-pVehicle->FrShape.BaseX)/300+1]+=pVehicle->CurRegion.zdata[i16Index];
		}
	}
	
	//考察每个块的高度，及整体的平、升、降等特性
	for (i16Index=0;i16Index<15;i16Index++)
	{
		if (BlockMap[0][i16Index]>0)
		{
			BlockMap[1][i16Index]=BlockMap[1][i16Index]/BlockMap[0][i16Index];
			if (BlockMap[1][i16Index]<MinHt)
			{
				MinHt=BlockMap[1][i16Index];
				MinHI=i16Index;
			}

			if (BlockMap[1][i16Index]>MaxHt)
			{
				MaxHt=BlockMap[1][i16Index];
				MaxHI=i16Index;
			}

			AvgHt+=BlockMap[1][i16Index];
			AvgCnt++;
		}
	}

	if (AvgCnt>1)
	{
		AvgHt=AvgHt/AvgCnt;

		if (MaxHt-MinHt>300&&MaxHI>MinHI)
		{
			//左低右高  可以是载货
			pVehicle->FrShape.u16FrShape[CurFramI]=3;
		}
		else if (MaxHt-MinHt>300&&MaxHI<MinHI)
		{
			//左高右低 可以是载货 或者是 车窗打进去
			pVehicle->FrShape.u16FrShape[CurFramI]=4;
		}
		else
		{
			if (MaxHt-AvgHt<100||AvgHt-MinHt<100)
			{
				//侧面竖直 宽度方向平坦 横竖结构 为车体
				pVehicle->FrShape.u16FrShape[CurFramI]=2;
			}
		}
	}
	else
	{
		pVehicle->FrShape.u16FrShape[CurFramI]=1;//侧面为一平板
	}

}


/************************************************************************/
/* 每帧外形类型检测                                                     */
/************************************************************************/
void FramShapeCalEx(VehicleSet *pVehicle)
{
	uint16 CurFramI=0;
	int16 i16PtSum=0;
	int16 i16Index=0;

	uint16 ThreshXZ=50;
	uint16 ThreshHt=800;
	int16 iOutLinePt=0;//轮廓线点
	int16 iThreshPt=0;//达到阈值的第一个点
	int16 FitCnt=0;

	uint16 BaseX=0;//边界X基

	int16 MaxHt=0;
	int16 MaxHI=0;
	int16 MinHt=9999;
	int16 MinHI=0;
	int16 AvgHt=0;
	int16 AvgCnt=0;

	int32 EdgeZ[FRAME_BUFLEN]={0};
	int32 EdgeX[FRAME_BUFLEN]={0};

	int16 i16IndexJ=0;
	int16 TmpCnt=0;
	int32 TmpSum=0;
	int32 PredictX=0;

//	uint16 DaFeiCnt=0;

	uint32 BlockMap[2][15]={0};

	if (pVehicle  ==  NULL)
	{
		return;
	}
	//提取每帧的轮廓值
	CurFramI=pVehicle->u16FrameCnt-1;
	i16PtSum=pVehicle->CurRegion.xdata[0];

	//拷贝坐标序列
	memcpy(EdgeX,pVehicle->CurRegion.xdata,sizeof(pVehicle->CurRegion.xdata));
	memcpy(EdgeZ,pVehicle->CurRegion.zdata,sizeof(pVehicle->CurRegion.xdata));


	if (pVehicle->CurFrmInfo.u16MaxHt<1000)
	{
		ThreshHt=600;
	}
	else
	{
		ThreshHt=800;
	}

	//第一步 寻找侧面底部ThreshHt第一个点 认为该点以下为外廓线部分
	for (i16Index=i16PtSum;i16Index>1;i16Index--)
	{
		if (EdgeZ[i16Index]>ThreshHt)
		{
			iThreshPt=i16Index;
			iOutLinePt=i16Index;
			break;
		}
	}


	//操作- 检查1m以上的连续打飞点个数
	for (i16Index=iThreshPt;i16Index>1;i16Index--)
	{
		if (EdgeZ[i16Index]<1&&EdgeZ[i16Index-1]<1)
		{
			pVehicle->FrShape.u8DaFeiCnt[CurFramI]++;
		}
	}

	//操作二 进行每帧的解剖

	//第二步 寻找到外边缘寻找的起点
	for (i16Index=iThreshPt;i16Index>1;i16Index--)
	{
		if (EdgeZ[i16Index-1]>0&&EdgeZ[i16Index]>0)
		{
			if (ABS(EdgeZ[i16Index-1]-EdgeZ[i16Index]<ThreshXZ)
				&&ABS(EdgeX[i16Index-1]-EdgeX[i16Index]<ThreshXZ))
			{
				iOutLinePt=i16Index;
				iThreshPt=i16Index;
				FitCnt++;
			}
			else
			{
				FitCnt=0;

			}
		}

		if (FitCnt>5)
		{
			break;
		}
	}

	//第三步进行预测和边缘线的生长
	for (i16Index=iOutLinePt;i16Index>0;i16Index--)
	{
		if (EdgeZ[i16Index]>0)
		{
			TmpSum=0;
			TmpCnt=0;
			PredictX=0;
			for (i16IndexJ=i16Index;i16IndexJ<iThreshPt;i16IndexJ++)
			{
				if (EdgeZ[i16IndexJ]>0)
				{
					TmpSum+=EdgeX[i16IndexJ];
					TmpCnt++;
				}
				if (TmpCnt>4)
				{
					break;
				}

			}
			if (TmpCnt>0)
			{
				PredictX=TmpSum/TmpCnt;
			}
			else
			{
				PredictX=EdgeZ[i16Index+1];
			}

			if (ABS(PredictX-EdgeX[i16Index])<100)
			{
				if (EdgeZ[i16Index]-600<pVehicle->CurRegion.zdata[iOutLinePt])
				{
					iOutLinePt=i16Index;
				}
				else
				{
					break;
				}
			}
			else 
			{
				if (ABS(PredictX-EdgeX[i16Index])<200&&EdgeZ[i16Index]>pVehicle->CurRegion.zdata[iOutLinePt])
				{
					if (EdgeZ[i16Index]-600<pVehicle->CurRegion.zdata[iOutLinePt])
					{
						iOutLinePt=i16Index;
					}
					else
					{
						break;
					}
				}
				EdgeZ[i16Index]=0;
				EdgeX[i16Index]=0;
			}
		}
	}



	//记录轮廓点
	if (iOutLinePt>0&&iOutLinePt<i16PtSum)
	{
		pVehicle->FrShape.u16LeftMaxHt[CurFramI]=pVehicle->CurRegion.zdata[iOutLinePt];
		BaseX=pVehicle->CurRegion.xdata[iOutLinePt];
	}
	else
	{
		return;
	}

	//按20cm为一块 进行分块处理
	for (i16Index=iOutLinePt-1;i16Index>0;i16Index--)
	{
		if (pVehicle->CurRegion.zdata[i16Index]>0&&ABS(pVehicle->CurRegion.xdata[i16Index]-BaseX)<3000)
		{
			BlockMap[0][ABS(pVehicle->CurRegion.xdata[i16Index]-BaseX)/200]++;
			BlockMap[1][ABS(pVehicle->CurRegion.xdata[i16Index]-BaseX)/200]+=pVehicle->CurRegion.zdata[i16Index];
		}
	}


	//考察每个块的高度，及整体的平、升、降等特性
	for (i16Index=0;i16Index<15;i16Index++)
	{
		if (BlockMap[0][i16Index]>0)
		{
			BlockMap[1][i16Index]=BlockMap[1][i16Index]/BlockMap[0][i16Index];
			if (BlockMap[1][i16Index]<MinHt)
			{
				MinHt=BlockMap[1][i16Index];
				MinHI=i16Index;
			}

			if (BlockMap[1][i16Index]>MaxHt)
			{
				MaxHt=BlockMap[1][i16Index];
				MaxHI=i16Index;
			}

			AvgHt+=BlockMap[1][i16Index];
			AvgCnt++;
		}
	}

	if (AvgCnt>0)
	{
		AvgHt=AvgHt/AvgCnt;
	}

	if (MaxHt-MinHt>300&&MaxHI>MinHI)
	{

		//左低右高
		pVehicle->FrShape.u16FrShape[CurFramI]=3;
	}
	else if (MaxHt-MinHt>300&&MaxHI<MinHI)
	{
		//左高右低
		pVehicle->FrShape.u16FrShape[CurFramI]=4;
	}
	else
	{
		if (MaxHt-AvgHt<100||AvgHt-MinHt<100)
		{
			pVehicle->FrShape.u16FrShape[CurFramI]=2;
		}
	}
}



///************************************************************************/
///* 每帧外形类型检测                                                     */
///************************************************************************/
//void FramShapeCal(VehicleSet *pVehicle)
//{
//	uint16 CurFramI=0;
//	int16 i16PtSum=0;
//	int16 i16Index=0;
//	
//	uint16 ThreshXZ=50;
//	uint16 ThreshHt=800;
//	int16 iLeftLinePt=0;//轮廓线点
//	int16 iThreshPt=0;//达到阈值的第一个点
//	int16 FitCnt=0;
//
//	uint16 BaseX=0;//边界X基
//
//	int16 MaxHt=0;
//	int16 MaxHI=0;
//	int16 MinHt=9999;
//	int16 MinHI=0;
//	int16 AvgHt=0;
//	int16 AvgCnt=0;
//
//	int32 EdgeZ[FRAME_BUFLEN]={0};
//	int32 EdgeX[FRAME_BUFLEN]={0};
//	
//	int16 i16IndexJ=0;
//	int16 TmpCnt=0;
//	int32 TmpSum=0;
//	int32 PredictX=0;
//
//	uint16 DaFeiCnt=0;
//
//	uint32 BlockMap[2][15]={0};
//
//	if (pVehicle  ==  NULL)
//	{
//		return;
//	}
//	//提取每帧的轮廓值
//	CurFramI=pVehicle->u16FrameCnt-1;
//	i16PtSum=pVehicle->CurRegion.xdata[0];
//	
//	//拷贝坐标序列
//	memcpy(EdgeX,pVehicle->CurRegion.xdata,sizeof(pVehicle->CurRegion.xdata));
//	memcpy(EdgeZ,pVehicle->CurRegion.zdata,sizeof(pVehicle->CurRegion.xdata));
//
//
//	if (pVehicle->CurFrmInfo.u16MaxHt<1000)
//	{
//		ThreshHt=600;
//	}
//	else
//	{
//		ThreshHt=800;
//	}
//
//	
//
//
//
//
//
//	//第一步 寻找侧面底部ThreshHt第一个点 认为该点以下为外廓线部分
//	for (i16Index=i16PtSum;i16Index>1;i16Index--)
//	{
//		if (EdgeZ[i16Index]>ThreshHt)
//		{
//			iThreshPt=i16Index;
//			iLeftLinePt=i16Index;
//			break;
//		}
//	}
//
//
//	//操作- 检查1m以上的连续打飞点个数
//	for (i16Index=iThreshPt;i16Index>1;i16Index--)
//	{
//		if (EdgeZ[i16Index]<1&&EdgeZ[i16Index-1]<1)
//		{
//			pVehicle->FrShape.u8DaFeiCnt[CurFramI]++;
//		}
//	}
//
//	//操作二 进行每帧的解剖
//
//	//第二步 寻找到外边缘寻找的起点
//	for (i16Index=iThreshPt;i16Index>1;i16Index--)
//	{
//		if (EdgeZ[i16Index-1]>0&&EdgeZ[i16Index]>0)
//		{
//			if (ABS(EdgeZ[i16Index-1]-EdgeZ[i16Index]<ThreshXZ)
//				&&ABS(EdgeX[i16Index-1]-EdgeX[i16Index]<ThreshXZ))
//			{
//				iLeftLinePt=i16Index;
//				FitCnt++;
//			}
//			else
//			{
//				FitCnt=0;
//
//			}
//		}
//		
//		if (FitCnt>5)
//		{
//			break;
//		}
//	}
//
//	//第三步进行预测和边缘线的生长
//	for (i16Index=iLeftLinePt;i16Index>0;i16Index--)
//	{
//		if (EdgeZ[i16Index]>0)
//		{
//			TmpSum=0;
//			TmpCnt=0;
//			PredictX=0;
//			for (i16IndexJ=i16Index;i16IndexJ<iThreshPt;i16IndexJ++)
//			{
//				if (EdgeZ[i16IndexJ]>0)
//				{
//					TmpSum+=EdgeX[i16IndexJ];
//					TmpCnt++;
//				}
//				if (TmpCnt>4)
//				{
//					break;
//				}
//				
//			}
//			if (TmpCnt>0)
//			{
//				PredictX=TmpSum/TmpCnt;
//			}
//			else
//			{
//				PredictX=EdgeZ[i16Index+1];
//			}
//
//			if (ABS(PredictX-EdgeX[i16Index])<100)
//			{
//				iLeftLinePt=i16Index;
//			}
//			else 
//			{
//				if (ABS(PredictX-EdgeX[i16Index])<200)
//				{
//					iLeftLinePt=i16Index;
//				}
//				EdgeZ[i16Index]=0;
//				EdgeX[i16Index]=0;
//			}
//		}
//	}
//
//
//
//	//记录轮廓点
//	if (iLeftLinePt>0&&iLeftLinePt<i16PtSum)
//	{
//		pVehicle->FrShape.u16LeftMaxHt[CurFramI]=pVehicle->CurRegion.zdata[iLeftLinePt];
//		BaseX=pVehicle->CurRegion.xdata[iLeftLinePt];
//	}
//	else
//	{
//		return;
//	}
//
//	//按20cm为一块 进行分块处理
//	for (i16Index=iLeftLinePt-1;i16Index>0;i16Index--)
//	{
//		if (pVehicle->CurRegion.zdata[i16Index]>0&&ABS(pVehicle->CurRegion.xdata[i16Index]-BaseX)<3000)
//		{
//			BlockMap[0][ABS(pVehicle->CurRegion.xdata[i16Index]-BaseX)/200]++;
//			BlockMap[1][ABS(pVehicle->CurRegion.xdata[i16Index]-BaseX)/200]+=pVehicle->CurRegion.zdata[i16Index];
//		}
//	}
//
//
//	//考察每个块的高度，及整体的平、升、降等特性
//	for (i16Index=0;i16Index<15;i16Index++)
//	{
//		if (BlockMap[0][i16Index]>0)
//		{
//			BlockMap[1][i16Index]=BlockMap[1][i16Index]/BlockMap[0][i16Index];
//			if (BlockMap[1][i16Index]<MinHt)
//			{
//				MinHt=BlockMap[1][i16Index];
//				MinHI=i16Index;
//			}
//
//			if (BlockMap[1][i16Index]>MaxHt)
//			{
//				MaxHt=BlockMap[1][i16Index];
//				MaxHI=i16Index;
//			}
//
//			AvgHt+=BlockMap[1][i16Index];
//			AvgCnt++;
//		}
//	}
//
//	if (AvgCnt>0)
//	{
//		AvgHt=AvgHt/AvgCnt;
//	}
//
//	if (MaxHt-MinHt>300&&MaxHI>MinHI)
//	{
//		
//		//左低右高
//		pVehicle->FrShape.u16FrShape[CurFramI]=3;
//	}
//	else if (MaxHt-MinHt>300&&MaxHI<MinHI)
//	{
//		//左高右低
//		pVehicle->FrShape.u16FrShape[CurFramI]=4;
//	}
//	else
//	{
//		if (MaxHt-AvgHt<100||AvgHt-MinHt<100)
//		{
//			pVehicle->FrShape.u16FrShape[CurFramI]=2;
//		}
//	}
//}


/************************************************************************/
/* 掷地检测区分货车                                                     */
/************************************************************************/
void CastGrdCheckCal(VehicleSet *pVehicle)
{
//	uint8 LowFixed=0;//底部锁定
//	uint8 TopFixed=0;//顶部锁定
	int16 i16Index=0;
//	uint16 u16StaI=0;
//	uint16 u16EndI=0;
	uint16 CurFramI=0;
	int16  i16PtSum=0;//当前帧点数
//	uint16 u16RealIndex=0;
//	uint16 u16CastThresh=520;//投掷到地上高度阈值
	uint16 u16EdgeX0=0;//车身边界x0起始位置
//	uint16 u16HeadEndI=0;//车头结束帧
//	uint16 u16CheckCnt=0;//检查段帧数
//	uint16 u16CheckMaxZ=0;//检查段最大高
//	uint16 u16CheckMinZ=9999;//检查段最小高
//	uint32 n32CheckSumZ=0;//检查段高之和
//	uint16 u16CheckAvgZ=0;//检查段高均值
//	uint16 u16CheckMaxX=0;//检查段边界x最大
//	uint16 u16CheckMinX=9999;//检查段边界x最小
//	uint32 n32CheckSumX=0;//检查段边界x之和
//	uint16 u16CheckAvgX=0;//检查段边界x均值

	uint16 FitPtCnt=0;
	


	if (pVehicle  ==  NULL)
	{
		return;
	}
	//提取每帧的轮廓值
	CurFramI=pVehicle->u16FrameCnt-1;
	i16PtSum=pVehicle->CurRegion.xdata[0];
	

	//进行掷地检测  显判断异常情况
	if (pVehicle->AxleSet.AxleSum<1||pVehicle->u16FrameCnt<8||i16PtSum<8)
	{
		//少于8个点直接退出去
		return;
	}
	
	u16EdgeX0=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleCastX;
	//遍历所有点 截取左侧30cm内的界面，取左上点为轮廓点

	//遍历高度800以下的所有点  标记是否中间投掷到地上
	for (i16Index=i16PtSum;i16Index>1;i16Index--)
	{
		if (pVehicle->CurRegion.zdata[i16Index]>VEHTH_LOW)
		{
			FitPtCnt++;
		}
		else if (FitPtCnt>1&& pVehicle->CurRegion.zdata[i16Index]<VEHTH_LOW 
			&& pVehicle->CurRegion.xdata[i16Index]>u16EdgeX0+300)
		{
			pVehicle->EdgeSet.u8CastGrdFlag[CurFramI]++;
		}
		else if (pVehicle->CurRegion.zdata[i16Index]>800)
		{
			break;
		}
	}

}



/************************************************************************/
/*x方向投影凸出物检查载货                                               */
/************************************************************************/
void CargoXEdgeCheck(VehicleSet *pVehicle)
{
	int16 i16Index=0;
	uint16 u16StaI=0;
	uint16 u16EndI=0;
//	uint16 u16RealIndex=0;
	uint16 u16TailCastX=0;//车尾的x边界值
//	uint16 u16HeadEndI=0;//车头结束帧
	uint16 u16CheckCnt=0;//检查段帧数
//	uint16 u16CheckMaxZ=0;//检查段最大高
//	uint16 u16CheckMinZ=9999;//检查段最小高
//	uint32 n32CheckSumZ=0;//检查段高之和
//	uint16 u16CheckAvgZ=0;//检查段高均值
	uint16 u16CheckMaxX=0;//检查段边界x最大
	uint16 u16CheckMinX=9999;//检查段边界x最小
	uint32 n32CheckSumX=0;
	uint16 u16CheckAvgX=0;

	//至少一个轴 1550皮卡的最低高度
	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT 
		|| pVehicle->u16FrameCnt < 10||pVehicle->AxleSet.AxleSum<1
		||pVehicle->EdgeSet.u16VehHeight<1550) 

		return ;

	if(pVehicle->AxleSet.Axle[1].AxleOriEndI>0)
	{
		u16TailCastX=pVehicle->AxleSet.Axle[1].AxleCastX;
	}
	else if (pVehicle->AxleSet.Axle[0].AxleOriEndI>0)
	{
		u16TailCastX=pVehicle->AxleSet.Axle[0].AxleCastX;
	}

	u16StaI=pVehicle->u16FrameCnt/2;
	u16EndI=pVehicle->u16FrameCnt-3;

	u16CheckCnt=0;

	for (i16Index=u16StaI;i16Index<u16EndI;i16Index++)
	{

		if (pVehicle->EdgeSet.u16MaxHeight[i16Index]>1000)
		{
			if (pVehicle->EdgeSet.u16MinXEdge[i16Index]<u16CheckMinX)
			{
				u16CheckMinX=pVehicle->EdgeSet.u16MinXEdge[i16Index];
			}
			if (pVehicle->EdgeSet.u16MinXEdge[i16Index]>u16CheckMaxX)
			{
				u16CheckMaxX=pVehicle->EdgeSet.u16MinXEdge[i16Index];
			}

//			n32CheckSumX+=pVehicle->EdgeSet.u16MinXEdge[i16Index];
			u16CheckCnt++;
		}
	}

	if (u16CheckCnt<2)
	{
		return;
	}

	u16CheckAvgX=n32CheckSumX/u16CheckCnt;

	if ((u16TailCastX-300>u16CheckAvgX && u16TailCastX-500>u16CheckMinX))
	{
		pVehicle->FeatureSet.IsCargo =pVehicle->FeatureSet.IsCargo|1;
	}
}





/************************************************************************/
/* 高度方向投影凹凸检查载货                                             */
/************************************************************************/
void CargoZEdgeCheck(VehicleSet *pVehicle)
{












}



/************************************************************************/
/*货车投掷到地上检测                                                    */
/************************************************************************/
void CastGroundCheck(VehicleSet *pVehicle)
{
	uint16  l_u16Index   = 0;
	uint16  TmpCnt=0;
	uint16  CastFrCnt=0;
	uint16  MinAxleFrCnt=9999;//轮轴最小帧数
	uint16  MaxAxleFrCnt=0;//最大轮轴帧数
	uint16  CalAxleFcCnt=0;
	uint16  StaI=0;
	uint16  EndI=0;

	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 10||pVehicle->AxleSet.AxleSum>2)
		return ;


	//进行多轴车的简单纠错
	if (pVehicle->AxleSet.AxleSum>2 && pVehicle->EdgeSet.u16VehHeight<2200)
	{
		pVehicle->AxleSet.AxleSum=TWO_AXLE;
	}

	for (l_u16Index=0;l_u16Index<pVehicle->AxleSet.AxleSum;l_u16Index++)
	{
		TmpCnt=pVehicle->AxleSet.Axle[l_u16Index].AxleOriEndI-pVehicle->AxleSet.Axle[l_u16Index].AxleOriStaI;
		if (TmpCnt<MinAxleFrCnt)
		{
			MinAxleFrCnt=TmpCnt;
		}

		if(MaxAxleFrCnt<TmpCnt)
		{
			MaxAxleFrCnt=TmpCnt;
		}
	}


	if (MinAxleFrCnt>0 && MaxAxleFrCnt>0)
	{
		if (MinAxleFrCnt*3>2*MaxAxleFrCnt)
		{
			CalAxleFcCnt=MaxAxleFrCnt;
		}
		else
		{
			CalAxleFcCnt=3*MinAxleFrCnt/2;
		}
	}
	else
	{
		CalAxleFcCnt=pVehicle->u16FrameCnt/3;
	}



	//确定车窗检测的范围段
	if (pVehicle->AxleSet.AxleSum>1)
	{
		StaI=pVehicle->AxleSet.Axle[0].AxleOriEndI+CalAxleFcCnt;
		if (pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI>CalAxleFcCnt)
		{
			EndI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI-CalAxleFcCnt;
		}
		else
		{
			EndI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI*2/3;
		}
	}
	else
	{
		if (pVehicle->AxleSet.AxleSum>0)
		{
			//第一轴为尾部轴
			if (pVehicle->AxleSet.Axle[0].AxleOriStaI>pVehicle->u16FrameCnt/3)
			{
				StaI=pVehicle->u16FrameCnt/2;
				EndI=pVehicle->AxleSet.Axle[0].AxleOriStaI-CalAxleFcCnt;
			}
			else
			{
				StaI=pVehicle->AxleSet.Axle[0].AxleOriEndI+CalAxleFcCnt;
				EndI=pVehicle->u16FrameCnt*2/3;
			}
		}
		else
		{
			StaI=pVehicle->u16FrameCnt/2;
			EndI=pVehicle->u16FrameCnt*2/3;
		}
	}


	for (l_u16Index=StaI;l_u16Index<EndI;l_u16Index++)
	{
		if (pVehicle->EdgeSet.u8CastGrdFlag[l_u16Index]>2
			 && pVehicle->EdgeSet.u16MaxHeight[l_u16Index]>1000)
		{
			CastFrCnt++;
		}
	}

	if (CastFrCnt>1)
	{
		pVehicle->FeatureSet.IsCastGrd=1;
	}
}

/************************************************************************/
/* 出车时多特征融合                                                     */
/************************************************************************/
void MutiFeatureFusion(VehicleSet *pVehicle)
{

	//轮轴特征 由轮轴识别器单独给定

	//车高、车头高、车高平整度判断
	VehHeightModify(pVehicle);//每帧高度调整
	VehHeightCheck(pVehicle);//计算最终车高
	VehHeadHtCheck(pVehicle);//计算第一轴高度 车头高
	VehRearHtCheck(pVehicle);//计算第二轴高度
	HtRoughnessCheck(pVehicle);//高度平整度检测  

	//底盘高判断
	DpHtCheck(pVehicle);
	CastGroundCheck(pVehicle);
	//高度方向聚类检测
	HtClusterCheck(pVehicle);

	//车斗与载货判断
	ClusterHopperCheck(pVehicle);
	AxleHopperCheck(pVehicle);
	AxleHopperCheckEx(pVehicle,pVehicle->FrShape.u16LeftMaxHt);

	//寻找高度突变点定货车 比如箱货 雨棚车等
	//HeadJumpCheck(pVehicle);
	FindHeadJump(pVehicle,0);
	FindHeadJump(pVehicle,1);
	HeadJumpCheckEx(pVehicle,pVehicle->FrShape.u16LeftMaxHt);
	FindTailJump(pVehicle,0);
	FindTailJump(pVehicle,1);
	//TailJumpCheck(pVehicle);
	CargoXEdgeCheck(pVehicle);
	FramStyleCheck(pVehicle);
	//货箱检测
	ContainerCheck(pVehicle);
	TruckCanCheck(pVehicle);
	//侧面一致性判断
	SideUinformCheck(pVehicle);
	//车窗检测
	GlassWinCheck(pVehicle);

	//油罐检测
	//ArcCheck(&pVehicle->ArcSet);


	//专车检测算子
	SpecialVehCheck(pVehicle);


	//在车高的基础上进行轮轴纠偏
	AxleAdjustMent(pVehicle);

	//方向判定
	DirCheckByDev(pVehicle);

}

/************************************************************************/
/* //特征识别器                                                         */
/************************************************************************/
void FeatureCal(VehicleSet *pVehicle)
{
	//第一类 轮轴特征提取
	//轮轴算子 由轮轴识别器单独给定
	AxleRecongnize(pVehicle);

	//第二类  外形特征提取

	//掷地检测
	CastGrdCheckCal(pVehicle);
	FramShapeCal(pVehicle);

	//车高算子 每帧区域寻找已经得出

	//底盘高算子
	DpHeightCal(pVehicle);

	//车斗算子

	//侧面一致性算子
	SideUniformCal(pVehicle);

	//货箱算子
	ContainerCal(pVehicle);


	//车窗检测算子
	IntoGlassCal(pVehicle);
	SlopeGlassCal(pVehicle);
	ThroughGlassCal(pVehicle);
	MapGlassCal(pVehicle);
	//第一轴之后最低高度
	Axle1MinHeightCal(pVehicle);


	//特殊车量及专车检测算子
	SpecialVehCal(pVehicle);
}

/************************************************************************/
/* //车辆方向判定基于外设                                               */
/************************************************************************/
void DirCheckByDev(VehicleSet *pVehicle)
{
#ifdef _KeilEnVi
	if(g_SysSetUp.u8DirDevEnable)
	{
		switch(g_SysSetUp.u8DirDevType)
		{
			case DEV_COIL:
				DirJudge_Coil(pVehicle);
				break;
			case DEV_GRAT:
				DirJudge_Grat(pVehicle);
				break;
			case DEV_LASE:
				DirJudge_Lase(pVehicle);
				break;
			case Dev_REFL:
				break;
			default:
				break;
		}
	}
	else
	{
		pVehicle->DirSet.u8VehDir = MOVE_FRNT;
	}
#else
		pVehicle->DirSet.u8VehDir = MOVE_FRNT;
#endif
}

#ifdef _KeilEnVi
/************************************************************************/
/* //车辆方向判定基于外设 --线圈                                        */
/************************************************************************/
void DirJudge_Coil(VehicleSet *pVehicle)
{
	//方向情况 1--前进；2--倒车；3--进入不完全倒出；4--倒车不完全又进入；5--异常未同时触发；6--删车处理
	if((pVehicle->DirSet.u8CL_StateHi<1) && (pVehicle->DirSet.u8CL_StateLv <= g_VehOutSet.u8DirLvCnt))  //倒车后退被激光扫描到又前进到发卡机
	{
		if(pVehicle->EdgeSet.u16HeadHeight < 2000)
		{
			 //对于底盘较低的小型车来说，出现此情况为倒车情况
			pVehicle->DirSet.u8VehDir=MOVE_FRBK;
		}
		else
		{			
			if(pVehicle->AxleSet.AxleSum<ONE_AXLE)
			{
				pVehicle->DirSet.u8VehDir = MOVE_FRBK;  //对于大车，没有检测到车轴、或检测到一个轴为双胎、或检测到轴的前两个轴都为双胎，此情况为倒车

			}
			else if(pVehicle->AxleSet.AxleSum<TWO_AXLE && pVehicle->AxleSet.Axle[0].AxleType==2)
			{
				pVehicle->DirSet.u8VehDir = MOVE_FRBK; 
			}
			else if(pVehicle->AxleSet.AxleSum<THREE_AXLE && pVehicle->AxleSet.Axle[0].AxleType==2
				&&pVehicle->AxleSet.Axle[1].AxleType==2)
			{
				pVehicle->DirSet.u8VehDir = MOVE_FRBK; 
			}				
			else
			{
				pVehicle->DirSet.u8VehDir = MOVE_BKFR;
			}
		}
	}	
	else if((pVehicle->DirSet.u8CL_StateHi <= g_VehOutSet.u8DirHiCnt) && (pVehicle->DirSet.u8CL_StateLv <= g_VehOutSet.u8DirLvCnt))	 //正方向的
	{
		pVehicle->DirSet.u8VehDir = MOVE_FRNT;	 
	}
	else if((pVehicle->DirSet.u8CL_StateHi == 0) && (pVehicle->DirSet.u8CL_StateLv > g_VehOutSet.u8DirLvCnt))	 //反方向的
	{
		pVehicle->DirSet.u8VehDir = MOVE_BACK;
	}
	else if((pVehicle->DirSet.u8CL_StateHi <= g_VehOutSet.u8DirHiCnt) && (pVehicle->DirSet.u8CL_StateLv > g_VehOutSet.u8DirLvCnt))  //正向进去不完全又退出车道的	
	{
		if((pVehicle->JudgeSet.u8JudgePattern == FIRST_KECHE 
			|| pVehicle->JudgeSet.u8JudgePattern == FIRST_HUOCHE) 
			&& pVehicle->DirSet.u8CL_StateLw == 0) //增加对小客车快速倒车的判断
		{
			pVehicle->DirSet.u8VehDir = MOVE_BACK;
		}	
		else
		{
			pVehicle->DirSet.u8VehDir = MOVE_FRBK;
		}
	}
	else   //异常情况
	{
		pVehicle->DirSet.u8VehDir = MOVE_ABNO;
	}
}

/************************************************************************/
/* //车辆方向判定基于外设 --光栅                                        */
/************************************************************************/
void DirJudge_Grat(VehicleSet *pVehicle)
{
	//方向情况 1--前进；2--倒车；3--进入不完全倒出；4--倒车不完全又进入；5--异常未同时触发；6--删车处理
	if( pVehicle->DirSet.u8GR_StateLG>5 )
	{
		if(pVehicle->DirSet.u8GR_StateLS  > 0 && pVehicle->DirSet.u8GR_StateGR >0)
		{
			pVehicle->DirSet.u8VehDir = MOVE_FRNT;
		}
		else if(pVehicle->DirSet.u8GR_StateLS  == 0 && pVehicle->DirSet.u8GR_StateGR == 0)
		{
			pVehicle->DirSet.u8VehDir = MOVE_BACK;
		}
		else if(pVehicle->DirSet.u8GR_StateLS  > 0 && pVehicle->DirSet.u8GR_StateGR == 0)
		{
			pVehicle->DirSet.u8VehDir = 3;
		}
		else if(pVehicle->DirSet.u8GR_StateLG  == 0 && pVehicle->DirSet.u8GR_StateGR > 0)
		{
			pVehicle->DirSet.u8VehDir = 4;
		}
		else
			pVehicle->DirSet.u8VehDir = 5;

	}
	else
		pVehicle->DirSet.u8VehDir = 6;
	
}

/************************************************************************/
/* //车辆方向判定基于外设 --激光                                        */
/************************************************************************/
void DirJudge_Lase(VehicleSet *pVehicle)
{
	//当前默认正向前进
	pVehicle->DirSet.u8VehDir=MOVE_FRNT;
}


#endif
