#include "stdafx.h"
#include "Avi_SpeCheck.h"
#include "Avi_MathProc.h"
#include <math.h>


/************************************************************************/
/* ���⳵���������                                                     */
/************************************************************************/
void SpecialVehCal(VehicleSet *pVehicle)
{
	//Ƥ���������
	PickUpFramCal(pVehicle);
	//��ճ����������
	EnCloseWinCal(pVehicle);
	EnCloseWinCalEx(pVehicle);

}


/************************************************************************/
/* ���⳵�����                                                         */
/************************************************************************/
void SpecialVehCheck(VehicleSet *pVehicle)
{
	//Ƥ�����
	PickUpCheck(pVehicle);

	//PickUpTailCheck(pVehicle); //20170803 �÷���Ч������
	//����β�����
	TruckTailCheck(pVehicle); 

	//�𱭼��
	JinBeiCheck(pVehicle);
	//��տͳ����
	EnCloseWinCheck(pVehicle);
	EnCloseWinCheckEx(pVehicle);

	//Axle2WinCheckEx(pVehicle);//20170803 �÷���Ч������
}


/************************************************************************/
/* IsJinBei �ж�С�ͳ��Ƿ��ǽ𱭳�                                      */
/* 2��ʾ�𱭣�1��ʾ��1													*/
/************************************************************************/
void  JinBeiCheck(VehicleSet *pVehicle)
{
//	uint8     l_u8RetValue = FIRST_KECHE;  //Ĭ�Ͽ�1

	if (pVehicle  ==  NULL) //�����쳣
	{
		return ;
	}

	//�����߶���1800��2000֮��
	if (pVehicle->EdgeSet.u16VehHeight >= 1800  &&  pVehicle->EdgeSet.u16VehHeight <= 2000)
	{
		//����������2�ᣬ��ͷ���복������С
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
/* �������                                                             */
/* 0--���ǻ���;1--�ǻ���                                                */
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

	if (pVehicle->EdgeSet.u16VehHeight > pVehicle->EdgeSet.u16RearHeight + 400  &&  //�����복β�߸߶ȱ仯��
		pVehicle->EdgeSet.u16Axle2FrCnt > 1  &&  pVehicle->EdgeSet.u16Axle3FrCnt < 1) //����������β 
	{
		for (l_u16Index = pVehicle->u16FrameCnt-1; l_u16Index > 0; l_u16Index--)
		{

				for (l_u16IndexTmp = pVehicle->AxleSet.Axle[1].AxleOriEndI; l_u16IndexTmp > pVehicle->AxleSet.Axle[1].AxleOriStaI; l_u16IndexTmp--) //����֡�߶ȱ仯��
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
/* Ƥ��β����� ��������Ը߶Ƚ���Ƥ��20170729�¼ӷ���������            */
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

	//�쳣���ֱ���˳� ͣ��ֱ���˳�
	if (pVehicle==NULL||pVehicle->u16FrameCnt<8||pVehicle->u16FrameCnt+1>MAX_CUT/2)
	{
		return;
	}


	//��ʼ�����ҳ��� Ҫ�󳵸�Ϊ1000����
	for (i16Index=pVehicle->u16FrameCnt-1;i16Index>pVehicle->u16FrameCnt/2;i16Index--)
	{
		//���ҵ���һ������1000��֡
		if (pVehicle->EdgeSet.u16MaxHeight[i16Index]>ThreshH&&FistFlag<1)
		{
			FistFlag=1;
			StaI=i16Index;
		}

		//���������ǰ֡�ߵ��ڳ��� ֱ����ֹ ��ʹ���߳����ڳ����ػ���
		if (pVehicle->EdgeSet.u16MaxHeight[i16Index]+1>pVehicle->EdgeSet.u16VehHeight)
		{
			break;
		}


		//Ȼ��Ѱ��ƽ̹�Ķ� ���복�߱Ƚ� �ж��Ƿ��Ƕ�
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
				//С��5֡ ֱ�Ӹ���� ������ǰ��
				if (FitCnt<MinFrCnt)
				{
					SumH=0;
					AvgH=0;
					FitCnt=0;
					StaI=i16Index;
				}
				else
				{
					//����5֡��Ҫ�ж��Ƿ�ƽ̹ ��ǰ�����ڸ߶Ƚ�
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
/* Ƥ�����֡�������                                                   */
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
	
	int32   l_n32LeftZData     = 0;  //��ߵ����߶�ֵ
	int32   l_n32LeftXData     = 0;  //��������߶�ֵ���Ӧ��Xֵ
	int32   l_n32BaseXData     = 0;  //��׼���Xֵ����
	int32   l_n32RightZData    = 0;
	int32   l_n32RightXData    = 0;


	uint16 CurFramI=0;
	uint16 PtSum=0;

	if (pVehicle == NULL || pVehicle->u16FrameCnt > FRAME_MAXCNT||pVehicle->u16FrameCnt<8)
		return ;

	CurFramI=pVehicle->u16FrameCnt-1;
	PtSum=pVehicle->CurRegion.zdata[0];

	if (PtSum +1 > FRAME_BUFLEN ) //����������
	{
		return;
	}
		
	if (pVehicle->EdgeSet.u16MaxHeight[CurFramI]< 900) //���߶�С��900
	{
		return;
	}

	//��һ�� ��������Ѱ�ҵ�һ������900�ĵ���Ϊ���ĵ������
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
	//�ڶ��� ������Ѱ�Ƿ������������x������500�ĵ� ͬʱ�Ը߶Ƚ��м���
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

	//���û�зֽ�����˳�
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
	//������ Ѱ���Ҳ൲��Ļ��� ��  ���е�
	if (pVehicle->CurRegion.xMax-l_n32LeftXData<1000)
	{
		//������൲����<1000ֱ�������ж�
		return;
	}

	for (i16TmpIndex1=1;i16TmpIndex1<i16BreakI;i16TmpIndex1++)
	{
		if (pVehicle->CurRegion.xdata[i16TmpIndex1]<1)
		{
			//���˴�ɵ�
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


			//������Ҳ���������ĵ㴦�ڷֽ����� ��Ϊ����ʻ�Ҵ����
			if (pVehicle->CurRegion.xdata[i16TmpIndex1]<l_n32LeftXData+500
				&&pVehicle->CurRegion.xdata[i16TmpIndex1]>l_n32LeftZData)
			{
				i16ErrPt++;
			}
		}
	}

	//���Ĳ� �ж����൲��X���������ֱ���Ƿ���ڶ���  20170804���Ӷ������͸���������
	if ((l_n32RightXData - l_n32LeftXData > 1000) && i16BreakX < l_n32RightXData
		&&i16DouCnt > 0 && (l_n32RightZData-i16DouHt > 250) && i16DouHt > 500 && i16DouHt <1500&& i16ErrPt<5) //
	{
		pVehicle->SpeVSet.u8PickUpFlag[CurFramI]=1;
	}
}

/************************************************************************/
/* Ƥ�����                                                             */
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

	//20170729 �޸�Ƥ����֡Ϊ����1֡(�����Ϊ2֡)
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
/* ���֡���                                                           */
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


	//�ҵ�������ֹ��
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

	//�쳣�ж�
	if (EndI>StaI||EndI<2||StaI<2)
	{
		return;
	}

	//��ʼ���з�ռ��
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

	//������Ҫ�� ֱ�ӷ���
	if (FitFlag>0)
	{
		pVehicle->SideSet.SideFlag[CurFramI]=pVehicle->SideSet.SideFlag[CurFramI]|2;
	}
}


/************************************************************************/
/* ��ճ���������ӸĽ���                                               */
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


	//�ҵ�������ֹ��
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

	//�쳣�ж�
	EndI=2;

	if (EndI>StaI||EndI<2||StaI<2)
	{
		return;
	}

	//��ʼ���з�ռ��
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

	////������Ҫ�� ֱ�ӷ���
	//if (FitFlag>0)
	//{
	//	pVehicle->SideSet.SideFlag[CurFramI]=pVehicle->SideSet.SideFlag[CurFramI]|2;
	//}
}

/************************************************************************/
/* ��տͳ������Ϊ����                                                 */
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
			EndI=MIN(3*pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI/2-StaI/2,pVehicle->u16FrameCnt-3);//�ж��յ������Ӱ������
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
		EndI=MIN(3*pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI/2-StaI/2,pVehicle->u16FrameCnt-3);//�ж��յ������Ӱ������
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

	//�ж��Ƿ��� 20170727 �޸���ֵ95%Ϊ90%
	if (CheckCnt && FitCnt*100/CheckCnt>90 && DisFitCnt*100/CheckCnt<20)
	{
		pVehicle->FeatureSet.IsFillWin=1;
	}
}


/************************************************************************/
/* ��տͳ������Ϊ���� �Ľ���	                                        */
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

	//����������Ϣȷ����ճ����ļ���
	if (pVehicle->AxleSet.AxleSum<2)
	{
		if (pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI>pVehicle->u16FrameCnt/2)
		{
			StaI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI;
			EndI=MIN(3*pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI/2-StaI/2,pVehicle->u16FrameCnt-3);//�ж��յ������Ӱ������
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
		EndI=MIN(3*pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI/2-StaI/2,pVehicle->u16FrameCnt-3);//�ж��յ������Ӱ������
	}


	//ͳ�Ƹöε�һ������յĵ�߶� �����ǲ�����͸ �����Ƕ����ֽ��
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

	//����ø߶�ֵ������ͬһ��Χ�������տͲյ�Ҫ��
	if (MaxH-100>AvgH||AvgH-100>MinH)
	{
		return;
	}

	//������ߡ���С�� �� ��ֵ�ߴ���ͬһˮƽ �����ж��Ƿ���Ԥ�ڵĲ�����
	if (pVehicle->EdgeSet.u16VehHeight-AvgH>300)
	{
		return;
	}
	
	//��ǰѰ����þ��ߴ���ͬһˮƽ��֡������м���ֽϵ͵�֡ �����ⷶΧ ȷ�ϲ�����Ҫ��
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
				//��չ��Χ
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
				//��չ��Χ
				StaI=i16Index;
			}
		}
		else
		{
			DisFitCnt++;
		}
	}

	//�ж��Ƿ��� 20170727 �޸���ֵ95%Ϊ90%
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
/* �ڶ��ᳵ����� ��Ϊ�ͳ� �Ľ���	                                    */
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

	//����������Ϣȷ���ڶ��ᳵ���ļ���
	if (pVehicle->AxleSet.AxleSum<2)
	{
		if (pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI>pVehicle->u16FrameCnt/2)
		{
			StaI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI;
			EndI=MIN(pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI,pVehicle->u16FrameCnt-3);//�ж��յ������Ӱ������
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
		EndI=MIN(pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI,pVehicle->u16FrameCnt-3);//�ж��յ������Ӱ������
	}


	//ͳ�Ƹöε�һ������յĵ�߶� �����ǲ�����͸ �����Ƕ����ֽ��
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
	

	//�ҵ��Ե�߶��복�߲����300���ϵ�֡
	if (FrCnt>0)
	{
		AvgH=SumH/FrCnt;

		//����ø߶�ֵ������ͬһ��Χ�������տͲյ�Ҫ��
		if (MaxH-100>AvgH || AvgH-100>MinH)
		{
			if (FitCnt*100/FrCnt<30 && MaxH-MinH>300 
				&& pVehicle->EdgeSet.u16VehHeight-MaxH > 300)
			{
				//�յ�߶Ȳ�ƽ�� ���ֶ�����
				pVehicle->FeatureSet.IsTruckTail=pVehicle->FeatureSet.IsTruckTail|16;
			}
		}
		else
		{
			////�ж��Ƿ�Ϊ�д��ͳ� �յ�߶�һֱ ���복�߲��150mm����
			//if (FrCnt*100>(EndI-StaI)*70 && FitCnt*100/FrCnt>70 
			//	&& pVehicle->EdgeSet.u16VehHeight-MaxH<800)
			//{
			//	//��ʶ��� SUV β���ͳ�
			//	pVehicle->FeatureSet.IsCar=1;
			//}
			//else 
				if (FitCnt*100/FrCnt>70 && pVehicle->EdgeSet.u16VehHeight-MaxH>1000)
			{
				pVehicle->FeatureSet.IsTruckTail=pVehicle->FeatureSet.IsTruckTail|32;
			}
			else if (FitCnt*100/FrCnt<25 && pVehicle->EdgeSet.u16VehHeight-MaxH>300)
			{
				//β���յ�߶�ͬ�� �Ҳ��복����ͬ �����߲300
				pVehicle->FeatureSet.IsTruckTail=pVehicle->FeatureSet.IsTruckTail|32;
			}
		}
	}
}
