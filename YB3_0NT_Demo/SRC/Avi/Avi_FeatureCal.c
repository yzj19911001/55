#include "stdafx.h"
#include "Avi_FeatureCal.h"
#include "Avi_MathProc.h"
#include "Avi_SpeCheck.h"
#include "Avi_AxleCheck.h"

/************************************************************************/
/* ��ȡ���յ��̸�����ֵ                                                 */
/************************************************************************/
void DpHtCheck(VehicleSet *pVehicle)
{
	uint8 TmpI=0;
	uint32 TmpSum=0;
	uint32 TmpCnt=0;
	//������̸�
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

			//20170720���ü�Ȩ���̸� ����ƽ�����̸�
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

		//20170720���ü�Ȩ���̸� ����ƽ�����̸�
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

		//20170720���ü�Ȩ���̸� ����ƽ�����̸�
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
/*��ȡ��Ӧ֡���̸�                                                      */
/************************************************************************/
void  DpHeightCal(VehicleSet *pVehicle)
{
	uint16   l_u16DaFeiPtCnt  = 0;
	uint16   l_u16LeftPoint = 0;
	uint16   l_u16RightPoint = 0;
	uint16   l_u16Index = 0;
//	uint16   l_u16TmpIndex = 0;
	uint16   l_u16MaxHeightThres = 800;  //�߶���ֵ����
	uint16   l_u16MinHeightThres = 300;  //�߶���ֵ����
	uint16   l_u16WideThres   = 150;  //��������ֵ
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

	//20150522 �޸�
	if (pVehicle->CurFrmInfo.u16MaxHt <= l_u16MaxHeightThres + 200)
		l_u16MaxHeightThres = 600;
	//20150522

	//��㷽������
	//if (g_FiltZ[l_u16RightPoint] > l_u16MaxHeightThres) //���������ֵ
	//{
	//	//20150427 ����
	//	//ֱ���жϵ��̴��Ƿ���
	//	if (g_FiltZ[l_u16RightPoint] > CHASSISHEIGHTTHRES  &&  (g_FiltZ[l_u16RightPoint]  ==  g_FiltZ[l_u16RightPoint-1]  && 
	//		g_FiltZ[l_u16RightPoint-1]  ==  g_FiltZ[l_u16RightPoint-2]))
	//		return;
	//	//20150427 �������
	//	if (pVehicle->AxleSet.AxleSum  ==  ONE_AXLE)//�洢��1��2��֮����̸�
	//	{
	//		pVehicle->Dp.DpHeight1_2 += g_FiltZ[l_u16RightPoint];
	//		pVehicle->Dp.DpFrSum1_2++;		
	//	}
	//	else if (pVehicle->AxleSet.AxleSum  ==  TWO_AXLE) //�洢��2��3��֮����̸�
	//	{
	//		pVehicle->Dp.DpHeight2_3 += g_FiltZ[l_u16RightPoint];
	//		pVehicle->Dp.DpFrSum2_3++;		
	//	}
	//	return;
	//}

	//ֱ���жϵ��̴��Ƿ���
	if (g_FiltZ[l_u16RightPoint] > CHASSISHEIGHTTHRES  &&  (g_FiltZ[l_u16RightPoint-1] == 0  && 
		g_FiltZ[l_u16RightPoint-2] == 0 && g_FiltZ[l_u16RightPoint-3] == 0))
		return;
	//��Ѱ�ҵ��߶���ֵ�µĵ�1����,Ȼ��ʼ�ҵ��̸�
	for (l_u16Index = l_u16RightPoint; l_u16Index > l_u16LeftPoint; l_u16Index--)
	{
		//�жϴ��
		if (0  ==  g_FiltZ[l_u16Index])
		{
			l_u16DaFeiPtCnt++;
			if (l_u16DaFeiPtCnt>2*DAFEIMAXPT)
			{
				//������ɳ���8����ֱ���˳�
				return;
			}
		}
		else if (g_FiltZ[l_u16Index] > l_u16MaxHeightThres) //����
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

	
	l_DpIndex=l_u16Index;//��һ���ҵ��ĵ��̵�
	l_u16DaFeiPtCnt = 0;

	//Ѱ�ҵ��̸����ڵ� 20170721  ���̵���������������ڶ�����Ѱ�ĵ��̵�
	for (l_u16Index=l_DpIndex;l_u16Index<l_u16RightPoint+1;l_u16Index++)
	{
		//�ų������ɵĿ��Ӱ��
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

	//�ڵڶ�����Ѱ�����̵�����������ж� 
	//�ж��Ƿ���
	if (l_u16DaFeiPtCnt > 1  &&  g_FiltZ[l_DpIndex-1] > CHASSISHEIGHTTHRES  &&  g_FiltZ[l_DpIndex] < l_u16MinHeightThres  &&  l_DpIndex <= l_u16RightPoint)
	{
		//ǰһ������ڵ�����ֵ����ǰ���̵����300���Ҵ��ڴ�ɵ�
		return;
	}
	else if (g_FiltZ[l_DpIndex] > CHASSISHEIGHTTHRES  &&  g_FiltZ[l_DpIndex-1]  == 0  &&  g_FiltZ[l_DpIndex-2] == 0  &&  g_FiltZ[l_DpIndex-3] == 0)
	{
		//��ǰ���̵�߶ȴ��ڵ��̸���ֵ��ǰ��������
		return;
	}
	else if (l_DpIndex < l_u16RightPoint  &&  g_FiltZ[l_DpIndex] > CHASSISHEIGHTTHRES)
	{
		//�ڶ��ε��̵��·��Ƿ���ڴ�� ������˳�������̸�
		for (l_u16Index2 = l_DpIndex; l_u16Index2 < l_u16RightPoint; l_u16Index2++)
		{
			if (g_FiltZ[l_u16Index2] > CHASSISHEIGHTTHRES)
			{
				if (g_FiltZ[l_u16Index2+1]  ==  0)
				{ 
					//���
					l_u16Count++;
				}
			}
			else if(g_FiltZ[l_u16Index2] < l_u16MinHeightThres)
			{
				//���ڵ�����ֵ ֱ���˳�
				return;
			}
			else 
			{
				l_DpIndex=l_u16Index2;
			}

			if (l_u16Count > 0  &&  g_FiltZ[l_u16Index2]  ==  0) //���
			{
				//�������
				l_u16Count++;
			}
			if (l_u16Count>1)
			{
				//�������������
				return;
			}
		}
	}
	else if (l_DpIndex<POINT_SUM-3)
	{
		//��ǰ���̵������������
		if (g_FiltZ[l_DpIndex] > CHASSISHEIGHTTHRES  &&  g_FiltZ[l_DpIndex+1]  == 0 
			&&  g_FiltZ[l_DpIndex+2] == 0 && g_FiltZ[l_DpIndex+3] == 0)
			return;
	}

	if (pVehicle->AxleSet.AxleSum  ==  ONE_AXLE && g_FiltZ[l_DpIndex] > l_u16MinHeightThres)//�洢��1��2��֮����̸�
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
	else if (pVehicle->AxleSet.AxleSum  ==  TWO_AXLE  && g_FiltZ[l_DpIndex] > l_u16MinHeightThres ) //�洢��2��3��֮����̸�
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
/*������ ���㵱ǰ֡1�����²����һ���ԣ��������ֿͻ�					*/
/*���룺pVehicle ������Ϣ�ṹ��											*/
/*����ֵ����                                                            */
/************************************************************************/
void SideUniformCal(VehicleSet *pVehicle)
{
	uint8   l_u8PtCount  = 0;  //�ڸ߶���ֵ�ڵĵ���
	uint16  l_u16Index   = 0;
	uint16  l_u16CurFramI = 0;
	int32   l_n32HeightThres = 1000;  //�߶���ֵ1000
	int32   l_n32HeightMinThres = 350; //�߶���ֵ
	int32   l_n32WideThres   = 150;   //�����ֵ
	int32   l_n32Max_X   = -99999;
	int32   l_n32Min_X   = 99999;
	int32   l_n32Min_Z   = 9999;

	if (pVehicle  ==  NULL || pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 1)
		return;
	l_u16CurFramI = pVehicle->u16FrameCnt - 1;

	if (pVehicle->CurRegion.xdata[0] > FRAME_BUFLEN-1 || //�����쳣
		pVehicle->CurRegion.xdata[0] < 8) 
		return;

	//�ұ���ɨ�赽��̥�ĵ�
	if (pVehicle->CurRegion.zdata[pVehicle->CurRegion.xdata[0]] >= l_n32HeightThres) //��͸߶ȴ�����ֵ
		return;

	//����һ���Լ���֡��
	pVehicle->SideSet.SideCheckCnt++;

	for (l_u16Index = pVehicle->CurRegion.zdata[0]; l_u16Index > 0; l_u16Index--)
	{
		if ( pVehicle->CurRegion.zdata[l_u16Index] <= l_n32HeightMinThres)
			continue;
		else if (pVehicle->CurRegion.zdata[l_u16Index] < l_n32HeightThres  && 
			pVehicle->CurRegion.zdata[l_u16Index]> l_n32HeightMinThres) //�߶�1m����
		{
			//Ѱ�������СX����ֵ
			//20150422 �޸�
			l_n32Min_Z = l_n32Min_Z > pVehicle->CurRegion.zdata[l_u16Index] ? pVehicle->CurRegion.zdata[l_u16Index] : l_n32Min_Z;
			//20150422 ���
			l_u8PtCount++;
			l_n32Max_X = (l_n32Max_X < pVehicle->CurRegion.xdata[l_u16Index]) ? pVehicle->CurRegion.xdata[l_u16Index] : l_n32Max_X;
			l_n32Min_X = (l_n32Min_X > pVehicle->CurRegion.xdata[l_u16Index]) ? pVehicle->CurRegion.xdata[l_u16Index] : l_n32Min_X;
		}
		else
			break;
	}

	//20150422 �޸�
	if ((l_n32Max_X > l_n32Min_X)  &&  (l_n32Max_X - l_n32Min_X < l_n32WideThres)  &&  (l_u8PtCount >= 3)   &&  (l_n32Min_Z < 550))//20150422 ���
	{
		pVehicle->SideSet.SideFitCnt++;
		pVehicle->SideSet.SideFlag[l_u16CurFramI]=pVehicle->SideSet.SideFlag[l_u16CurFramI]|1;  //�������һ����
	}
}

/************************************************************************/
/*CheckSide ������ ���1�����²����һ���ԣ��������ֿͻ�				*/
/*���룺pVehicle ������Ϣ�ṹ��											*/
/*����ֵ�� 1��ʾ����һ�£�0����											*/
/************************************************************************/
void SideUinformCheck(VehicleSet *pVehicle)
{
//	uint8   l_u8RetValue  = 0;
	uint16  l_u16Index    = 0;
	uint16  l_u16SideTotal = 0;
	uint16  l_u16TmpCnt    = 0;
	uint16  l_u16MaxCnt   = 5;  //������������ʶ��1�ĸ���
	uint16  l_u16CheckCnt=0;

	if (pVehicle  ==  NULL || pVehicle->u16FrameCnt > FRAME_MAXCNT)
		return ;


	if ((pVehicle->EdgeSet.u16RearHeight<1000 && pVehicle->EdgeSet.u16RearHeight > 0) 
		|| pVehicle->u16FrameCnt >= FRAME_MAXCNT) // 20150422 ����֡���ﵽ���֡�����ж�һ����
	{
		return ;
	}

	if (pVehicle->u16FrameCnt < 5) //����5֡
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

	if (l_u16MaxCnt <= l_u16SideTotal/8  &&   //���������1�ĸ���������3  
		l_u16SideTotal*5 >= l_u16CheckCnt*4)  //����֡����4/5
	{
		pVehicle->FeatureSet.IsSideUiform=pVehicle->FeatureSet.IsSideUiform|1;
	}
}

/************************************************************************/
/* IsJiZhuangXiang();  //�ж�ȡ�õ�ĳ֡�Ƿ��Ǽ�װ��                     */
/* ����pFrameInfo ��ֱ����������֡��Ϣ�ṹ��							*/
/*����ֵ��0��ʾ���Ǽ�װ�䣬1��ʾ�Ǽ�װ��                                */
/************************************************************************/
uint8  IsJiZhuangXiang(FramInfo *pFrameInfo)
{
	uint8   l_u8ReturnValue     = 0;   
	uint16  l_u16Index          = 0;
	int32   l_n32MinDistance    = 0;  //��Сˮƽ����
	int32   l_n32MaxDistance    = 0;  //���ˮƽ����
	int32   l_n32HeightThres    = 0;  //���ڼ���ĸ߶���ֵ

	if (pFrameInfo  ==  NULL || pFrameInfo->u16StaPos >= pFrameInfo->u16EndPos ||
		pFrameInfo->u16StaPos >= POINT_SUM || pFrameInfo->u16EndPos >= POINT_SUM) //�����쳣
		return 0;

	if (pFrameInfo->u16MaxHt < MAXHEIGHT_JIZHUANGXIANG)  //��֡�߶�С�ڼ�װ����͸߶� 
	{
		return 0;
	}

	if (pFrameInfo->u16MaxHt > MAXHEIGHT_JIZHUANGXIANG + 300) //���Ӷ�̬�ı�߶���ֵ
		l_n32HeightThres = pFrameInfo->u16MaxHt - 300;
	else
		l_n32HeightThres = MAXHEIGHT_JIZHUANGXIANG;

	//��㷽��������(ɨ�赽�����͵Ĳ���)	

	//�ȴ���ʹ��ҵ���С�߶���ֵ��
	for (l_u16Index = pFrameInfo->u16EndPos; l_u16Index > pFrameInfo->u16StaPos; l_u16Index--)
	{
		if (g_FiltZ[l_u16Index] >= MINHEIGHT_JIZHUANGXIANG)  //��С����ֵ��ֱ������
			break;
	}

	//����С�߶���ֵ�����߶���ֵ֮������ˮƽ�����
	l_n32MinDistance = g_FiltX[l_u16Index];
	for (; l_u16Index > pFrameInfo->u16StaPos; l_u16Index--)
	{
		if (g_FiltZ[l_u16Index] < l_n32HeightThres)
		{
			if (l_n32MinDistance > g_FiltX[l_u16Index])
			{
				l_n32MinDistance = g_FiltX[l_u16Index];  //ȡ��С����
			}	
			if (l_n32MaxDistance < g_FiltX[l_u16Index])
			{
				l_n32MaxDistance = g_FiltX[l_u16Index];  //ȡ������
			}
		}
		else
			break;
	}

	if (l_n32MaxDistance > 0  &&  l_n32MinDistance > 0  &&  l_n32MaxDistance >= l_n32MinDistance) //�������ˮƽ�����ֵ
	{
		if (l_n32MaxDistance - l_n32MinDistance < MAXDIFFER_JIZHUANGXIANG) //С���������ֵ��ֵ
		{
			l_u8ReturnValue = 1;
		}
	}

	return l_u8ReturnValue;
}

/************************************************************************/
/* ���������ж�                                                         */
/************************************************************************/
void ContainerCheck(VehicleSet *pVehicle)
{

//	uint8    l_u8RetValue = 0;
//	uint8    l_u8CarWindowFlag = 0; //������ʶ
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
	uint32  HtHist[3][2]={0};//3�Σ��ֱ�����ֵ ȷ������߶� 1--�ۼƸ߶�ֵ;0--����;
	

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


	//�������ĸ߶� ����ͳ�ƹ���ʵ��
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



	for (l_u16Index = StaI; l_u16Index < EndI; l_u16Index++) //����һ��֡��	
	{
		if (ContainerHt < pVehicle->EdgeSet.u16MaxHeight[l_u16Index] + 150) //�߶��޶�
		{
			CheckCnt++;
			if (pVehicle->ContainerSet.ContainerFlag[l_u16Index]>0)
			{
				pVehicle->ContainerSet.ContainerCnt++;
			}
			
		}
	}

	//20170719 �޸���ռ�ȵ�ʱ��Ϊ����߶�15cm��Χ�ڵ�֡��ռ��
	if (pVehicle->ContainerSet.ContainerCnt*3>CheckCnt*2)
	{
		pVehicle->FeatureSet.IsContainer=pVehicle->FeatureSet.IsContainer|1;
	}
}

/************************************************************************/
/*������ ContainerCal �����麯��	                                    */
/*���룺 *pVehicle ������Ϣ�ṹ��					                    */
/*�������                                                              */
/************************************************************************/
void ContainerCal(VehicleSet *pVehicle)
{
//	uint8   l_u8Ret       = 0;
	uint8   l_u8Flag      = 0;
	uint16  l_u16Index    = 0;
	uint16  l_u16CurFramI = 0;
	int32   l_n32Max_X    = -99999;
	int32   l_n32Min_X    = 99999;
//	int32   l_n32MaxHeight_Thres = 0;     //���߶���ֵ
	int32   l_n32MinHeight_Thres = 1200;  //��С�߶���ֵ
	int32   l_n32MaxWide_Thres   = 100;

	if (pVehicle  ==  NULL || pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 1||
		pVehicle->CurRegion.zdata[0] > FRAME_BUFLEN - 1 ||
		pVehicle->CurRegion.zdata[0] < 6)
		return ;

	l_u16CurFramI = pVehicle->u16FrameCnt-1;

	//20170919 �޸ĵ�ǰ֡��>2000
	if (pVehicle->AxleSet.AxleSum<1||pVehicle->EdgeSet.u16MaxHeight[l_u16CurFramI]<1700)
	{
		return;
	}

	

	for (l_u16Index = pVehicle->CurRegion.zdata[0]; l_u16Index > 5; l_u16Index--) //��С�߶���ֵ�������е�
	{
		if (pVehicle->CurRegion.zdata[l_u16Index] <= l_n32MinHeight_Thres  &&  (!l_u8Flag))
		{
			continue;
		}
		else  //�����߶���ֵ����
		{
			//Ѱ�������СX����ֵ
			l_u8Flag = 1;
			l_n32Max_X = (l_n32Max_X < pVehicle->CurRegion.xdata[l_u16Index]) ? pVehicle->CurRegion.xdata[l_u16Index] : l_n32Max_X;
			l_n32Min_X = (l_n32Min_X > pVehicle->CurRegion.zdata[l_u16Index]) ? pVehicle->CurRegion.xdata[l_u16Index] : l_n32Min_X;
		}
	}
	
	
	//���߽�x�����Ҳ�ֵ����100mm���� �� ������ƽ̹��
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
/*MinHeightCheck //�����͸߶� ��Ҫ���С�� ����1������֡��        */
/*����:������Ϣ�ṹ�� pVehicle											*/
/*�����                                                                */
/************************************************************************/
void Axle1MinHeightCal(VehicleSet *pVehicle)
{
	uint16   l_u16PtIndex = 0;
	int32    l_n32DiffThres = 100;
	int32    l_n32HeightThres  = 500;
	int32    l_n32Height = 0;
	uint16   l_u16HeightIndex = 0;

	if (pVehicle  ==  NULL || pVehicle->CurFrmInfo.u16StaPos >= pVehicle->CurFrmInfo.u16EndPos 
		||pVehicle->AxleSet.AxleSum<1||pVehicle->AxleSet.AxleSum>1) //�Ϸ����ж�
	{
		return;
	}

	l_u16PtIndex = pVehicle->CurFrmInfo.u16EndPos;
	//��㷽��ɨ�赽�����·��ĵ���

	if (g_FiltZ[l_u16PtIndex-1] - g_FiltZ[l_u16PtIndex] > l_n32DiffThres || //��͵�֮��仯�ϴ�
		(g_FiltZ[l_u16PtIndex-2] - g_FiltZ[l_u16PtIndex-1])*2 > g_FiltZ[l_u16PtIndex-1] - g_FiltZ[l_u16PtIndex])//���3�����ֵ�Ƚ�
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
		g_FiltZ[l_u16HeightIndex-1]  ==  g_FiltZ[l_u16HeightIndex-2]) //��Ϊ��ɣ�ֱ�ӷ���
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
/* �����һ�����Ӧ������Ϊ��ͷ��                                       */
/************************************************************************/
void VehHeadHtCheck(VehicleSet *pVehicle)
{
//	uint8    l_u8Flag       = 0;
	uint8    l_u8Count      = 0;
//	uint8    l_u8DouPengFlag = 0;
	uint16   l_u16RetHeight = 0;
	uint16   l_u16Index     = 0;
//	uint16   l_u16Index1    = 0;
//	uint16   l_u16PointCnt  = 0;  //���ڼ��㳵ͷ�ߵĵ���
	uint16   l_uFrameCnt    = 0;
//	uint16   l_u16MaxHeight = 0;
//	uint16   l_u16MinHeight = 0; 
//	uint16   l_u16PreHeight = 0; //ǰһ֡�ĸ߶�ֵ
	uint32   l_u32Height    = 0;

	if (pVehicle  ==  NULL || pVehicle->AxleSet.AxleSum<1)
		return ;


	//20150423 �޸� ֱ�Ӳ��ó�ͷǰ3֡ƽ����Ϊ��ͷ��
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
/*�ڶ������Ӧ�ĳ���                                                    */
/************************************************************************/
void VehRearHtCheck(VehicleSet *pVehicle)
{
	uint8  l_u8Index = 0;
//	uint16 l_u16RetHeight = 0;
	int32  l_n32Height = 0;
	uint32 l_u32FrCnt=0;

	if (pVehicle  ==  NULL||pVehicle->AxleSet.AxleSum<2)
		return ;

	//�ڶ����� ֡�����Ӧ�߶�
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

	
	

	//�������� �������Ӧ�߶�
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
/*���㳵���߶�                                                          */
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
/* �߶�ƽ���ȼ��                                                       */
/* 0Ĭ��ֵ��1��ʾ���߶�ƽ�� 2��ʾ��β�복���нϴ�߶Ȳ�               */
/************************************************************************/
void HtRoughnessCheck(VehicleSet *pVehicle)
{
	uint8    l_u8RetValue = 0;
	uint8    l_u8HeightFlag = 0;  //��ʾ��ͷ��1���ӽ����ߵı�ʶ
	uint16   l_u16Count   = 0;
	uint16   l_u16Index   = 0;
	// 	uint16   l_u16MinHeight = 0;
	// 	uint16   l_u16MaxHeight = 0;
	uint32   l_u32HeightSum = 0;

	if (pVehicle  ==  NULL)
		return ;

	if (pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 8) //С��10֡��֡���쳣
		return ;

	for (l_u16Index = 0; l_u16Index < pVehicle->u16FrameCnt/3; l_u16Index++) //Ѱ��ǰ1/3֡���е�1���ӽ��߶�ֵ������
	{
		if (pVehicle->EdgeSet.u16VehHeight < pVehicle->EdgeSet.u16MaxHeight[l_u16Index] + 150)
		{
			l_u8HeightFlag = 1;
			break;
		}
	}

	if (l_u8HeightFlag) //��⳵���߶�ƽ���� ָһ��ƽ��
	{
		for (; l_u16Index < pVehicle->u16FrameCnt*4/5; l_u16Index++)
		{
			if (l_u16Index > pVehicle->u16FrameCnt - 8)
				break;
			else if (pVehicle->EdgeSet.u16VehHeight < pVehicle->EdgeSet.u16MaxHeight[l_u16Index] + 150) //����һ��ƽ����
				continue;
			else
			{
				l_u8HeightFlag = 2;  //������һ��ƽ����
				break;
			}

		}

		if (l_u8HeightFlag < 2)
			l_u8RetValue = 1;   //�߶�ƽ��
	}

	if (2  ==  l_u8HeightFlag)
	{
		//Ѱ�Һ�1/3֡����ƽ���߶�ֵ
		for (l_u16Index = pVehicle->u16FrameCnt*2/3; l_u16Index < pVehicle->u16FrameCnt-8; l_u16Index++)	
		{
			//��⳵β�߶�
			l_u32HeightSum += pVehicle->EdgeSet.u16MaxHeight[l_u16Index];
			l_u16Count++;
		}	

		if (l_u16Count > 4) //������Ҫ5֡
		{
			l_u32HeightSum /= l_u16Count;

			if (pVehicle->EdgeSet.u16VehHeight > l_u32HeightSum + 500) //�и߶Ȳ�
			{
				l_u8RetValue = 2; //��ʾ�и߶Ȳ�
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
/* ÿ֡�߶ȼ�����������Ҫ��ֹ�����㵼�¸߶Ȳ�׼                       */
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
		if (pVehicle->EdgeSet.u16MaxHeight[l_u16Index] != pVehicle->EdgeSet.u16SecHeight[l_u16Index]) //�洢�ĸ߶Ȳ���
		{
			if (pVehicle->EdgeSet.u16MaxHeight[l_u16Index]>pVehicle->EdgeSet.u16MaxHeight[l_u16Index-1]+1000
				 && ABS(pVehicle->EdgeSet.u16SecHeight[l_u16Index]-pVehicle->EdgeSet.u16MaxHeight[l_u16Index-1])<500)
			{
				pVehicle->EdgeSet.u16MaxHeight[l_u16Index]=pVehicle->EdgeSet.u16SecHeight[l_u16Index];//��ǰ֡�ڶ�����ǰһ֡���߽ӽ� ��֡�߶��쳣 ȡ�ڶ���
			}
			else if (pVehicle->EdgeSet.u16MaxHeight[l_u16Index-1] > pVehicle->EdgeSet.u16MaxHeight[l_u16Index] + l_u16Height)
			{//ǰһ֡�߶ȱȵ�ǰ�߶ȸ�
				l_u16Count++;
				l_u16StartIndex = l_u16StartIndex < 1 ? l_u16Index : l_u16StartIndex; //��¼��ʼ����
			}
			else if (l_u16Count > 0)
			{
				l_u16Count++;
			}
		}
		else
		{
			//�߶����
			if (l_u16Count > 0  &&  pVehicle->EdgeSet.u16MaxHeight[l_u16Index] > pVehicle->EdgeSet.u16MaxHeight[l_u16Index-1] + l_u16Height) //˵���а��ݴ�����
			{//��������
				for(l_u16IndexTmp = l_u16StartIndex; l_u16IndexTmp < l_u16Index; l_u16IndexTmp++) //�޸ĸ߶�ֵ
				{
					pVehicle->EdgeSet.u16MaxHeight[l_u16IndexTmp] = pVehicle->EdgeSet.u16SecHeight[l_u16IndexTmp];
					pVehicle->EdgeSet.u8HeightAdjFlag[l_u16IndexTmp] = 1; //�޸ĸ߶ȱ�ʶ
				}
			}
			else if (l_u16Count > 0)
			{//�ҵ��Ŀ�ʼ�ط��������ط�ǰ��֡�߶�û����������
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
					pVehicle->EdgeSet.u8HeightAdjFlag[l_u16IndexTmp] = 1; //�޸ĸ߶ȱ�ʶ
				}
			}
			l_u16Count = 0;
			l_u16StartIndex = 0;
		}
	}


	//��������һ֡ 20170714
	if (pVehicle->EdgeSet.u16MaxHeight[0] != pVehicle->EdgeSet.u16SecHeight[0]
	 && pVehicle->EdgeSet.u16MaxHeight[1]+1000<pVehicle->EdgeSet.u16MaxHeight[0])
	{
		pVehicle->EdgeSet.u16MaxHeight[0]=pVehicle->EdgeSet.u16MaxHeight[1];
		pVehicle->EdgeSet.u8HeightAdjFlag[0] = 1; //�޸ĸ߶ȱ�ʶ
	}


}

/************************************************************************/
/*�������		                                                        */
/************************************************************************/
void AxleAdjustMent(VehicleSet *pVehicle)
{
	//��6�� ��6��
	if (pVehicle->AxleSet.AxleSum> SIX_AXLE)
		pVehicle->AxleSet.AxleSum = SIX_AXLE;

	//����<2200�ľ�ƫ0���1��ĳ���
	if (pVehicle->EdgeSet.u16VehHeight< 2200)
	{
		if (pVehicle->AxleSet.AxleSum < ONE_AXLE)
		{
			pVehicle->AxleSet.AxleSum = TWO_AXLE;  //2��
			pVehicle->AxleSet.Axle[0].AxleType=1;
			pVehicle->AxleSet.Axle[1].AxleType=1;
		}
		else if (ONE_AXLE  ==  pVehicle->AxleSet.AxleSum) //ɨ�赽1����
		{
			pVehicle->AxleSet.AxleSum = TWO_AXLE;  //2��
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
			pVehicle->AxleSet.AxleSum = TWO_AXLE;  //2��
			pVehicle->AxleSet.Axle[0].AxleType=1;
			pVehicle->AxleSet.Axle[1].AxleType=1;
		}
	}
}

/************************************************************************/
/* �߶ȱ仯�ʾ��෽��                                                   */
/************************************************************************/
void HtClusterCheck(VehicleSet *pVehicle)
{
	int16 i=0;
	int16 j=0;
	int16 iTmpI=0;
	uint16 ThreshPt=5;
	uint16 u16Index=0;
	uint16 PtSum=0;//������ֵ����
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
	
	//�������߽���5�����ֵ�˲�  �� ��Ѱ��ֵ��λ��
	for (u16Index=0;u16Index<u16FramCnt;u16Index++)
	{
		//5�����ֵ�˲�
		if (u16Index+1>ThreshPt/2 && u16Index<u16FramCnt-ThreshPt/2)
		{
			uFilterH[PtSum]=MeanFilterU(&pVehicle->EdgeSet.u16MaxHeight[u16Index-ThreshPt/2],ThreshPt);
			PtSum++;
		}

		//����ֵ
		if (pVehicle->EdgeSet.u16MaxHeight[u16Index]>pVehicle->Cluster.Hmax)
		{
			pVehicle->Cluster.Hmax=pVehicle->EdgeSet.u16MaxHeight[u16Index];
			pVehicle->Cluster.HmaxIdx=u16Index;
		}

		//��Сֵ
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


	//��߶��ܶ�����
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
	
	//��߶��ܶ����߲������
	for (u16Index=1;u16Index<PtSum;u16Index++)
	{
		iDifferH[u16Index-1]=(iChangeH[u16Index]-iChangeH[u16Index-1]);
	}
	u8Offset=u8Offset+1;

	//��ǲ�����߱仯һ���Զ�
	IndicateTrends(iDifferH,iSignH,PtSum-1);
	
	//��˳�����ξ��� �ֳɼ�������
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
		for (j=i+1;j<iRealCnt+1;j++)//���һλ ���0 ȷ����������ͳ��
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
		//������̫�� ��Ϊ����仯̫�� ʵ����Ϊ���� ����Ϊ�쳣�ӵ� 
		if (pVehicle->Cluster.ClNum>19)
		{
			pVehicle->Cluster.IsOk=0;
			return;
		}
	}

	//����ÿ������ε�����
	for (i=0;i<pVehicle->Cluster.ClNum;i++)
	{
		if (pVehicle->Cluster.CI[i].Num)
		{
			ShapeAnalysisEx(uFilterH,&pVehicle->Cluster.CI[i]);
		}
	}

	//����εĺϲ�
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

	//ͳ��ÿ������������ɵ�֡��
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
/* ���෨�������                                                       */
/************************************************************************/
void ClusterHopperCheck(VehicleSet *pVehicle)
{
	int i=0;
//	uint8 l_u8retValue=0;
	int HopperIdx=-1;//�����������
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

	//���˵�С�ͳ�
	if (pVehicle->Cluster.CI[0].Num>1 && pVehicle->Cluster.CI[0].AvgH<1100
		 && pVehicle->Cluster.CI[pVehicle->Cluster.ClNum-1].AvgH<1000)
	{
		return ;
	}

	//Ѱ�ҳ������ڵ���
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
		//�����������֡��ռ������ ��ֹС�γ�β��ȫ������ ��������20170729
		if (pVehicle->Cluster.CI[HopperIdx].DaFeiCnt*100/pVehicle->Cluster.CI[HopperIdx].Num<50)
		{
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|2;
		}
	}
	
}

/************************************************************************/
/* ͨ�����������ж��Ƿ��ػ�                                             */
/************************************************************************/
void FramStyleCheck(VehicleSet *pVehicle)
{
	int16 iIndex=0;
	int16 PtNum=0;
	int16 StaIndex=0;//�ڶ�����һ��һ������1000�ĵ����
	int16 EndIndex=0;//�ڶ�������һ������1000�ĵ����

	uint16 HopperTh=1000;//�����߶�����


	//�������� �򳵸ߣ�1450 �� ֡����8 ֱ���������ж�
	if (pVehicle->AxleSet.AxleSum!=TWO_AXLE||
		pVehicle->EdgeSet.u16VehHeight>2200||pVehicle->u16FrameCnt<8
		&&pVehicle->EdgeSet.u16HeadHeight>2200)
	{
		return;
	}

	//����֮���֡ ���η���

	//��Ѱ�����Ƴ����Ĳ��ֶ�
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

	//����ҵͣ�����ɣ����߸߲��300
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
/* ���ᶨλ������ⷨ                                                   */
/************************************************************************/
void AxleHopperCheckEx(VehicleSet *pVehicle,uint16 *pH)
{
	int16 iIndex=0;
	int16 PtNum=0;
	int16 StaIndex=0;//�ڶ�����һ��һ������1000�ĵ����
	int16 EndIndex=0;//�ڶ�������һ������1000�ĵ����

	uint16 HopperTh=1000;//�����߶�����

	ClusterInfo HopperCI;//���������
	ClusterInfo CrashCI;//��ײ�������

	memset(&HopperCI,0,sizeof(HopperCI));
	memset(&CrashCI,0,sizeof(CrashCI));

	//�������� �򳵸ߣ�1450 �� ֡����8 ֱ���������ж�
	if (pVehicle->AxleSet.AxleSum!=TWO_AXLE||
		pVehicle->EdgeSet.u16VehHeight<1450||pVehicle->u16FrameCnt<8)
	{
		return;
	}

	//����֮���֡ ���η���

	//��Ѱ�����Ƴ����Ĳ��ֶ�
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

	//�����۸öε�����
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

	//��λ�����εļ���Сλ��
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

	//���㳵���εľ�ֵ�߶�
	HopperCI.AvgH=HopperCI.SumH/HopperCI.Num;
	HopperCI.AvgX=HopperCI.SumX/HopperCI.Num;

	ShapeAnalysisEx(pH,&HopperCI);


	//�۶����ж� x���������λ��ƫ��� ���ڸ߶Ƚ��ͻ�߶ȶ���
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
			//���߳����ڵڶ���ǰ �ҵڶ����ϸ߶��복���и߲�
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|4;
			return;
		}
		else if (pVehicle->EdgeSet.u16VehHtIndex>HopperCI.Sta && HopperCI.Style == 1
			&& HopperCI.MaxZ-HopperCI.AvgH>400 && HopperCI.AvgH-HopperCI.MinZ<100)
		{
			//���߳����ڵڶ����ϣ��ҳ����½����ƣ�����߶������400 
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
/* ���ᶨλ������ⷨ                                                   */
/************************************************************************/
void AxleHopperCheck(VehicleSet *pVehicle)
{
	int16 iIndex=0;
	int16 PtNum=0;
	int16 StaIndex=0;//�ڶ�����һ��һ������1000�ĵ����
	int16 EndIndex=0;//�ڶ�������һ������1000�ĵ����

	uint16 HopperTh=1000;//�����߶�����

	ClusterInfo HopperCI;//���������
	ClusterInfo CrashCI;//��ײ�������

	memset(&HopperCI,0,sizeof(HopperCI));
	memset(&CrashCI,0,sizeof(CrashCI));

	//�������� �򳵸ߣ�1450 �� ֡����8 ֱ���������ж�
	if (pVehicle->AxleSet.AxleSum!=TWO_AXLE||
		pVehicle->EdgeSet.u16VehHeight<1450||pVehicle->u16FrameCnt<8)
	{
		return;
	}

	//����֮���֡ ���η���

	//��Ѱ�����Ƴ����Ĳ��ֶ�
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

	//�����۸öε�����
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
	
	//��λ�����εļ���Сλ��
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

	//���㳵���εľ�ֵ�߶�
	HopperCI.AvgH=HopperCI.SumH/HopperCI.Num;
	HopperCI.AvgX=HopperCI.SumX/HopperCI.Num;

	ShapeAnalysisEx(pVehicle->EdgeSet.u16MaxHeight,&HopperCI);


	//�۶����ж� x���������λ��ƫ��� ���ڸ߶Ƚ��ͻ�߶ȶ���
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
			//���߳����ڵڶ���ǰ �ҵڶ����ϸ߶��복���и߲�
			pVehicle->FeatureSet.IsHopper=pVehicle->FeatureSet.IsHopper|4;
			return;
		}
		else if (pVehicle->EdgeSet.u16VehHtIndex>HopperCI.Sta && HopperCI.Style == 1
			 && HopperCI.MaxZ-HopperCI.AvgH>400 && HopperCI.AvgH-HopperCI.MinZ<100)
		{
			//���߳����ڵڶ����ϣ��ҳ����½����ƣ�����߶������400 
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


//����б�ʵĳ�������
void SlopeGlassCal(VehicleSet *pVehicle)
{
	int16 ThreshSt=0;//������ֵ�߶ȵĵ�һ����
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
	l_u16ThreshH = pVehicle->CurFrmInfo.u16MaxHt/2 > 1000 ? pVehicle->CurFrmInfo.u16MaxHt/2 : 1000;   //���1000

	//��һ���ҵ��߶ȴ���1000�ĵ�һ����
	for (i16Index=l_u16PtSum;i16Index>1;i16Index--)
	{
		if (pVehicle->CurRegion.zdata[i16Index]>l_u16ThreshH)
		{
			ThreshSt=i16Index;
			break;
		}
	}
    //�ڶ���ͨ��б��ʶ��͸����
	for (i16Index=ThreshSt;i16Index>2;i16Index--)
	{
		
		if (pVehicle->CurRegion.zdata[i16Index] == 0||pVehicle->CurRegion.zdata[i16Index-1] == 0||pVehicle->CurRegion.zdata[i16Index]<1200
			||pVehicle->CurRegion.xdata[i16Index]-pVehicle->CurRegion.xdata[i16Index-1]  == 0)
		{
			//���ֱ������
			continue;
		}
		else
		{
			if ((ABS(pVehicle->CurRegion.xdata[i16Index]-pVehicle->CurRegion.xdata[i16Index-1])>ThreshX
				 && ABS(pVehicle->CurRegion.zdata[i16Index]-pVehicle->CurRegion.zdata[i16Index-1])>ThreshX/10)
				||ABS(pVehicle->CurRegion.zdata[i16Index]-pVehicle->CurRegion.zdata[i16Index-1])>ThreshZ)
			{

				//�ֱ��󼤹⵽�����ϵ��б�� k1 k2,�복�������ڵ��б��k12;
				K1 = 1000* ( pVehicle->CurRegion.zdata[i16Index] -Smp.JGHeight ) / (pVehicle->CurRegion.xdata[i16Index] );//б�� y/x,y��Ҫת��Ϊ������ĸ߶� 
				K2 = 1000 * (  pVehicle->CurRegion.zdata[i16Index-1]-Smp.JGHeight) / (pVehicle->CurRegion.xdata[i16Index-1]);

				K12 = 1000 * (  pVehicle->CurRegion.zdata[i16Index]  - pVehicle->CurRegion.zdata[i16Index-1] ) / ( pVehicle->CurRegion.xdata[i16Index]  -  pVehicle->CurRegion.xdata[i16Index-1] )  ; 

				
				if (K1*K12 > 0  &&  K1*K2>0   &&   K12 > K2  &&  K2 > K1 )
				{
					//�жϵ�Ĺ�ϵ���������ϵĵ㣬�����ĵ㣬Ӧ���ڳ������ص㣨��͸�㣩
					WinStaPt =  i16Index-1;//������ʼ��
					WinStaX =pVehicle->CurRegion.xdata[i16Index+2];
					GlassPtCnt=0;
					//�������� �жϳ����ϲ�ƽ����
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

//�����ɳ���ͶӰͼ ��������
void MapGlassCal(VehicleSet *pVehicle)
{
	int16 i16Index=0;
	uint16 ThreshH=900;//�߶���ֵ
	uint16 PtSum=0;//��ǰ֡�ܵ���
	uint16 CurFramI=0;//��ǰ֡���
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
	//����һ�� Ѱ�Ҵ�ɿ�
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
						//ȷ����ɵ���������3��������γɵĿհ׸߶�����150 ��ɴ���ͬһ����
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
					//�����֡�����ص����ֲ����� �����ǰһ֡
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
					//С�ͳ��Ĳ����߶�ҪС
					if (pVehicle->WindowsSet.GlassWinFlag[TmpIndex]&16)
					{

					}
					else
					{
						pVehicle->WindowsSet.GlassWinFlag[TmpIndex]=pVehicle->WindowsSet.GlassWinFlag[TmpIndex]|16;
					}
					//���ͳ��Ĳ���Ҫ��
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


//����͸���� �°� 
void ThroughGlassCal(VehicleSet *pVehicle)
{

	int16 i16Index=0;
	uint16 PtSum=0;//��ǰ֡�ܵ���
	uint16 CurFramI=0;//��ǰ֡���

	uint16 xMax=0;//x���ֵ
	uint16 xMin=0;//x��Сֵ
//	uint16 StaX0=0;//�ֲ���ʼx
//	uint16 nStep=0;//�ֲ�����
//	uint16 xStep=50;//�ֲ�����
//	uint16 xHist[60]={0};//0-3000m�ֲ�
//	uint16 nHistCnt=0;//����ͳ�Ƶĵ���
//	uint16 nEdge=0;//����߽�x �ֲ���
	uint16 xEdge=0;//����߽�x
	uint16 ThreshH=1000;//�߶���ֵ
	int32 CastX=0;//��͸����x֮��
	uint16 CastCnt=0;//��͸����

	uint16 StaPt=0;
	uint16 EndPt=0;

	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > MAX_CUT 
		|| pVehicle->u16FrameCnt < 8 ||pVehicle->AxleSet.AxleSum<1||pVehicle->CurFrmInfo.u16MaxHt<1000||
		pVehicle->CurRegion.zdata[0] > FRAME_BUFLEN-1 || pVehicle->CurRegion.zdata[0] < 10)
		return ;

	PtSum=pVehicle->CurRegion.zdata[0];

	CurFramI=pVehicle->u16FrameCnt-1;

	ThreshH=pVehicle->CurRegion.zMax/2>800?pVehicle->CurRegion.zMax/2:800;

	//�ҵ��߽���
	xEdge=(2*pVehicle->CurRegion.xMin+pVehicle->CurRegion.xMax)/3;//����������չ��ȵ�1/3��Ϊ�߽�

	//�ҵ������μ��������Ϸ���
	xMin=pVehicle->CurRegion.xMax;
	for (i16Index=PtSum-2;i16Index>2;i16Index--)
	{
		if (pVehicle->CurRegion.zdata[i16Index]>ThreshH)
		{
			if (pVehicle->CurRegion.xdata[i16Index]>xEdge && StaPt<1)
			{
				//�������
				EndPt=i16Index;
				StaPt=i16Index;
			}
			else if (pVehicle->CurRegion.xdata[i16Index]<xEdge && pVehicle->CurRegion.xdata[i16Index-1]<xEdge
				 && StaPt>0 && StaPt-EndPt<1)
			{
				EndPt=i16Index+1;
			}

			//��͸����
			if (StaPt>0 && StaPt-EndPt<1)
			{
				//��͸����
				CastX+=pVehicle->CurRegion.xdata[i16Index]-xEdge;
				CastCnt++;
			}

			//�����϶�
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



//����͸����
void IntoGlassCal(VehicleSet *pVehicle)
{
	int32 iCoorX[FRAME_BUFLEN]={0};
	int32 iCoorZ[FRAME_BUFLEN]={0};
	int32 iFilterX[FRAME_BUFLEN]={0};
	int32 iFilterZ[FRAME_BUFLEN]={0};

	uint16 FitPoint=0;
	uint16 PointCnt=0;
	uint16 DafeiPt=0;

	//��ֵ����  �߶����� �˲�����
	uint16 ThreshH =1000;//��ֵ
	uint16 ThreshPt=4;
	uint16 ThreshX=300;
//	uint16 ThreshZ=300;


	uint16 MaxXPt=0;//���xλ��
	int32  MaxX=0;//���Xλ��
//	uint16 MaxXHt=0;//���x��Ӧ�߶�

	uint16 MinX1=9999;//�ϲ�����Сxֵ
	uint16 MinX1Pt=0;//�ϲ�����Сxֵ��Ӧ�ĵ���
	uint16 MinX2=9999;//�²�����Сxֵ
	uint16 MinX2Pt=0;//�²�����Сxֵ��Ӧ�ĵ���

	int32 SumDiffX=0;


//	int32 MaxHtx=0;//����xλ��
//	int32  MaxHt=0;//�����ֵ
//	uint16 MinHtPt=0;//����Сֵλ��
//	int32  MinHt=65535;//����Сֵ
//	int32 MinHtx=0;//��С��xλ��

	uint16 PtSum=0;//������ֵ����
	uint16 zNearCnt=0;

	uint16 u16Index=0;
	uint16 CurFramI=0;

//	uint16 xHist[35]={0};//x����ֱ��ͼ
	uint16 xMax=0;
	uint16 xMin=0;


	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > MAX_CUT 
		|| pVehicle->u16FrameCnt < 80||pVehicle->AxleSet.AxleSum<1||pVehicle->CurFrmInfo.u16MaxHt<1000||
		pVehicle->CurRegion.zdata[0] > FRAME_BUFLEN-1 || pVehicle->CurRegion.zdata[0] < 10)
		return ;

	PointCnt=pVehicle->CurRegion.zdata[0];

	CurFramI=pVehicle->u16FrameCnt-1;

	//��һ�� �ҵ���������ұ߽�
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


	//ɸѡ���� ���˴�����
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


	//�����������ֵ�˲� ��������������
	PtSum=0;
	for (u16Index=ThreshPt/2;u16Index<FitPoint-ThreshPt/2;u16Index++)
	{
		iFilterX[PtSum]=MeanFilterI(&iCoorX[u16Index-ThreshPt/2],ThreshPt);
		iFilterZ[PtSum]=MeanFilterI(&iCoorZ[u16Index-ThreshPt/2],ThreshPt);
		PtSum++;
	}

	//��λ���ҵ�λ��
	for (u16Index=0;u16Index<PtSum-1;u16Index++)
	{
		if (iFilterX[u16Index]>MaxX)
		{
			MaxXPt=u16Index;
			MaxX=iFilterX[u16Index];
//			MaxXHt=iFilterZ[u16Index];
		}
	}



	//�����ҵ�Ϊ�ֽ�Ѱ�����¶�����x
	for (u16Index=0;u16Index<PtSum-1;u16Index++)
	{
		if (u16Index<MaxXPt)
		{
			//�ϲ��ֵ���Сxֵ��λ��
			if (MinX1>iFilterX[u16Index])
			{
				MinX1=iFilterX[u16Index];
				MinX1Pt=u16Index;
			}

		}
		else
		{
			//�²��ֵ���Сx��λ��
			if (MinX1>iFilterX[u16Index])
			{
				MinX2=iFilterX[u16Index];
				MinX2Pt=u16Index;
			}
		}
	}


	//û�ҵ��������±߽�ֱ���˳�
	if (MinX1Pt<1||MinX2Pt<1)
	{
		return;
	}



	//�����ۼ����
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
/* �������				                                                */
/************************************************************************/
void GlassWinCheck(VehicleSet *pVehicle)
{
//	uint8    l_u8RetValue = 0;
//	uint8    l_u8CarWindowFlag = 0; //������ʶ
//	uint8    l_u8Count = 0;
	uint16   l_u16Index   = 0;
//	uint16   l_u16TmpIndex = 0;
//	uint16   l_u16CurHeight = 0;
	uint16   TmpCnt=0;
	uint16   MinAxleFrCnt=9999;//������С֡��


	uint16  StaI=0;
	uint16  EndI=0;

	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 10)
		return ;


	//���ж��ᳵ�ļ򵥾���
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

	


	//ȷ���������ķ�Χ��
	if (pVehicle->AxleSet.AxleSum>1)
	{
		if (pVehicle->u16FrameCnt/2-pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI>0)
		{
			//ͣ���������һ������β��һֱ��ɨ�赽 ֻѡȡ��һ̥����֡�����һ̥����֡��һ��  �� ���һ�������λ�ý���ͳ�Ʒ���
			StaI=(pVehicle->AxleSet.Axle[0].AxleOriEndI+pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI)/2;
			EndI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriEndI;
		}
		else if (pVehicle->u16FrameCnt/3-pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI>0
			 && pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI-pVehicle->AxleSet.Axle[0].AxleOriEndI>MinAxleFrCnt/2)
		{
			//ͣ���������һ�����Ὺʼһֱ��ɨ�� ѡȡ��һ����������һ����Ŀ�ʼ�ν���ͳ�Ʒ���  20170714 ����ǰ����������
			StaI=(pVehicle->AxleSet.Axle[0].AxleOriEndI+pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI)/2;
			EndI=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI;

		}
		else if (pVehicle->u16FrameCnt<pVehicle->AxleSet.Axle[0].AxleOriEndI*2
			 && pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI-pVehicle->AxleSet.Axle[0].AxleOriEndI>MinAxleFrCnt)
		{
			//��һ������ǰ��ͣ�� ѡȡ��һ����������ڶ����Ὺʼλ�ý���ͳ�� 20170714
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



	for (l_u16Index = StaI; l_u16Index < EndI; l_u16Index++) //����һ��֡��	
	{
		//20170714 �ɸ߶�200��Χ����300��Χ�ڵ�֡�����ж�
		if (pVehicle->EdgeSet.u16VehHeight < pVehicle->EdgeSet.u16MaxHeight[l_u16Index] + 300  && 
			ABS(pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleCastX-pVehicle->EdgeSet.u16MinXEdge[l_u16Index])<200) //�߶��޶�
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
		if (pVehicle->WindowsSet.GlassWinCnt > 1) //��2֡
		{
			pVehicle->FeatureSet.IsGlassWin=1;
		}
	}
	else
	{
		if (pVehicle->WindowsSet.GlassWinCnt > 5
			 && pVehicle->WindowsSet.GlassWinCnt*3>EndI-StaI) //��2֡
		{
			pVehicle->FeatureSet.IsGlassWin=1;
		}
	}

}
/************************************************************************/
/* β������ ��⳵�����ػ�                                              */
/************************************************************************/
void TailJumpCheck(VehicleSet *pVehicle)
{
	int16 i16Index=0;
	uint16 u16StaI=0;
	uint16 u16EndI=0;
	uint16 u16RealIndex=0;
	uint16 u16HeadCastX=0;//��ͷ��x�߽�ֵ
//	uint16 u16HeadEndI=0;//��ͷ����֡
	uint16 u16CheckCnt=0;//����֡��
	uint16 u16CheckMaxZ=0;//��������
	uint16 u16CheckMinZ=9999;//������С��
	uint32 n32CheckSumZ=0;//���θ�֮��
	uint16 u16CheckAvgZ=0;//���θ߾�ֵ
	uint16 u16CheckMaxX=0;//���α߽�x���
	uint16 u16CheckMinX=9999;//���α߽�x��С
	uint32 n32CheckSumX=0;//���α߽�x֮��
//	uint16 u16CheckAvgX=0;//���α߽�x��ֵ
	int32 n32HeadHeight=0;//��ͷ�߶�
	int32 n32BodyHeight=0;//����θ߶�
	int32 n32TailHeight=0;//��λ�θ߶�
	uint16 u16CalCnt=0;//��ʱ�������


	//����һ����
	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT 
		|| pVehicle->u16FrameCnt < 10||pVehicle->AxleSet.AxleSum<1
		||pVehicle->EdgeSet.u16HeadHeight<1800)

		return ;

	if (pVehicle->AxleSet.AxleSum>1)
	{
		//����������� ֱ�����ᶨλ�������
		u16StaI=pVehicle->AxleSet.Axle[0].AxleOriEndI;
		u16EndI=pVehicle->AxleSet.Axle[1].AxleOriStaI;
		n32HeadHeight=pVehicle->EdgeSet.u16HeadHeight;

		if (u16StaI<1||u16EndI<1)
		{
			//��ֹ�����Ǿ����õ���
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
		//��һ��Ϊβ���� ���㳵ͷ�߽����ж�����
		u16StaI=pVehicle->AxleSet.Axle[0].AxleOriStaI;
		u16EndI=pVehicle->AxleSet.Axle[0].AxleOriEndI;
		if (u16StaI<1||u16StaI<pVehicle->u16FrameCnt/3)
		{
			//��ֹ�����Ǿ����õ���
			return;
		}

		//���㳵ͷ��
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

		//���㳵���м�θ߶�
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

		
		//�Ѿ���֤β���߶�>1000
		if (ABS(n32BodyHeight-n32HeadHeight)>300 && ABS(n32TailHeight-n32HeadHeight)>300)
		{
			//������복ͷ�߲����300  ��β���복ͷ�θ߶Ȳ����300
			pVehicle->FeatureSet.IsTailJump|=32;
		}
		else if (ABS(n32BodyHeight-n32HeadHeight)>300 && ABS(n32TailHeight-n32BodyHeight)>300)
		{
			//������복ͷ�߲����300  ��β���복β�θ߶Ȳ����300
			pVehicle->FeatureSet.IsTailJump|=64;
		}
	}
}

/************************************************************************/
/* ��������ʻ��		                                                */
/************************************************************************/
void TruckCanCheck(VehicleSet *pVehicle)
{
	uint16 u16indx=0;
	uint16 u16TmpI=0;
	int32  shape[MAX_CUT]={0};//��������״
	int32  zlen=0;
	uint8 temp=0;

	uint32 ContainSumH=0;
	uint32 CabSumH=0;

	uint16 CheckFram=0;//����֡��


	uint8 ContainRat=0;  //����֡������
	uint16 ContainCnt=0; //����֡��
	uint16 ContainSt=MAX_CUT; //�����������
	uint16 ContainH=0;  //����ƽ���߶�
	uint16 CabSt=0; //��ʻ����ʼ����
	int16 CabEd=0;//��ʻ�ҵ���ֹ����
	uint16 CabCnt=0;  //��ʻ��֡��
	int16 CabH=0;  //��ʻ�Ҿ���
	uint16 CabHmax=0;

	uint16 CabHst=0;//��ʻ��ǰ�߶�
	uint16 CabFlat=0;//��ʻ��ƽ̹֡
	uint16 FlatHt=0;
	uint32 CabFrH=0;//��ʻ��ǰһ�����
	uint32 CabBaH=0;//��ʻ�Һ�һ�����

	uint8 CabStlye=0;//0-δ��⣻1--������2--ƽ̹��3--�½���



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
			//�����ֻ�е�һ���ᣬ�������޷���λ ֱ���˳�
			if (pVehicle->AxleSet.Axle[0].AxleEndIndex<pVehicle->u16FrameCnt/3)
			{
				return;
			}
			else
			{
				CheckFram=MIN(pVehicle->AxleSet.Axle[0].AxleOriStaI,MAX_CUT);//���һ��
			}

		}
		else
		{
			if (pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI<pVehicle->u16FrameCnt/2)
			{
				CheckFram=MIN(pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleOriStaI,MAX_CUT);//���һ��
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
	/*������ݻ�����Ϊ���״                                                */
	/************************************************************************/
	for (u16indx=0;u16indx<CheckFram;u16indx++)//���ݻ�
	{
		//���ݻ�
		temp=pVehicle->EdgeSet.u16MaxHeight[u16indx]/100;
		if (temp>7)
		{
			shape[u16indx]=temp-7;
		}
		else
		{
			shape[u16indx]=0;
		}
		//����֡�������
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



	//�������΢����
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

	//����ռ��
	ContainRat=ContainCnt*10/CheckFram;
	//�������
	if (ContainCnt > 0)
		ContainH=ContainSumH/ContainCnt;
	//�����Լ�ʻ�һ���С�� ��ǰ����Ѱ�Ҽ�ʻ��
	if(ContainRat>5 && ContainH>12 && ContainSt>2) 
	{
		//�Ƿ���ڼ�ʻ�Ҽ���ʻ�Ҹ߶�
		CabEd=ContainSt-1;//��ʼ����ʻ���յ�
		CabSt=CabEd;//��ʼ����ʻ�����
		for (u16indx=ContainSt-1;u16indx>0;u16indx--)
		{
			if (zlen<9)
			{
				//����ֱ���˳�
				break;
			}

			//Ѱ�Ҽ�ʻ��
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

					//�ж��Ƿ������ʻ��ƽ̹Ҫ��
					if (CabCnt<5)
					{	//����Ѱ�Ҽ�ʻ��
						CabEd=u16indx;
						CabSt=u16indx;
						CabHmax=0;
						CabSumH=0;
						CabCnt=0;
					}
					else
					{
						//�ж��Ƿ�Ϊƽ̹��ʻ��
						//�жϼ�ʻ�ҵ���״
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
			//��ʻ�ҵĸ߶�
			CabH=CabSumH/CabCnt;
			//��������Ҹ߶�����Ϊ����С���� 
			if (CabH>12 && CabH<16 && CabSt>1 && ContainH-CabH>3)//��ʻ�Ҹ߶�2000-2200֮�䣬��ͷ֡����6֡����ֹ�ͳ�����
			{
				pVehicle->FeatureSet.IsContainer=pVehicle->FeatureSet.IsContainer|2;
			}
		
		}
	}
}

/************************************************************************/
/* ������Ѱ��β����                                                     */
/************************************************************************/
void FindTailJump(VehicleSet *pVehicle,uint8 nMode)
{
	int32 iIndex=0;

	int32 StaIndex=0;//������ʼ֡���
	int32 EndIndex=0;//������ֹ֡���
	uint16 HopperTh=1000;//�����߶�����
	int32 iBreakPt=0;
	uint16 FitCnt=0;
	uint32 FrontHt=0;
	uint16 FallCnt=0;

	uint32 FallHt=0;
	int32 iIndex1=0;

	uint16 *pH=NULL; 
	uint8  nBreaks=0;

	//���ж��ᳵ�ļ򵥾���  20170803 �ͳ��������Ҳ�д˾�������
	if (pVehicle->AxleSet.AxleSum>2 && pVehicle->EdgeSet.u16VehHeight<2200)
	{
		pVehicle->AxleSet.AxleSum=TWO_AXLE;
	}

	//�������� �򳵸ߣ�1450 �� ֡����8 ֱ���������ж�
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


	//��λ����������ʼλ�ú���ֹλ��
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


	//����Ѱ�ҵ�һ������1000��֡��Ϊ������ֹ֡
	for (iIndex=pVehicle->u16FrameCnt-1;iIndex>pVehicle->u16FrameCnt/2;iIndex--)
	{
		if (MIN_N_NEAR(&pH[iIndex],3,0)>HopperTh)
		{
			EndIndex=iIndex;
			break;
		}
	}


	//Ѱ�Ҹ߶ȼ���ڶ������350��֡
	for (iIndex=EndIndex;iIndex>StaIndex;iIndex--)
	{
		
		if (iIndex-3<2)
		{
			break;
		}
		else if (iIndex>pVehicle->u16FrameCnt-3)
		{
			//β������3֡
			continue;
		}

		//��ֹǰ��֡���쳣 ����ǰ��֡��ֵ��������
		FrontHt=(pH[iIndex-1]+pH[iIndex-2]+pH[iIndex-3])/3;
		//if (FrontHt+100>pH[iIndex-1])
		//{
		//	FrontHt=pH[iIndex-1];
		//}

		//��ʼ�Ҹ߶ȶ�����
		if ((FrontHt+1>pH[iIndex-1]&&FrontHt-100<pH[iIndex-1])
			||(FrontHt<pH[iIndex-1]&&FrontHt+100>pH[iIndex-1]))
		{
			//ǰ����β��������Ҫ�����ж�����350
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
				//20170804���� ��Ӧ���д���֤
				iBreakPt=iIndex;
				nBreaks=2;
				break;
			}
		}
		else
		{

			//ǰ����֡�����ɸߵ������� �복�������Ϳͳ�β����������
			FallHt=0;
			for(iIndex1=iIndex;iIndex1>StaIndex;iIndex1--)
			{
				//������ ���彵���ﵽ300 ֡֡������100
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

			//�ų�������
			if (FallHt>300&&iIndex-iIndex1>1)
			{	
				iBreakPt=iIndex;
				break;
			}
		}
	}

	//û���ҵ����ʵĶ����
	if (iBreakPt<StaIndex)
	{
		return;
	}

	//��ǰ��5֡ ȷ������� ��ֹ�ͳ������ɼٶ� ���� 20170729
	for (iIndex=iBreakPt-1;iIndex>MAX(iBreakPt-6,6);iIndex--)
	{
		if (pVehicle->FrShape.u8DaFeiCnt[iIndex]>2*DAFEIMAXPT)
		{
			FitCnt++;
		}

		//���֡���� ֱ���˳�
		if (FitCnt>2)
		{
			return;
		}
	}

	FitCnt=0;
	//����Ѱ��5֡���Ƿ����ƽ̹��������ڶ�����֡ ��������ӽ��ͳ�β��2֡�ϵ�֡����
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

	//6֡��ͷ��4֡��ƽ̹�� ������ƽ̹�� + ���Ӹ߶����½����ж� �ų��ͳ�β��ͣ��(20170804)
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
/* ��ͷ���� ���ֿͻ�                                                    */
/************************************************************************/
void HeadJumpCheckEx(VehicleSet *pVehicle,uint16 *pH)
{
	uint16 u16Index=0;
	uint16 u16StaI=0;
	uint16 u16EndI=0;
	uint16 u16RealIndex=0;
	uint16 u16HeadCastX=0;//��ͷ��x�߽�ֵ
	uint16 u16HeadEndI=0;//��ͷ����֡
	uint16 u16CheckCnt=0;//����֡��
	uint16 u16CheckMaxZ=0;//��������
	uint16 u16CheckMinZ=9999;//������С��
	uint32 n32CheckSumZ=0;//���θ�֮��
	uint16 u16CheckAvgZ=0;//���θ߾�ֵ
	uint16 u16CheckMaxX=0;//���α߽�x���
	uint16 u16CheckMinX=9999;//���α߽�x��С
	uint32 n32CheckSumX=0;//���α߽�x֮��
//	uint16 u16CheckAvgX=0;//���α߽�x��ֵ

	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 10
		||pVehicle->EdgeSet.u16HeadHeight<1800||pVehicle->AxleSet.AxleSum<1)

		return ;


	u16StaI=pVehicle->AxleSet.Axle[0].AxleOriStaI;
	u16EndI=pVehicle->AxleSet.Axle[0].AxleOriEndI;
	u16HeadCastX=pVehicle->AxleSet.Axle[0].AxleCastX;


	if (u16StaI<1||u16StaI>pVehicle->u16FrameCnt/3)
	{
		//��ֹ�����Ǿ����õ���
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

	//�������һ��ȳ�����εľ�ֵ�߶��������С�߶� �ж��Ƿ��������
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
/* ��ͷ������  ���ֿͻ�20170803                                       */
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


//	uint16 CabHst=0;//��ʻ��ǰ�߶�
//	uint16 CabFlat=0;//��ʻ��ƽ̹֡
//	uint16 FlatHt=0;
//	uint32 CabFrH=0;//��ʻ��ǰһ�����
//	uint32 CabBaH=0;//��ʻ�Һ�һ�����

	uint16 PassMaxHt=0;//���ɶ�����
	uint16 PassMinHt=0;//���ɶ���С��
	uint32 PassAvgHt=0;//���ɶξ�ֵ
	uint32 PassCnt=0;//���ɶ�֡��

	uint16 *pH=NULL;
	uint16 u16VehHeight=0;
	uint16 u16MaxIndx=0;

	//׼������ 
	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 10
		||pVehicle->EdgeSet.u16HeadHeight>2500||pVehicle->EdgeSet.u16HeadHeight<1000
		||pVehicle->EdgeSet.u16VehHeight<1800)

		return ;


	//��������Ĳ�ͬ �Զ�ѡȡ
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


	//�ҵ�����ƽ̹��
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

	//��ǰѰ�Ҹ߶�ƽ̹���Ҵ��ڸ߶Ƚ��Ĳ�λ--�����ʻ�һ�ϵ͵Ļ����դ����
	//�Ƿ���ڼ�ʻ�Ҽ���ʻ�Ҹ߶�
	CabEd=ContainSt-1;//��ʼ����ʻ���յ�
	CabSt=CabEd;//��ʼ����ʻ�����
	for (i16Index=CabEd;i16Index>5;i16Index--)
	{
		if (pH[i16Index]<1200)
		{
			//����ֱ���˳�
			break;
		}

		//Ѱ�Ҽ�ʻ��
		if (pH[ContainSt]-pH[i16Index]>200 
			&&((pH[i16Index]+1>pH[CabEd]
		&&pH[CabEd]+150>pH[i16Index])
			||(pH[i16Index]<pH[CabEd]
		&&pH[i16Index]+150>pH[CabEd])))
		{
			//���ʻ���յ�߶���150����
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
				//�ж��Ƿ������ʻ��ƽ̹Ҫ��
				if (CabCnt<5)
				{	//����Ѱ�Ҽ�ʻ��
					CabEd=i16Index;
					CabSt=i16Index;
					CabHmax=0;
					CabSumH=0;
					CabCnt=0;
				}
				else
				{
					//�ж��Ƿ�Ϊƽ̹��ʻ��
					if (ContainSt-CabEd>3)
					{
						//���ڹ��ɶ� ������� ��� ��ֵ �ж��Ƿ�Ϊƽ̹���� �д��ͳ�
						PassMaxHt=0;//���ɶ�����
						PassMinHt=0;//���ɶ���С��
						PassAvgHt=0;//���ɶξ�ֵ
						PassCnt=0;//���ɶ�֡��
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
							//���ɶ�Ϊƽ̹�� �� ������2100����  �ų��𱭵ȸ���
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
						//�����ڹ��ɶ� ֱ���ж�
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
/* ��ͷ���� ���ֿͻ�                                                    */
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


//	uint16 CabHst=0;//��ʻ��ǰ�߶�
//	uint16 CabFlat=0;//��ʻ��ƽ̹֡
//	uint16 FlatHt=0;
//	uint32 CabFrH=0;//��ʻ��ǰһ�����
//	uint32 CabBaH=0;//��ʻ�Һ�һ�����

	uint16 PassMaxHt=0;//���ɶ�����
	uint16 PassMinHt=0;//���ɶ���С��
	uint32 PassAvgHt=0;//���ɶξ�ֵ
	uint32 PassCnt=0;//���ɶ�֡��


	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 10
		||pVehicle->EdgeSet.u16HeadHeight>2500||pVehicle->EdgeSet.u16HeadHeight<1000||pVehicle->EdgeSet.u16VehHeight<1800)

		return ;

	//�ҵ�����ƽ̹��
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

	//��ǰѰ�Ҹ߶�ƽ̹���Ҵ��ڸ߶Ƚ��Ĳ�λ--�����ʻ�һ�ϵ͵Ļ����դ����
	//�Ƿ���ڼ�ʻ�Ҽ���ʻ�Ҹ߶�
	CabEd=ContainSt-1;//��ʼ����ʻ���յ�
	CabSt=CabEd;//��ʼ����ʻ�����
	for (i16Index=CabEd;i16Index>5;i16Index--)
	{
		if (pVehicle->EdgeSet.u16MaxHeight[i16Index]<1200)
		{
			//����ֱ���˳�
			break;
		}

		//Ѱ�Ҽ�ʻ��
		if (pVehicle->EdgeSet.u16MaxHeight[ContainSt]-pVehicle->EdgeSet.u16MaxHeight[i16Index]>200 
			&&((pVehicle->EdgeSet.u16MaxHeight[i16Index]+1>pVehicle->EdgeSet.u16MaxHeight[CabEd]
		&&pVehicle->EdgeSet.u16MaxHeight[CabEd]+150>pVehicle->EdgeSet.u16MaxHeight[i16Index])
			||(pVehicle->EdgeSet.u16MaxHeight[i16Index]<pVehicle->EdgeSet.u16MaxHeight[CabEd]
		&&pVehicle->EdgeSet.u16MaxHeight[i16Index]+150>pVehicle->EdgeSet.u16MaxHeight[CabEd])))
		{
			//���ʻ���յ�߶���150����
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
				//�ж��Ƿ������ʻ��ƽ̹Ҫ��
				if (CabCnt<5)
				{	//����Ѱ�Ҽ�ʻ��
					CabEd=i16Index;
					CabSt=i16Index;
					CabHmax=0;
					CabSumH=0;
					CabCnt=0;
				}
				else
				{
					//�ж��Ƿ�Ϊƽ̹��ʻ��
					if (ContainSt-CabEd>3)
					{
						//���ڹ��ɶ� ������� ��� ��ֵ �ж��Ƿ�Ϊƽ̹���� �д��ͳ�
						PassMaxHt=0;//���ɶ�����
						PassMinHt=0;//���ɶ���С��
						PassAvgHt=0;//���ɶξ�ֵ
						PassCnt=0;//���ɶ�֡��
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
							//���ɶ�Ϊƽ̹�� �� ������2100����  �ų��𱭵ȸ���
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
						//�����ڹ��ɶ� ֱ���ж�
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
/* ÿ֡�������ͼ�� 20170803�Ľ���                                      */
/************************************************************************/
void FramShapeCal(VehicleSet *pVehicle)
{
	uint8  TmpFlag=0;
	uint16 CurFramI=0;
	int16 i16PtSum=0;
	int16 i16Index=0;

//	uint16 ThreshXZ=50;
//	uint16 ThreshHt=800;
//	int16 iOutLinePt=0;//�����ߵ�
//	int16 iThreshPt=0;//�ﵽ��ֵ�ĵ�һ����
//	int16 FitCnt=0;

//	uint16 BaseX=0;//�߽�X��

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
	//��ȡÿ֡������ֵ
	CurFramI=pVehicle->u16FrameCnt-1;
	i16PtSum=pVehicle->CurRegion.xdata[0];

	//����- ���1m���º�1m���ϵ�������ɵ����
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
	//��¼������
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

	//������ ��ȡ����20CM���ڵĵ� �γ�����
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
			//��ֹ�м�Ͷ�����������͸߶������������Ե����� �������ǻ�����������������γ�
			continue;
		}
		if (pVehicle->CurRegion.xdata[i16Index]<pVehicle->FrShape.BaseX+300)
		{
			if (pVehicle->CurRegion.zdata[i16Index]>pVehicle->FrShape.u16LeftMaxHt[CurFramI])
			{
				pVehicle->FrShape.u16LeftMaxHt[CurFramI]=pVehicle->CurRegion.zdata[i16Index];
			}
			BlockMap[0][0]++;
			//���ڵ�һ���� �������ɨ�赽�����ײ� ��ȡ���ߴ���
			BlockMap[1][0]=pVehicle->FrShape.u16LeftMaxHt[CurFramI]*BlockMap[0][0];
		}
		else 
		{
			BlockMap[0][ABS(pVehicle->CurRegion.xdata[i16Index]-pVehicle->FrShape.BaseX)/300+1]++;
			BlockMap[1][ABS(pVehicle->CurRegion.xdata[i16Index]-pVehicle->FrShape.BaseX)/300+1]+=pVehicle->CurRegion.zdata[i16Index];
		}
	}
	
	//����ÿ����ĸ߶ȣ��������ƽ��������������
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
			//����Ҹ�  �������ػ�
			pVehicle->FrShape.u16FrShape[CurFramI]=3;
		}
		else if (MaxHt-MinHt>300&&MaxHI<MinHI)
		{
			//����ҵ� �������ػ� ������ �������ȥ
			pVehicle->FrShape.u16FrShape[CurFramI]=4;
		}
		else
		{
			if (MaxHt-AvgHt<100||AvgHt-MinHt<100)
			{
				//������ֱ ��ȷ���ƽ̹ �����ṹ Ϊ����
				pVehicle->FrShape.u16FrShape[CurFramI]=2;
			}
		}
	}
	else
	{
		pVehicle->FrShape.u16FrShape[CurFramI]=1;//����Ϊһƽ��
	}

}


/************************************************************************/
/* ÿ֡�������ͼ��                                                     */
/************************************************************************/
void FramShapeCalEx(VehicleSet *pVehicle)
{
	uint16 CurFramI=0;
	int16 i16PtSum=0;
	int16 i16Index=0;

	uint16 ThreshXZ=50;
	uint16 ThreshHt=800;
	int16 iOutLinePt=0;//�����ߵ�
	int16 iThreshPt=0;//�ﵽ��ֵ�ĵ�һ����
	int16 FitCnt=0;

	uint16 BaseX=0;//�߽�X��

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
	//��ȡÿ֡������ֵ
	CurFramI=pVehicle->u16FrameCnt-1;
	i16PtSum=pVehicle->CurRegion.xdata[0];

	//������������
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

	//��һ�� Ѱ�Ҳ���ײ�ThreshHt��һ���� ��Ϊ�õ�����Ϊ�����߲���
	for (i16Index=i16PtSum;i16Index>1;i16Index--)
	{
		if (EdgeZ[i16Index]>ThreshHt)
		{
			iThreshPt=i16Index;
			iOutLinePt=i16Index;
			break;
		}
	}


	//����- ���1m���ϵ�������ɵ����
	for (i16Index=iThreshPt;i16Index>1;i16Index--)
	{
		if (EdgeZ[i16Index]<1&&EdgeZ[i16Index-1]<1)
		{
			pVehicle->FrShape.u8DaFeiCnt[CurFramI]++;
		}
	}

	//������ ����ÿ֡�Ľ���

	//�ڶ��� Ѱ�ҵ����ԵѰ�ҵ����
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

	//����������Ԥ��ͱ�Ե�ߵ�����
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



	//��¼������
	if (iOutLinePt>0&&iOutLinePt<i16PtSum)
	{
		pVehicle->FrShape.u16LeftMaxHt[CurFramI]=pVehicle->CurRegion.zdata[iOutLinePt];
		BaseX=pVehicle->CurRegion.xdata[iOutLinePt];
	}
	else
	{
		return;
	}

	//��20cmΪһ�� ���зֿ鴦��
	for (i16Index=iOutLinePt-1;i16Index>0;i16Index--)
	{
		if (pVehicle->CurRegion.zdata[i16Index]>0&&ABS(pVehicle->CurRegion.xdata[i16Index]-BaseX)<3000)
		{
			BlockMap[0][ABS(pVehicle->CurRegion.xdata[i16Index]-BaseX)/200]++;
			BlockMap[1][ABS(pVehicle->CurRegion.xdata[i16Index]-BaseX)/200]+=pVehicle->CurRegion.zdata[i16Index];
		}
	}


	//����ÿ����ĸ߶ȣ��������ƽ��������������
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

		//����Ҹ�
		pVehicle->FrShape.u16FrShape[CurFramI]=3;
	}
	else if (MaxHt-MinHt>300&&MaxHI<MinHI)
	{
		//����ҵ�
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
///* ÿ֡�������ͼ��                                                     */
///************************************************************************/
//void FramShapeCal(VehicleSet *pVehicle)
//{
//	uint16 CurFramI=0;
//	int16 i16PtSum=0;
//	int16 i16Index=0;
//	
//	uint16 ThreshXZ=50;
//	uint16 ThreshHt=800;
//	int16 iLeftLinePt=0;//�����ߵ�
//	int16 iThreshPt=0;//�ﵽ��ֵ�ĵ�һ����
//	int16 FitCnt=0;
//
//	uint16 BaseX=0;//�߽�X��
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
//	//��ȡÿ֡������ֵ
//	CurFramI=pVehicle->u16FrameCnt-1;
//	i16PtSum=pVehicle->CurRegion.xdata[0];
//	
//	//������������
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
//	//��һ�� Ѱ�Ҳ���ײ�ThreshHt��һ���� ��Ϊ�õ�����Ϊ�����߲���
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
//	//����- ���1m���ϵ�������ɵ����
//	for (i16Index=iThreshPt;i16Index>1;i16Index--)
//	{
//		if (EdgeZ[i16Index]<1&&EdgeZ[i16Index-1]<1)
//		{
//			pVehicle->FrShape.u8DaFeiCnt[CurFramI]++;
//		}
//	}
//
//	//������ ����ÿ֡�Ľ���
//
//	//�ڶ��� Ѱ�ҵ����ԵѰ�ҵ����
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
//	//����������Ԥ��ͱ�Ե�ߵ�����
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
//	//��¼������
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
//	//��20cmΪһ�� ���зֿ鴦��
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
//	//����ÿ����ĸ߶ȣ��������ƽ��������������
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
//		//����Ҹ�
//		pVehicle->FrShape.u16FrShape[CurFramI]=3;
//	}
//	else if (MaxHt-MinHt>300&&MaxHI<MinHI)
//	{
//		//����ҵ�
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
/* ���ؼ�����ֻ���                                                     */
/************************************************************************/
void CastGrdCheckCal(VehicleSet *pVehicle)
{
//	uint8 LowFixed=0;//�ײ�����
//	uint8 TopFixed=0;//��������
	int16 i16Index=0;
//	uint16 u16StaI=0;
//	uint16 u16EndI=0;
	uint16 CurFramI=0;
	int16  i16PtSum=0;//��ǰ֡����
//	uint16 u16RealIndex=0;
//	uint16 u16CastThresh=520;//Ͷ�������ϸ߶���ֵ
	uint16 u16EdgeX0=0;//����߽�x0��ʼλ��
//	uint16 u16HeadEndI=0;//��ͷ����֡
//	uint16 u16CheckCnt=0;//����֡��
//	uint16 u16CheckMaxZ=0;//��������
//	uint16 u16CheckMinZ=9999;//������С��
//	uint32 n32CheckSumZ=0;//���θ�֮��
//	uint16 u16CheckAvgZ=0;//���θ߾�ֵ
//	uint16 u16CheckMaxX=0;//���α߽�x���
//	uint16 u16CheckMinX=9999;//���α߽�x��С
//	uint32 n32CheckSumX=0;//���α߽�x֮��
//	uint16 u16CheckAvgX=0;//���α߽�x��ֵ

	uint16 FitPtCnt=0;
	


	if (pVehicle  ==  NULL)
	{
		return;
	}
	//��ȡÿ֡������ֵ
	CurFramI=pVehicle->u16FrameCnt-1;
	i16PtSum=pVehicle->CurRegion.xdata[0];
	

	//�������ؼ��  ���ж��쳣���
	if (pVehicle->AxleSet.AxleSum<1||pVehicle->u16FrameCnt<8||i16PtSum<8)
	{
		//����8����ֱ���˳�ȥ
		return;
	}
	
	u16EdgeX0=pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum-1].AxleCastX;
	//�������е� ��ȡ���30cm�ڵĽ��棬ȡ���ϵ�Ϊ������

	//�����߶�800���µ����е�  ����Ƿ��м�Ͷ��������
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
/*x����ͶӰ͹�������ػ�                                               */
/************************************************************************/
void CargoXEdgeCheck(VehicleSet *pVehicle)
{
	int16 i16Index=0;
	uint16 u16StaI=0;
	uint16 u16EndI=0;
//	uint16 u16RealIndex=0;
	uint16 u16TailCastX=0;//��β��x�߽�ֵ
//	uint16 u16HeadEndI=0;//��ͷ����֡
	uint16 u16CheckCnt=0;//����֡��
//	uint16 u16CheckMaxZ=0;//��������
//	uint16 u16CheckMinZ=9999;//������С��
//	uint32 n32CheckSumZ=0;//���θ�֮��
//	uint16 u16CheckAvgZ=0;//���θ߾�ֵ
	uint16 u16CheckMaxX=0;//���α߽�x���
	uint16 u16CheckMinX=9999;//���α߽�x��С
	uint32 n32CheckSumX=0;
	uint16 u16CheckAvgX=0;

	//����һ���� 1550Ƥ������͸߶�
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
/* �߶ȷ���ͶӰ��͹����ػ�                                             */
/************************************************************************/
void CargoZEdgeCheck(VehicleSet *pVehicle)
{












}



/************************************************************************/
/*����Ͷ�������ϼ��                                                    */
/************************************************************************/
void CastGroundCheck(VehicleSet *pVehicle)
{
	uint16  l_u16Index   = 0;
	uint16  TmpCnt=0;
	uint16  CastFrCnt=0;
	uint16  MinAxleFrCnt=9999;//������С֡��
	uint16  MaxAxleFrCnt=0;//�������֡��
	uint16  CalAxleFcCnt=0;
	uint16  StaI=0;
	uint16  EndI=0;

	if (pVehicle  ==  NULL||pVehicle->u16FrameCnt > FRAME_MAXCNT || pVehicle->u16FrameCnt < 10||pVehicle->AxleSet.AxleSum>2)
		return ;


	//���ж��ᳵ�ļ򵥾���
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



	//ȷ���������ķ�Χ��
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
			//��һ��Ϊβ����
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
/* ����ʱ�������ں�                                                     */
/************************************************************************/
void MutiFeatureFusion(VehicleSet *pVehicle)
{

	//�������� ������ʶ������������

	//���ߡ���ͷ�ߡ�����ƽ�����ж�
	VehHeightModify(pVehicle);//ÿ֡�߶ȵ���
	VehHeightCheck(pVehicle);//�������ճ���
	VehHeadHtCheck(pVehicle);//�����һ��߶� ��ͷ��
	VehRearHtCheck(pVehicle);//����ڶ���߶�
	HtRoughnessCheck(pVehicle);//�߶�ƽ���ȼ��  

	//���̸��ж�
	DpHtCheck(pVehicle);
	CastGroundCheck(pVehicle);
	//�߶ȷ��������
	HtClusterCheck(pVehicle);

	//�������ػ��ж�
	ClusterHopperCheck(pVehicle);
	AxleHopperCheck(pVehicle);
	AxleHopperCheckEx(pVehicle,pVehicle->FrShape.u16LeftMaxHt);

	//Ѱ�Ҹ߶�ͻ��㶨���� ������� ���ﳵ��
	//HeadJumpCheck(pVehicle);
	FindHeadJump(pVehicle,0);
	FindHeadJump(pVehicle,1);
	HeadJumpCheckEx(pVehicle,pVehicle->FrShape.u16LeftMaxHt);
	FindTailJump(pVehicle,0);
	FindTailJump(pVehicle,1);
	//TailJumpCheck(pVehicle);
	CargoXEdgeCheck(pVehicle);
	FramStyleCheck(pVehicle);
	//������
	ContainerCheck(pVehicle);
	TruckCanCheck(pVehicle);
	//����һ�����ж�
	SideUinformCheck(pVehicle);
	//�������
	GlassWinCheck(pVehicle);

	//�͹޼��
	//ArcCheck(&pVehicle->ArcSet);


	//ר���������
	SpecialVehCheck(pVehicle);


	//�ڳ��ߵĻ����Ͻ��������ƫ
	AxleAdjustMent(pVehicle);

	//�����ж�
	DirCheckByDev(pVehicle);

}

/************************************************************************/
/* //����ʶ����                                                         */
/************************************************************************/
void FeatureCal(VehicleSet *pVehicle)
{
	//��һ�� ����������ȡ
	//�������� ������ʶ������������
	AxleRecongnize(pVehicle);

	//�ڶ���  ����������ȡ

	//���ؼ��
	CastGrdCheckCal(pVehicle);
	FramShapeCal(pVehicle);

	//�������� ÿ֡����Ѱ���Ѿ��ó�

	//���̸�����
	DpHeightCal(pVehicle);

	//��������

	//����һ��������
	SideUniformCal(pVehicle);

	//��������
	ContainerCal(pVehicle);


	//�����������
	IntoGlassCal(pVehicle);
	SlopeGlassCal(pVehicle);
	ThroughGlassCal(pVehicle);
	MapGlassCal(pVehicle);
	//��һ��֮����͸߶�
	Axle1MinHeightCal(pVehicle);


	//���⳵����ר���������
	SpecialVehCal(pVehicle);
}

/************************************************************************/
/* //���������ж���������                                               */
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
/* //���������ж��������� --��Ȧ                                        */
/************************************************************************/
void DirJudge_Coil(VehicleSet *pVehicle)
{
	//������� 1--ǰ����2--������3--���벻��ȫ������4--��������ȫ�ֽ��룻5--�쳣δͬʱ������6--ɾ������
	if((pVehicle->DirSet.u8CL_StateHi<1) && (pVehicle->DirSet.u8CL_StateLv <= g_VehOutSet.u8DirLvCnt))  //�������˱�����ɨ�赽��ǰ����������
	{
		if(pVehicle->EdgeSet.u16HeadHeight < 2000)
		{
			 //���ڵ��̽ϵ͵�С�ͳ���˵�����ִ����Ϊ�������
			pVehicle->DirSet.u8VehDir=MOVE_FRBK;
		}
		else
		{			
			if(pVehicle->AxleSet.AxleSum<ONE_AXLE)
			{
				pVehicle->DirSet.u8VehDir = MOVE_FRBK;  //���ڴ󳵣�û�м�⵽���ᡢ���⵽һ����Ϊ˫̥�����⵽���ǰ�����ᶼΪ˫̥�������Ϊ����

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
	else if((pVehicle->DirSet.u8CL_StateHi <= g_VehOutSet.u8DirHiCnt) && (pVehicle->DirSet.u8CL_StateLv <= g_VehOutSet.u8DirLvCnt))	 //�������
	{
		pVehicle->DirSet.u8VehDir = MOVE_FRNT;	 
	}
	else if((pVehicle->DirSet.u8CL_StateHi == 0) && (pVehicle->DirSet.u8CL_StateLv > g_VehOutSet.u8DirLvCnt))	 //�������
	{
		pVehicle->DirSet.u8VehDir = MOVE_BACK;
	}
	else if((pVehicle->DirSet.u8CL_StateHi <= g_VehOutSet.u8DirHiCnt) && (pVehicle->DirSet.u8CL_StateLv > g_VehOutSet.u8DirLvCnt))  //�����ȥ����ȫ���˳�������	
	{
		if((pVehicle->JudgeSet.u8JudgePattern == FIRST_KECHE 
			|| pVehicle->JudgeSet.u8JudgePattern == FIRST_HUOCHE) 
			&& pVehicle->DirSet.u8CL_StateLw == 0) //���Ӷ�С�ͳ����ٵ������ж�
		{
			pVehicle->DirSet.u8VehDir = MOVE_BACK;
		}	
		else
		{
			pVehicle->DirSet.u8VehDir = MOVE_FRBK;
		}
	}
	else   //�쳣���
	{
		pVehicle->DirSet.u8VehDir = MOVE_ABNO;
	}
}

/************************************************************************/
/* //���������ж��������� --��դ                                        */
/************************************************************************/
void DirJudge_Grat(VehicleSet *pVehicle)
{
	//������� 1--ǰ����2--������3--���벻��ȫ������4--��������ȫ�ֽ��룻5--�쳣δͬʱ������6--ɾ������
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
/* //���������ж��������� --����                                        */
/************************************************************************/
void DirJudge_Lase(VehicleSet *pVehicle)
{
	//��ǰĬ������ǰ��
	pVehicle->DirSet.u8VehDir=MOVE_FRNT;
}


#endif
