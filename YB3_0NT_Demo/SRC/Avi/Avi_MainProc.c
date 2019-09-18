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
uint32 g_n32WaveNumber;//vc����
SysModelParam Smp;//ϵͳģ�Ͳ���
FramInfo CurFramInfo;//��ǰ֡��Ϣ
VehOutSet g_VehOutSet;//������β�����ֵ
int32 g_FiltX[POINT_SUM]; //ɨ���X�����˲�������
int32 g_FiltZ[POINT_SUM]; //ɨ���Z�����˲�������
int32 LMS_data_1[POINT_SUM+1];//ɨ���������
VehIndexSet  g_VehIndex;//������ż�¼��
int32 g_CoorX[POINT_SUM] = {0}; //ɨ���X����
int32 g_CoorZ[POINT_SUM] = {0}; //ɨ���Z����
#endif



//uint8 g_VehicleSetIndex[VEHICLE_MAX];//����������������	
//VehicleSet g_VehicleSet[VEHICLE_MAX];//������¼���ṹ��
VehicleSet g_VehicleSet;
//uint8 g_totalVehicle=0;//��ǰ�ܳ���

/************************************************************************/
/* ����ʶ����                                                         */
/************************************************************************/
void VehIdentProc(void)
{
	//����м��
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
/* ����м��                                                           */
/************************************************************************/
void InitMiddleWare(void)
{
	//�����������
	memset(g_FiltZ,0,sizeof(g_FiltZ));
	memset(g_FiltX,0,sizeof(g_FiltX));
	memset(g_CoorX,0,sizeof(g_CoorX));
	memset(g_CoorZ,0,sizeof(g_CoorZ));

	//���֡��¼��
	memset(&CurFramInfo,0,sizeof(CurFramInfo));
}

/************************************************************************/
/* ģ�Ͳ�����ʼ��                                                       */
/************************************************************************/
void InitModeParam(void)
{	
#ifdef _KeilEnVi	
	
#else 
	//���ɽ���� 20170524
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
/* ������תֱ������ �����˲�   �ִ�С�����                             */
/************************************************************************/
uint8 Polar2Rect(void)
{
	int16 u16Index=0;
	uint32 Rx=0;
	uint32 Rz=0;
	int32 dif = 0;
	uint32 ThresDis[POINT_SUM] = {0};//ÿ���˲���ֵ
	uint32 min_neighour = 65535;
	int32 TmpL=0;
	int32 abs_temp = 0;

	if( Smp.ZeroPos > POINT_CEN ) 
	{
		//�ɸߵ���
	}
	else 
	{
		// �ɵ͵���
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
			//��Ϊ��ɻ��಻����ɨ���
			g_CoorX[u16Index]=0;
			g_CoorZ[u16Index]=0;
		}
		else
		{
			//����ֱ������ת��
			if(u16Index < Mid_Point_Cal && u16Index >=0)//����ת����
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

			//���ڳ�����Χ�Ͳ��������߶ȷ�Χ��ɨ��� ��Ϊ0
			if (g_CoorX[u16Index]>Smp.StartX+Smp.LaneWidth||g_CoorZ[u16Index]>Smp.MaxHt)
			{
				g_CoorX[u16Index]=0;
				g_CoorZ[u16Index]=0;
			}
		}

		//ֻ��һ���߶Ⱥ�X��Χ�ڵĵ�����˲����� ����CPUʹ����
		if(g_CoorX[u16Index]<Smp.StartX-300||g_CoorX[u16Index]>g_CoorX[u16Index]<Smp.StartX+Smp.LaneWidth/2
			||g_CoorZ[u16Index]>1200)
		{
			g_FiltX[u16Index]=g_CoorX[u16Index];
			g_FiltZ[u16Index]=g_CoorZ[u16Index];
			continue;
		}
		
		
		Rx = TabRsin[ABS(u16Index - Mid_Point_Cal)];
		Rz = TabRcos[ABS(u16Index - Mid_Point_Cal)];


		//�����ֵΪ�˲���ֵ
		if( Rx > Rz )
			ThresDis[u16Index] = Rx*LMS_data_1[u16Index]/1000;
		else
			ThresDis[u16Index] = Rz*LMS_data_1[u16Index]/1000;

		//���û�У���ֵ��СĬ��Ϊ30mm
		if( ThresDis[u16Index] < 30 )
			ThresDis[u16Index] = 30;

		//3�����˲�
		if( u16Index > 2 && u16Index < 358 )
		{
			min_neighour = 65535;
			//abs_temp = 0;
			//if (g_CoorX[u16Index] < 100 && g_CoorX[u16Index] > Smp.StartX  &&  g_CoorX[u16Index] < Smp.StartX+Smp.LaneWidth ) //�Ը߶�100���µĵ����3���ֵ�˲�
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
				//�˵���Ҫ
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
/* ֡����                                                           */
/************************************************************************/
void FramAnlsye()
{
	uint16 u16Index=0;
	int32 iTmpMaxX=0;
	int32 iTmpMinX=0xFFFF;
	uint32 TmpSumX=0;
	//��ʼ��֡��¼��
	CurFramInfo.u16MaxHt=0;
	CurFramInfo.u16SecHt=0;
	CurFramInfo.n32xMin=0xFFFF;
	CurFramInfo.n32xMax=0;
	CurFramInfo.u32TimeStamp=LMS_data_1[POINT_SUM];
	//��һ�α���  Ѱ�ҳ���ɨ���  �����߶�����ʼ��
	for( u16Index = Mid_Point_Cal; u16Index>1; u16Index--)
	{
		//���ڳ���ʶ�� Ѱ�ҳ�������ɨ���
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
					//�ҵ�������ֹ��
					CurFramInfo.u16StaPos=u16Index;
					CurFramInfo.u16EndPos=u16Index;
				}
				else
				{
					//������ʼ��
					CurFramInfo.u16StaPos=u16Index;
				}

				if (CurFramInfo.u16MaxHt<g_FiltZ[u16Index])
				{
					//������ߵ� 
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
					//������Զ��
					CurFramInfo.n32xMax=g_FiltX[u16Index];
					CurFramInfo.u16xMaxPt=u16Index;
				}

				if (g_FiltX[u16Index]<CurFramInfo.n32xMin)
				{
					//���������
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
				//�������͵� ��Ϊ-1 ������
				//g_FiltZ[u16Index]=-1;
			}
		}
		else
		{
			//���ڷ�Χ�ڵĵ�ֱ��ȥ������ֹƽ�澵�������ɢ�� 20170729
			g_FiltZ[u16Index]=0;
			g_FiltX[u16Index]=0;
		}


	}

	//�жϸ������Ƿ�Ϊ�г��Ϸ�����
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
/* ������ƥ��                                                         */
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
			//������δƥ��ɹ�����Ϊ���³�
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
/* ��������Ƿ�ƥ��                                                     */
/* ������Ĭ��ƥ��ɹ�    �˴������������λ��Լ�����ڷֳ�������         */
/************************************************************************/
uint8 IsRegionMatch(FramInfo FI1,FramInfo FI2)
{
	//ͬһ���� Ĭ��ƥ��ɹ�  �˴������������λ��Լ�����ڷֳ�������
	if (IS_INSIDE(FI1.n32xMin,FI1.n32xMax,FI2.n32xMin,FI2.n32xMax))
	{
		//�����ص�
		return 1;
	}
	else if (abs((FI1.n32xMax+FI1.n32xMin)-(FI2.n32xMax+FI2.n32xMin))<800)
	{
		//�������ĵ���400����
		return 1;
	}
	else if ((FI1.n32xMax<FI2.n32xMin&&FI2.n32xMin-FI1.n32xMax<400)
		||(FI2.n32xMax<FI1.n32xMin&&FI1.n32xMin-FI2.n32xMax<400))
	{
		//����߽���400����
		return 1;
	}
	else
	{
		return 1;
	}
}

#ifdef _KeilEnVi
/************************************************************************/
/* ��֪�����н�����                                                     */
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
/* ��Ȧ��֪�������                                                     */
/************************************************************************/
void Dev_CoilDirCheck(void)
{
	if(g_DirTrigged)
	{
		//�����豸ʹ�� �뿪״̬������Ч
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
/* ��դ��֪�������                                                     */
/************************************************************************/
void Dev_GratDirCheck(void)
{
	
	//g_VehOutSet.u8DirLwCnt �Ƽ�Ϊ5
	//g_VehOutSet.u8DirHiCnt �Ƽ�Ϊ40
	if (g_VehicleSet.DirSet.u8GR_Valided && g_VehicleSet.DirSet.u16GR_FitFrCnt< g_VehOutSet.u8DirLwCnt && !g_DirTrigged) //ǰ5֡�����г� ��դ�޳�
			g_VehicleSet.DirSet.u8GR_StateLS++;
	
	if (g_VehicleSet.DirSet.u8GR_Valided && g_DirTrigged)//�����й�դҲ��
	{
		g_VehicleSet.DirSet.u8GR_StateLG++;
		if (g_VehicleSet.DirSet.u8GR_StateLG> g_VehOutSet.u8DirHiCnt)
			g_VehicleSet.DirSet.u8GR_StateLG = g_VehOutSet.u8DirHiCnt;
	}
	
	if (g_VehicleSet.DirSet.u8GR_Valided)
		g_VehicleSet.DirSet.u8GR_StateGR = 0;
	else
	{
		//���м����޳�����դ�г�
		if (g_DirTrigged)
			g_VehicleSet.DirSet.u8GR_StateGR++;
	}
}

#endif


/************************************************************************/
/* �����֪�������                                                     */
/************************************************************************/
void Dev_LaseDirCheck(void)
{
	
}

#ifdef _KeilEnVi

/************************************************************************/
/* ֡���������		                                                    */
/************************************************************************/
void CheckAbFramCnt(void)
{
	g_DllAtState.CheckFrCnt++;
	//����֡������6000�����Ҷ������г� ��ôɾ��ͷ��	
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
/* ������			                                                    */
/************************************************************************/
void CheckQueState(void)
{
	if(g_DllAtState.QueCheckFlag)
	{
		g_DllAtState.QueCheckFlag = 0;
		VehQueCheck(); //���ж��еļ��
	
//		if( ((uint32)RTC_UCOUNT - g_u32SecondSecCnt) > 60 )
//		{
//			g_u32SecondSecCnt		= (uint32)RTC_UCOUNT; // ǿ�Ƴ�����Ҳ���г�
//			SendClearVeh();
//		}
		
	}
}

/************************************************************************/
/* ǿ�Ƴ�������		                                                    */
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
			//�ϴ�����ǰ���ж��м��
			if(g_u8VehQueNum) //�������г���
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
			
			//ǿ����β������Ϣ��ӵ��ϴ���������20150519
			g_u8VehQue[g_u8VehQueIndex] = JudgeType; //ǿ����β�ķ���Ϊ0��ֱ������λ����������ӳ�����Ϣ
			g_u8VehQueIndex = (g_u8VehQueIndex + 1) % MAXVEHQUE; //���¶�������
		}
		else
		{
			//��յ�ǰ������¼
			memset(&g_VehicleSet,0,sizeof(g_VehicleSet));
		}	
	}
}

/************************************************************************/
/* ǿ�Ƴ��������ж�	                                                    */
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
/* ����ǿ�Ƴ�������			                                            */
/************************************************************************/
void ReportForceVeh(VehicleSet *pVehicle,uint8 ForcType)
{
	uint8 UpBuff[100]={0};
	uint8 TmpI=0;
	uint8 u8Index=0;
	uint8 u8TyreCnt=0;

	//���Կڳ���
	//���Կ��ϴ�����	
	//֡ͷ�����
	TmpI=0;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=CMD_UPLOADVEHS;
	
	//�ϴ�����
	TmpI=5;
	
	//�����
	g_VehIndex.u32Index++;//ȫ�ּ���++��ÿ���0��ʼ
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
	
	//����
	UpBuff[TmpI++]=ForcType;
	UpBuff[TmpI++]=MOVE_ABNO;
	//����
	UpBuff[TmpI++]=pVehicle->AxleSet.AxleSum;
	//����
	for(u8Index=0;u8Index<pVehicle->AxleSet.AxleSum;u8Index++)
	{
		UpBuff[TmpI++]=pVehicle->AxleSet.Axle[u8Index].AxleType;
	}
	
	//�ж�׼��
	UpBuff[TmpI++]=(pVehicle->JudgeSet.u8JudgeRule&0xFF);
	//����׼��
	UpBuff[TmpI++]=(pVehicle->JudgeSet.u8ModifyRule&0xFF);
	
	//ר����
	UpBuff[TmpI++]=(pVehicle->JudgeSet.u8SpeVehCode&0xFF);
	
	//���̸�
	UpBuff[TmpI++]=((pVehicle->Dp.DpHeight1_2>>8)&0xFF);
	UpBuff[TmpI++]=(pVehicle->Dp.DpHeight1_2&0xFF);
	//��ͷ��
	UpBuff[TmpI++]=((pVehicle->EdgeSet.u16HeadHeight>>8)&0xFF);
	UpBuff[TmpI++]=(pVehicle->EdgeSet.u16HeadHeight&0xFF);
	//����
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsHopper&0xFF);
	
	//ͷ����
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsHeadJump&0xFF);
	
	//β����
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsTailJump&0xFF);
	
	//�ػ�
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsCargo&0xFF);
	
	//����
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsContainer&0xFF);
	
	//����β
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsTruckTail&0xFF);
	
	//Ͷ��������
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsCastGrd&0xFF);


	//��ճ���
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsFillWin&0xFF);	
	
	//����
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsGlassWin&0xFF);
	
	//����һ��
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsSideUiform&0xFF);	
	

	//�����豸����
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
	
	//Ԥ��24 ����ע ��¼�ļ�������
	memcpy(&UpBuff[TmpI],g_u8ItemName,24);
	TmpI+=24;


	//����λ
	UpBuff[3]=(((TmpI+2)>>8)&0xFF);
	UpBuff[4]=((TmpI+2)&0xFF);
	
	//У��λ
	AddCrc16(UpBuff,TmpI);
	SendBuf_full(UpBuff,TmpI + 2,g_UartSet.TS_CHCOM);	

	if(g_DllAtState.DllOpened)
	{
		//�ϱ����ϴ�����
		TmpI=0;
		memset(UpBuff,0,100);
		//֡ͷ+�����
		UpBuff[TmpI++]=0xFF;
		UpBuff[TmpI++]=0xFF;
		UpBuff[TmpI++]=DLL_VHCHECKINFO;
		
		//�ϴ�����
		TmpI=5;
		UpBuff[TmpI++]=g_VehIndex.u32Index&0xFF;//�����
		UpBuff[TmpI++]=0;//Ԥ��1λ
		UpBuff[TmpI++]=pVehicle->AxleSet.AxleSum;//����
		UpBuff[TmpI++]=pVehicle->JudgeSet.u8JudgePattern;//����
		UpBuff[TmpI++]=0;//Ԥ��1λ
		if(pVehicle->AxleSet.Axle[1].AxleType== 0x01 )
			u8TyreCnt = 20;	 //����
		else
			u8TyreCnt = 22;	 //����		
		UpBuff[TmpI++]=u8TyreCnt;//��̥����
		UpBuff[TmpI++]=0;//���ٶ�
		UpBuff[TmpI++]=0;//�н�����
		UpBuff[TmpI++]=0;//ר������
		
		//����λ
		UpBuff[3]=(((TmpI+2)>>8)&0xFF);
		UpBuff[4]=((TmpI+2)&0xFF);
		
		//У��λ
		AddCrc16(UpBuff,TmpI);
		SendBuf_full(UpBuff,TmpI + 2,g_UartSet.RM_CHCOM);	
	}
}

/************************************************************************/
/* //ɾ������ͷһ����		                                            */
/************************************************************************/
void SendHeadVeh(void) 
{
	uint8 UpBuff[100]={0};
	uint8 TmpI=0;
	uint8 u8Index=0;
	
	//���Կ��ϴ�����	
	//֡ͷ�����
	TmpI=0;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=CMD_UPLOADVEHS;
	
	//�ϴ�����
	TmpI=5;
	
	//�����
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
	
	//����
	UpBuff[TmpI++]=1;
	//����
	UpBuff[TmpI++]=6;
	//����
	UpBuff[TmpI++]=2;
	//����
	for(u8Index=0;u8Index<2;u8Index++)
	{
		UpBuff[TmpI++]=1;
	}
	
	//�ж�׼��
	UpBuff[TmpI++]=0;
	//����׼��
	UpBuff[TmpI++]=0;
	
	//ר����
	UpBuff[TmpI++]=0;
	
	//���̸�
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	//��ͷ��
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	//����
	UpBuff[TmpI++]=0;
	
	//ͷ����
	UpBuff[TmpI++]=0;
	
	//β����
	UpBuff[TmpI++]=0;
	
	//�ػ�
	UpBuff[TmpI++]=0;
	
	//����
	UpBuff[TmpI++]=0;
	
	//����β
	UpBuff[TmpI++]=0;
	
	//Ͷ��������
	UpBuff[TmpI++]=0;

	//��ճ���
	UpBuff[TmpI++]=0;	
	
	//����
	UpBuff[TmpI++]=0;
	
	//����һ��
	UpBuff[TmpI++]=0;	
	
	//�����豸����
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;	
	
	//Ԥ��24 ����ע ��¼�ļ�������
	memcpy(&UpBuff[TmpI],g_u8ItemName,24);
	TmpI+=24;



	//����λ
	UpBuff[3]=(((TmpI+2)>>8)&0xFF);
	UpBuff[4]=((TmpI+2)&0xFF);
	
	//У��λ
	AddCrc16(UpBuff,TmpI);
	SendBuf_full(UpBuff,TmpI + 2,g_UartSet.TS_CHCOM);
	
	
	//�ϴ��ڳ��� 
	if(g_DllAtState.DllOpened)
	{
		memset(UpBuff,0,100);
		TmpI=0;
		UpBuff[TmpI++] = 0xFF;
		UpBuff[TmpI++] = 0xFF;
		UpBuff[TmpI++]= DLL_VHCHECKINFO; 
		
		TmpI=5;
		UpBuff[TmpI++] = g_VehIndex.u32Index &0xFF;	  //����ID
		UpBuff[TmpI++] = 0;  //������         
		UpBuff[TmpI++] = 2;	 //����
		UpBuff[TmpI++] = 1; //����
		UpBuff[TmpI++] = 0;	 //������
		UpBuff[TmpI++] = 0;	 //����
		UpBuff[TmpI++] = 0;	 //���ٶ�
		UpBuff[TmpI++] = 6;	 //ɾ��
		UpBuff[TmpI++]=0;//ר������
		
		//����λ
		UpBuff[3]=(((TmpI+2)>>8)&0xFF);
		UpBuff[4]=((TmpI+2)&0xFF);
		
		AddCrc16(UpBuff,TmpI); 
		SendBuf_full(UpBuff,TmpI + 2,g_UartSet.RM_CHCOM);	
	}		
}

/************************************************************************/
/* //����һ����				                                            */
/************************************************************************/
void SendAddVeh(uint8 u8AddType)
{
	uint8 UpBuff[100]={0};
	uint8 TmpI=0;
	uint8 u8Index=0;
	uint8 u8AxleCnt=0;
	
	//ͨ�����Ͷ�����
	switch(u8AddType)
	{
		case 1:
		case 2:
		case 3:
		case 4:
		case 5:
		case 6:
			u8AxleCnt = 2;           //������
		  break;
		case 7:
			u8AxleCnt = 3;           //������
		  break;
		case 8:
			u8AxleCnt = 4;           //������
		  break;
		case 9:
			u8AxleCnt = 5;           //������
		  break;
		default:
			u8AxleCnt = 2;           //������
		  break;
	}
	
	
	//���Կ��ϴ�����	
	//֡ͷ�����
	TmpI=0;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=CMD_UPLOADVEHS;
	
	//�ϴ�����
	TmpI=5;
	
	//�����
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
	
	//����
	UpBuff[TmpI++]=u8AddType;
	//����
	UpBuff[TmpI++]=MOVE_FRNT;
	//����
	UpBuff[TmpI++]=u8AxleCnt;
	//����
	for(u8Index=0;u8Index<2;u8Index++)
	{
		UpBuff[TmpI++]=1;
	}
	
	//�ж�׼��
	UpBuff[TmpI++]=0;
	//����׼��
	UpBuff[TmpI++]=0;
	
	//ר����
	UpBuff[TmpI++]=0;
	
	//���̸�
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	//��ͷ��
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	//����
	UpBuff[TmpI++]=0;
	
	//ͷ����
	UpBuff[TmpI++]=0;
	
	//β����
	UpBuff[TmpI++]=0;
	
	//�ػ�
	UpBuff[TmpI++]=0;
	
	//����
	UpBuff[TmpI++]=0;
	
	//����β
	UpBuff[TmpI++]=0;
	
	//Ͷ��������
	UpBuff[TmpI++]=0;

	//��ճ���
	UpBuff[TmpI++]=0;	
	
	//����
	UpBuff[TmpI++]=0;
	
	//����һ��
	UpBuff[TmpI++]=0;	
	
	//�������
	UpBuff[TmpI++]=0;	
	UpBuff[TmpI++]=0;	
	UpBuff[TmpI++]=0;	
	
	//Ԥ��24 ����ע ��¼�ļ�������
	memcpy(&UpBuff[TmpI],g_u8ItemName,24);
	TmpI+=24;



	//����λ
	UpBuff[3]=(((TmpI+2)>>8)&0xFF);
	UpBuff[4]=((TmpI+2)&0xFF);
	
	//У��λ
	AddCrc16(UpBuff,TmpI);
	SendBuf_full(UpBuff,TmpI + 2,g_UartSet.TS_CHCOM);
	
	
	//�ϴ��ڳ��� 
	if(g_DllAtState.DllOpened)
	{
		memset(UpBuff,0,100);
		TmpI=0;
		UpBuff[TmpI++] = 0xFF;
		UpBuff[TmpI++] = 0xFF;
		UpBuff[TmpI++]= DLL_VHCHECKINFO; 
		
		TmpI=5;
		UpBuff[TmpI++] = g_VehIndex.u32Index &0xFF;	  //����ID
		UpBuff[TmpI++] = 0;  //������         
		UpBuff[TmpI++] = u8AxleCnt;	 //����
		UpBuff[TmpI++] = u8AddType; //����
		UpBuff[TmpI++] = 0;	 //������
		UpBuff[TmpI++] = 0;	 //����
		UpBuff[TmpI++] = 0;	 //���ٶ�
		UpBuff[TmpI++] = 1;	 //����
		UpBuff[TmpI++]=0;//ר������
		
		//����λ
		UpBuff[3]=(((TmpI+2)>>8)&0xFF);
		UpBuff[4]=((TmpI+2)&0xFF);
		
		AddCrc16(UpBuff,TmpI); 
		SendBuf_full(UpBuff,TmpI + 2,g_UartSet.RM_CHCOM);	
	}		
	g_u8VehQue[g_u8VehQueIndex] = u8AddType;
	g_u8VehQueIndex = (g_u8VehQueIndex+1)%MAXVEHQUE;
	
}

/************************************************************************/
/* ����Back����		                                                    */
/************************************************************************/
void SendBackVeh(void)
{
	uint8 UpBuff[100]={0};
	uint8 TmpI=0;
	uint8 u8Index=0;
	
	//���Կ��ϴ�����	
	//֡ͷ�����
	TmpI=0;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=CMD_UPLOADVEHS;
	
	//�ϴ�����
	TmpI=5;
	
	//�����
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
	
	//����
	UpBuff[TmpI++]=1;
	//����
	UpBuff[TmpI++]=MOVE_BACK;
	//����
	UpBuff[TmpI++]=2;
	//����
	for(u8Index=0;u8Index<2;u8Index++)
	{
		UpBuff[TmpI++]=1;
	}
	
	//�ж�׼��
	UpBuff[TmpI++]=0;
	//����׼��
	UpBuff[TmpI++]=0;
	
	//ר����
	UpBuff[TmpI++]=0;
	
	//���̸�
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	//��ͷ��
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	//����
	UpBuff[TmpI++]=0;
	
	//ͷ����
	UpBuff[TmpI++]=0;
	
	//β����
	UpBuff[TmpI++]=0;
	
	//�ػ�
	UpBuff[TmpI++]=0;
	
	//����
	UpBuff[TmpI++]=0;
	
	//����β
	UpBuff[TmpI++]=0;
	
	//Ͷ��������
	UpBuff[TmpI++]=0;


	//��ճ���
	UpBuff[TmpI++]=0;	
	
	//����
	UpBuff[TmpI++]=0;
	
	//����һ��
	UpBuff[TmpI++]=0;	
	
	//�����豸����
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;
	UpBuff[TmpI++]=0;	
	
	//Ԥ��24 ����ע ��¼�ļ�������
	memcpy(&UpBuff[TmpI],g_u8ItemName,24);
	TmpI+=24;

	//����λ
	UpBuff[3]=(((TmpI+2)>>8)&0xFF);
	UpBuff[4]=((TmpI+2)&0xFF);
	
	//У��λ
	AddCrc16(UpBuff,TmpI);
	SendBuf_full(UpBuff,TmpI + 2,g_UartSet.TS_CHCOM);
	
	
	//�ϴ��ڳ��� 
	if(g_DllAtState.DllOpened)
	{
		memset(UpBuff,0,100);
		TmpI=0;
		UpBuff[TmpI++] = 0xFF;
		UpBuff[TmpI++] = 0xFF;
		UpBuff[TmpI++]= DLL_VHCHECKINFO; 
		
		TmpI=5;
		UpBuff[TmpI++] = g_VehIndex.u32Index &0xFF;	  //����ID
		UpBuff[TmpI++] = 0;  //������         
		UpBuff[TmpI++] = 2;	 //����
		UpBuff[TmpI++] = 1; //����
		UpBuff[TmpI++] = 0;	 //������
		UpBuff[TmpI++] = 0;	 //����
		UpBuff[TmpI++] = 0;	 //���ٶ�
		UpBuff[TmpI++] = 2;	 //����
		UpBuff[TmpI++]=0;//ר������
		
		//����λ
		UpBuff[3]=(((TmpI+2)>>8)&0xFF);
		UpBuff[4]=((TmpI+2)&0xFF);
		
		AddCrc16(UpBuff,TmpI); 
		SendBuf_full(UpBuff,TmpI + 2,g_UartSet.RM_CHCOM);	
	}	
}

/************************************************************************/
/* ���Ͷ�����Ϣ				                                            */
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
		TmpBuff[3] = ((tempindex + 2)>>8)&0xFF;//Э��֡�ֽ���
		TmpBuff[4] = (tempindex + 2)&0xFF;

		AddCrc16(TmpBuff,	tempindex);
		SendBuf_full(TmpBuff,tempindex + 2,CH);	
	}
	else
	{
		
	}	
}


/************************************************************************/
/* ������״̬				                                            */
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
	if(g_u8VehQueNum == 2) //��������Ϊ2�����
	{
		Ke3Cnt = 0;
		Huo3Cnt = 0;
		for(i=1;i<=2;i++)
		{
		
			Index = (g_u8VehQueIndex >= i) ? (g_u8VehQueIndex-i):(g_u8VehQueIndex + MAXVEHQUE - i); //��ȡ��ǰ������Ϣ����
//			if(g_u8VehQue[Index] >= 7 || //��������һ����Ϊ��3����4����5����
//				 g_u8VehQue[Index] == 3 || g_u8VehQue[Index] == 4) // ����Ϊ��3����4����  ����
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
		if(Flag) //����Ϊ���������ǳ����˴󳵣���ʾ���ж೵
		{
			SendHeadVeh(); //ɾ������ͷһ��������λ���������鲻����в���
//			SendBackVeh(); //ɾ��һ����		
//			g_u8VehQueIndex = (g_u8VehQueIndex >= 1) ? (g_u8VehQueIndex - 1):(g_u8VehQueIndex + MAXVEHQUE - 1);
//			g_u8VehQue[g_u8VehQueIndex] = 0;
		}
	}
	else if(g_u8VehQueNum == 3) //��������Ϊ3�����
	{
		Ke1Cnt = 0;
		Huo1Cnt = 0;
		Huo3Cnt = 0;
		Ke3Cnt = 0;
		Flag = 0;
		for(i=1;i<=3;i++)
		{
			Index = (g_u8VehQueIndex >= i) ? (g_u8VehQueIndex-i):(g_u8VehQueIndex + MAXVEHQUE - i); //��ȡ��ǰ������Ϣ����
			if(g_u8VehQue[Index] == 1) //������3�����п�1����
				Ke1Cnt++;
			else if(g_u8VehQue[Index] == 5) //������3�����л�1����
				Huo1Cnt++; 
			else if(g_u8VehQue[Index] >= 8 )
				Huo3Cnt ++;
			else if(g_u8VehQue[Index] == 3 || g_u8VehQue[Index] == 4)
				Ke3Cnt ++;
			else
				;
//				Flag = 1; //�����˷ǿ�1�ͻ�1�ĳ�
		}
		if(Flag || (Huo3Cnt >= 1 ) || (Ke3Cnt >= 1) ) //���ж೵��ɾһ����
		{
			SendBackVeh(); //ɾ��һ����
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
			Index = (g_u8VehQueIndex >= i) ? (g_u8VehQueIndex-i):(g_u8VehQueIndex + MAXVEHQUE - i); //��ȡ��ǰ������Ϣ����
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
				SendBackVeh(); //ɾ��һ����
				g_u8VehQueIndex = (g_u8VehQueIndex >= 1) ? (g_u8VehQueIndex - 1):(g_u8VehQueIndex + MAXVEHQUE - 1);
				g_u8VehQue[g_u8VehQueIndex] = 0;
			}
	}
	else if(g_u8VehQueNum > 4) //����������3�����ж೵
	{
//		SendBackVeh(); //ɾ��һ����
//		g_u8VehQueIndex = (g_u8VehQueIndex >= 1) ? (g_u8VehQueIndex - 1):(g_u8VehQueIndex + MAXVEHQUE - 1);
//		g_u8VehQue[g_u8VehQueIndex] = 0;
		SendHeadVeh(); //ɾ������ͷһ��������λ���������鲻����в���
	}
		
}

#endif

/************************************************************************/
/* ������                                                             */
/************************************************************************/
void ClearVehicleErr()
{

}

/************************************************************************/
/* ���³���������Ϣ                                                     */
/************************************************************************/
void UpdateVehRegion(VehicleSet *pVehicle)
{
	//��������
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

	//�������һ֡����
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
		//��ֹɢ�� ��ɱ߽�ͶӰ�쳣 20170725
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
	//����������������
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

	//����������ȡ
	FeatureCal(pVehicle);
}


/************************************************************************/
/* ��鳵��״̬                                                         */
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
			g_VehicleSet.u8State = PASSED_USED;  //�ѽ���������β�ĳ�	
		}
#ifndef _KeilEnVi
		if (g_ForceOut)
		{
			g_VehicleSet.u8State = PASSED_USED;//ǿ����β������
		}
#endif

		if(g_VehicleSet.u8State == PASSED_USED) 
		{  
			//��β���� ���г���ʶ��	
			OutPutVeh(&g_VehicleSet);
#ifdef _KeilEnVi				
			Catch_Frame_RT(&g_VehicleSet,2);
			if(g_WeigthAt.u8wCarClassify)
			{
				//��IO���� 
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
				//�����˳�3��ǿ�Ƴ���
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
/*IsVehCheck ����Ƿ�Ϊ����                                             */
/*1-�ǳ�����0-���ǳ���		                                            */
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
/* ��������                                                             */
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
		
		//�ϴ�����ǰ���ж��м��
		if(g_u8VehQueNum && pVehicle->DirSet.u8VehDir != MOVE_BACK) //�������г���,�ҵ�ǰ��������Ϊ2
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
			
			//��������Ϣ��ӵ�����
			if(pVehicle->DirSet.u8VehDir != 2 && pVehicle->DirSet.u8VehDir != 3) //����Ϊ2��3�ĳ����������
			{
				g_u8VehQue[g_u8VehQueIndex] = pVehicle->JudgeSet.u8JudgePattern; //������Ϣ�����
				g_u8VehQueIndex = (g_u8VehQueIndex + 1) % MAXVEHQUE; //���¶�������
			}
			else if(pVehicle->DirSet.u8VehDir == 2) //����Ϊ2�Ľ��м�������
			{
				if(g_u8VehQueIndex >= 1)
					g_u8VehQueIndex = g_u8VehQueIndex - 1;
				else
					g_u8VehQueIndex = MAXVEHQUE - 1;

					g_u8VehQue[g_u8VehQueIndex] = 0; //���ö���β������Ϣ���
			}
		}
		else
		{
			if(pVehicle->AxleSet.AxleSum > SIX_AXLE && g_u8VehQueNum == 1 &&
			pVehicle->AxleSet.Axle[pVehicle->AxleSet.AxleSum - 1].AxleType == 1) //ǿ�ư��������󣬳����ֵ�����������Ҫ��ǿ�Ƴ��ĳ�ɾ��
			{ 
				//���ж��е�ɾ������
				SendBackVeh();

				//����Ϊ2�����ж���β������Ϣ���
				if(g_u8VehQueIndex >= 1)
					g_u8VehQueIndex = g_u8VehQueIndex - 1;
				else
					g_u8VehQueIndex = MAXVEHQUE - 1;

				g_u8VehQue[g_u8VehQueIndex] = 0; //���ö���β������Ϣ���

			}						
		}
#endif
	}
	
}

#ifndef _KeilEnVi
/************************************************************************/
/* д������txt�ı�                                                    */
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

	//��֡��
	itoa(pVehicle->u16FrameCnt, &l_acVehicleInfo[index], 10); 
	index+=6;


	//��ʼ֡
	itoa(pVehicle->StaWaveIndex, &l_acVehicleInfo[index], 10); 
	index+=6;

	//��ֹ֡
	itoa(pVehicle->EndWaveIndex, &l_acVehicleInfo[index], 10); 
	index+=6;


	//����
	itoa(pVehicle->AxleSet.AxleSum, &l_acVehicleInfo[index], 10); 
	index+=4;

	//����
	for (u16Idex=0;u16Idex<pVehicle->AxleSet.AxleSum;u16Idex++)
	{
		itoa(pVehicle->AxleSet.Axle[u16Idex].AxleType, &l_acVehicleInfo[index], 10); 
		index+=1;
	}

	index=index+8-pVehicle->AxleSet.AxleSum;
	

	//����
	itoa(pVehicle->JudgeSet.u8JudgePattern, &l_acVehicleInfo[index], 10); 
	index+=4;

	//�ж�׼��
	itoa(pVehicle->JudgeSet.u8JudgeRule, &l_acVehicleInfo[index], 10); 
	index+=4;

	//����׼��
	itoa(pVehicle->JudgeSet.u8ModifyRule, &l_acVehicleInfo[index], 10); 
	index+=4;

	//ר������
	itoa(pVehicle->JudgeSet.u8SpeVehCode, &l_acVehicleInfo[index], 10);
	index+=2;


	//��ͷ��
	itoa(pVehicle->EdgeSet.u16HeadHeight, &l_acVehicleInfo[index], 10); 
	index+=6;

	//���̸�
	itoa(pVehicle->Dp.DpHeight1_2, &l_acVehicleInfo[index], 10); 
	index+=6;

	//����
	itoa(pVehicle->FeatureSet.IsHopper, &l_acVehicleInfo[index], 10); 
	index+=2;


	//ͷ����
	itoa(pVehicle->FeatureSet.IsHeadJump, &l_acVehicleInfo[index], 10); 
	index+=2;
	

	//β����
	itoa(pVehicle->FeatureSet.IsTailJump, &l_acVehicleInfo[index], 10); 
	index+=2;
	
	//�ػ�
	itoa(pVehicle->FeatureSet.IsCargo, &l_acVehicleInfo[index], 10); 
	index+=2;


	//����
	itoa(pVehicle->FeatureSet.IsContainer, &l_acVehicleInfo[index], 10); 
	index+=2;

	//����β
	itoa(pVehicle->FeatureSet.IsTruckTail, &l_acVehicleInfo[index], 10); 
	index+=2;

	//Ͷ��������
	itoa(pVehicle->FeatureSet.IsCastGrd, &l_acVehicleInfo[index], 10); 
	index+=2;


	//��ճ���
	itoa(pVehicle->FeatureSet.IsFillWin, &l_acVehicleInfo[index], 10); 
	index+=2;

	//����
	itoa(pVehicle->FeatureSet.IsGlassWin, &l_acVehicleInfo[index], 10); 
	index+=2;

	//����һ��
	itoa(pVehicle->FeatureSet.IsSideUiform, &l_acVehicleInfo[index], 10); 
	index+=2;

	l_acVehicleInfo[index] = '\r';
	index++;
	l_acVehicleInfo[index] = '\n';

	if ((l_pFile = fopen("avi.txt","a+")) != NULL) //�򿪳ɹ�
	{
		fwrite(l_acVehicleInfo,index+1,1,l_pFile);
	}

	fclose(l_pFile);
}
#else 
/************************************************************************/
/* �ϴ�������Ϣ  		                                                */
/************************************************************************/
void ReportNormalVeh(VehicleSet *pVehicle)
{
	uint8 UpBuff[100]={0};
	uint8 TmpI=0;
	uint8 u8Index=0;
	uint8 u8TyreCnt=0;
	
	//���Կ��ϴ�����	
	//֡ͷ�����
	TmpI=0;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=0xFF;
	UpBuff[TmpI++]=CMD_UPLOADVEHS;
	
	//�ϴ�����
	TmpI=5;
	
	//�����
	g_VehIndex.u32Index++;//ȫ�ּ���++��ÿ���0��ʼ
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
	
	//����
	UpBuff[TmpI++]=pVehicle->JudgeSet.u8JudgePattern;
	//����
	UpBuff[TmpI++]=pVehicle->DirSet.u8VehDir;
	
	//����
	UpBuff[TmpI++]=pVehicle->AxleSet.AxleSum;
	//����
	for(u8Index=0;u8Index<pVehicle->AxleSet.AxleSum;u8Index++)
	{
		UpBuff[TmpI++]=pVehicle->AxleSet.Axle[u8Index].AxleType;
	}
	
	//�ж�׼��
	UpBuff[TmpI++]=(pVehicle->JudgeSet.u8JudgeRule&0xFF);
	//����׼��
	UpBuff[TmpI++]=(pVehicle->JudgeSet.u8ModifyRule&0xFF);
	
	//ר����
	UpBuff[TmpI++]=(pVehicle->JudgeSet.u8SpeVehCode&0xFF);
	
	//���̸�
	UpBuff[TmpI++]=((pVehicle->Dp.DpHeight1_2>>8)&0xFF);
	UpBuff[TmpI++]=(pVehicle->Dp.DpHeight1_2&0xFF);
	//��ͷ��
	UpBuff[TmpI++]=((pVehicle->EdgeSet.u16HeadHeight>>8)&0xFF);
	UpBuff[TmpI++]=(pVehicle->EdgeSet.u16HeadHeight&0xFF);
	//����
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsHopper&0xFF);
	
	//ͷ����
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsHeadJump&0xFF);
	
	//β����
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsTailJump&0xFF);
	
	//�ػ�
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsCargo&0xFF);
	
	//����
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsContainer&0xFF);
	
	//����β
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsTruckTail&0xFF);
	
	//Ͷ��������
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsCastGrd&0xFF);


	//��ճ���
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsFillWin&0xFF);	
	
	//����
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsGlassWin&0xFF);
	
	//����һ��
	UpBuff[TmpI++]=(pVehicle->FeatureSet.IsSideUiform&0xFF);
	
	//�����豸����
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
	
	//Ԥ��24 ����ע ��¼�ļ�������
	memcpy(&UpBuff[TmpI],g_u8ItemName,24);
	TmpI+=24;

	

	//����λ
	UpBuff[3]=(((TmpI+2)>>8)&0xFF);
	UpBuff[4]=((TmpI+2)&0xFF);
	
	//У��λ
	AddCrc16(UpBuff,TmpI);
	SendBuf_full(UpBuff,TmpI + 2,g_UartSet.TS_CHCOM);

	if(g_DllAtState.DllOpened
		&& pVehicle->DirSet.u8DirTrigged)
	{
		//�ϱ����ϴ�����
		TmpI=0;
		memset(UpBuff,0,100);
		//֡ͷ+�����
		UpBuff[TmpI++]=0xFF;
		UpBuff[TmpI++]=0xFF;
		UpBuff[TmpI++]=DLL_VHCHECKINFO;
		
		//�ϴ�����
		TmpI=5;
		UpBuff[TmpI++]=g_VehIndex.u32Index&0xFF;//�����
		UpBuff[TmpI++]=0;//Ԥ��1λ
		UpBuff[TmpI++]=pVehicle->AxleSet.AxleSum;//����
		UpBuff[TmpI++]=pVehicle->JudgeSet.u8JudgePattern;//����
		UpBuff[TmpI++]=0;//Ԥ��1λ
		for(u8Index=0;u8Index<pVehicle->AxleSet.AxleSum;u8Index++)
		{
			u8TyreCnt+=pVehicle->AxleSet.Axle[u8Index].AxleType*2;//����������������
		}
		UpBuff[TmpI++]=u8TyreCnt;//��̥����
		UpBuff[TmpI++]=0;//���ٶ�
		UpBuff[TmpI++]=pVehicle->DirSet.u8VehDir;//�н�����
		UpBuff[TmpI++]=pVehicle->JudgeSet.u8SpeVehCode;//ר������
		
		//����λ
		UpBuff[3]=(((TmpI+2)>>8)&0xFF);
		UpBuff[4]=((TmpI+2)&0xFF);
		
		//У��λ
		AddCrc16(UpBuff,TmpI);
		SendBuf_full(UpBuff,TmpI + 2,g_UartSet.RM_CHCOM);	
	}
}

#endif


