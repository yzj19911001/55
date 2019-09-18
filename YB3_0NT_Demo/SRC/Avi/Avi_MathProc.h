#ifndef _AVIMATHPROC_H
#define _AVIMATHPROC_H

#include "JZGlobal.h"



#define   ABS(X)   ((X) >= 0 ? (X) : -(X))

#ifndef MAX
#define MAX(a,b)            (((a) > (b)) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a,b)            (((a) < (b)) ? (a) : (b))
#endif


#ifndef IS_INSIDE
#define IS_INSIDE(x,y,dx,dy)	( (MIN(dx,dy) >= MAX(x,y) || MAX(dx,dy) <= MIN(x,y)) ? 0:1 )
#endif


#ifndef _KeilEnVi
#include <math.h>
#endif



JZG_EXT unsigned short Tabsin[181];//���ұ�
JZG_EXT unsigned short Tabcos[181];//���ұ�
//JZG_EXT unsigned short Tabtan[181];//���б�
//JZG_EXT unsigned short Tabcot[181];//���б�

JZG_EXT unsigned  short TabRsin[361];//��Rx��sinϵ��
JZG_EXT unsigned int TabRcos[361];//��Rz��cosϵ��

JZG_EXT int32 Average(const int32 *a,uint16 num);//��ƽ��ֵ

int32 MeanFilterU(uint16 *pData,int n);//n�����ֵ�˲�
int32 MeanFilterI(int32 *pData,int n);//n�����ֵ�˲�
void IndicateTrends(int32 *pdata,int32 *pIndicate,int len);//������߱仯һ����
void IndicatePeaks(int32 *pdata,int32 *pIndicate,int len);//��Ƿ��
void ShapeAnalysisEx(uint16 *pData,ClusterInfo *pCluster);//���μ���㷨

void IndicateTrend(int32 *pdata,int32 *pIndicate,int len);
void ShapeAnalysis(uint16 *pData,ClusterInfo *pCluster);//���μ�ⷽ��
uint16 MIN_N_NEAR(uint16 *pSta,uint16 N,uint16 B_F);//N������Сֵ
void CalcVehHeight(uint16 *pH,uint16 u16FrameCnt,uint16 *CalHt,uint16 *MaxIndex);//����߶ȷ���

#endif
