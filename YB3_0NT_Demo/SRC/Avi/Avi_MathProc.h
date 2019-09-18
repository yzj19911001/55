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



JZG_EXT unsigned short Tabsin[181];//正弦表
JZG_EXT unsigned short Tabcos[181];//余弦表
//JZG_EXT unsigned short Tabtan[181];//正切表
//JZG_EXT unsigned short Tabcot[181];//余切表

JZG_EXT unsigned  short TabRsin[361];//求Rx的sin系数
JZG_EXT unsigned int TabRcos[361];//求Rz的cos系数

JZG_EXT int32 Average(const int32 *a,uint16 num);//求平均值

int32 MeanFilterU(uint16 *pData,int n);//n邻域均值滤波
int32 MeanFilterI(int32 *pData,int n);//n邻域均值滤波
void IndicateTrends(int32 *pdata,int32 *pIndicate,int len);//标记曲线变化一致性
void IndicatePeaks(int32 *pdata,int32 *pIndicate,int len);//标记峰谷
void ShapeAnalysisEx(uint16 *pData,ClusterInfo *pCluster);//外形检测算法

void IndicateTrend(int32 *pdata,int32 *pIndicate,int len);
void ShapeAnalysis(uint16 *pData,ClusterInfo *pCluster);//外形检测方法
uint16 MIN_N_NEAR(uint16 *pSta,uint16 N,uint16 B_F);//N邻域最小值
void CalcVehHeight(uint16 *pH,uint16 u16FrameCnt,uint16 *CalHt,uint16 *MaxIndex);//计算高度方法

#endif
