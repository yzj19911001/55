#ifndef _AVIAXLECHECK_H
#define _AVIAXLECHECK_H


#include "JZGlobal.h"

#define   RET_TRUE    0
#define   RET_FALSE	  1
#define   RET_ERROR  -1
#define   RET_VALUE   2

#define DOUBLE_AXLE 155

#define	MAX_AXLE_COUNT	20	///最大轴数

#define SafeIsland_Height Smp.IsLanHt
#define MAX_FRAME FRAME_MAXCNT

extern uint16  Mid_Point_Cal ;

void AxleRecongnize(VehicleSet *pVehicle); //轮轴识别器处理函数
int	clear_global(void);
int PreProcess(void);//预处理
int pre_process(void);//轮轴识别预处理
void SetAxleParam(void);//轮轴需要的参数设置
void CheckAxleState(VehicleSet *pVehicle);//检查当前轮轴识别状态
int judge_tyre(void);//轮轴检测
int get_veh_info_frame(int32 *pFiltX,int32 *pFiltZ,int32 *pX,int32 *pZ);
int32 axle_speed(int axle_present);//轮轴速度计算
int32 axle_group(int axle_present);//轴组判断
void AxleClearGlobal(void);//轮轴清空全局变量
#ifdef _KeilEnVi
void ReportAxleInfo(uint8 AxleType);//上报轮轴信息
void ReprotAckSInfo(void);//上报心跳信息
#endif

#endif

