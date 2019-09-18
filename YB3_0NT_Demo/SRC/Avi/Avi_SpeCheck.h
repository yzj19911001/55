#ifndef _AVISPECHECK_H_
#define _AVISPECHECK_H_

#include "JZGlobal.h"


void JinBeiCheck(VehicleSet *pVehicle);//金杯车检测
void TruckTailCheck(VehicleSet *pVehicle);//货车检测
void PickUpCheck(VehicleSet *pVehicle);//皮卡检测
void PickUpFramCal(VehicleSet *pVehicle);//皮卡帧检测算子
void SpecialVehCheck(VehicleSet *pVehicle);//特殊车辆检测
void SpecialVehCal(VehicleSet *pVehicle);//特殊车辆检测算子
void PickUpTailCheck(VehicleSet *pVehicle);//皮卡尾部检测
void EnCloseWinCal(VehicleSet *pVehicle);//封闭车窗检测算子
void EnCloseWinCheck(VehicleSet *pVehicle);//封闭车窗检测
void Axle2WinCheckEx(VehicleSet *pVehicle);//第二轴车窗客车检测

void EnCloseWinCalEx(VehicleSet *pVehicle);//封闭车窗检测算子
void EnCloseWinCheckEx(VehicleSet *pVehicle);//封闭车窗检测

uint8 FramArcCal(int32 *pX,int32 *pZ,uint16 PointCnt, VehicleSet *pVehilceSet);//每帧弧段算子
void ArcCheck(ArcInfo *pArcSet);//油罐检测

#endif
