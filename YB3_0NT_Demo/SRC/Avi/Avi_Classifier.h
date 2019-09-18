#ifndef _AVICLASSIFIER_H_
#define _AVICLASSIFIER_H_


#include "JZGlobal.h"

void VehIdentifier(VehicleSet *pVehicle);//车型识别器
void VehPatternClassify(VehicleSet *pVehicle);//车型分类
void OneAxleClassify(VehicleSet *pVehicle);//一轴车分类器
void TwoAxleClassify(VehicleSet *pVehicle);//二轴车分类器
void ThreeAxleClassify(VehicleSet *pVehicle);//三轴车分类器
void MutiAxleClassify(VehicleSet *pVehicle);//多轴车分类器
void VehPatternModify(VehicleSet *pVehicle);//车型调整与纠偏

#endif
