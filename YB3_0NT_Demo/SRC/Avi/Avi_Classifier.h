#ifndef _AVICLASSIFIER_H_
#define _AVICLASSIFIER_H_


#include "JZGlobal.h"

void VehIdentifier(VehicleSet *pVehicle);//����ʶ����
void VehPatternClassify(VehicleSet *pVehicle);//���ͷ���
void OneAxleClassify(VehicleSet *pVehicle);//һ�ᳵ������
void TwoAxleClassify(VehicleSet *pVehicle);//���ᳵ������
void ThreeAxleClassify(VehicleSet *pVehicle);//���ᳵ������
void MutiAxleClassify(VehicleSet *pVehicle);//���ᳵ������
void VehPatternModify(VehicleSet *pVehicle);//���͵������ƫ

#endif
