#ifndef _VEHIDENTPROC_H
#define _VEHIDENTPROC_H


#include "JZGlobal.h"


JZG_EXT void VehIdentProc(void);//����ʶ���㷨����
JZG_EXT void InitModeParam(void);//ģ�Ͳ�����ʼ��


uint8 Polar2Rect(void);//������תֱ�����긽���˲� �ִ�С��
void FramAnlsye(void);//֡����
void ClearVehicleErr(void);//������
uint8 IsRegionMatch(FramInfo FI1,FramInfo FI2);//�Ƿ�����ƥ��
void UpdateVehRegion(VehicleSet *pVehicle);//���³���������Ϣ
void OutPutVeh(VehicleSet *pVehicle);//����ʶ��������
void Write2TxtFile(VehicleSet *pVehicle);//���д��txt�ļ�
void VehIdentifier(VehicleSet *pVehicle);//����ʶ����
void CheckVehState(void);//��鳵��״̬
void VehRegionMatch(void);//������ƥ��
uint8 IsVehCheck(VehicleSet *pVehicle);//�ж��Ƿ�Ϊ����
void InitMiddleWare(void);//����м��
void VehDirCheck(void);//��֪�����н����� 
#ifdef _KeilEnVi
void ReportNormalVeh(VehicleSet *pVehicle);//�ϱ�������Ϣ
void ForceCloseProc(void);//ǿ�Ƴ�������
uint8 ForceVehJudge(VehicleSet *pVehicle);//ǿ�Ƴ��������ж�
void SendBackVeh(void);//���з���Back
void ReportForceVeh(VehicleSet *pVehicle,uint8 ForcType);//ǿ�Ƴ������ͳ�����Ϣ
void Dev_CoilDirCheck(void);//��Ȧ��֪������� 
void Dev_GratDirCheck(void);//��դ��֪������� 
void Dev_LaseDirCheck(void);//�����֪�������  
void SendAddVeh(uint8 u8AddType);//�������ӵ�һ����
void SendHeadVeh(void);//ɾ��ͷһ����
void SendQueNum(uint8 CH,uint8 PRTL);//���Ͷ�����Ϣ
void CheckAbFramCnt(void);//֡���������
void CheckQueState(void);//����״̬���
void VehQueCheck(void);//���г������м�����


#endif 

#endif
