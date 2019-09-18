#ifndef _AVIFEATURECAL_H
#define _AVIFEATURECAL_H

#include "JZGlobal.h"
void FeatureCal(VehicleSet *pVehicle);//����ʶ����
void SideUniformCal(VehicleSet *pVehicle);//1m���²���һ��������
void SideUinformCheck(VehicleSet *pVehicle);//1m���²���һ���Լ��
void DpHeightCal(VehicleSet *pVehicle);//��ȡ��Ӧ֡���̸�
void DpHtCheck(VehicleSet *pVehicle);//ȷ�����յĵ��̸�
uint8 IsJiZhuangXiang(FramInfo *pFrameInfo);//��װ����
void ContainerCheck(VehicleSet *pVehicle);//���������ж�
void ContainerCal(VehicleSet *pVehicle);//����������
void MinHeightCheck(VehicleSet *pVehicle);//��͸߶ȼ��
void VehHeadHtCheck(VehicleSet *pVehicle);//��һ�����Ӧ�ĳ��� ����ͷ������
void VehRearHtCheck(VehicleSet *pVehicle);//�ڶ������Ӧ�ĳ��� �ڶ��������
void VehHeightCheck(VehicleSet *pVehicle);//�����߶ȼ��
void VehHeightModify(VehicleSet *pVehicle);//���ߵ���
void MutiFeatureFusion(VehicleSet *pVehicle);//�������ں�
void AxleAdjustMent(VehicleSet *pVehicle);//������� С��Χ��ƫ
void Axle1MinHeightCal(VehicleSet *pVehicle);//
void HtClusterCheck(VehicleSet *pVehicle);//�߶ȷ���������
void ClusterHopperCheck(VehicleSet *pVehicle);//���෨�������;
void AxleHopperCheck(VehicleSet *pVehicle);//���ᶨλ����⳵��
void GlassWinCheck(VehicleSet *pVehicle);//�������
void IntoGlassCal(VehicleSet *pVehicle);//���ڴ�͸�ĳ������
void SlopeGlassCal(VehicleSet *pVehicle);//����б�ʵĳ������
void ThroughGlassCal(VehicleSet *pVehicle);////����͸����
void MapGlassCal(VehicleSet *pVehicle);//����������������
void HeadJumpCheck(VehicleSet *pVehicle);//��ͷ���� ���ֿͻ�
void TailJumpCheck(VehicleSet *pVehicle);//β������ ���ֿͻ�
void CargoXEdgeCheck(VehicleSet *pVehicle);//x����ͶӰ͹�������ػ�
void CastGrdCheckCal(VehicleSet *pVehicle);//���ؼ�����ֻ���
void TruckCanCheck(VehicleSet *pVehicle);//ͨ����ͷ���߼�����
void FramShapeCal(VehicleSet *pVehicle);//ÿ֡�������ͼ��
void FramStyleCheck(VehicleSet *pVehicle);//���μ���ػ�
void AxleHopperCheckEx(VehicleSet *pVehicle,uint16 *pH);
void HeadJumpCheckEx(VehicleSet *pVehicle,uint16 *pH);
void FindTailJump(VehicleSet *pVehicle,uint8 nMode);//�ҵ���β���䶨�ͻ�
void FindHeadJump(VehicleSet *pVehicle,uint8 nMode);//�ҵ���ͷ���䶨�ͻ�
void DirCheckByDev(VehicleSet *pVehicle);//���������ж���������  
#ifdef _KeilEnVi
void DirJudge_Coil(VehicleSet *pVehicle);//��Ȧ�жϷ���
void DirJudge_Grat(VehicleSet *pVehicle);//��դ�жϷ���
void DirJudge_Lase(VehicleSet *pVehicle);//�����жϷ���
#endif

#endif
