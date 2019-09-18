#ifndef _AVIFEATURECAL_H
#define _AVIFEATURECAL_H

#include "JZGlobal.h"
void FeatureCal(VehicleSet *pVehicle);//特征识别器
void SideUniformCal(VehicleSet *pVehicle);//1m以下侧面一致性算子
void SideUinformCheck(VehicleSet *pVehicle);//1m以下侧面一致性检测
void DpHeightCal(VehicleSet *pVehicle);//获取对应帧底盘高
void DpHtCheck(VehicleSet *pVehicle);//确定最终的底盘高
uint8 IsJiZhuangXiang(FramInfo *pFrameInfo);//集装箱检测
void ContainerCheck(VehicleSet *pVehicle);//货箱最终判定
void ContainerCal(VehicleSet *pVehicle);//货箱检测算子
void MinHeightCheck(VehicleSet *pVehicle);//最低高度检测
void VehHeadHtCheck(VehicleSet *pVehicle);//第一轴相对应的车高 即车头高算子
void VehRearHtCheck(VehicleSet *pVehicle);//第二轴相对应的车高 第二轴高算子
void VehHeightCheck(VehicleSet *pVehicle);//车辆高度检测
void VehHeightModify(VehicleSet *pVehicle);//车高调整
void MutiFeatureFusion(VehicleSet *pVehicle);//多特征融合
void AxleAdjustMent(VehicleSet *pVehicle);//轮轴调整 小范围纠偏
void Axle1MinHeightCal(VehicleSet *pVehicle);//
void HtClusterCheck(VehicleSet *pVehicle);//高度方向聚类分析
void ClusterHopperCheck(VehicleSet *pVehicle);//聚类法车斗检测;
void AxleHopperCheck(VehicleSet *pVehicle);//轮轴定位法检测车斗
void GlassWinCheck(VehicleSet *pVehicle);//车窗检测
void IntoGlassCal(VehicleSet *pVehicle);//基于穿透的车窗检测
void SlopeGlassCal(VehicleSet *pVehicle);//基于斜率的车窗检测
void ThroughGlassCal(VehicleSet *pVehicle);////车身穿透算子
void MapGlassCal(VehicleSet *pVehicle);//车窗玻璃块检测算子
void HeadJumpCheck(VehicleSet *pVehicle);//车头跳变 区分客货
void TailJumpCheck(VehicleSet *pVehicle);//尾部跳变 区分客货
void CargoXEdgeCheck(VehicleSet *pVehicle);//x方向投影凸出物检查载货
void CastGrdCheckCal(VehicleSet *pVehicle);//掷地检测区分货车
void TruckCanCheck(VehicleSet *pVehicle);//通过车头跳高检测箱货
void FramShapeCal(VehicleSet *pVehicle);//每帧外形类型检测
void FramStyleCheck(VehicleSet *pVehicle);//外形检测载货
void AxleHopperCheckEx(VehicleSet *pVehicle,uint16 *pH);
void HeadJumpCheckEx(VehicleSet *pVehicle,uint16 *pH);
void FindTailJump(VehicleSet *pVehicle,uint8 nMode);//找到车尾陡变定客货
void FindHeadJump(VehicleSet *pVehicle,uint8 nMode);//找到车头陡变定客货
void DirCheckByDev(VehicleSet *pVehicle);//车辆方向判定基于外设  
#ifdef _KeilEnVi
void DirJudge_Coil(VehicleSet *pVehicle);//线圈判断方向
void DirJudge_Grat(VehicleSet *pVehicle);//光栅判断方向
void DirJudge_Lase(VehicleSet *pVehicle);//激光判断方向
#endif

#endif
