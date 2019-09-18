#ifndef _AVIAXLECHECK_H
#define _AVIAXLECHECK_H


#include "JZGlobal.h"

#define   RET_TRUE    0
#define   RET_FALSE	  1
#define   RET_ERROR  -1
#define   RET_VALUE   2

#define DOUBLE_AXLE 155

#define	MAX_AXLE_COUNT	20	///�������

#define SafeIsland_Height Smp.IsLanHt
#define MAX_FRAME FRAME_MAXCNT

extern uint16  Mid_Point_Cal ;

void AxleRecongnize(VehicleSet *pVehicle); //����ʶ����������
int	clear_global(void);
int PreProcess(void);//Ԥ����
int pre_process(void);//����ʶ��Ԥ����
void SetAxleParam(void);//������Ҫ�Ĳ�������
void CheckAxleState(VehicleSet *pVehicle);//��鵱ǰ����ʶ��״̬
int judge_tyre(void);//������
int get_veh_info_frame(int32 *pFiltX,int32 *pFiltZ,int32 *pX,int32 *pZ);
int32 axle_speed(int axle_present);//�����ٶȼ���
int32 axle_group(int axle_present);//�����ж�
void AxleClearGlobal(void);//�������ȫ�ֱ���
#ifdef _KeilEnVi
void ReportAxleInfo(uint8 AxleType);//�ϱ�������Ϣ
void ReprotAckSInfo(void);//�ϱ�������Ϣ
#endif

#endif

