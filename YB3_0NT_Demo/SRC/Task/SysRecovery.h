#ifndef _SysRecovery_H
#define _SysRecovery_H

#include "JZGlobal.h"

JZG_EXT void RecoverySystem(void);////�ָ�ϵͳ
void DefualtSysParam(void);//Ĭ��ϵͳ����
void RecoModelParam(void);//�ָ�ģ�Ͳ���
void DefualtModelParam(void);//Ĭ��ģ�Ͳ���
void RecoSystemParam(void);//�ָ�ϵͳ����
void RecordReset(void);//������������
void RecoveryNetParam(void);////�ָ��������
void DefualtNetParam(void);	//Ĭ���������
void InitGlobalParam(void);//��ʼ��ȫ�ֱ���
void RecoveryUartSet(void);//�ָ���������
void DefualtUartSet(void);//Ĭ���������
uint32 GetUBR(uint8 UBR_I);//ͨ�������ʵȼ��õ�������ֵ
uint8 CheckMdlParam(void);//���ģ�Ͳ����Ƿ�ǡ��
void MakeCtr2Factory(void);//ʹ�������ָ���������
void RecBkModelParam(void);	//���ñ��ݲ�������ģ�;��廯
void DefualtVehIndex(void);//���ó�����ż�¼��ΪĬ��ֵ
void RecoVehIndex(void);//�ָ�����ID���
void SaveVehIndex(void);//�洢����ID
void RecoVehOutThresh(void);//�ָ��������������ֵ���� 
void DefualtVehOutTH(void);//Ĭ�ϳ�����ֵ���� 
uint8 CheckVehOutTH(void);//��������ֵ����
void RecoWeightAtSet(void);//�ָ����̨������������
void DefualtWeightAtSet(void);//Ĭ�����̨������������
uint8 CheckWeightAtSet(void);//����̨��������

#endif
