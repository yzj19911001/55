/******************************************************************************
  * @file    Task_AqcWave.h
  * @author  Laser Application Department
  * @version V1.0.0
  * @date    21-November-2016
  * @brief   Header file of Task_AqcWave.c module.
******************************************************************************
  * @attention
  *
  * <h2><center>&copy COPYRIGHT(c) 2016 Wanji Technology(WJ)</center></h2>
	*
	<h2><center>&</center></h2>
	*
  * All rights reserved.
******************************************************************************/
#ifndef _Task_FrameSave_H_
#define _Task_FrameSave_H_
#include "config.h"

////����洢�õ�ַ
//#define  SAVEADDRINFO_SDADDR      0x3FFFFF //�洢������ص�ַ������
//#define  STARTSECTORNO_FRAMESAVE  0x400000 //��ʼ���� 2Gλ��
//#define  ENDSECTORNO_FRAMESAVE    0x800000 //��ֹ���� 4Gλ��
//#define  SIZESECTOR_FRAMESAVE     0x400000//������

#define  SAVEADDRINFO_SDADDR      0x3FFFFF //�洢������ص�ַ������
#define  STARTSECTORNO_FRAMESAVE  0xA00000 //��ʼ���� 5Gλ��
#define  ENDSECTORNO_FRAMESAVE    0x1A00000 //��ֹ���� 13Gλ��
#define  SIZESECTOR_FRAMESAVE     0x1000000//������


//SD��������
#define WLC864I89    89
#define WLC864I892   892
#define CONTROLLER   WLC864I892  //������� 89  ��С������Ϊ892
#define SD_INSERT (P3_INP_STATE & (1<<25))


//���δ洢��SD����¼�ṹ��
#define BHeaderSize 137
#define CACHENUM 16
#define SECTORSIZE 512
#define FRAMESIZE 500 //���� ��Ϊ500  ʵ��Ϊ1000
#define ISOPENED  0xAA
#define ISCLOSED  0xBB
#define F1BlockNum 2

typedef struct tagFramSave
{
	uint8  FgFlag; //���Ǳ�־λ
    uint16 SvIndx ;//�沨��֡���
    uint16 IFrIndx[CACHENUM];//�뻺���֡���
    //uint16 OFrIndx[CACHENUM];//�������֡���
    uint32 CurSvAddr;//��ǰ�洢��sd����ַ
    uint32 SwSdCnt;//Ӧ��д��sd��֡��
    uint32 WrSdCnt;//ʵ��д��sd����֡��
    uint8  Disabled;//�沨��ʹ�ܱ�ʶλ 0xAA - ʹ�ܣ� 0xBB - ��ʹ��
    uint8  ICacheIndx;//����������
    uint8  OCacheIndx;//����������
    uint8 FrameSaveBuf[CACHENUM][FRAMESIZE][F1BlockNum*SECTORSIZE];//��������
} FramSave;

extern FramSave g_WvSave;//�沨�μ�¼�ṹ��
void Task_AqcWave(void *tdata);
void Init_FrameSave(void);//��ʼ������
void Task_Aqc_Wave(void *tdata);//�沨������
uint8 Fram_Sv2SD(uint8 *pData,uint16 nSize,uint32 Sd_Addr);//�洢������sd��
void Catch_Frame_RT(VehicleSet* pVehicle,uint8 nCmd);//����ץ֡
uint8 SaveAddrInfoTsd(void);//�洢��ַ�����Ϣ
void ReadAddrInfoFsd(void);//��ȡsd����ַ��Ϣ
void SetDefualtAddr(void);//���洢��ַ�������Ϣ����ΪĬ��ֵ
uint8 Fram_Sv2SD_Ex(uint8 *pData,uint8 nNum,uint32 Sd_Addr);//һ��д�������洢��ʽ��һ֡����

#endif
