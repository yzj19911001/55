#ifndef _PRTL_PROC_H_
#define _PRTL_PROC_H_

#include "JZGlobal.h"

//������
#define CMD_OPERATEERRS 0xFE //��ѯ���ô���
#define CMD_GETSYSPARAM 0x01 //��ѯϵͳ�����
#define CMD_SETSYSPARAM 0x02 //����ϵͳ����
#define CMD_GETMDLPARAM 0x03 //��ѯģ�Ͳ���
#define CMD_SETMDLPARAM 0x04 //����ģ�Ͳ���
#define CMD_GETRESETINF 0x05 //��ѯ������Ϣ
#define CMD_CLRRESETINF 0x06 //���������Ϣ 
#define CMD_CTLSYSRESET 0x07 //����������
#define CMD_CTL2FACTROY 0x08 //�ָ���������
#define CMD_GETNETPARAM 0x09 //��ȡ�������
#define CMD_SETNETPARAM 0x0A //�����������
#define CMD_GETUARTINFO 0x0B //��ѯ���ڲ���
#define CMD_SETUARTINFO 0x0C //���ô��ڲ���
#define CMD_GETSYSTIMES 0x0D //��ѯϵͳʱ��
#define CMD_SETSYSTIMES 0x0E//����ϵͳʱ��
#define CMD_GETAVITHRES 0x0F //��ȡ����ʶ����ֵ
#define CMD_SETAVITHRES 0x10 //���ó���ʶ����ֵ
#define CMD_GETSINGWAVE 0x11 //��ȡ��֡����
#define CMD_UPLOADVEHS  0x12 //����ʱ�ϴ�����ָ��
#define CMD_GETAQCSTATE 0x13 //��ȡ���Ⲩ�ν��մ���״̬
#define CMD_CLRAQCSTATE 0x14 //��ռ��Ⲩ�ν��մ���״̬
#define CMD_GETVEHOUTTH 0x15 //��ȡ������β��ֵ����
#define CMD_SETVEHOUTTH 0x16 //���ó�����β��ֵ����
#define CMD_GETWEIGHTAT 0x17 //��ѯ��̨������������
#define CMD_SETWEIGHTAT 0x18 //���ó�̨������������


#define DLL_OPENCHANNEL 0x30  //���豸ͨ��
#define DLL_GETDEVSTATE 0x31  //�豸״̬
#define DLL_VHCHECKINFO 0x32  //������Ϣ
#define DLL_DEVISYNCHRO 0x33  //�豸ͬ��
#define DLL_MANCLOSEOUT 0x34  //�ֶ���β
#define DLL_CLOSECHANEL 0x35  //�ر��豸ͨ��
#define DLL_ADD1VEHICLE 0x36  //����һ����
#define DLL_DEL1VEHICLE 0x37  //��ȥһ����
#define DLL_UPLDQUEINFO 0x38  //�ϴ�������Ϣ




//������
#define ERR_OK 0x01 //�ɹ�
#define ERR_TD 0x02 //�������ʧ��
#define ERR_LG 0x03 //�߼�����
#define ERR_AL 0x04 //�ڴ�������
#define ERR_AB 0x05 //�����쳣

void PassThroughToJG(uint8 *TouchuanBuff);//͸��
uint8 SeekSocketNo(uint8 *p_IPbuff);//ͨ��IP�˿���Ѱ��ӦSocket
void GetSysTime(uint8 CH,uint8 PRTL);//��ȡϵͳʱ��
void SetSysTime(uint8 *pBuff,uint8 CH,uint8 PRTL);//����ϵͳʱ��
void GetSysParam(uint8 CH,uint8 PRTL);//��ȡϵͳ����
void SetSysParam(uint8 *pBuff,uint8 CH,uint8 PRTL);//����ϵͳ����
void GetModelParam(uint8 CH,uint8 PRTL);//��ȡģ�Ͳ���
void SetModelParam(uint8 *pBuff,uint8 CH,uint8 PRTL);//����ģ�Ͳ���
void GetCtlResetInfo(uint8 CH,uint8 PRTL);//��ȡ������������Ϣ
void ClrCtlResetInfo(uint8 CH,uint8 PRTL);//��տ�����������Ϣ
void CtlSysReset(uint8 CH,uint8 PRTL);//����������
void GetNetParam(uint8 CH,uint8 PRTL);//��ȡ�������
void SetNetParam(uint8 *pBuff,uint8 CH,uint8 PRTL);//�����������
void GetUartInfoParam(uint8 CH,uint8 PRTL);//��ȡϵͳ����
void SetUartInfo(uint8 *pBuff,uint8 CH,uint8 PRTL);//����ϵͳ����
void GetSingleWave(uint8 CH,uint8 PRTL);//��ȡ��֡����	
void Ctl2FactoryState(uint8 CH,uint8 PRTL);//�ָ�������������״̬
void PassThroughToUart(uint8 SocketIdx);//͸��������
void PassThroughToJG(uint8 *ToJGBuff);//͸��������
void ClrAqcWvState(uint8 CH,uint8 PRTL);//��ղɼ����ε�״̬
void GetAqcWvState(uint8 CH,uint8 PRTL);//��ȡ�ɼ����ε�״̬
void GetVehOutTH(uint8 CH,uint8 PRTL);//��ѯ������ֵ����
void SetVehOutTH(uint8 *pBuff,uint8 CH,uint8 PRTL);//���ó�����ֵ����
void GetWeightAtSet(uint8 CH,uint8 PRTL);//��ѯ���̨����״̬
void SetWeightAtSet(uint8 *pBuff,uint8 CH,uint8 PRTL);//���ó�̨��������

void DllGetDeviState(uint8 CH,uint8 PRTL);//DLL��ѯ�豸״̬
void DllOpenCHannel(uint8 CH,uint8 PRTL);//DLL������豸
void DllReplyManOut(uint8 CH,uint8 PRTL);//DLLӦ���ֶ���β����
void DllCloseCHannel(uint8 CH,uint8 PRTL);//DLL����ر��豸
void DllAdd1VehInfo(uint8 *pBuff,uint8 CH,uint8 PRTL);//DLL����1����
void DllDel1VehInfo(uint8 *pBuff,uint8 CH,uint8 PRTL);//DLLɾ��1����
void DllUpLoadQueInfo(uint8 *pBuff,uint8 CH,uint8 PRTL);//DLL�ϴ�������Ϣ

JZG_EXT void SendBuf_full(uint8  *RcvBufPt, uint32  Snd_Len,uint8 CH);//ȫͨ�����ͺ���

#endif
