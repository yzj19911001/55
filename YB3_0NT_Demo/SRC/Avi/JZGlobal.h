/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			JZGlobal.h
** Last modified Date:  2011511
** Last Version:		1.0
** Descriptions:		���س���ȫ�ֱ���
**
**--------------------------------------------------------------------------------------------------------
** Created by:			ZHANG Ye
** Created date:		2011511
** Version:				1.0
** Descriptions:		
**
**--------------------------------------------------------------------------------------------------------
** Modified by:			Wang ZiFeng
** Modified date:		2013-03-11
** Version:				2.0
** Descriptions:
**
*********************************************************************************************************/
#ifndef	__JZGLOBAL_H
#define	__JZGLOBAL_H


#ifdef	__JZGLOBAL
#define	JZG_EXT	
#else
#define	JZG_EXT	extern
#endif

#include "JZStructure.h"


#ifdef _KeilEnVi
#include "WDT.h"

//�ź���  
JZG_EXT OS_EVENT *g_TdOpMutex;
JZG_EXT OS_EVENT *g_SdOpMutex;	
JZG_EXT	OS_EVENT *g_WavesProc;
JZG_EXT	OS_EVENT *g_Uart1Recv;
JZG_EXT	OS_EVENT *g_Uart1Send;
JZG_EXT	OS_EVENT *g_Uart5Send;
JZG_EXT OS_EVENT *g_Uart5Recv;
JZG_EXT OS_EVENT *g_NetS0Recv;
JZG_EXT	OS_EVENT *g_NetS1Recv;
JZG_EXT	OS_EVENT *g_NetS2Recv;
JZG_EXT OS_EVENT *g_NetS3Recv;//�˿�3���ź���
JZG_EXT OS_EVENT *g_UpdateApp;	
     

JZG_EXT	SystemTime		g_SysCurTime;//��ǰʱ�� 
JZG_EXT DevUartSet      g_UartSet;//�豸�ӿ�
JZG_EXT LaserPassTo     g_LaserPass;//����͸��
JZG_EXT NetInfo         g_NetInfo;//ϵͳ�������
JZG_EXT SysTemSet       g_SysSetUp;//ϵͳ����
JZG_EXT VehIndexSet     g_VehIndex;//������ż�¼��


#if	YBVERSION >= 30		//3.0�Ǳ���

#else	//2.2�Ǳ���
JZG_EXT	void	(*bootloader_entry)(void);
#endif	//#if	YBVERSION >= 30		//3.0�Ǳ���


JZG_EXT uint32 Net4_DisconnectNum ;
JZG_EXT uint32 Net1_DisconnectNum ;
JZG_EXT uint32 Net2_DisconnectNum ;
JZG_EXT uint32 Net3_DisconnectNum ;

JZG_EXT uint32 Net1_InvalidRecNum ;
JZG_EXT uint32 Net2_InvalidRecNum ;
JZG_EXT uint32 Net3_InvalidRecNum ;


//���ڽ��ս�����

JZG_EXT volatile uint8 uart1_timeout ;
JZG_EXT volatile uint8 uart5_timeout ;

#define	UARTRCVBUFSIZE		8192		//����5�����С
#define MAX_FRAME_LENGTH 256  //Э�����֡����
#define MIN_FRAME_LENGTH 7	  //Э����С֡����
#define MIN_FRAME_LENGTH_TC 18	  //͸��Э����С֡����
#define MAX_FRAME_LENGTH_TC 1800	  //͸��Э����С֡����
JZG_EXT	volatile uint16	 g_UART5_Recv_Index;      //����5�����������
JZG_EXT	volatile uint16	 g_UART5_Proc_Index;      //����5���洦������
JZG_EXT uint8 g_u8U5RcvBuf[UARTRCVBUFSIZE] ; //����5���ջ���
JZG_EXT	volatile uint16	 g_UART1_Recv_Index;      //����1�����������
JZG_EXT	volatile uint16	 g_UART1_Proc_Index;      //����1���洦������
JZG_EXT uint8 g_u8U1RcvBuf[UARTRCVBUFSIZE] ; //����1���ջ���


#define MAX_FRAME_LENGTH_AA 200  //Э�����֡����
#define MIN_FRAME_LENGTH_AA 8	  //Э����С֡����	
#define MAX_FRAME_LENGTH_FA 200  //Э�����֡����
#define MIN_FRAME_LENGTH_FA 7	  //Э����С֡����	 
#define MAX_FRAME_LENGTH_FF 8000  //Э�����֡����
#define MIN_FRAME_LENGTH_FF 8	  //Э����С֡����
#define WAVE_FRAME_LENGTH   12    //���䲢������Э��֡��  �̶�


//���³�����
JZG_EXT	SystemTime	g_sstCurTime_Remote; 
JZG_EXT uint8 g_Update_App_Step;
JZG_EXT uint8 g_bStarted;
JZG_EXT uint8 g_Update_App_CH;//���³���ͨ��
JZG_EXT uint8 ZYJ_MJH;	  //�����ڼ��ֹ����02 08��������ݰ�
JZG_EXT uint8 g_u8ParseBuf[2500];
JZG_EXT uint8 g_u8FrmNo;
JZG_EXT uint8 MIN_Remote; //Զ�̸���ר��ʱ����Ϣ
JZG_EXT uint32 g_u32ParseLen;


//ϵͳ����
#define ResetSystem()  	{WDTInit(1),WDTIM_COUNTER =  0x10000000-100;while(1); 	}


//���ղ����û���
//���ռ��������λ���
#define LMDMAXCACHE 500
JZG_EXT uint8 g_LmdCache[LMDMAXCACHE][Full_Size+1]; //�������ݻ��� 500����	
JZG_EXT uint16 g_LmdCacheIn; //�����⻺�������
JZG_EXT uint16 g_LmdCacheOut; //�����⻺�������
JZG_EXT DllAtState g_DllAtState;//����λ��DLL������״̬
#endif

//�¼ӵĶ��� 20170628

//������
#define ERROR_NONE 0 //�޴���
#define ERROR_LOGI 1 //�߼�����

//LMS_Data��ض���

#define POINT_CEN 180 //�������ĵ�
#define POINT_SUM 361 //ɨ�����
#define POINT_END 360 //ɨ���������
#define LMS_MINTH 20 //���������� 
JZG_EXT int32 LMS_data_1[POINT_SUM+1];//ɨ���������


//���������ֵ
#define THRESH_H1 350  //��һ�߶���ֵ 350 
#define THRESH_H2 500  //�ڶ��߶���ֵ 500
#define THRESH_H3 520  //�����߶���ֵ 520

#define VEHTH_LOW 150  //������߶���ֵ����
#define VEHTH_UP  4800 //������߶���ֵ����

#define ISLANHT  800 //��ȫ���߶�
#define ISLANWD  300 //��ȫ�����

//�߼��ж������ֵ
#define     MINHEIGHT_JIZHUANGXIANG   1600   //���ڼ���5�Ƿ��Ǽ�װ�����͸߶�
#define     MAXHEIGHT_JIZHUANGXIANG   3800   //���ڼ���5�Ƿ��Ǽ�װ������߶�
#define     MAXDIFFER_JIZHUANGXIANG   150    //���ڼ���5�Ƿ��Ǽ�װ��Ŀ������ֵ
#define     CHASSISHEIGHTTHRES        430    //�ͻ����ֵĵ��̸߶���ֵ

#define  DAFEIMAXPT     4             //����ת���д�ɵ����ת������

//���峵������
#define     ONE_AXLE                  1
#define     TWO_AXLE                  2   
#define     THREE_AXLE                3
#define     FOUR_AXLE                 4
#define     FIVE_AXLE                 5
#define     SIX_AXLE                  6	

//���峵�ͺ�
#define     FIRST_KECHE               1      //��һ��ͳ� <=7��
#define     SECOND_KECHE         	  2      //�ڶ���ͳ� 8��19��
#define     THIRD_KECHE               3      //������ͳ� 20��39��
#define     FOURTH_KECHE              4      //������ͳ� >=40��
#define     FIRST_HUOCHE              5      //��һ����� <=2��
#define     SECOND_HUOCHE			  6      //�ڶ������ 2��5�֣�����
#define     THIRD_HUOCHE              7      //��������� 5��10�֣�����
#define     FOURTH_HUOCHE             8      //��������� 10��15�֣�����
#define     FIFTH_HUOCHE              9      //��������� >15��	
#define     FIRST_JIZHUANGXIANG       10     //��һ�༯װ��
#define     SECOND_JIZHUANGXIANG      11     //�ڶ��༯װ��

//�㷨��ض��� ȫ�ֱ���
JZG_EXT uint32 g_n32WaveNumber;//vc����
JZG_EXT SysModelParam Smp;//ϵͳģ�Ͳ���
JZG_EXT uint8  g_u8ItemName[24];//��¼���������ļ�������
JZG_EXT FramInfo CurFramInfo;//��ǰ֡��Ϣ
JZG_EXT VehOutSet g_VehOutSet;//������β�����ֵ
JZG_EXT WeightAtSet g_WeigthAt;//���̨��������

//��λ������ά���ĺ궨��
#define	MAXVEHQUE	10	//������λ�����д洢����Ĵ�С
JZG_EXT uint8   g_u8VehQueNum;       //��̬�������Ŀ
JZG_EXT uint8   g_u8VehQue[MAXVEHQUE];  //ѭ�����飬��������ϴ���10��������Ϣ
JZG_EXT uint8 	g_u8VehQueIndex;  //g_u8VehQue�����������ֵΪ0-9


JZG_EXT int32 g_CoorX[POINT_SUM]; //ɨ���X��������
JZG_EXT int32 g_CoorZ[POINT_SUM]; //ɨ���Z��������
JZG_EXT int32 g_FiltX[POINT_SUM]; //ɨ���X�����˲�������
JZG_EXT int32 g_FiltZ[POINT_SUM]; //ɨ���Z�����˲�������




#endif



