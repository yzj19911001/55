#ifndef	__JZSTRUCTURE_H_
#define	__JZSTRUCTURE_H_

#include "stdafx.h"

#include "Config.h"



#ifndef _KeilEnVi
#define _KeilEnVi
#endif

#ifdef _KeilEnVi


#define POINT_SUM               361	
#define Full_Size               831

#define CH_NOCH         0x00
#define CH_COM1         0xC1
#define CH_COM2         0xC2
#define CH_COMA         0xC3

#define PTRL_FF         0x01
#define PTRL_FA         0x02
#define PTRL_NO         0x00

typedef struct tagDevUartSet
{
	uint8 BR_COM1;//COM1�����ʴ��� 1-5
	uint8 BR_COM2;//COM2�����ʴ��� 1-5
	uint8 TS_CHCOM;//����ָ��ͨ��	
	uint8 RM_CHCOM;//��ӦԶ��ָ��ͨ��
	uint16 u16CRC;//CRCУ��	
}DevUartSet;


#define MAXPASSLEN 200
typedef struct tagLaserPassTO
{
	uint8 PassCH;//͸��ͨ��
	uint8 PassCont[MAXPASSLEN];
}LaserPassTo;

#define Dev_NONO 0 //���豸
#define DEV_COIL 1 //��Ȧ
#define DEV_GRAT 2 //��դ
#define DEV_LASE 3 //����
#define Dev_REFL 4 //���⾵

typedef struct tagSystemParam	
{
	uint8   au8ProgramVersion[11];//����汾
	uint16  hResetCnt;			 //Ӳ����������
	uint16  sResetCnt;			 //����������
	uint8 	u8DOGEnable;		 //���Ź�ʹ��
	uint8   u8SDEnable;          //sd��ʹ��
	uint8   u8DirDevType;        //�н�����ʶ���豸���� 1--COIL;2--GRATING 3--LASER;
	uint8   u8DirDevEnable;      //�����豸ʹ��
	uint8   u8WorkMode;			 //����ģʽ 0-�ֳ����У�1--ģ������;
	uint8   u8DeviceID[15];		 //�豸ʶ����
	uint8   u8StationID[16];	 //վ����
	uint16	u16CRC;				 //CRCУ��		
} SysTemSet;

typedef	struct tagNetInfo
{
	uint8	au8IPAddr[4];	//������IP��ַ
	uint16	u32LocalPortNO;	//�������˿ں�
	uint8	au8SubMask[4];	//��������
	uint8	au8GatewayIP[4];//Ĭ������		
	uint8	au8MACAddr[6];	//MAC
	uint8  SocketIP[8][4];//socket��Ӧ��ip
	uint32 SocketPort[4];//socket��Ӧ�Ķ˿�
	uint16	u16CRC;				 //CRCУ��
} NetInfo;	 


typedef	struct tagSystemTime
{
	uint16	u16Year;				//��
	uint8	u8Month;				//��
	uint8	u8Day;					//��
	uint8	u8Week;					//���ڼ�
	uint8	u8Hour;					//ʱ
	uint8	u8Minute;				//��
	uint8	u8Second;				//��
} SystemTime;


#define	MAX_RESETCNT	10		

typedef struct tagResetRecord
{
	uint8	u8CurPos;
	uint8	u8CurCnt;
 	SystemTime SysTimeTable[MAX_RESETCNT];
}ResetRecord;	   

typedef struct tagJGRecSet
{
	uint8  nCmd;			  //֡���� 1 ������ݣ� 2 �ǲ������;
	uint8  Data[2*Full_Size]; //���⻺������
	uint32 RecLen;            //���ճ���
	uint32 PostCnt;           //�жϼ���
	uint32 ErrFrCnt;          //����֡���� ��֡ͷ��Ϊ02
	uint32 RecWvCnt;		  //���������ݼ���
	uint32 ProWvCnt;          //�����ನ�μ���
	uint32 RecCmdCnt;		  //��������֡����
	uint32 ZeroCnt;			  //����Ϊ0����
	uint8  WaitPass;          //�ȴ�͸��
}JGRecSet;


typedef struct tagDLLAtState
{
	uint8 DllOpened;//ͨ���Ƿ�������ϴ�
	uint8 ManHanded;//�ֶ���β
	uint8 ForceVout;//��3��ǿ�Ƴ���
	uint8 QueCheckFlag;//���м����
	uint16 CheckFrCnt;//���֡����
}DllAtState;//��DLL����״̬

#endif


typedef struct tagVehOutSet
{
	uint8 u8EmptyFrCnt;//�հ�֡��
	uint8 u8Ax3Force;//��3��ǿ�Ƴ���
	uint8 u8DirHiCnt;//�����豸����ֵ����
	uint8 u8DirLwCnt;//�����豸����ֵ����
	uint8 u8DirLvCnt;//�����豸�뿪����
	uint16 u16GratHt;//��դ�����߶�����
	uint16 u16CRC;//CRCУ��
}VehOutSet; //������β��ֵ����


typedef struct tagWeightAtSet
{
	uint8 u8wAtChannel;//��̨����ͨ��
	uint8 u8wAckEnable;//��̨����ʹ��
	uint8 u8wAxleReprot;//��̨�����ϱ�
	uint8 u8wCarClassify;//��̨�ֳ��ϱ�
	uint8 u8wAckSendFlag;//��ʼ��������
	uint16 u16wAckTmCnt;//������������ ÿ1s�ӿ�ʼ����
	uint16 u16Crc;//crcУ��
}WeightAtSet;

typedef struct tagVehIndexSet
{
	uint16 u16Year;
	uint8 u8Mon;
	uint8 u8Day;
	uint32 u32Index;
	uint16 u16Crc;	
}VehIndexSet;

typedef struct tagSysModelParam
{
	uint16 ZeroPos;//���ĵ�
	uint16 JGHeight;//�������߶�
	uint16 StartX;//��������ʼ��X(��Ѱ�������ʼX)
	uint16 LaneWidth;//��·���
	uint16 MaxHt;//��󾻿ո߶�
	uint16 IsLanHt;//��ȫ���߶�
	uint16 IsLanWd;//��ȫ�����
	//uint8 ScanMode;//ɨ�跽ʽ  1--�ɸߵ��ͣ�0---�ɵ͵���
    uint16 u16CRC;//CRCУ��	
}SysModelParam;//ϵͳģ�Ͳ����ṹ��

#define VEHICLE_MASK	0x07 //���������
#define VEHICLE_MAX		0x08 //�����


#define FRAME_MAXCNT 30000 //���֡�� 60000
#define FRAME_BUFLEN 0x100 //�����������Ч���� 256	
#define FRAME_MASK      FRAME_MAXCNT-1 //���֡������

#define NO_USED			0x00  //��Ч
#define OCCURING_USED	0x01  //���ڽ���ʱ
#define PASSED_USED		0x02  //��ʻ��

#define NORMAL_MAX_EMPTYFRAME  0x0A //�����հ�֡����   0x0A

typedef struct tagRegionSet
{
	int32  xdata[FRAME_BUFLEN];
	int32  zdata[FRAME_BUFLEN];
	int32  xMax; //x���λ��
	int32  xMin;//x��Сλ�� 
	int32  zMax; 
}RegionSet;


typedef struct tagFramInfo
{
	uint8   u8Valid;//������Ч
	uint8   u8Matched;//ƥ����
	uint16  u16GrFitCnt;//�����դ�߶ȵ���
	uint16  u16PtSum;//��������Ч�����
	uint16  u16MaxHt; //���߶�ֵ
	uint16  u16MaxHtPt;//���߶ȵ�λ��
	uint16  u16SecHt;//�ڶ��߶�ֵ
	uint16  u16xMinPt; //x��Сλ�õ�
	uint16  u16xMaxPt; //x���λ�õ�
	uint16  u16Width;//�����
	uint16  u16StaPos;  //ÿ֡��ʼ��Ч��Ŀ�ʼ����
	uint16  u16EndPos;  //ÿ֡��ʼ��Ч�����ֹ����
	int32   n32xMin;  //x��Сλ�� 
	int32   n32xMax; //x���λ��
	uint32  u16AvgX;//x��ƽ��λ��
	uint32  u32TimeStamp;//ʱ���
}FramInfo;


typedef struct tagArcSet
{
	uint8 IsArcShape;//�Ƿ�Ϊ���Ƚṹ
	uint8 ExistArc;//�Ƿ���ڻ���
	uint16 StFramIdex;//���ο�ʼ֡��
	uint16 EdFramIdex;//���ν���֡��
	uint16 ArcFramCnt;//����֡����
	uint16 SimArcCnt;//���ƻ��ε�֡����
	uint8  Arcflag[FRAME_MAXCNT]; //������� 1-��ʶ���Σ�2-���ƻ��Σ�0-δ�ж���
	uint16 ArcCnt;//�»��μ��֡��
}ArcInfo;


typedef struct tagAxleInfo
{
	int AxleOriIndex;//���ֳ���֡ ԭʼ֡���
	int AxleStaIndex;//������ʼ֡���
	int AxleEndIndex;//�������֡���
	int AxleType;//��������
	int AxleCastX;//����ͶӰ���x
	int AxleLeftDelt;//������벿�ֱ仯��
	int AxleRightDlet;//�����Ұ벿�ֱ仯��
	int AxleOriStaI;//������ʼ֡ ԭʼ֡���
	int AxleOriEndI;//���ֽ���֡ ԭʼ֡���
	int AxleWidthMax;//�߶���ֵ�µĵ��X���ֵ
	int AxleSpeed;//�����ٶ�
	int AxleGroup;//������
}AxleInfo;


#define	MAX_AXLE 20	///�������

typedef struct tagAxleSetInfo
{
	int8 AxleSum;//������
	AxleInfo Axle[MAX_AXLE];//ÿ����ϸ��Ϣ
}AxleSetInfo;


typedef struct tagSideUniform
{
	uint16 SideCheckCnt;//����1m���²���һ���Լ����֡��
	uint16 SideFitCnt;//�������һ���Ե�֡����
	uint8 SideFlag[FRAME_MAXCNT];//ÿ֡һ���Ա��;1--����һ���ԣ�2--��գ�
}SideUniform;


typedef struct tagVehEdgeSet
{
	uint16 u16Width;//���
	uint16 u16HeadHeight;//��һ���Ӧ�ĳ��� ����ͷ��
	uint16 u16RearHeight;//��β���߶� 
	uint16 u16Axle2Ht;//�ڶ����Ӧ�ĸ߶�
	uint16 u16Axle2FrCnt;//�ڶ����Ӧ��֡��
	uint16 u16Axle3Ht;//�������Ӧ�ĸ߶�
	uint16 u16Axle3FrCnt;//�������Ӧ��֡��
	uint16 u16VehHeight;//����
	uint16 u16VehHtIndex;//�����߶�����֡
	uint16 u16MaxHeight[FRAME_MAXCNT];//ÿ֡�����߶�
	uint8  u8CastGrdFlag[FRAME_MAXCNT];//ÿ֡��500�����Ƿ�����м�δ�͸ֻ���ϵ�
	uint16 u16SecHeight[FRAME_MAXCNT];//ÿ֡�ĵڶ���߶�
	uint16 u16MinHeight[FRAME_MAXCNT];//ÿ֡����͸߶�
	uint16 u16A1MinHeight[2];//��һ�������ڶ���ǰÿ֡��С�߶�
	uint16 u16A1MinHtCnt[2];//��һ�������ڶ���ǰ�ܹ�֡��
	uint8  u8HeightAdjFlag[FRAME_MAXCNT]; //�߶��޸ı�ʶ
	uint16 u16MinXEdge[FRAME_MAXCNT];//ÿ֡�����x
	uint16 u16MinX;//����֡�����x
	uint16 u16MinXIndex;//�����x֡����
}VehEdgeSet;


typedef struct tagFramShape
{
	int32 BaseX;//���X��׼
	int32 FrLoDFCnt;//1m���´�ɼ���
	int32 FrHiDFCnt;//1m���ϴ�ɼ���
	uint8 LoDaFeiR;//1m���´��ռ��
	uint8 HiDaFeiR;//1m���ϴ��ռ��
	uint8 u8DaFeiCnt[FRAME_MAXCNT];//1m����������ɵ���
	uint8 u16FrShape[FRAME_MAXCNT];//ÿ֡����״ 1--��������ƽ��(���߽ṹ)��2--����ƽ��+����ƽ��(�����ṹ)��3--����ƽ��+�в��ػ�(�����ṹ)��
	uint16 u16LeftMaxHt[FRAME_MAXCNT];//ÿ֡�����ϵ� --�洢Zֵ��
}FramShape;

typedef struct tagSpecialVehSet
{
	uint8 u8PickUpFlag[FRAME_MAXCNT];//Ƥ�����֡
	uint16 u16CloseWin[FRAME_MAXCNT];//��ճ����������;

}SpeVehSet; //���⳵����⼯




typedef struct tagDpHeight
{
	uint32 DpHeight1_2;//����1-2֮����̸߶�
	uint32 DpHtHist1_2[5][2];//����1-2֮����̸߶�һ�δ洢380-430-480-530-580
	uint16 DpFrSum1_2;//����1-2֮���֡��
	uint32 DpHeight2_3;//����2-3֮��ĵ��̸߶�
	uint32 DpHtHist2_3[5][2];//����2-3֮����̸߶�һ�δ洢380-430-480-530-580
	uint16 DpFrSum2_3;//����2-3֮���֡��
}DpHeight;


typedef struct tagCheDouInfo
{
	uint16 CheDouStaI;//������ʼ֡
	uint16 CheDouEndI;//��������֡
	uint16 CheDouCnt;//����֡��
	uint8  CheDouFlag[FRAME_MAXCNT];//����֡���
}CheDouInfo;


typedef struct tagContainerInfo
{
	uint16 ContainerStaI;//���俪ʼ֡
	uint16 ContainerEndI;//�������֡
	uint16 ContainerCnt;//����֡��
	uint8  ContainerFlag[FRAME_MAXCNT];//����֡���
}ContainerInfo;


#define MAPBLOCKCNT 3

typedef struct tagGlassBlock
{
	uint16 FramIndex;//֡���
	uint16 StaHt;//������ʼ�߶�
	uint16 EndHt;//���������߶�
	uint16 GlassHt;//�����θ߶�
}GlassBlock;

typedef struct tagGlassWinInfo
{
	uint16 GlassWinStaI;//����������ʼ֡
	uint16 GlassWinEndI;//������������֡
	uint16 GlassWinCnt;//��������֡��
	uint8  GlassWinFlag[FRAME_MAXCNT];//��������֡��� 2--��͸������; 4--б�ʷ�����;8--��͸�����;16--С�ͳ������ɳ���;32--���ͳ������ɳ���;
	uint8 BlockCnt;//������ͼ֡��
	uint8 BlockIndex;//���������
	GlassBlock BlockSet[MAPBLOCKCNT];//������ͼ��
}GlassWinInfo;

typedef struct tagJudgeInfo
{
	uint8 u8JudgePattern;//�ж��ĳ���;
	uint8 u8JudgeRule;//�ж�׼������
	uint8 u8ModifyRule;//���͵���׼������
	uint8 u8SpeVehCode;//ר������
}JudgeInfo;



typedef struct tagFeatureFlag
{
	uint8 IsSideUiform;//�Ƿ�߱�����һ����
	uint8 IsCastGrd;//�Ƿ���м�Ͷ��������
	uint8 IsHopper;//�Ƿ�Ϊ������1-Ϊԭʼ��������2-������භ��4-���ᶨλ��
	uint8 IsContainer;//�Ƿ�߱����� 0--�����ڻ��䣻1--����ƽ����2--��ʻ�ҵ���ߣ�
	uint8 IsHeadJump;//�Ƿ���ڳ�ͷ���� 0--�����ڣ�1--ֱ���ҵ����ڣ�2--���ɶδ��ڣ�
	uint8 IsTailJump;//�Ƿ����β������ 0--�����ڣ�1--���ڣ�
	uint8 IsCargo;//�Ƿ��ػ� 1--X�����ػ���2--�����ػ���
	uint8 IsFillWin;//�Ƿ��ճ�����
	uint8 IsGlassWin;//�Ƿ���ڲ�������
	uint8 IsTruckTail;//�Ƿ���ڳ�β;

	uint8 IsHtRough;//�߶�ƽ��
	

	uint8 IsCar;//�Ƿ�Ϊ�ͳ�
	uint8 IsTruck;//�Ƿ�Ϊ����

	uint8 IsJinBei;//�Ƿ�𱭳�
	uint8 IsJzx;//�Ƿ�Ϊ��װ��;
	uint8 IsYgc;//�Ƿ�Ϊ�͹�
	uint8 IsMotor;//�Ƿ�ΪĦ�г�

}FeatureFlag;

typedef struct tagClusterInfo
{
	uint16 Sta;//��ʼ
	uint16 End;//��ֹ
	uint16 Num;//����
	uint16 MaxZ;//����
	uint16 MaxZI;//�������
	uint16 MinZ;//��С��
	uint16 MinZI;//��С�����
	uint32 SumH;//���ۼƺ�
	uint16 AvgH;//�߶��ۼ�ֵ��ֵ
	uint16 MinX;//��С��xֵ
	uint16 MinXI;//��Сxֵ��λ�ã�
	uint16 MaxX;//���xֵ��
	uint16 MaxXI;//���xֵ��λ��
	uint16 AvgX;//x�ľ�ֵ;
	uint32 SumX;//x���ۼ�ֵ
	uint16 FlatCnt;//ƽ̹֡��
	uint8 Style;//�ڲ��仯���� 1-�½���2-ƽ̹��3-������4-δ�ж���
	uint8 FlatR;//ƽ̹ռ��
	uint16 DaFeiCnt;//�öδ��֡��
}ClusterInfo;


#define MAX_CUT 512 //���������� ��ֹͣ�������쳣
#define MAX_CLUSTER 20 //���������
typedef struct tagHeightCluster
{
	uint8 IsOk;//��������
	uint16 Hmin;//��Сֵ
	uint16 Hmax;//�ݶ����ֵ
	uint16 HmaxIdx;//�����ֵ�λ��
	uint8 ClNum;//�������
	ClusterInfo CI[MAX_CLUSTER];//����μ���
	//uint8 TrendNum;
	//int Trend[20];
}HeightCluster;

#define MOVE_NONO 0  //Ĭ�� ����
#define MOVE_FRNT 1  //ǰ��
#define MOVE_BACK 2  //����
#define MOVE_FRBK 3  //���벻��ȫ����
#define MOVE_BKFR 4  //��������ȫ�ֽ���
#define MOVE_ABNO 5  //�쳣 ��ͬʱ����
#define MOVE_DELE 6  //ɾ�����

typedef struct tagDirCheck
{
	uint8 u8DirTrigged;//������ʻ�������д���
	uint8 u8CL_StateHi;//��ֱ�����⵽��ʱ���ظ���Ȧ��״̬����������ֵ�����ڴ󳵷���
	uint8 u8CL_StateLw;//��ֱ�����⵽��ʱ���ظ���Ȧ��״̬����������ֵ������С�ͳ�����
	uint8 u8CL_StateLv;//��ֱ������û�м�⵽��ʱ���ظ���Ȧ��״̬����
	uint8 u8GR_Valided;//��դ��ǰ֡��Ч
	uint16 u16GR_FitFrCnt;//��դ��Ч֡
	uint8 u8GR_StateLS;//�����г�����դ�޳�����
	uint8 u8GR_StateLG;//����͹�դ���м���
	uint8 u8GR_StateGR;//�����޳�����դ�г�
	uint8 u8VehDir;//ǰ������״̬	
}DirCheck;

#define NORMAL 0
#define FORCED 1
#define HANDED 2

typedef struct tagVehicleSet
{
	uint8 u8State;//����״̬
	uint8 u8VoutFlag;//������������ 0--����������1--ǿ�Ƴ�����2--�ֶ�����
	uint32 StaWaveIndex;//��ʼ�������
	uint32 EndWaveIndex;//�����������

	uint16 EmptyFrame; //�հ�֡��

	uint16 u16FrameCnt; //��֡��

	FramInfo CurFrmInfo;//��һ֡ɨ����Ϣ

	RegionSet CurRegion;//��һ֡�����; 

	DirCheck DirSet;//�������¼��
	
	ArcInfo ArcSet;//���μ�¼��

	AxleSetInfo AxleSet;//�������¼�� 

	SideUniform SideSet;//����һ���Լ���¼��

	VehEdgeSet EdgeSet;//�����߶���Ϣ��¼��

	DpHeight Dp;//���̸߶���Ϣ

	CheDouInfo HopperSet;//������⼯

	ContainerInfo ContainerSet;//�����⼯

	GlassWinInfo WindowsSet;//����������⼯

	FeatureFlag FeatureSet;//�ۺ�������

	JudgeInfo JudgeSet;//�����ж���¼��

	HeightCluster Cluster;//������༯��

	FramShape FrShape;//ÿ֡�����η���

	SpeVehSet SpeVSet;//���⳵����⼯

#ifdef _DEBUG
	RegionSet PC[FRAME_MAXCNT];	 //����֡��Ϣ
#endif

}VehicleSet;//������Ϣ��¼





#endif



