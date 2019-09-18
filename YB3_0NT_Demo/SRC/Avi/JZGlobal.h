/****************************************Copyright (c)****************************************************
**                         			BEIJING	WANJI(WJ)                               
**                                     
**
**--------------File Info---------------------------------------------------------------------------------
** File name:			JZGlobal.h
** Last modified Date:  2011511
** Last Version:		1.0
** Descriptions:		计重程序全局变量
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

//信号量  
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
JZG_EXT OS_EVENT *g_NetS3Recv;//端口3的信号量
JZG_EXT OS_EVENT *g_UpdateApp;	
     

JZG_EXT	SystemTime		g_SysCurTime;//当前时间 
JZG_EXT DevUartSet      g_UartSet;//设备接口
JZG_EXT LaserPassTo     g_LaserPass;//激光透传
JZG_EXT NetInfo         g_NetInfo;//系统网络参数
JZG_EXT SysTemSet       g_SysSetUp;//系统设置
JZG_EXT VehIndexSet     g_VehIndex;//车辆序号记录集


#if	YBVERSION >= 30		//3.0仪表功能

#else	//2.2仪表功能
JZG_EXT	void	(*bootloader_entry)(void);
#endif	//#if	YBVERSION >= 30		//3.0仪表功能


JZG_EXT uint32 Net4_DisconnectNum ;
JZG_EXT uint32 Net1_DisconnectNum ;
JZG_EXT uint32 Net2_DisconnectNum ;
JZG_EXT uint32 Net3_DisconnectNum ;

JZG_EXT uint32 Net1_InvalidRecNum ;
JZG_EXT uint32 Net2_InvalidRecNum ;
JZG_EXT uint32 Net3_InvalidRecNum ;


//串口接收解析用

JZG_EXT volatile uint8 uart1_timeout ;
JZG_EXT volatile uint8 uart5_timeout ;

#define	UARTRCVBUFSIZE		8192		//串口5缓存大小
#define MAX_FRAME_LENGTH 256  //协议最大帧长度
#define MIN_FRAME_LENGTH 7	  //协议最小帧长度
#define MIN_FRAME_LENGTH_TC 18	  //透传协议最小帧长度
#define MAX_FRAME_LENGTH_TC 1800	  //透传协议最小帧长度
JZG_EXT	volatile uint16	 g_UART5_Recv_Index;      //串口5缓存接收索引
JZG_EXT	volatile uint16	 g_UART5_Proc_Index;      //串口5缓存处理索引
JZG_EXT uint8 g_u8U5RcvBuf[UARTRCVBUFSIZE] ; //串口5接收缓存
JZG_EXT	volatile uint16	 g_UART1_Recv_Index;      //串口1缓存接收索引
JZG_EXT	volatile uint16	 g_UART1_Proc_Index;      //串口1缓存处理索引
JZG_EXT uint8 g_u8U1RcvBuf[UARTRCVBUFSIZE] ; //串口1接收缓存


#define MAX_FRAME_LENGTH_AA 200  //协议最大帧长度
#define MIN_FRAME_LENGTH_AA 8	  //协议最小帧长度	
#define MAX_FRAME_LENGTH_FA 200  //协议最大帧长度
#define MIN_FRAME_LENGTH_FA 7	  //协议最小帧长度	 
#define MAX_FRAME_LENGTH_FF 8000  //协议最大帧长度
#define MIN_FRAME_LENGTH_FF 8	  //协议最小帧长度
#define WAVE_FRAME_LENGTH   12    //传输并车波形协议帧长  固定


//更新程序用
JZG_EXT	SystemTime	g_sstCurTime_Remote; 
JZG_EXT uint8 g_Update_App_Step;
JZG_EXT uint8 g_bStarted;
JZG_EXT uint8 g_Update_App_CH;//更新程序通道
JZG_EXT uint8 ZYJ_MJH;	  //更新期间禁止传输02 08等相关数据包
JZG_EXT uint8 g_u8ParseBuf[2500];
JZG_EXT uint8 g_u8FrmNo;
JZG_EXT uint8 MIN_Remote; //远程更新专用时钟信息
JZG_EXT uint32 g_u32ParseLen;


//系统重启
#define ResetSystem()  	{WDTInit(1),WDTIM_COUNTER =  0x10000000-100;while(1); 	}


//接收波形用缓存
//接收激光器波形缓存
#define LMDMAXCACHE 500
JZG_EXT uint8 g_LmdCache[LMDMAXCACHE][Full_Size+1]; //激光数据缓存 500个；	
JZG_EXT uint16 g_LmdCacheIn; //两激光缓存计数；
JZG_EXT uint16 g_LmdCacheOut; //两激光缓存计数；
JZG_EXT DllAtState g_DllAtState;//与上位机DLL交互的状态
#endif

//新加的定义 20170628

//错误定义
#define ERROR_NONE 0 //无错误
#define ERROR_LOGI 1 //逻辑错误

//LMS_Data相关定义

#define POINT_CEN 180 //理想中心点
#define POINT_SUM 361 //扫描点数
#define POINT_END 360 //扫描最后点序号
#define LMS_MINTH 20 //测距距离下限 
JZG_EXT int32 LMS_data_1[POINT_SUM+1];//扫描距离数组


//定义相关阈值
#define THRESH_H1 350  //第一高度阈值 350 
#define THRESH_H2 500  //第二高度阈值 500
#define THRESH_H3 520  //第三高度阈值 520

#define VEHTH_LOW 150  //车辆点高度阈值下限
#define VEHTH_UP  4800 //车辆点高度阈值上限

#define ISLANHT  800 //安全岛高度
#define ISLANWD  300 //安全岛宽度

//逻辑判断相关阈值
#define     MINHEIGHT_JIZHUANGXIANG   1600   //用于检测货5是否是集装箱的最低高度
#define     MAXHEIGHT_JIZHUANGXIANG   3800   //用于检测货5是否是集装箱的最大高度
#define     MAXDIFFER_JIZHUANGXIANG   150    //用于检测货5是否是集装箱的宽度最大差值
#define     CHASSISHEIGHTTHRES        430    //客货区分的底盘高度阈值

#define  DAFEIMAXPT     4             //坐标转换中打飞的最大转换点数

//定义车辆轴数
#define     ONE_AXLE                  1
#define     TWO_AXLE                  2   
#define     THREE_AXLE                3
#define     FOUR_AXLE                 4
#define     FIVE_AXLE                 5
#define     SIX_AXLE                  6	

//定义车型宏
#define     FIRST_KECHE               1      //第一类客车 <=7座
#define     SECOND_KECHE         	  2      //第二类客车 8—19座
#define     THIRD_KECHE               3      //第三类客车 20—39座
#define     FOURTH_KECHE              4      //第四类客车 >=40座
#define     FIRST_HUOCHE              5      //第一类货车 <=2吨
#define     SECOND_HUOCHE			  6      //第二类货车 2—5吨（含）
#define     THIRD_HUOCHE              7      //第三类货车 5—10吨（含）
#define     FOURTH_HUOCHE             8      //第四类货车 10—15吨（含）
#define     FIFTH_HUOCHE              9      //第五类货车 >15吨	
#define     FIRST_JIZHUANGXIANG       10     //第一类集装箱
#define     SECOND_JIZHUANGXIANG      11     //第二类集装箱

//算法相关定义 全局变量
JZG_EXT uint32 g_n32WaveNumber;//vc中用
JZG_EXT SysModelParam Smp;//系统模型参数
JZG_EXT uint8  g_u8ItemName[24];//记录波形所在文件夹名称
JZG_EXT FramInfo CurFramInfo;//当前帧信息
JZG_EXT VehOutSet g_VehOutSet;//车辆收尾相关阈值
JZG_EXT WeightAtSet g_WeigthAt;//与称台交互参数

//下位机队列维护的宏定义
#define	MAXVEHQUE	10	//定义下位机队列存储数组的大小
JZG_EXT uint8   g_u8VehQueNum;       //动态库队列数目
JZG_EXT uint8   g_u8VehQue[MAXVEHQUE];  //循环数组，保存最近上传的10个车辆信息
JZG_EXT uint8 	g_u8VehQueIndex;  //g_u8VehQue数组的索引，值为0-9


JZG_EXT int32 g_CoorX[POINT_SUM]; //扫描点X坐标坐标
JZG_EXT int32 g_CoorZ[POINT_SUM]; //扫描点Z坐标坐标
JZG_EXT int32 g_FiltX[POINT_SUM]; //扫描点X坐标滤波后坐标
JZG_EXT int32 g_FiltZ[POINT_SUM]; //扫描点Z坐标滤波后坐标




#endif



