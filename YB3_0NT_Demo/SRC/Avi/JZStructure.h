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
	uint8 BR_COM1;//COM1波特率代码 1-5
	uint8 BR_COM2;//COM2波特率代码 1-5
	uint8 TS_CHCOM;//调试指令通道	
	uint8 RM_CHCOM;//响应远程指令通道
	uint16 u16CRC;//CRC校验	
}DevUartSet;


#define MAXPASSLEN 200
typedef struct tagLaserPassTO
{
	uint8 PassCH;//透传通道
	uint8 PassCont[MAXPASSLEN];
}LaserPassTo;

#define Dev_NONO 0 //无设备
#define DEV_COIL 1 //线圈
#define DEV_GRAT 2 //光栅
#define DEV_LASE 3 //激光
#define Dev_REFL 4 //反光镜

typedef struct tagSystemParam	
{
	uint8   au8ProgramVersion[11];//软件版本
	uint16  hResetCnt;			 //硬件重启次数
	uint16  sResetCnt;			 //软重启次数
	uint8 	u8DOGEnable;		 //看门狗使能
	uint8   u8SDEnable;          //sd卡使能
	uint8   u8DirDevType;        //行进方向识别设备种类 1--COIL;2--GRATING 3--LASER;
	uint8   u8DirDevEnable;      //方向设备使能
	uint8   u8WorkMode;			 //工作模式 0-现场运行；1--模拟运行;
	uint8   u8DeviceID[15];		 //设备识别码
	uint8   u8StationID[16];	 //站点编号
	uint16	u16CRC;				 //CRC校验		
} SysTemSet;

typedef	struct tagNetInfo
{
	uint8	au8IPAddr[4];	//控制器IP地址
	uint16	u32LocalPortNO;	//控制器端口号
	uint8	au8SubMask[4];	//子网掩码
	uint8	au8GatewayIP[4];//默认网关		
	uint8	au8MACAddr[6];	//MAC
	uint8  SocketIP[8][4];//socket对应的ip
	uint32 SocketPort[4];//socket对应的端口
	uint16	u16CRC;				 //CRC校验
} NetInfo;	 


typedef	struct tagSystemTime
{
	uint16	u16Year;				//年
	uint8	u8Month;				//月
	uint8	u8Day;					//日
	uint8	u8Week;					//星期几
	uint8	u8Hour;					//时
	uint8	u8Minute;				//分
	uint8	u8Second;				//秒
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
	uint8  nCmd;			  //帧类型 1 测距数据； 2 非测距数据;
	uint8  Data[2*Full_Size]; //激光缓存数组
	uint32 RecLen;            //接收长度
	uint32 PostCnt;           //中断计数
	uint32 ErrFrCnt;          //错误帧计数 即帧头不为02
	uint32 RecWvCnt;		  //激光测距数据计数
	uint32 ProWvCnt;          //处理测距波形计数
	uint32 RecCmdCnt;		  //激光命令帧计数
	uint32 ZeroCnt;			  //长度为0数据
	uint8  WaitPass;          //等待透传
}JGRecSet;


typedef struct tagDLLAtState
{
	uint8 DllOpened;//通道是否打开允许上传
	uint8 ManHanded;//手动收尾
	uint8 ForceVout;//超3轴强制出车
	uint8 QueCheckFlag;//队列检查标记
	uint16 CheckFrCnt;//检查帧计数
}DllAtState;//与DLL交互状态

#endif


typedef struct tagVehOutSet
{
	uint8 u8EmptyFrCnt;//空白帧数
	uint8 u8Ax3Force;//超3轴强制出车
	uint8 u8DirHiCnt;//方向设备高阈值计数
	uint8 u8DirLwCnt;//方向设备低阈值计数
	uint8 u8DirLvCnt;//方向设备离开计数
	uint16 u16GratHt;//光栅触发高度上限
	uint16 u16CRC;//CRC校验
}VehOutSet; //车辆收尾阈值设置


typedef struct tagWeightAtSet
{
	uint8 u8wAtChannel;//称台交互通道
	uint8 u8wAckEnable;//称台心跳使能
	uint8 u8wAxleReprot;//称台过轴上报
	uint8 u8wCarClassify;//称台分车上报
	uint8 u8wAckSendFlag;//开始发送心跳
	uint16 u16wAckTmCnt;//发送心跳计数 每1s钟开始发送
	uint16 u16Crc;//crc校验
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
	uint16 ZeroPos;//中心点
	uint16 JGHeight;//激光器高度
	uint16 StartX;//即车道开始的X(搜寻轮轴的起始X)
	uint16 LaneWidth;//道路宽度
	uint16 MaxHt;//最大净空高度
	uint16 IsLanHt;//安全岛高度
	uint16 IsLanWd;//安全岛宽度
	//uint8 ScanMode;//扫描方式  1--由高到低；0---由低到高
    uint16 u16CRC;//CRC校验	
}SysModelParam;//系统模型参数结构体

#define VEHICLE_MASK	0x07 //最大车数掩码
#define VEHICLE_MAX		0x08 //最大车数


#define FRAME_MAXCNT 30000 //最大帧数 60000
#define FRAME_BUFLEN 0x100 //车辆的最大有效点数 256	
#define FRAME_MASK      FRAME_MAXCNT-1 //最大帧数掩码

#define NO_USED			0x00  //无效
#define OCCURING_USED	0x01  //正在进行时
#define PASSED_USED		0x02  //已驶过

#define NORMAL_MAX_EMPTYFRAME  0x0A //正常空白帧上限   0x0A

typedef struct tagRegionSet
{
	int32  xdata[FRAME_BUFLEN];
	int32  zdata[FRAME_BUFLEN];
	int32  xMax; //x最大位置
	int32  xMin;//x最小位置 
	int32  zMax; 
}RegionSet;


typedef struct tagFramInfo
{
	uint8   u8Valid;//区域有效
	uint8   u8Matched;//匹配结果
	uint16  u16GrFitCnt;//满足光栅高度点数
	uint16  u16PtSum;//区域有有效点个数
	uint16  u16MaxHt; //最大高度值
	uint16  u16MaxHtPt;//最大高度点位置
	uint16  u16SecHt;//第二高度值
	uint16  u16xMinPt; //x最小位置点
	uint16  u16xMaxPt; //x最大位置点
	uint16  u16Width;//最大宽度
	uint16  u16StaPos;  //每帧开始有效点的开始索引
	uint16  u16EndPos;  //每帧开始有效点的终止索引
	int32   n32xMin;  //x最小位置 
	int32   n32xMax; //x最大位置
	uint32  u16AvgX;//x的平均位置
	uint32  u32TimeStamp;//时间戳
}FramInfo;


typedef struct tagArcSet
{
	uint8 IsArcShape;//是否为弧度结构
	uint8 ExistArc;//是否存在弧段
	uint16 StFramIdex;//弧段开始帧序
	uint16 EdFramIdex;//弧段结束帧序
	uint16 ArcFramCnt;//弧度帧计数
	uint16 SimArcCnt;//类似弧段的帧计数
	uint8  Arcflag[FRAME_MAXCNT]; //标记数组 1-标识弧段；2-类似弧段；0-未判定；
	uint16 ArcCnt;//新弧段检测帧数
}ArcInfo;


typedef struct tagAxleInfo
{
	int AxleOriIndex;//车轮出轴帧 原始帧序号
	int AxleStaIndex;//轮轴起始帧序号
	int AxleEndIndex;//轮轴结束帧序号
	int AxleType;//车轮类型
	int AxleCastX;//车轮投影最近x
	int AxleLeftDelt;//车轮左半部分变化量
	int AxleRightDlet;//车轮右半部分变化量
	int AxleOriStaI;//车轮起始帧 原始帧序号
	int AxleOriEndI;//车轮结束帧 原始帧序号
	int AxleWidthMax;//高度阈值下的点的X相差值
	int AxleSpeed;//轮轴速度
	int AxleGroup;//轴组标记
}AxleInfo;


#define	MAX_AXLE 20	///最大轴数

typedef struct tagAxleSetInfo
{
	int8 AxleSum;//总轴数
	AxleInfo Axle[MAX_AXLE];//每轴详细信息
}AxleSetInfo;


typedef struct tagSideUniform
{
	uint16 SideCheckCnt;//进入1m以下侧面一致性检测总帧数
	uint16 SideFitCnt;//满足侧面一致性的帧计数
	uint8 SideFlag[FRAME_MAXCNT];//每帧一致性标记;1--侧面一致性；2--封闭；
}SideUniform;


typedef struct tagVehEdgeSet
{
	uint16 u16Width;//宽度
	uint16 u16HeadHeight;//第一轴对应的车高 即车头高
	uint16 u16RearHeight;//车尾部高度 
	uint16 u16Axle2Ht;//第二轴对应的高度
	uint16 u16Axle2FrCnt;//第二轴对应的帧数
	uint16 u16Axle3Ht;//第三轴对应的高度
	uint16 u16Axle3FrCnt;//第三轴对应的帧数
	uint16 u16VehHeight;//车高
	uint16 u16VehHtIndex;//车辆高度所在帧
	uint16 u16MaxHeight[FRAME_MAXCNT];//每帧的最大高度
	uint8  u8CastGrdFlag[FRAME_MAXCNT];//每帧中500以下是否出现中间段穿透只地上点
	uint16 u16SecHeight[FRAME_MAXCNT];//每帧的第二大高度
	uint16 u16MinHeight[FRAME_MAXCNT];//每帧的最低高度
	uint16 u16A1MinHeight[2];//第一个轴后与第二轴前每帧最小高度
	uint16 u16A1MinHtCnt[2];//第一个轴后与第二轴前总共帧数
	uint8  u8HeightAdjFlag[FRAME_MAXCNT]; //高度修改标识
	uint16 u16MinXEdge[FRAME_MAXCNT];//每帧最近点x
	uint16 u16MinX;//所有帧最近点x
	uint16 u16MinXIndex;//最近点x帧索引
}VehEdgeSet;


typedef struct tagFramShape
{
	int32 BaseX;//左侧X基准
	int32 FrLoDFCnt;//1m以下打飞计数
	int32 FrHiDFCnt;//1m以上打飞计数
	uint8 LoDaFeiR;//1m以下打飞占比
	uint8 HiDaFeiR;//1m以上打飞占比
	uint8 u8DaFeiCnt[FRAME_MAXCNT];//1m以上连续打飞点数
	uint8 u16FrShape[FRAME_MAXCNT];//每帧的形状 1--整个侧面平板(竖线结构)；2--侧面平板+顶部平板(横竖结构)；3--侧面平板+中部载货(竖曲结构)；
	uint16 u16LeftMaxHt[FRAME_MAXCNT];//每帧的左上点 --存储Z值；
}FramShape;

typedef struct tagSpecialVehSet
{
	uint8 u8PickUpFlag[FRAME_MAXCNT];//皮卡检测帧
	uint16 u16CloseWin[FRAME_MAXCNT];//封闭车窗检测算子;

}SpeVehSet; //特殊车辆检测集




typedef struct tagDpHeight
{
	uint32 DpHeight1_2;//轮轴1-2之间底盘高度
	uint32 DpHtHist1_2[5][2];//轮轴1-2之间底盘高度一次存储380-430-480-530-580
	uint16 DpFrSum1_2;//轮轴1-2之间的帧数
	uint32 DpHeight2_3;//轮轴2-3之间的底盘高度
	uint32 DpHtHist2_3[5][2];//轮轴2-3之间底盘高度一次存储380-430-480-530-580
	uint16 DpFrSum2_3;//轮轴2-3之间的帧数
}DpHeight;


typedef struct tagCheDouInfo
{
	uint16 CheDouStaI;//车斗开始帧
	uint16 CheDouEndI;//车斗结束帧
	uint16 CheDouCnt;//车斗帧数
	uint8  CheDouFlag[FRAME_MAXCNT];//车斗帧标记
}CheDouInfo;


typedef struct tagContainerInfo
{
	uint16 ContainerStaI;//货箱开始帧
	uint16 ContainerEndI;//货箱结束帧
	uint16 ContainerCnt;//货箱帧数
	uint8  ContainerFlag[FRAME_MAXCNT];//货箱帧标记
}ContainerInfo;


#define MAPBLOCKCNT 3

typedef struct tagGlassBlock
{
	uint16 FramIndex;//帧序号
	uint16 StaHt;//玻璃开始高度
	uint16 EndHt;//玻璃结束高度
	uint16 GlassHt;//玻璃段高度
}GlassBlock;

typedef struct tagGlassWinInfo
{
	uint16 GlassWinStaI;//玻璃车窗开始帧
	uint16 GlassWinEndI;//玻璃车窗结束帧
	uint16 GlassWinCnt;//玻璃车窗帧数
	uint8  GlassWinFlag[FRAME_MAXCNT];//玻璃车窗帧标记 2--穿透法车窗; 4--斜率法车窗;8--穿透面积法;16--小型车区域打飞车窗;32--大型车区域打飞车窗;
	uint8 BlockCnt;//玻璃地图帧数
	uint8 BlockIndex;//玻璃块序号
	GlassBlock BlockSet[MAPBLOCKCNT];//玻璃地图集
}GlassWinInfo;

typedef struct tagJudgeInfo
{
	uint8 u8JudgePattern;//判定的车型;
	uint8 u8JudgeRule;//判定准则依据
	uint8 u8ModifyRule;//车型调整准则依据
	uint8 u8SpeVehCode;//专车代码
}JudgeInfo;



typedef struct tagFeatureFlag
{
	uint8 IsSideUiform;//是否具备侧面一致性
	uint8 IsCastGrd;//是否从中间投掷到地上
	uint8 IsHopper;//是否为车斗；1-为原始斗方法；2-纵向聚类斗；4-轮轴定位斗
	uint8 IsContainer;//是否具备货箱 0--不存在货箱；1--侧面平整；2--驾驶室低箱高；
	uint8 IsHeadJump;//是否存在车头跳变 0--不存在；1--直接找到存在；2--过渡段存在；
	uint8 IsTailJump;//是否存在尾部跳变 0--不存在；1--存在；
	uint8 IsCargo;//是否载货 1--X基线载货；2--外形载货；
	uint8 IsFillWin;//是否封闭车窗；
	uint8 IsGlassWin;//是否存在玻璃车窗
	uint8 IsTruckTail;//是否存在车尾;

	uint8 IsHtRough;//高度平整
	

	uint8 IsCar;//是否为客车
	uint8 IsTruck;//是否为货车

	uint8 IsJinBei;//是否金杯车
	uint8 IsJzx;//是否为集装箱;
	uint8 IsYgc;//是否为油罐
	uint8 IsMotor;//是否为摩托车

}FeatureFlag;

typedef struct tagClusterInfo
{
	uint16 Sta;//起始
	uint16 End;//终止
	uint16 Num;//计数
	uint16 MaxZ;//最大高
	uint16 MaxZI;//最大高序号
	uint16 MinZ;//最小高
	uint16 MinZI;//最小高序号
	uint32 SumH;//高累计和
	uint16 AvgH;//高度累计值均值
	uint16 MinX;//最小的x值
	uint16 MinXI;//最小x值的位置；
	uint16 MaxX;//最大x值；
	uint16 MaxXI;//最大x值的位置
	uint16 AvgX;//x的均值;
	uint32 SumX;//x的累计值
	uint16 FlatCnt;//平坦帧数
	uint8 Style;//内部变化趋势 1-下降；2-平坦；3-上升；4-未判定；
	uint8 FlatR;//平坦占比
	uint16 DaFeiCnt;//该段打飞帧数
}ClusterInfo;


#define MAX_CUT 512 //最大切面个数 防止停车引起异常
#define MAX_CLUSTER 20 //最大聚类个数
typedef struct tagHeightCluster
{
	uint8 IsOk;//聚类正常
	uint16 Hmin;//最小值
	uint16 Hmax;//梯度最大值
	uint16 HmaxIdx;//最大出现的位置
	uint8 ClNum;//聚类个数
	ClusterInfo CI[MAX_CLUSTER];//聚类段集合
	//uint8 TrendNum;
	//int Trend[20];
}HeightCluster;

#define MOVE_NONO 0  //默认 不定
#define MOVE_FRNT 1  //前进
#define MOVE_BACK 2  //倒退
#define MOVE_FRBK 3  //进入不完全倒车
#define MOVE_BKFR 4  //倒出不完全又进入
#define MOVE_ABNO 5  //异常 无同时触发
#define MOVE_DELE 6  //删除情况

typedef struct tagDirCheck
{
	uint8 u8DirTrigged;//车辆行驶过程中有触发
	uint8 u8CL_StateHi;//垂直激光检测到车时，地感线圈的状态计数，高阈值，用于大车方向
	uint8 u8CL_StateLw;//垂直激光检测到车时，地感线圈的状态计数，低阈值，用于小客车方向
	uint8 u8CL_StateLv;//垂直激光器没有检测到车时，地感线圈的状态计数
	uint8 u8GR_Valided;//光栅当前帧有效
	uint16 u16GR_FitFrCnt;//光栅有效帧
	uint8 u8GR_StateLS;//激光有车，光栅无车计数
	uint8 u8GR_StateLG;//激光和光栅都有计数
	uint8 u8GR_StateGR;//激光无车，光栅有车
	uint8 u8VehDir;//前进倒退状态	
}DirCheck;

#define NORMAL 0
#define FORCED 1
#define HANDED 2

typedef struct tagVehicleSet
{
	uint8 u8State;//过车状态
	uint8 u8VoutFlag;//车辆出车类型 0--正常出车；1--强制出车；2--手动出车
	uint32 StaWaveIndex;//起始波形序号
	uint32 EndWaveIndex;//结束波形序号

	uint16 EmptyFrame; //空白帧数

	uint16 u16FrameCnt; //总帧数

	FramInfo CurFrmInfo;//上一帧扫描信息

	RegionSet CurRegion;//上一帧区域点; 

	DirCheck DirSet;//方向检测记录集
	
	ArcInfo ArcSet;//弧段记录集

	AxleSetInfo AxleSet;//轮轴检测记录集 

	SideUniform SideSet;//侧面一致性检测记录集

	VehEdgeSet EdgeSet;//车辆高度信息记录集

	DpHeight Dp;//底盘高度信息

	CheDouInfo HopperSet;//车斗检测集

	ContainerInfo ContainerSet;//货箱检测集

	GlassWinInfo WindowsSet;//玻璃车窗检测集

	FeatureFlag FeatureSet;//综合特征集

	JudgeInfo JudgeSet;//车型判定记录集

	HeightCluster Cluster;//车身聚类集合

	FramShape FrShape;//每帧的外形分析

	SpeVehSet SpeVSet;//特殊车辆检测集

#ifdef _DEBUG
	RegionSet PC[FRAME_MAXCNT];	 //所有帧信息
#endif

}VehicleSet;//车辆信息记录





#endif



