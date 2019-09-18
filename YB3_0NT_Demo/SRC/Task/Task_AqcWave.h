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

////定义存储用地址
//#define  SAVEADDRINFO_SDADDR      0x3FFFFF //存储波形相关地址的扇区
//#define  STARTSECTORNO_FRAMESAVE  0x400000 //起始扇区 2G位置
//#define  ENDSECTORNO_FRAMESAVE    0x800000 //终止扇区 4G位置
//#define  SIZESECTOR_FRAMESAVE     0x400000//扇区数

#define  SAVEADDRINFO_SDADDR      0x3FFFFF //存储波形相关地址的扇区
#define  STARTSECTORNO_FRAMESAVE  0xA00000 //起始扇区 5G位置
#define  ENDSECTORNO_FRAMESAVE    0x1A00000 //终止扇区 13G位置
#define  SIZESECTOR_FRAMESAVE     0x1000000//扇区数


//SD卡插入检测
#define WLC864I89    89
#define WLC864I892   892
#define CONTROLLER   WLC864I892  //大控制器 89  ；小控制器为892
#define SD_INSERT (P3_INP_STATE & (1<<25))


//波形存储至SD卡记录结构体
#define BHeaderSize 137
#define CACHENUM 16
#define SECTORSIZE 512
#define FRAMESIZE 500 //测试 改为500  实际为1000
#define ISOPENED  0xAA
#define ISCLOSED  0xBB
#define F1BlockNum 2

typedef struct tagFramSave
{
	uint8  FgFlag; //覆盖标志位
    uint16 SvIndx ;//存波形帧序号
    uint16 IFrIndx[CACHENUM];//入缓存的帧序号
    //uint16 OFrIndx[CACHENUM];//出缓存的帧序号
    uint32 CurSvAddr;//当前存储的sd卡地址
    uint32 SwSdCnt;//应该写入sd卡帧数
    uint32 WrSdCnt;//实际写入sd卡的帧数
    uint8  Disabled;//存波形使能标识位 0xAA - 使能； 0xBB - 不使能
    uint8  ICacheIndx;//进缓存索引
    uint8  OCacheIndx;//出缓存索引
    uint8 FrameSaveBuf[CACHENUM][FRAMESIZE][F1BlockNum*SECTORSIZE];//缓存数组
} FramSave;

extern FramSave g_WvSave;//存波形记录结构体
void Task_AqcWave(void *tdata);
void Init_FrameSave(void);//初始化函数
void Task_Aqc_Wave(void *tdata);//存波形任务
uint8 Fram_Sv2SD(uint8 *pData,uint16 nSize,uint32 Sd_Addr);//存储波形至sd卡
void Catch_Frame_RT(VehicleSet* pVehicle,uint8 nCmd);//在线抓帧
uint8 SaveAddrInfoTsd(void);//存储地址相关信息
void ReadAddrInfoFsd(void);//读取sd卡地址信息
void SetDefualtAddr(void);//将存储地址的相关信息设置为默认值
uint8 Fram_Sv2SD_Ex(uint8 *pData,uint8 nNum,uint32 Sd_Addr);//一次写多扇区存储方式存一帧波形

#endif
