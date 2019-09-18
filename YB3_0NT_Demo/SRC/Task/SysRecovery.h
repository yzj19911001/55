#ifndef _SysRecovery_H
#define _SysRecovery_H

#include "JZGlobal.h"

JZG_EXT void RecoverySystem(void);////恢复系统
void DefualtSysParam(void);//默认系统参数
void RecoModelParam(void);//恢复模型参数
void DefualtModelParam(void);//默认模型参数
void RecoSystemParam(void);//恢复系统参数
void RecordReset(void);//更新重启次数
void RecoveryNetParam(void);////恢复网络参数
void DefualtNetParam(void);	//默认网络参数
void InitGlobalParam(void);//初始化全局变量
void RecoveryUartSet(void);//恢复串口设置
void DefualtUartSet(void);//默认网络参数
uint32 GetUBR(uint8 UBR_I);//通过波特率等级得到波特率值
uint8 CheckMdlParam(void);//检查模型参数是否恰当
void MakeCtr2Factory(void);//使控制器恢复出厂设置
void RecBkModelParam(void);	//采用备份参数进行模型具体化
void DefualtVehIndex(void);//设置车辆序号记录集为默认值
void RecoVehIndex(void);//恢复车辆ID序号
void SaveVehIndex(void);//存储车辆ID
void RecoVehOutThresh(void);//恢复车辆出车相关阈值设置 
void DefualtVehOutTH(void);//默认出车阈值参数 
uint8 CheckVehOutTH(void);//检查出车阈值参数
void RecoWeightAtSet(void);//恢复与称台交互参数设置
void DefualtWeightAtSet(void);//默认与称台交互参数设置
uint8 CheckWeightAtSet(void);//检查称台交互参数

#endif
